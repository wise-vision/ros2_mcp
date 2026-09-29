#!/usr/bin/env bash
#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
# docs-sync patch producer. Runs the model over a merged ros2_mcp change and
# writes a unified diff against the website docs to $OUT/patch.diff.
#
# The model only edits a SCRATCH COPY of the website checkout with file tools
# (Read/Grep/Glob/Edit/Write): no shell, no network tools, no MCP servers.
# The patch is computed by git afterwards and restricted to src/content/docs;
# scripts/docs_sync_validate.py is the real gate, this is defence in depth.
#
# Usage: docs_sync_propose.sh <ros2_mcp-dir> <site-dir> <base-sha> <head-sha> <out-dir>
# Env:   ANTHROPIC_API_KEY (required), DOCS_SYNC_MODEL (default: sonnet),
#        CLAUDE_BIN (default: claude), DOCS_SYNC_DRY_RUN=1 (skip the model).
set -euo pipefail

if [ "$#" -ne 5 ]; then
  echo "usage: $0 <ros2_mcp-dir> <site-dir> <base-sha> <head-sha> <out-dir>" >&2
  exit 2
fi
REPO=$(cd "$1" && pwd)
SITE=$(cd "$2" && pwd)
BASE=$3
HEAD=$4
mkdir -p "$5"
OUT=$(cd "$5" && pwd)
PROMPT_FILE="$REPO/.github/docs-sync/prompt.md"
MAX_INPUT_BYTES=200000

if [ -z "$BASE" ] || [ "$BASE" = "0000000000000000000000000000000000000000" ]; then
  BASE=$(git -C "$REPO" rev-parse "$HEAD^")
fi

SCRATCH=$(mktemp -d)
trap 'rm -rf "$SCRATCH"' EXIT
WORK="$SCRATCH/work"
mkdir -p "$WORK"
# No .git in the model's workspace: a model-written .git/config (fsmonitor,
# pager, hooks) must never be read by a later git command.
tar -C "$SITE" --exclude=./.git -cf - . | tar -C "$WORK" -xf -
IN="$WORK/.docs-sync-input"
mkdir -p "$IN"

git -C "$REPO" diff "$BASE" "$HEAD" -- server README.md installation \
  | head -c "$MAX_INPUT_BYTES" > "$IN/change.diff"
git -C "$REPO" log --format='%H%n%B%n----' "$BASE..$HEAD" \
  | head -c 20000 > "$IN/commits.txt"
if [ -f "$REPO/docs/generated/tools.md" ]; then
  head -c "$MAX_INPUT_BYTES" "$REPO/docs/generated/tools.md" > "$IN/tools.md"
fi

if [ ! -s "$IN/change.diff" ]; then
  echo "no server/README/installation change between $BASE and $HEAD"
  : > "$OUT/patch.diff"
  exit 0
fi

if [ "${DOCS_SYNC_DRY_RUN:-0}" != "1" ]; then
  (
    cd "$WORK"
    "${CLAUDE_BIN:-claude}" -p --bare --restricted --strict-mcp-config --no-session-persistence \
      --disable-slash-commands --permission-mode acceptEdits --permission-prompts none \
      --model "${DOCS_SYNC_MODEL:-sonnet}" --max-budget-usd 2 \
      --tools "Read,Grep,Glob,Edit,Write" \
      --disallowedTools "Bash,WebFetch,WebSearch,NotebookEdit,Task" \
      --system-prompt "$(cat "$PROMPT_FILE")" \
      "Update the docs for the change in .docs-sync-input/ following your rules." \
      > "$OUT/model-reply.txt"
  )
fi

# Diff ONLY the docs tree, in a fresh repo built from the pristine website
# checkout (anything else the model wrote is discarded here, and would be
# rejected by the validator anyway). Plain files only; no model-controlled
# git config is ever read.
DIFF="$SCRATCH/diff"
mkdir -p "$DIFF/src/content"
export GIT_CONFIG_NOSYSTEM=1 GIT_CONFIG_GLOBAL=/dev/null
git -C "$DIFF" init -q
if [ -d "$SITE/src/content/docs" ]; then
  cp -r "$SITE/src/content/docs" "$DIFF/src/content/docs"
fi
git -C "$DIFF" add -A
git -C "$DIFF" -c user.name=docs-sync -c user.email=docs-sync@localhost commit -q --allow-empty -m base
rm -rf "$DIFF/src/content/docs"
if [ -d "$WORK/src/content/docs" ]; then
  find "$WORK/src/content/docs" -type l -delete
  cp -r "$WORK/src/content/docs" "$DIFF/src/content/docs"
fi
git -C "$DIFF" add -A
git -C "$DIFF" diff --cached --no-color --no-ext-diff --no-renames -- src/content/docs > "$OUT/patch.diff"
echo "patch.diff: $(wc -l < "$OUT/patch.diff") lines"

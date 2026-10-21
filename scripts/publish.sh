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
# Registry side of publishing Docker images. .github/workflows/docker_image.yml calls it, and it
# runs unchanged against any registry (a local `registry:2` container works), so the publish path
# can be rehearsed before it ever touches Docker Hub.
#
#   publish.sh check-immutable <repo> <tag>...
#       Fail if <tag>, <tag>-amd64 or <tag>-arm64 already exists. A registry error fails too:
#       "could not check" must never read as "free".
#   publish.sh push <repo> <local-image> <arch> <tag>...
#       Push the local image, the exact one that was smoke-tested, as <tag>-<arch>.
#   publish.sh merge <repo> <run-id> <experimental> <tag>...
#       Create each multi-arch <tag> from <tag>-amd64 + <tag>-arm64, but only when both variants
#       were built by this workflow run (label RUN_LABEL). Otherwise the arch whose build failed
#       would silently be filled in from an older run. For an experimental distro that tag is
#       skipped with a warning; for a supported one it is an error.
#
# Images must be built with: --label "io.github.wise-vision.ros2-mcp.run-id=<run-id>".
set -euo pipefail

RUN_LABEL=io.github.wise-vision.ros2-mcp.run-id

die() { echo "::error::$*" >&2; exit 1; }

# 0 = exists, 1 = does not exist, 2 = the registry could not say.
exists() {
  local out
  if out=$(docker buildx imagetools inspect "$1" 2>&1); then
    return 0
  fi
  if grep -qiE 'not found|manifest unknown|404' <<<"$out"; then
    return 1
  fi
  echo "::error::cannot check whether $1 exists: $out" >&2
  return 2
}

check_immutable() {
  local repo=$1 tag ref rc
  shift
  for tag in "$@"; do
    for ref in "$repo:$tag" "$repo:$tag-amd64" "$repo:$tag-arm64"; do
      rc=0
      exists "$ref" || rc=$?
      case $rc in
        0) die "$ref already exists; release tags are immutable. To finish a partially failed" \
               "release, use 'Re-run failed jobs' (it keeps this plan). Never 'Re-run all jobs'." ;;
        1) ;;
        *) exit 2 ;;
      esac
    done
  done
  echo "immutable tags are free: $*"
}

push() {
  local repo=$1 image=$2 arch=$3 tag
  shift 3
  case $arch in amd64|arm64) ;; *) die "unknown arch $arch" ;; esac
  for tag in "$@"; do
    docker tag "$image" "$repo:$tag-$arch"
    docker push "$repo:$tag-$arch"
  done
}

merge() {
  local repo=$1 run_id=$2 experimental=$3 tag arch got msg out
  shift 3
  for tag in "$@"; do
    for arch in amd64 arm64; do
      got=$(docker buildx imagetools inspect "$repo:$tag-$arch" \
              --format "{{ index .Image.Config.Labels \"$RUN_LABEL\" }}" 2>/dev/null || true)
      if [ "$got" != "$run_id" ]; then
        msg="$repo:$tag-$arch comes from run '${got:-unknown}', not this run ($run_id): its build failed or was skipped"
        if [ "$experimental" = true ]; then
          echo "::warning::$msg. Leaving $repo:$tag unchanged."
          continue 2
        fi
        die "$msg"
      fi
    done
    docker buildx imagetools create --tag "$repo:$tag" "$repo:$tag-amd64" "$repo:$tag-arm64"
    # Captured, not piped into head: under pipefail an early-closing head can SIGPIPE inspect.
    out=$(docker buildx imagetools inspect "$repo:$tag")
    sed -n 1,3p <<<"$out"
  done
}

cmd=${1:-}
shift || true
case $cmd in
  check-immutable) [ $# -ge 2 ] || die "usage: publish.sh check-immutable <repo> <tag>..."; check_immutable "$@" ;;
  push) [ $# -ge 4 ] || die "usage: publish.sh push <repo> <local-image> <arch> <tag>..."; push "$@" ;;
  merge) [ $# -ge 4 ] || die "usage: publish.sh merge <repo> <run-id> <experimental> <tag>..."; merge "$@" ;;
  *) die "usage: publish.sh check-immutable|push|merge ..." ;;
esac

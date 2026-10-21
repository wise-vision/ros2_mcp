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
#       Push the local image, the exact one that was smoke-tested, as <tag>-<arch>. A release's
#       immutable variant (<distro>-RELEASE-<arch>) that already exists is never pushed again:
#       if THIS run (RUN_ID) pushed it on an earlier attempt ("Re-run failed jobs"), it is kept
#       as is; if another run did, the push fails.
#   publish.sh merge <repo> <run-id> <experimental> <tag>...
#       Create each multi-arch <tag> from <tag>-amd64 + <tag>-arm64, but only when both variants
#       were built by this workflow run (label RUN_LABEL). Otherwise the arch whose build failed
#       would silently be filled in from an older run. For an experimental distro that tag is
#       skipped with a warning; for a supported one it is an error.
#
# merge also creates a floating tag dropped by that check when both its variants carry this
# run's label: this run already moved them, so the multi-arch tag must follow (the newer
# release's run is serialized behind this one and cannot have published yet).
#
# push and merge, when RELEASE is set (release and refresh runs), first re-check against the live
# tag list (git ls-remote RELEASE_REMOTE) that RELEASE is still the newest release. If a newer one
# exists, they publish only RELEASE's own immutable tags (*-RELEASE) and leave every floating tag
# alone. The plan was made when the run started; "Re-run failed jobs" on an older release reuses
# that plan, and without this check it would move :<distro> back. An unreadable tag list fails.
#
# Images must be built with: --label "io.github.wise-vision.ros2-mcp.run-id=<run-id>".
set -euo pipefail

RUN_LABEL=io.github.wise-vision.ros2-mcp.run-id

die() { echo "::error::$*" >&2; exit 1; }

RELEASE_RE='^[0-9]{4}([.][1-9][0-9]?)?$'   # same rule as scripts/plan_images.py

# Print, space-separated, the tags that may be published now (see the header).
publishable() {
  local release=${RELEASE:-} listing newest tag kept=()
  if [ -z "$release" ]; then
    echo "$*"
    return
  fi
  [ -n "${RELEASE_REMOTE:-}" ] || die "RELEASE is set but RELEASE_REMOTE is not"
  listing=$(git ls-remote --tags "$RELEASE_REMOTE") || die "cannot list release tags of $RELEASE_REMOTE"
  # sort -V orders 2610 < 2610.1 < 2610.2 < 2610.10 < 2611, like release_key in plan_images.py
  # Peeled entries (refs/tags/2610^{}) fail RELEASE_RE and drop out; the plain entry remains.
  # shellcheck disable=SC2001  # a per-line edit of a multi-line listing
  newest=$(sed 's#^.*refs/tags/##' <<<"$listing" | grep -E "$RELEASE_RE" | sort -uV | tail -n 1) || true
  [ -n "$newest" ] || die "no release tags found at $RELEASE_REMOTE"
  if [ "$newest" = "$release" ]; then
    echo "$*"
    return
  fi
  for tag in "$@"; do
    case $tag in *-"$release") kept+=("$tag") ;; esac
  done
  echo "::warning::$release is no longer the newest release ($newest is): publishing only ${kept[*]:-nothing}, leaving the floating tags alone." >&2
  echo "${kept[*]}"
}

# True when inspect output is the registry saying this exact reference has no manifest. buildx
# prints "ERROR: <normalized ref>: not found"; other registries say "manifest unknown". Anything
# else (a missing docker/buildx, a proxy's "404 page not found", a timeout) is NOT "missing".
registry_says_missing() {  # <ref> <inspect output>
  grep -qF "$1: not found" <<<"$2" || grep -qi 'manifest unknown' <<<"$2"
}

# 0 = exists, 1 = does not exist, 2 = the registry could not say.
exists() {
  local out rc=0
  out=$(docker buildx imagetools inspect "$1" 2>&1) || rc=$?
  if [ "$rc" = 0 ]; then
    return 0
  fi
  if [ "$rc" != 126 ] && [ "$rc" != 127 ] && registry_says_missing "$1" "$out"; then
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

# The run-id label of a pushed image; '' if the image or the label is absent. A registry that
# cannot answer is fatal (exit 2), never read as "absent".
label_of() {
  local out rc=0
  out=$(docker buildx imagetools inspect "$1" --format "{{ index .Image.Config.Labels \"$RUN_LABEL\" }}" 2>&1) || rc=$?
  if [ "$rc" = 0 ]; then
    printf '%s' "$out"
  elif [ "$rc" != 126 ] && [ "$rc" != 127 ] && registry_says_missing "$1" "$out"; then
    printf ''
  else
    echo "::error::cannot read the labels of $1: $out" >&2
    exit 2
  fi
}

push() {
  local repo=$1 image=$2 arch=$3 tag tags rc got
  shift 3
  case $arch in amd64|arm64) ;; *) die "unknown arch $arch" ;; esac
  tags=$(publishable "$@")
  read -ra tags <<<"$tags"
  for tag in "${tags[@]}"; do
    if [ -n "${RELEASE:-}" ] && [[ $tag == *-"$RELEASE" ]]; then
      rc=0
      exists "$repo:$tag-$arch" || rc=$?
      case $rc in
        0) got=$(label_of "$repo:$tag-$arch")
           if [ -n "${RUN_ID:-}" ] && [ "$got" = "$RUN_ID" ]; then
             echo "$repo:$tag-$arch was already pushed by this run; keeping it (immutable)."
             continue
           fi
           die "$repo:$tag-$arch already exists (run '${got:-unknown}'); release tags are immutable" ;;
        1) ;;
        *) exit 2 ;;
      esac
    fi
    docker tag "$image" "$repo:$tag-$arch"
    docker push "$repo:$tag-$arch"
  done
}

merge() {
  local repo=$1 run_id=$2 experimental=$3 tag arch got msg out kept dropped
  shift 3
  kept=" $(publishable "$@") "
  for tag in "$@"; do
    dropped=false
    [[ $kept == *" $tag "* ]] || dropped=true
    for arch in amd64 arm64; do
      got=$(label_of "$repo:$tag-$arch")
      if [ "$got" != "$run_id" ]; then
        if $dropped; then
          continue 2  # superseded floating tag this run never moved: leave it alone
        fi
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

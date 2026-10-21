#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""The publish path: scripts/publish.sh (driven through a stub `docker`) and the structure of
.github/workflows/docker_image.yml that keeps a broken, untested or forged image off the registry.
Pure Python + bash: no Docker, no network."""
import os
import pathlib
import stat
import subprocess

import pytest
import yaml

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
PUBLISH = REPO_ROOT / "scripts" / "publish.sh"
WORKFLOWS = REPO_ROOT / ".github" / "workflows"
LABEL = "io.github.wise-vision.ros2-mcp.run-id"
REPO = "example/ros2_mcp"

GIT_STUB = r"""#!/usr/bin/env bash
# Stub git: `ls-remote --tags <remote>` lists STUB_TAGS (annotated ones also with a peeled ^{} line).
echo "git $*" >> "$STUB_LOG"
[ -n "${STUB_GIT_FAIL:-}" ] && { echo "fatal: unable to access" >&2; exit 128; }
for t in $STUB_TAGS; do
  printf '%s\trefs/tags/%s\n' 0123456789abcdef0123456789abcdef01234567 "$t"
  printf '%s\trefs/tags/%s^{}\n' 89abcdef0123456789abcdef0123456789abcdef "$t"
done
"""

STUB = r"""#!/usr/bin/env bash
# Stub docker: imagetools inspect answers from env, everything is logged.
echo "$*" >> "$STUB_LOG"
if [ "$1 $2 $3" = "buildx imagetools inspect" ]; then
  ref=$4
  for e in $STUB_ERROR; do [ "$e" = "$ref" ] && { echo "dial tcp: i/o timeout" >&2; exit 1; }; done
  for e in $STUB_EXISTING; do
    if [ "$e" = "$ref" ]; then
      if [ "${5:-}" = "--format" ]; then
        for kv in $STUB_LABELS; do [ "${kv%%=*}" = "$ref" ] && { echo "${kv#*=}"; exit 0; }; done
        echo ""; exit 0
      fi
      echo "Name: $ref"; exit 0
    fi
  done
  echo "ERROR: $ref: not found" >&2; exit 1
fi
exit 0
"""


@pytest.fixture
def docker(tmp_path):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    stub = bin_dir / "docker"
    stub.write_text(STUB)
    stub.chmod(stub.stat().st_mode | stat.S_IEXEC)
    git = bin_dir / "git"
    git.write_text(GIT_STUB)
    git.chmod(git.stat().st_mode | stat.S_IEXEC)
    log = tmp_path / "docker.log"
    log.write_text("")

    def run(*args, existing=(), error=(), labels=None, release="", tags=(), git_fail=False):
        env = dict(os.environ, PATH=f"{bin_dir}:{os.environ['PATH']}", STUB_LOG=str(log),
                   STUB_EXISTING=" ".join(existing), STUB_ERROR=" ".join(error),
                   STUB_LABELS=" ".join(f"{k}={v}" for k, v in (labels or {}).items()),
                   RELEASE=release, RELEASE_REMOTE="https://example.invalid/repo",
                   STUB_TAGS=" ".join(tags), STUB_GIT_FAIL="1" if git_fail else "")
        r = subprocess.run(["bash", str(PUBLISH), *args], capture_output=True, text=True, env=env, check=False)
        return r, log.read_text().splitlines()
    return run


# --- check-immutable ---------------------------------------------------------------------------

def test_check_immutable_passes_when_nothing_exists(docker):
    r, calls = docker("check-immutable", REPO, "jazzy-2610.1", "humble-2610.1")
    assert r.returncode == 0, r.stderr
    assert len([c for c in calls if "imagetools inspect" in c]) == 6  # tag, -amd64, -arm64 each


@pytest.mark.parametrize("existing", ["jazzy-2610.1", "jazzy-2610.1-amd64", "jazzy-2610.1-arm64"])
def test_check_immutable_refuses_any_existing_variant(docker, existing):
    r, _ = docker("check-immutable", REPO, "jazzy-2610.1", existing=[f"{REPO}:{existing}"])
    assert r.returncode == 1
    assert "immutable" in r.stderr and "Re-run failed jobs" in r.stderr


def test_check_immutable_fails_closed_when_the_registry_cannot_answer(docker):
    r, _ = docker("check-immutable", REPO, "jazzy-2610.1", error=[f"{REPO}:jazzy-2610.1-amd64"])
    assert r.returncode == 2
    assert "cannot check" in r.stderr


# --- push --------------------------------------------------------------------------------------

def test_push_pushes_the_tested_local_image_under_each_arch_tag(docker):
    r, calls = docker("push", REPO, "ros2_mcp", "amd64", "jazzy-2610.1", "jazzy")
    assert r.returncode == 0, r.stderr
    assert calls == [
        f"tag ros2_mcp {REPO}:jazzy-2610.1-amd64", f"push {REPO}:jazzy-2610.1-amd64",
        f"tag ros2_mcp {REPO}:jazzy-amd64", f"push {REPO}:jazzy-amd64",
    ]
    assert not any("build" in c for c in calls), "push must not rebuild"


# --- floating tags only for the release that is still the newest ------------------------------

def test_push_without_release_does_not_consult_git(docker):
    r, calls = docker("push", REPO, "img", "amd64", "jazzy-main")
    assert r.returncode == 0, r.stderr
    assert not any(c.startswith("git ") for c in calls)


def test_push_of_the_newest_release_publishes_floating_tags(docker):
    r, calls = docker("push", REPO, "img", "amd64", "jazzy-2610.1", "jazzy",
                      release="2610.1", tags=["2606", "2610", "2610.1"])
    assert r.returncode == 0, r.stderr
    assert f"push {REPO}:jazzy-amd64" in calls and f"push {REPO}:jazzy-2610.1-amd64" in calls


def test_rerun_of_an_older_release_keeps_floating_tags(docker):
    # "Re-run failed jobs" on 2610 after 2610.1 shipped: the cached plan still lists :jazzy.
    r, calls = docker("push", REPO, "img", "arm64", "jazzy-2610", "jazzy", "latest",
                      release="2610", tags=["2610", "2610.1"])
    assert r.returncode == 0, r.stderr
    pushed = [c for c in calls if c.startswith("push ")]
    assert pushed == [f"push {REPO}:jazzy-2610-arm64"]
    assert "no longer the newest" in r.stderr


def test_merge_of_an_older_release_keeps_floating_tags(docker):
    existing, labels = _variants("jazzy-2610", "42", "42")
    existing.append(f"{REPO}:jazzy-2610")
    r, calls = docker("merge", REPO, "42", "false", "jazzy-2610", "jazzy", existing=existing,
                      labels=labels, release="2610", tags=["2610", "2610.10", "2610.2"])
    assert r.returncode == 0, r.stderr
    creates = [c for c in calls if "imagetools create" in c]
    assert creates == [f"buildx imagetools create --tag {REPO}:jazzy-2610 {REPO}:jazzy-2610-amd64 {REPO}:jazzy-2610-arm64"]


def test_rolling_only_floating_older_release_publishes_nothing(docker):
    r, calls = docker("push", REPO, "img", "amd64", "rolling", release="2610", tags=["2610", "2610.1"])
    assert r.returncode == 0, r.stderr
    assert not any(c.startswith("push ") for c in calls)


def test_newest_is_numeric_not_lexical(docker):
    # 2610.10 > 2610.9 > 2610.2: a lexical sort would call 2610.9 the newest.
    r, calls = docker("push", REPO, "img", "amd64", "jazzy-2610.10", "jazzy",
                      release="2610.10", tags=["2610.2", "2610.9", "2610.10", "v9999", "2610.rc1"])
    assert r.returncode == 0, r.stderr
    assert f"push {REPO}:jazzy-amd64" in calls


def test_unreadable_tag_list_fails_closed(docker):
    r, calls = docker("push", REPO, "img", "amd64", "jazzy-2610.1", "jazzy", release="2610.1",
                      tags=["2610.1"], git_fail=True)
    assert r.returncode == 1
    assert not any(c.startswith("push ") for c in calls)


def test_push_rejects_unknown_arch(docker):
    r, _ = docker("push", REPO, "ros2_mcp", "riscv64", "jazzy")
    assert r.returncode == 1


# --- merge -------------------------------------------------------------------------------------

def _variants(tag, run_amd, run_arm):
    refs = {f"{REPO}:{tag}-amd64": run_amd, f"{REPO}:{tag}-arm64": run_arm}
    return list(refs), refs


def test_merge_creates_multiarch_tag_from_this_runs_variants(docker):
    existing, labels = _variants("jazzy", "42", "42")
    existing.append(f"{REPO}:jazzy")  # what imagetools create produced; merge inspects it after
    r, calls = docker("merge", REPO, "42", "false", "jazzy", existing=existing, labels=labels)
    assert r.returncode == 0, r.stderr
    assert f"buildx imagetools create --tag {REPO}:jazzy {REPO}:jazzy-amd64 {REPO}:jazzy-arm64" in calls


def test_merge_refuses_a_stale_variant_for_a_supported_distro(docker):
    existing, labels = _variants("jazzy", "42", "41")  # arm64 failed this run; last week's remains
    r, calls = docker("merge", REPO, "42", "false", "jazzy", existing=existing, labels=labels)
    assert r.returncode == 1
    assert "not this run" in r.stderr
    assert not any("imagetools create" in c for c in calls)


def test_merge_skips_a_stale_variant_for_an_experimental_distro(docker):
    existing, labels = _variants("rolling", "41", "42")
    r, calls = docker("merge", REPO, "42", "true", "rolling", existing=existing, labels=labels)
    assert r.returncode == 0
    assert "::warning::" in r.stdout
    assert not any("imagetools create" in c for c in calls)


def test_merge_treats_a_missing_variant_as_stale(docker):
    r, calls = docker("merge", REPO, "42", "false", "jazzy", existing=[f"{REPO}:jazzy-amd64"],
                      labels={f"{REPO}:jazzy-amd64": "42"})
    assert r.returncode == 1
    assert not any("imagetools create" in c for c in calls)


@pytest.mark.parametrize("args", [[], ["bogus"], ["push", REPO], ["merge", REPO, "42"]])
def test_usage_errors(docker, args):
    r, _ = docker(*args)
    assert r.returncode == 1


# --- workflow structure ------------------------------------------------------------------------

def _wf(name):
    return yaml.safe_load((WORKFLOWS / name).read_text())


DOCKER = _wf("docker_image.yml")
PUBLISH_JOBS = ["build_and_push_amd64", "build_and_push_arm64", "merge_manifests"]


@pytest.mark.parametrize("job", PUBLISH_JOBS)
def test_publish_jobs_never_run_for_pull_requests(job):
    # On a pull request the planner is the PR's own code, so its outputs cannot be the only gate.
    assert "github.event_name != 'pull_request'" in DOCKER["jobs"][job]["if"]


def test_pull_request_build_job_is_pull_request_only():
    assert "github.event_name == 'pull_request'" in DOCKER["jobs"]["build_image"]["if"]


@pytest.mark.parametrize("name", ["docker_image.yml", "server_tests.yml", "release.yml"])
def test_no_expression_is_interpolated_into_a_script(name):
    for job_name, job in _wf(name)["jobs"].items():
        for step in job.get("steps", []):
            assert "${{" not in step.get("run", ""), f"{name}:{job_name}:{step.get('name')} interpolates"


def test_floating_tag_movers_are_serialized():
    conc = DOCKER["concurrency"]
    assert "floating-tags" in conc["group"]
    for trigger in ("'schedule'", "github.ref_type == 'tag'", "inputs.release"):
        assert trigger in conc["group"]
    assert conc["cancel-in-progress"] == "${{ github.event_name == 'pull_request' }}"


def test_nothing_rebuilds_between_smoke_test_and_push():
    text = (WORKFLOWS / "docker_image.yml").read_text()
    assert "--push" not in text, "a --push build is a second, untested build"
    for job in ("build_and_push_amd64", "build_and_push_arm64"):
        spec = DOCKER["jobs"][job]
        # unique per distro and run, so jobs sharing a Docker daemon can't swap images
        assert "${{ matrix.distro }}" in spec["env"]["LOCAL_IMAGE"]
        assert "${{ github.run_id }}" in spec["env"]["LOCAL_IMAGE"]
        runs = [s.get("run", "") for s in spec["steps"]]
        build = next(r for r in runs if "buildx build" in r)
        assert "--load" in build and "--pull" in build and '-t "$LOCAL_IMAGE"' in build
        assert LABEL in build and "org.opencontainers.image.revision" in build
        smoke = [r for r in runs if "smoke_e2e.sh" in r]
        assert smoke and all('"$LOCAL_IMAGE"' in r for r in smoke)
        push = next(r for r in runs if "publish.sh push" in r)
        assert '"$REPO" "$LOCAL_IMAGE" ' in push, f"{job} must push the image it built and tested"


@pytest.mark.parametrize("job", ["build_and_push_arm64", "build_image_arm64"])
def test_arm64_builds_natively(job):
    spec = DOCKER["jobs"][job]
    assert spec["runs-on"] == "ubuntu-24.04-arm"
    assert "container" not in spec
    assert not any(s.get("uses", "").startswith("docker/setup-qemu-action") for s in spec["steps"])


@pytest.mark.parametrize("job", ["build_and_push_amd64", "build_and_push_arm64"])
def test_both_arches_get_the_end_to_end_smoke_before_push(job):
    runs = [s.get("run", "") for s in DOCKER["jobs"][job]["steps"]]
    smoke = next(i for i, r in enumerate(runs) if 'smoke_e2e.sh "$LOCAL_IMAGE"' in r)
    push = next(i for i, r in enumerate(runs) if "publish.sh push" in r)
    assert smoke < push


def test_pull_requests_build_and_smoke_both_arches():
    for job, runner in (("build_image", "ubuntu-latest"), ("build_image_arm64", "ubuntu-24.04-arm")):
        spec = DOCKER["jobs"][job]
        assert spec["runs-on"] == runner
        assert "github.event_name == 'pull_request'" in spec["if"]
        runs = " ".join(s.get("run", "") for s in spec["steps"])
        assert "smoke_e2e.sh ros2_mcp" in runs and "--type-support-only" in runs


def test_immutable_check_runs_in_plan_job_through_publish_script():
    runs = [s.get("run", "") for s in DOCKER["jobs"]["plan"]["steps"]]
    assert any("publish.sh check-immutable" in r for r in runs)


def test_merge_only_accepts_this_runs_variants():
    runs = [s.get("run", "") for s in DOCKER["jobs"]["merge_manifests"]["steps"]]
    assert any('publish.sh merge "$REPO" "$RUN_ID"' in r for r in runs)


@pytest.mark.parametrize("job", PUBLISH_JOBS)
def test_publish_steps_recheck_that_the_release_is_still_newest(job):
    steps = [s for s in DOCKER["jobs"][job]["steps"] if "publish.sh push" in s.get("run", "")
             or "publish.sh merge" in s.get("run", "")]
    assert steps
    for s in steps:
        assert s["env"]["RELEASE"] == "${{ needs.plan.outputs.release }}"
        assert "github.repository" in s["env"]["RELEASE_REMOTE"]
    assert DOCKER["jobs"]["plan"]["outputs"]["release"] == "${{ steps.plan.outputs.release }}"


def test_one_failed_distro_does_not_freeze_the_others():
    cond = DOCKER["jobs"]["merge_manifests"]["if"]
    assert "always()" in cond and "!cancelled()" in cond and "needs.plan.result == 'success'" in cond


@pytest.mark.parametrize("name", ["docker_image.yml", "release.yml"])
def test_third_party_actions_are_pinned_to_a_commit(name):
    import re
    for job in _wf(name)["jobs"].values():
        for step in job.get("steps", []):
            uses = step.get("uses", "")
            if uses and not uses.startswith("actions/"):
                assert re.fullmatch(r"[\w.-]+/[\w.-]+@[0-9a-f]{40}", uses), uses

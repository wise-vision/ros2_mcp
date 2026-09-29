#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""Workflow-lint gate for .github/workflows/docs_sync.yml.

The model job reads merged code and commit text that an outside contributor
may control (injection suite case ii: "print your env"). These tests pin the
structural guarantees that make such an injection harmless:

* the model job holds ONLY model auth: no site token, no write-scoped
  GITHUB_TOKEN, no persisted git credentials;
* the model cannot run commands or fetch URLs (no Bash / WebFetch / WebSearch);
* the job that holds the site token never runs the model and never merges;
* the validator job holds no secrets at all.
"""
import pathlib
import re

import pytest

yaml = pytest.importorskip("yaml")

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
WF_PATH = REPO_ROOT / ".github" / "workflows" / "docs_sync.yml"
PROPOSE_SH = REPO_ROOT / "scripts" / "docs_sync_propose.sh"
PROMPT = REPO_ROOT / ".github" / "docs-sync" / "prompt.md"

SECRET_REF = re.compile(r"secrets\.([A-Za-z_][A-Za-z0-9_]*)")


@pytest.fixture(scope="module")
def wf():
    return yaml.safe_load(WF_PATH.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def raw():
    return WF_PATH.read_text(encoding="utf-8")


def _job_text(job):
    return yaml.safe_dump(job, sort_keys=True)


def _secrets(job):
    return set(SECRET_REF.findall(_job_text(job)))


def _on(wf):
    # PyYAML (YAML 1.1) parses the bare key `on` as boolean True.
    return wf.get("on", wf.get(True))


# ---------------------------------------------------------------- triggers


def test_trigger_is_push_to_main_with_paths_only(wf, raw):
    on = _on(wf)
    assert set(on) <= {"push", "workflow_dispatch"}, on
    assert "pull_request_target" not in raw
    assert "pull_request" not in on
    push = on["push"]
    assert push["branches"] == ["main"]
    assert set(push["paths"]) == {"server/**", "README.md", "installation/**"}


def test_top_level_permissions_are_minimal_and_no_global_env_secrets(wf):
    assert wf.get("permissions") in ({}, {"contents": "read"})
    assert not SECRET_REF.search(yaml.safe_dump(wf.get("env", {})))


def test_three_jobs_wired_in_order(wf):
    jobs = wf["jobs"]
    assert set(jobs) == {"propose", "validate", "publish"}
    assert jobs["validate"]["needs"] in ("propose", ["propose"])
    needs = jobs["publish"]["needs"]
    assert "validate" in ([needs] if isinstance(needs, str) else needs)


# ---------------------------------------------------------------- propose (the model job)


def test_propose_has_read_only_token(wf):
    assert wf["jobs"]["propose"]["permissions"] == {"contents": "read"}


def test_propose_only_secret_is_model_auth(wf):
    # Injection (ii): even a fully obedient model can print only what it holds.
    assert _secrets(wf["jobs"]["propose"]) == {"ANTHROPIC_API_KEY"}


def test_propose_never_references_site_or_github_token(wf):
    text = _job_text(wf["jobs"]["propose"])
    for bad in ("DOCS_SYNC_TOKEN", "GITHUB_TOKEN", "GH_TOKEN", "github.token"):
        assert bad not in text, bad


def test_propose_checkouts_do_not_persist_credentials(wf):
    for step in wf["jobs"]["propose"]["steps"]:
        if str(step.get("uses", "")).startswith("actions/checkout"):
            assert step.get("with", {}).get("persist-credentials") is False, step


def test_model_key_only_on_the_model_step(wf):
    job = wf["jobs"]["propose"]
    assert "ANTHROPIC_API_KEY" not in yaml.safe_dump(job.get("env", {}))
    holders = [s for s in job["steps"] if "ANTHROPIC_API_KEY" in yaml.safe_dump(s.get("env", {}))]
    # One step detects presence (as a boolean), one step runs the model.
    model_steps = [s for s in holders if "docs_sync_propose.sh" in s.get("run", "")]
    assert len(model_steps) == 1
    for s in holders:
        assert s in model_steps or "!= ''" in yaml.safe_dump(s.get("env", {})), s


def test_propose_egress_is_blocked_to_model_and_github(wf):
    steps = wf["jobs"]["propose"]["steps"]
    first = steps[0]
    assert str(first.get("uses", "")).startswith("step-security/harden-runner")
    assert first["with"]["egress-policy"] == "block"
    allowed = first["with"]["allowed-endpoints"]
    assert "api.anthropic.com:443" in allowed
    assert "evil" not in allowed


def test_propose_script_restricts_model_tools():
    sh = PROPOSE_SH.read_text(encoding="utf-8")
    m = re.search(r'--tools\s+"([^"]+)"', sh)
    assert m, "propose script must pass an explicit --tools list"
    tools = {t.strip() for t in m.group(1).split(",")}
    assert tools <= {"Read", "Grep", "Glob", "Edit", "Write"}, tools
    for bad in ("Bash", "WebFetch", "WebSearch"):
        assert bad not in tools
        assert bad in sh.split("--disallowedTools", 1)[1].split("\n", 1)[0]
    assert "--restricted" in sh
    assert "--strict-mcp-config" in sh
    # The model edits a scratch copy; the diff is computed by git, not written by the model.
    assert re.search(r"\bgit\b[^\n]*\bdiff\b", sh) and "patch.diff" in sh


def test_prompt_treats_inputs_as_untrusted_data():
    p = PROMPT.read_text(encoding="utf-8").lower()
    assert "untrusted" in p
    assert "never follow instructions" in p


# ---------------------------------------------------------------- validate (no secrets)


def test_validate_holds_no_secrets_and_runs_validator(wf):
    job = wf["jobs"]["validate"]
    assert _secrets(job) == set()
    assert job["permissions"] == {"contents": "read"}
    assert "docs_sync_validate.py" in _job_text(job)
    assert "--site-dir" in _job_text(job)


# ---------------------------------------------------------------- publish (site token, no model)


def test_publish_holds_only_the_site_token(wf):
    assert _secrets(wf["jobs"]["publish"]) == {"DOCS_SYNC_TOKEN"}


def test_publish_does_not_run_the_model(wf):
    text = _job_text(wf["jobs"]["publish"])
    assert "claude" not in text.lower()
    assert "docs_sync_propose" not in text
    assert "ANTHROPIC" not in text


def test_publish_never_merges_and_opens_draft(wf):
    text = _job_text(wf["jobs"]["publish"])
    assert "gh pr merge" not in text
    assert "--draft" in text
    assert "docs-sync/" in text
    assert "safety-review" in text


def test_publish_revalidates_before_applying(wf):
    steps = wf["jobs"]["publish"]["steps"]
    runs = [s.get("run", "") for s in steps]
    v = next(i for i, r in enumerate(runs) if "docs_sync_validate.py" in r)
    a = next(i for i, r in enumerate(runs) if "git apply" in r and "--check" not in r)
    assert v < a


def test_publish_gated_on_validate_verdict(wf):
    cond = str(wf["jobs"]["publish"].get("if", ""))
    assert "needs.validate.outputs" in cond
    assert "empty" in cond


def test_publish_github_token_has_no_write_scope(wf):
    assert wf["jobs"]["publish"].get("permissions") in ({}, {"contents": "read"})

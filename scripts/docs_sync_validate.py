#!/usr/bin/env python3
#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""Deterministic gate for model-produced docs patches (docs-sync loop).

The ``propose`` job of ``.github/workflows/docs_sync.yml`` runs a model over a
merged diff of this repo and writes ``patch.diff``: a unified diff against the
``wise-vision/wisevision-website`` docs. The model read text an outside
contributor may control, so its output is UNTRUSTED. This script is the only
thing between that output and the ``publish`` job that holds the website token.

A patch passes only when ALL of these hold:

* it is a well-formed text unified diff (no binary, symlink or submodule entries);
* every touched path is a ``.md``/``.mdx`` file under ``src/content/docs/``;
* at most ``MAX_CHANGED_LINES`` (400) lines are added + removed;
* every URL in an added line is https on the allowlist (``ALLOWED_HOSTS``;
  GitHub only under ``github.com/wise-vision``), including markdown inline and
  reference-style links, autolinks, bare ``www.`` links, e-mail links and
  HTML-entity / backslash-escape obfuscations;
* no added line (or diff header) matches a secret pattern;
* no added line carries raw HTML (other than a small set of Starlight
  components), ``<script``/``<iframe``, MDX ``import``/``export`` (other than
  ``@astrojs/starlight/components``) or MDX ``{expressions}`` outside code
  fences;
* with ``--site-dir``: ``git apply --check`` succeeds against the website checkout.

It also decides whether the resulting PR needs the ``safety-review`` label: the
patch mentions security / safety / actuation / read-only mode, or names a tool
classified ``mutating`` in ``docs/generated/tools.json``.

Exit codes: 0 valid (possibly empty), 1 rejected, 2 usage error.
"""
from __future__ import annotations

import argparse
import html
import json
import pathlib
import posixpath
import re
import subprocess
import sys
from dataclasses import dataclass, field
from urllib.parse import urlsplit

DOCS_PREFIX = "src/content/docs/"
DOC_SUFFIXES = (".md", ".mdx")
MAX_CHANGED_LINES = 400

# Host suffix allowlist (the host itself or any subdomain of it).
ALLOWED_HOSTS = (
    "wisevision.tech",
    "docs.ros.org",
    "hub.docker.com",
    "modelcontextprotocol.io",
)
# github.com is allowed only for these org/user path prefixes.
ALLOWED_GITHUB_OWNERS = ("wise-vision",)

SECRET_PATTERNS = [
    ("aws-access-key", re.compile(r"\b(?:AKIA|ASIA)[0-9A-Z]{16}\b")),
    ("github-token", re.compile(r"\bgh[pousr]_[A-Za-z0-9]{30,}")),
    ("github-fine-grained-pat", re.compile(r"\bgithub_pat_[A-Za-z0-9_]{20,}")),
    ("anthropic-key", re.compile(r"\bsk-ant-[A-Za-z0-9_\-]{16,}")),
    ("openai-key", re.compile(r"\bsk-(?:proj-)?[A-Za-z0-9]{32,}")),
    ("google-key", re.compile(r"\bAIza[0-9A-Za-z_\-]{35}")),
    ("slack-token", re.compile(r"\bxox[abposr]-[A-Za-z0-9-]{10,}")),
    ("pem-block", re.compile(r"-----BEGIN [A-Z0-9 ]*-----")),
    ("bearer-token", re.compile(r"(?i)\bbearer\s+[A-Za-z0-9._~+/=\-]{20,}")),
    (
        "secret-env-name",
        re.compile(
            r"\b(?:DOCS_SYNC_TOKEN|ANTHROPIC_API_KEY|CLAUDE_CODE_OAUTH_TOKEN|GITHUB_TOKEN|GH_TOKEN"
            r"|CLOUDFLARE_API_TOKEN|CF_API_TOKEN|ACTIONS_RUNTIME_TOKEN|ACTIONS_ID_TOKEN_REQUEST_TOKEN"
            r"|ACTIONS_ID_TOKEN_REQUEST_URL|AWS_SECRET_ACCESS_KEY)\b"
        ),
    ),
]

# Starlight components a docs page may use without raw HTML.
ALLOWED_COMPONENTS = {
    "Aside", "Badge", "Card", "CardGrid", "Code", "FileTree", "Icon",
    "LinkButton", "LinkCard", "Steps", "TabItem", "Tabs",
}
ALLOWED_IMPORT = re.compile(
    r"""^import\s+\{[\sA-Za-z0-9_,]+\}\s+from\s+['"]@astrojs/starlight/components['"];?\s*$"""
)
ALWAYS_BANNED_HTML = re.compile(r"(?i)<\s*/?\s*(script|iframe|object|embed|frame|frameset)\b")

SAFETY_KEYWORDS = re.compile(
    r"(?i)\b(security|safety|actuation|actuat\w*|read[- ]?only|readonly|mutating|cmd_vel|e-?stop)\b"
)

# URL extraction (run on normalised text).
BARE_URL = re.compile(r"(?i)\b[a-z][a-z0-9+.\-]*://[^\s<>\"'`)\]]+")
WWW_URL = re.compile(r"(?i)(?<![\w/.@-])www\.[^\s<>\"'`)\]]+")
INLINE_LINK_DEST = re.compile(r"\]\(\s*<?([^)\s>]*)")
REF_DEF = re.compile(r"^\s{0,3}\[[^\]]+\]:\s*<?([^\s>]*)")
AUTOLINK = re.compile(r"<([a-zA-Z][a-zA-Z0-9+.\-]{1,31}:[^\s<>]*)>")
ATTR_URL = re.compile(r"(?i)\b(?:href|src|action|data|to|link)\s*=\s*[\"']?([^\"'\s>]*)")
EMAIL = re.compile(r"(?i)(?<![\w.+-])[\w.+-]+@([a-z0-9-]+(?:\.[a-z0-9-]+)+)")
SCHEME = re.compile(r"^([A-Za-z][A-Za-z0-9+.\-]*):")
TAG = re.compile(r"<\s*(/?)\s*([A-Za-z][\w.:\-]*)([^>]*)>?")
INLINE_CODE = re.compile(r"(`+)(.+?)\1")
BACKSLASH_ESCAPE = re.compile(r"\\([!-/:-@\[-`{-~])")
FENCE = re.compile(r"^\s{0,3}(```+|~~~+)")


@dataclass
class Result:
    ok: bool = True
    empty: bool = False
    errors: list[str] = field(default_factory=list)
    files: list[str] = field(default_factory=list)
    added: int = 0
    removed: int = 0
    safety_review: bool = False
    safety_reasons: list[str] = field(default_factory=list)

    def fail(self, msg: str) -> None:
        self.ok = False
        self.errors.append(msg)

    def as_dict(self) -> dict:
        return {
            "ok": self.ok,
            "empty": self.empty,
            "errors": self.errors,
            "files": self.files,
            "added": self.added,
            "removed": self.removed,
            "safety_review": self.safety_review,
            "safety_reasons": self.safety_reasons,
        }


@dataclass
class Hunk:
    old_start: int
    lines: list[tuple[str, str]]  # (kind, text) kind in {'+', '-', ' '}


@dataclass
class FileDiff:
    paths: set[str]
    new_path: str | None
    header: list[str]
    hunks: list[Hunk]
    new_file: bool = False
    deleted: bool = False


# ---------------------------------------------------------------- parsing


def _strip_prefix(p: str) -> str | None:
    p = p.strip()
    if p.startswith('"') and p.endswith('"'):
        p = p[1:-1]
    p = p.split("\t", 1)[0]
    if p == "/dev/null":
        return None
    if p.startswith(("a/", "b/")):
        p = p[2:]
    return p


HUNK_RE = re.compile(r"^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@")


def parse_patch(text: str, res: Result) -> list[FileDiff]:
    lines = text.splitlines()
    files: list[FileDiff] = []
    cur: FileDiff | None = None
    hunk: Hunk | None = None
    for n, line in enumerate(lines, 1):
        if line.startswith("diff --git "):
            m = re.match(r"^diff --git (\S+) (\S+)$", line)
            if not m:
                res.fail(f"line {n}: unparseable diff header (path with spaces?)")
                return files
            a, b = _strip_prefix(m.group(1)), _strip_prefix(m.group(2))
            cur = FileDiff(paths={p for p in (a, b) if p}, new_path=b, header=[line], hunks=[])
            files.append(cur)
            hunk = None
            continue
        if cur is None:
            if line.strip():
                res.fail(f"line {n}: text before the first 'diff --git' header (not a unified diff)")
                return files
            continue
        if hunk is None or line.startswith(("--- ", "+++ ")) and not hunk.lines:
            # header region of a file entry
            if line.startswith("@@"):
                m = HUNK_RE.match(line)
                if not m:
                    res.fail(f"line {n}: bad hunk header")
                    return files
                hunk = Hunk(old_start=int(m.group(1)), lines=[])
                cur.hunks.append(hunk)
                continue
            cur.header.append(line)
            if line.startswith(("--- ", "+++ ")):
                p = _strip_prefix(line[4:])
                if p:
                    cur.paths.add(p)
            elif line.startswith(("rename from ", "rename to ", "copy from ", "copy to ")):
                cur.paths.add(line.split(" ", 2)[2].strip())
            elif line.startswith("new file mode"):
                cur.new_file = True
            elif line.startswith("deleted file mode"):
                cur.deleted = True
            continue
        if line.startswith("@@"):
            m = HUNK_RE.match(line)
            if not m:
                res.fail(f"line {n}: bad hunk header")
                return files
            hunk = Hunk(old_start=int(m.group(1)), lines=[])
            cur.hunks.append(hunk)
        elif line.startswith("\\"):
            continue  # "\ No newline at end of file"
        elif line == "" or line[0] in "+- ":
            kind = line[0] if line else " "
            hunk.lines.append((kind, line[1:]))
        else:
            res.fail(f"line {n}: unexpected line inside hunk")
            return files
    return files


# ---------------------------------------------------------------- checks


def _path_ok(p: str) -> bool:
    if p.startswith("/") or "\\" in p or any(ord(c) < 32 for c in p):
        return False
    parts = p.split("/")
    if any(part in ("", ".", "..") for part in parts):
        return False
    norm = posixpath.normpath(p)
    return norm == p and norm.startswith(DOCS_PREFIX) and norm.endswith(DOC_SUFFIXES)


def normalise(s: str) -> str:
    """Undo the obfuscations markdown/HTML renderers undo for us."""
    prev = None
    out = s
    while prev != out:  # nested entity encodings (&amp;#104;)
        prev = out
        out = html.unescape(out)
    return BACKSLASH_ESCAPE.sub(r"\1", out)


def host_allowed(host: str) -> bool:
    host = host.lower().rstrip(".")
    return any(host == h or host.endswith("." + h) for h in ALLOWED_HOSTS)


def url_problem(url: str) -> str | None:
    """None when `url` may appear in docs, else the reason it may not."""
    url = url.strip()
    if not url or url.startswith("#"):
        return None
    if any(ord(c) < 33 for c in url) or "\\" in url:
        return "control character or backslash in link"
    if url.startswith("//"):
        return "protocol-relative link"
    m = SCHEME.match(url)
    if not m:
        return None  # relative link on the docs site itself
    scheme = m.group(1).lower()
    if scheme == "mailto":
        dom = url.split("@", 1)[-1].split("?", 1)[0]
        return None if "@" in url and host_allowed(dom) else "mailto outside allowlist"
    if scheme != "https":
        return f"scheme '{scheme}' not allowed (https only)"
    parts = urlsplit(url)
    if "@" in parts.netloc:
        return "userinfo in URL"
    host = (parts.hostname or "").lower()
    if host in ("github.com", "www.github.com"):
        owner = parts.path.lstrip("/").split("/", 1)[0].lower()
        if owner in ALLOWED_GITHUB_OWNERS:
            return None
        return "github.com outside the wise-vision org"
    if host_allowed(host):
        return None
    return f"host '{host}' not on allowlist"


def extract_urls(s: str) -> list[str]:
    urls: list[str] = []
    urls += BARE_URL.findall(s)
    urls += ["https://" + w for w in WWW_URL.findall(s)]
    urls += INLINE_LINK_DEST.findall(s)
    urls += REF_DEF.findall(s)
    urls += AUTOLINK.findall(s)
    urls += ATTR_URL.findall(s)
    urls += ["mailto:x@" + d for d in EMAIL.findall(s)]
    return [u for u in urls if u]


def check_secrets(text: str, where: str, res: Result) -> None:
    for name, rx in SECRET_PATTERNS:
        if rx.search(text) or rx.search(normalise(text)):
            res.fail(f"{where}: secret pattern '{name}'")


def check_prose(text: str, where: str, res: Result) -> None:
    """MDX/HTML checks for a line that renders as markdown (not in a code fence)."""
    stripped = INLINE_CODE.sub("", normalise(text))
    s = stripped.strip()
    if s.startswith("import ") or s.startswith("import{"):
        if not ALLOWED_IMPORT.match(s):
            res.fail(f"{where}: MDX import not allowed (only @astrojs/starlight/components)")
        return
    if re.match(r"^export\b", s):
        res.fail(f"{where}: MDX export not allowed")
    if "{" in stripped or "}" in stripped:
        res.fail(f"{where}: MDX expression braces outside a code block")
    for m in TAG.finditer(stripped):
        name, attrs = m.group(2), m.group(3) or ""
        if m.group(1) == "" and AUTOLINK.fullmatch(m.group(0)):
            continue  # an autolink <https://...>, URL-checked elsewhere
        if name in ALLOWED_COMPONENTS and not re.search(r"(?i)\bon\w+\s*=", attrs):
            continue
        res.fail(f"{where}: raw HTML/JSX tag <{m.group(1)}{name}> not allowed")


def _fence_state_at(site_dir: pathlib.Path | None, path: str, old_start: int, new_file: bool):
    """True/False = inside/outside a code fence before `old_start`; None = unknown."""
    if new_file:
        return False
    if site_dir is None:
        return None
    f = site_dir / path
    try:
        old = f.read_text(encoding="utf-8").splitlines()
    except OSError:
        return None
    inside = False
    for line in old[: max(old_start - 1, 0)]:
        if FENCE.match(line):
            inside = not inside
    return inside


def check_file(fd: FileDiff, res: Result, site_dir, mutating: set[str]) -> None:
    for i, h in enumerate(fd.header):
        check_secrets(h, f"header of {fd.new_path}", res)
    for p in sorted(fd.paths):
        if not _path_ok(p):
            res.fail(f"path not allowed: {p!r} (only {DOCS_PREFIX}**/*.md|mdx)")
    for h in fd.header:
        if h.startswith(("new file mode", "old mode", "new mode", "deleted file mode")):
            if not h.rstrip().endswith(("100644",)):
                res.fail(f"{fd.new_path}: file mode not allowed ({h.strip()})")
        if "GIT binary patch" in h or h.startswith("Binary files"):
            res.fail(f"{fd.new_path}: binary patch not allowed (path check failed)")
    if fd.new_path:
        res.files.append(fd.new_path)

    for hi, h in enumerate(fd.hunks):
        inside = _fence_state_at(site_dir, fd.new_path or "", h.old_start, fd.new_file)
        for kind, text in h.lines:
            where = f"{fd.new_path} hunk {hi + 1}"
            if kind in "+-":
                if kind == "+":
                    res.added += 1
                else:
                    res.removed += 1
                if SAFETY_KEYWORDS.search(text) or any(t in text for t in mutating):
                    res.safety_review = True
                    res.safety_reasons.append(f"{where}: {text.strip()[:80]}")
            if kind == "-":
                continue
            fence_line = bool(FENCE.match(text))
            if kind == "+":
                norm = normalise(text)
                check_secrets(text, where, res)
                for u in extract_urls(text) + extract_urls(norm):
                    why = url_problem(u)
                    if why:
                        res.fail(f"{where}: disallowed URL {u!r}: {why}")
                if ALWAYS_BANNED_HTML.search(norm):
                    res.fail(f"{where}: <script>/<iframe>-class HTML not allowed")
                # Unknown fence state -> treat every added line as prose (strict).
                if not fence_line and inside is not True:
                    check_prose(text, where, res)
            if fence_line and inside is not None:
                inside = not inside
    path_hit = any(SAFETY_KEYWORDS.search(p) for p in fd.paths)
    if path_hit:
        res.safety_review = True
        res.safety_reasons.append(f"path: {fd.new_path}")


def apply_check(patch: str, site_dir: pathlib.Path, res: Result) -> None:
    r = subprocess.run(
        ["git", "apply", "--check", "--verbose", "-"],
        cwd=site_dir, input=patch, capture_output=True, text=True,
    )
    if r.returncode != 0:
        res.fail("git apply --check failed: " + (r.stderr.strip() or r.stdout.strip())[:500])


def load_mutating_tools(tools_json: pathlib.Path) -> set[str]:
    try:
        data = json.loads(pathlib.Path(tools_json).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return set()
    return {t["name"] for t in data.get("tools", []) if t.get("mutating")}


def validate(patch: str, site_dir=None, mutating_tools=None) -> Result:
    res = Result()
    if not patch.strip():
        res.empty = True
        return res
    mutating = set(mutating_tools or ())
    site = pathlib.Path(site_dir) if site_dir else None
    files = parse_patch(patch, res)
    if res.ok and not files:
        res.fail("no file entries in patch")
    for fd in files:
        check_file(fd, res, site, mutating)
    total = res.added + res.removed
    if total > MAX_CHANGED_LINES:
        res.fail(f"patch changes {total} lines; cap is {MAX_CHANGED_LINES}")
    if res.ok and site is not None:
        apply_check(patch, site, res)
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("patch", help="unified diff produced by the propose job")
    ap.add_argument("--site-dir", help="checkout of wise-vision/wisevision-website for git apply --check")
    ap.add_argument("--tools-json", default=str(pathlib.Path(__file__).resolve().parents[1] / "docs/generated/tools.json"))
    ap.add_argument("--verdict-out", help="write the JSON verdict here")
    a = ap.parse_args(argv)
    try:
        text = pathlib.Path(a.patch).read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as e:
        print(f"cannot read patch: {e}", file=sys.stderr)
        return 2
    res = validate(text, site_dir=a.site_dir, mutating_tools=load_mutating_tools(pathlib.Path(a.tools_json)))
    verdict = res.as_dict()
    if a.verdict_out:
        pathlib.Path(a.verdict_out).write_text(json.dumps(verdict, indent=2) + "\n")
    if res.ok:
        state = "EMPTY (nothing to publish)" if res.empty else f"OK: {len(res.files)} file(s), +{res.added}/-{res.removed}"
        print(f"docs-sync patch {state}; safety_review={res.safety_review}")
        return 0
    print("docs-sync patch REJECTED:")
    for e in res.errors:
        print(f"  - {e}")
    return 1


if __name__ == "__main__":
    sys.exit(main())

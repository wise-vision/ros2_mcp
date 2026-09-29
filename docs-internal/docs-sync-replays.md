# docs-sync manual replays (wvrevive_2809 W8.1)

Before the workflow went in, the patch producer (`scripts/docs_sync_propose.sh`)
was run by hand over the last three merged PRs to `main` that touched
`server/**` or `README.md`, against a fresh anonymous clone of
`wise-vision/wisevision-website@main`. Each output then went through
`scripts/docs_sync_validate.py --site-dir <site> --tools-json docs/generated/tools.json`.

Local-run difference from CI: CI runs `claude -p --bare`, which only accepts
`ANTHROPIC_API_KEY`. The local host has no API key, so a wrapper dropped
`--bare` and used the local Claude Code login instead. All other flags were the
same (`--restricted --strict-mcp-config --tools Read,Grep,Glob,Edit,Write
--disallowedTools Bash,WebFetch,WebSearch,NotebookEdit,Task --permission-mode
acceptEdits --permission-prompts none --model sonnet`). Claude Code 2.1.283.

| PR | merge SHA | change | patch | validator | would a reviewer merge it? |
|---|---|---|---|---|---|
| #63 | 47b4918 | Pro tools merged into the public repo, rename to ROS2 MCP, read-only mode, Humble+Jazzy CI | +9/-0, `quickstart.mdx` | OK, `safety_review=True` | **Yes, after one edit.** It is accurate (env var, CLI flag, tools not registered, unknown-tool error all match `server/`). The PR gets the `safety-review` label because it is about read-only mode. A reviewer would probably also list the five hidden tools by name, as the README does. |
| #61 | 66f3f2d | README: commercial contact and forklift rosbag demo quickstart | empty | EMPTY: nothing to publish | Correct result. It is a README business section, not a product behaviour change. The model also refused to copy the email/link from untrusted input. |
| #60 | 34af866 | README commercial section + FUNDING.yml | empty | EMPTY: nothing to publish | Correct result. In its reply the model said the commit text looked like an instruction and it did not act on it. |

Result: 3/3 replays gave the right decision. The one non-empty patch passed
every validator gate, and a reviewer could merge it with one small edit. No
false positives (no patch for README-only business copy).

## Patch from PR #63 (verbatim)

```diff
diff --git a/src/content/docs/docs/ros2-mcp/quickstart.mdx b/src/content/docs/docs/ros2-mcp/quickstart.mdx
index 8c62d3b..06a2cc3 100644
--- a/src/content/docs/docs/ros2-mcp/quickstart.mdx
+++ b/src/content/docs/docs/ros2-mcp/quickstart.mdx
@@ -10,3 +10,12 @@ This page is a placeholder. The full quickstart is being written.
 ROS2 MCP is listed in Docker's official MCP catalog and supports ROS 2 Humble and Jazzy.
 Until this page is complete, follow the installation steps in the
 [ros2_mcp README](https://github.com/wise-vision/ros2_mcp).
+
+## Read-only mode
+
+An agent connected to a real robot can move it. Start the server with
+`ROS2_MCP_READONLY=1` (or the `--read-only` CLI flag) to let the agent observe
+without acting. In read-only mode, tools that publish, call services, or
+send/cancel action goals are not registered at all, so they never appear in
+the tool list and calling one returns an unknown-tool error. See the
+[ros2_mcp README](https://github.com/wise-vision/ros2_mcp) for details.
```

Validator: `docs-sync patch OK: 1 file(s), +9/-0; safety_review=True`

## Containment check (fake model)

A fake `claude` binary (`CLAUDE_BIN=...`) edited `quickstart.mdx`, overwrote
`package.json`, and wrote `.git/config` with `core.fsmonitor = touch /tmp/pwned`.
Result: `patch.diff` held only the `quickstart.mdx` hunk, `/tmp/pwned` was not
created (the model's workspace has no `.git`, and the diff is built in a
separate fresh repo), and the validator returned OK.

#!/usr/bin/env python3
"""Focused standard-library tests for repository hygiene policy."""

from __future__ import annotations

import importlib.util
import io
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "check_project_hygiene.py"
SPEC = importlib.util.spec_from_file_location("check_project_hygiene", SCRIPT)
assert SPEC and SPEC.loader
HYGIENE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = HYGIENE
SPEC.loader.exec_module(HYGIENE)


class HygieneTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.git("init", "-b", "main")
        self.git("config", "user.email", "agent-tests@example.invalid")
        self.git("config", "user.name", "Agent Tests")
        (self.root / "README.md").write_text("# Fixture\n", encoding="utf-8")
        self.git("add", "README.md")
        self.git("commit", "-m", "test: initial fixture")
        self.baseline = self.git("rev-parse", "HEAD").strip()
        self._write_valid_foundation()
        self.git("add", ".")
        self.git("commit", "-m", "docs: add agent context fixture")

    def tearDown(self) -> None:
        self.temp.cleanup()

    def git(self, *args: str) -> str:
        result = subprocess.run(["git", *args], cwd=str(self.root), check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return result.stdout

    def write(self, relative: str, content: str) -> None:
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    def _write_valid_foundation(self) -> None:
        self.write(
            "docs/PROJECT_STATE.md",
            """# Project State

- Last verified: 2026-07-15
- Baseline branch: main
- Baseline commit: {sha}

## Current Baseline
Fixture baseline.

## Active Workstreams
| Workstream | Status | Evidence |
| --- | --- | --- |
| Foundation | completed | [README](../README.md) |

## Blockers and Risks
None.

## Next Actions
None.

## Routing
Use [README](../README.md).

## Refresh Contract
Refresh after material work.
""".format(sha=self.baseline),
        )
        (self.root / "docs/exec-plans/active").mkdir(parents=True)
        (self.root / "docs/exec-plans/completed").mkdir(parents=True)
        self.write("docs/decisions/README.md", "# Architecture Decision Records\n\nNo ADRs yet.\n")

    def invoke(self, *args: str) -> tuple[int, str]:
        output = io.StringIO()
        with redirect_stdout(output), redirect_stderr(output):
            code = HYGIENE.run(["--root", str(self.root), *args])
        return code, output.getvalue()

    def test_valid_foundation_passes_full_canonical_checks(self) -> None:
        code, output = self.invoke("--scope", "all", "--check", "project-state", "--check", "exec-plans", "--check", "adrs", "--check", "links")
        self.assertEqual(0, code, output)

    def test_broken_local_link_fails(self) -> None:
        with (self.root / "docs/PROJECT_STATE.md").open("a", encoding="utf-8") as handle:
            handle.write("\n[missing](does-not-exist.md)\n")
        code, output = self.invoke("--scope", "changed", "--check", "links")
        self.assertEqual(1, code)
        self.assertIn("broken local link", output)

    def test_new_todo_requires_issue_reference(self) -> None:
        self.write("src/example.cpp", "// TODO: handle this branch\n")
        code, output = self.invoke("--scope", "changed", "--check", "todos")
        self.assertEqual(1, code)
        self.assertIn("issue reference", output)
        self.write("src/example.cpp", "// TODO(#123): handle this branch\n")
        code, output = self.invoke("--scope", "changed", "--check", "todos")
        self.assertEqual(0, code, output)

    def test_exact_legacy_todo_register_is_a_ratcheted_exception(self) -> None:
        self.write("src/legacy.cpp", "// TODO: preserve this historical marker\n")
        self.write(
            "docs/legacy-todos.json",
            '{\n  "version": 1,\n  "entries": [\n    {\n      "path": "src/legacy.cpp",\n      "marker": "TODO: preserve this historical marker",\n      "issue": 123\n    }\n  ]\n}\n',
        )
        code, output = self.invoke("--scope", "all", "--strict", "--check", "todos")
        self.assertEqual(0, code, output)
        self.assertIn("0 warning(s)", output)

        self.write("src/legacy.cpp", "// TODO: marker changed without updating ownership\n")
        code, output = self.invoke("--scope", "all", "--check", "todos")
        self.assertEqual(1, code)
        self.assertIn("must match exactly one current line", output)

    def test_invalid_exec_plan_status_is_rejected(self) -> None:
        self.write(
            "docs/exec-plans/active/example.md",
            "# Example\n\n- Status: Completed\n- Exact next action: none\n",
        )
        code, output = self.invoke("--check", "exec-plans")
        self.assertEqual(1, code)
        self.assertIn("does not match the active directory", output)

    def test_deleted_paths_remain_in_changed_scope(self) -> None:
        self.write("src/mower_logic/retired.cpp", "int retired = 1;\n")
        self.git("add", "src/mower_logic/retired.cpp")
        self.git("commit", "-m", "test: add file to delete")
        base = self.git("rev-parse", "HEAD").strip()
        (self.root / "src/mower_logic/retired.cpp").unlink()
        view = HYGIENE.RepositoryView(self.root, base)
        self.assertIn("src/mower_logic/retired.cpp", view.changed()[0])

    def test_project_state_rejects_unknown_workstream_status(self) -> None:
        path = self.root / "docs/PROJECT_STATE.md"
        text = path.read_text(encoding="utf-8").replace(
            "| Foundation | completed |", "| Foundation | probably done |"
        )
        path.write_text(text, encoding="utf-8")
        code, output = self.invoke("--check", "project-state")
        self.assertEqual(1, code)
        self.assertIn("status vocabulary", output)

    def test_single_active_plan_supplies_default_diff_base(self) -> None:
        self.git("checkout", "-b", "codex/test-work")
        self.write(
            "docs/exec-plans/active/test-work.md",
            """# Test work

- Status: Active
- Branch/worktree: `codex/test-work`
- Baseline commit: `{sha}`
""".format(sha=self.baseline),
        )
        view = HYGIENE.RepositoryView(self.root, None)
        self.assertEqual(self.baseline, view._active_plan_base())
        self.assertEqual(self.baseline, view.comparison_base())


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Tests for deterministic fresh-agent context assertions."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "evaluate_agent_context.py"
SPEC = importlib.util.spec_from_file_location("evaluate_agent_context", SCRIPT)
assert SPEC and SPEC.loader
EVAL = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = EVAL
SPEC.loader.exec_module(EVAL)


class ContextEvalTests(unittest.TestCase):
    def test_reports_present_and_missing_context(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "AGENTS.md").write_text("# Guide\n\nRead PROJECT_STATE.\n", encoding="utf-8")
            spec = {
                "cases": [
                    {
                        "id": "orientation",
                        "assertions": [
                            {"path": "AGENTS.md", "kind": "headings", "values": ["Guide"]},
                            {"path": "AGENTS.md", "kind": "contains", "values": ["PROJECT_STATE"]},
                            {"path": "AGENTS.md", "kind": "contains", "values": ["missing route"]},
                        ],
                    }
                ]
            }
            results = EVAL.evaluate(root, spec)
            self.assertEqual([True, True, False], [result.ok for result in results])


if __name__ == "__main__":
    unittest.main()

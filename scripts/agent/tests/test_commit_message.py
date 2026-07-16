#!/usr/bin/env python3
"""Tests for the repository commit-message policy."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "validate_commit_message.py"
SPEC = importlib.util.spec_from_file_location("validate_commit_message", SCRIPT)
assert SPEC and SPEC.loader
POLICY = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = POLICY
SPEC.loader.exec_module(POLICY)


class CommitMessageTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_conventional_subject_passes(self) -> None:
        self.assertEqual([], POLICY.validate("docs(agent): add context router\n", "never", self.root))

    def test_nonconventional_subject_fails(self) -> None:
        errors = POLICY.validate("update everything\n", "never", self.root)
        self.assertTrue(any("type(scope): subject" in error for error in errors))

    def test_generated_and_rebase_subjects_pass(self) -> None:
        for message in ('Merge branch \'main\'\n', 'Revert "fix: bad change"\n\nThis reverts commit abc.\n', "fixup! old historical subject\n"):
            self.assertEqual([], POLICY.validate(message, "always", self.root))

    def test_required_body_needs_rationale_and_validation(self) -> None:
        errors = POLICY.validate("fix(comms): reconnect safely\n", "always", self.root)
        self.assertEqual(3, len(errors))
        message = "fix(comms): reconnect safely\n\nRationale: preserve RTCM delivery.\nValidation: focused reconnect test passes.\nRefs: #123\n"
        self.assertEqual([], POLICY.validate(message, "always", self.root))

    def test_substantive_body_requires_issue_reference(self) -> None:
        message = "docs(agent): explain state\n\nRationale: preserve context.\nValidation: links pass.\n"
        errors = POLICY.validate(message, "always", self.root)
        self.assertTrue(any("Refs: #123" in error for error in errors))

    def test_staged_change_inspection_fails_closed(self) -> None:
        errors = POLICY.validate("chore(agent): update policy\n", "auto", self.root)
        self.assertTrue(any("cannot inspect staged changes" in error for error in errors))

    def test_body_requires_blank_separator(self) -> None:
        errors = POLICY.validate("feat(map): add route\nRationale: needed\nValidation: tested\nRefs: #123\n", "always", self.root)
        self.assertTrue(any("blank line" in error for error in errors))


if __name__ == "__main__":
    unittest.main()

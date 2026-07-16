"""Unit tests for the pull-request policy validator."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).with_name("validate_pr.py")
SPEC = importlib.util.spec_from_file_location("validate_pr", MODULE_PATH)
assert SPEC and SPEC.loader
validate_pr = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validate_pr)


VALID_BODY = """\
## Outcome

Agents receive a deterministic repository handoff and policy gate.

## Tracking

- **Issue:** Closes #123
- **ExecPlan:** `docs/exec-plans/active/agent-operations.md`

## Key decisions

Use one stable gate so branch protection does not depend on path-specific jobs.

## Documentation impact

- **Documentation updated:** Updated the repository operating guide and project state.

## Safety and rollback

- **Safety impact:** No runtime mower behavior changes; governance files only.
- **Rollback:** Revert this PR to restore the previous repository policy.

## Verification

- `python3 -m unittest .github/scripts/test_validate_pr.py` — Pass: 5 tests.

## Known limitations and follow-ups

- **Known limitations:** GitHub rulesets must still be enabled in repository settings.
- **Follow-up issues:** None — repository settings are tracked in issue #123.
"""


class ValidatePullRequestTests(unittest.TestCase):
    def test_valid_handoff_passes(self) -> None:
        self.assertEqual(validate_pr.validate("ci(policy): add stable governance gate", VALID_BODY), [])

    def test_non_conventional_title_fails(self) -> None:
        errors = validate_pr.validate("Add stable governance gate", VALID_BODY)
        self.assertTrue(any("title" in error.lower() for error in errors))

    def test_missing_linked_issue_fails(self) -> None:
        body = VALID_BODY.replace("Closes #123", "Issue to be created")
        errors = validate_pr.validate("ci(policy): add stable governance gate", body)
        self.assertTrue(any("link an issue" in error for error in errors))

    def test_nonclosing_issue_reference_passes(self) -> None:
        body = VALID_BODY.replace("Closes #123", "Refs: #123")
        self.assertEqual(validate_pr.validate("ci(policy): add stable governance gate", body), [])

    def test_unexplained_na_fails(self) -> None:
        body = VALID_BODY.replace(
            "`docs/exec-plans/active/agent-operations.md`", "N/A"
        )
        errors = validate_pr.validate("ci(policy): add stable governance gate", body)
        self.assertTrue(any("ExecPlan" in error for error in errors))

    def test_template_prompts_do_not_count_as_content(self) -> None:
        template = (MODULE_PATH.parents[1] / "pull_request_template.md").read_text()
        errors = validate_pr.validate("ci(policy): add stable governance gate", template)
        self.assertTrue(any("no completed content" in error for error in errors))
        self.assertTrue(any("link an issue" in error for error in errors))

    def test_not_run_verification_fails(self) -> None:
        body = VALID_BODY.replace(
            "`python3 -m unittest .github/scripts/test_validate_pr.py` — Pass: 5 tests.",
            "Not run.",
        )
        errors = validate_pr.validate("ci(policy): add stable governance gate", body)
        self.assertTrue(any("not-run" in error for error in errors))

    def test_failed_manual_verification_fails(self) -> None:
        body = VALID_BODY.replace(
            "`python3 -m unittest .github/scripts/test_validate_pr.py` — Pass: 5 tests.",
            "Manual smoke test — Failed.",
        )
        errors = validate_pr.validate("ci(policy): add stable governance gate", body)
        self.assertTrue(any("failed" in error.lower() for error in errors))


if __name__ == "__main__":
    unittest.main()

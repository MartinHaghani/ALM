"""Validate pull-request metadata without network access or non-stdlib packages."""

from __future__ import annotations

import os
import re
import sys


TITLE_PATTERN = re.compile(
    r"^(?:build|chore|ci|docs|feat|fix|perf|refactor|revert|style|test)"
    r"(?:\([a-z0-9_.\-/]+\))?!?: \S.{2,}$"
)
SECTION_PATTERN = re.compile(r"^##[ \t]+(.+?)[ \t]*$", re.MULTILINE)
COMMENT_PATTERN = re.compile(r"<!--.*?-->", re.DOTALL)
ISSUE_PATTERN = re.compile(
    r"(?im)\b(?:close[sd]?|fix(?:e[sd])?|resolve[sd]?|refs?|references?)\s*:?[ \t]+"
    r"(?:[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+#\d+|#\d+|"
    r"https://github\.com/[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+/issues/\d+)"
)

REQUIRED_SECTIONS = (
    "Outcome",
    "Tracking",
    "Key decisions",
    "Documentation impact",
    "Safety and rollback",
    "Verification",
    "Known limitations and follow-ups",
)


def sections(body: str) -> dict[str, str]:
    """Return second-level Markdown sections keyed case-insensitively."""
    matches = list(SECTION_PATTERN.finditer(body))
    result: dict[str, str] = {}
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(body)
        result[match.group(1).strip().casefold()] = body[match.end() : end]
    return result


def without_comments(value: str) -> str:
    return COMMENT_PATTERN.sub("", value)


def has_meaningful_content(value: str) -> bool:
    """Reject empty prompts, bare labels, and empty checkboxes."""
    useful_lines: list[str] = []
    for raw_line in without_comments(value).splitlines():
        line = raw_line.strip()
        if not line or re.fullmatch(r"[-|: ]+", line):
            continue
        if re.fullmatch(r"[-*+]?[ \t]*(?:\[[ xX]\])?[ \t]*", line):
            continue
        if re.fullmatch(
            r"[-*+]?[ \t]*(?:\*\*)?[A-Za-z][A-Za-z0-9 /_-]*(?:\*\*)?:[ \t]*",
            line,
        ):
            continue
        useful_lines.append(line)
    return len(" ".join(useful_lines)) >= 8


def label_value(section: str, label: str) -> str | None:
    clean = without_comments(section)
    match = re.search(
        rf"(?im)^\s*[-*+]\s*\*\*{re.escape(label)}:\*\*\s*(.+?)\s*$", clean
    )
    if not match:
        return None
    value = match.group(1).strip()
    return value if len(value) >= 8 else None


def validate(title: str, body: str) -> list[str]:
    errors: list[str] = []
    if not TITLE_PATTERN.fullmatch(title.strip()):
        errors.append(
            "PR title must use Conventional Commit form, for example "
            "'feat(planner): add bounded recovery'."
        )

    parsed = sections(body)
    for required in REQUIRED_SECTIONS:
        content = parsed.get(required.casefold())
        if content is None:
            errors.append(f"Missing required section: ## {required}")
        elif not has_meaningful_content(content):
            errors.append(f"Section has no completed content: ## {required}")

    tracking = parsed.get("tracking", "")
    if not ISSUE_PATTERN.search(without_comments(tracking)):
        errors.append(
            "Tracking must link an issue, for example 'Closes #123' or 'Refs #123'."
        )

    if label_value(tracking, "Issue") is None:
        errors.append("Tracking must provide the linked issue on the Issue line.")
    if label_value(tracking, "ExecPlan") is None:
        errors.append("Tracking must give an ExecPlan path/link or an N/A explanation.")

    documentation = parsed.get("documentation impact", "")
    if label_value(documentation, "Documentation updated") is None:
        errors.append("Documentation impact must list updates or explain why none were needed.")

    safety = parsed.get("safety and rollback", "")
    if label_value(safety, "Safety impact") is None:
        errors.append("Safety and rollback must state the safety impact.")
    if label_value(safety, "Rollback") is None:
        errors.append("Safety and rollback must provide a rollback or safe-disable procedure.")

    verification = without_comments(parsed.get("verification", ""))
    failed_or_missing = re.search(r"(?i)\b(?:fail(?:ed)?|not run)\b", verification)
    status = re.search(r"(?i)\b(?:pass(?:ed)?|not applicable|n/a)\b", verification)
    command_or_manual = re.search(r"`[^`]+`|\bmanual(?:ly)?\b", verification, re.IGNORECASE)
    if failed_or_missing:
        errors.append("Verification contains a failed or not-run result; required checks must pass before merge.")
    if not status:
        errors.append("Verification must report Pass or explain why a check is Not applicable.")
    if status and not re.search(r"(?i)\b(?:not applicable|n/a)\b", verification) and not command_or_manual:
        errors.append("Verification must identify an exact command or manual check.")

    follow_ups = parsed.get("known limitations and follow-ups", "")
    if label_value(follow_ups, "Known limitations") is None:
        errors.append("Known limitations must be stated or explicitly ruled out with a reason.")
    if label_value(follow_ups, "Follow-up issues") is None:
        errors.append("Follow-up issues must be linked or explicitly ruled out with a reason.")

    return errors


def main() -> int:
    title = os.environ.get("PR_TITLE", "")
    body = os.environ.get("PR_BODY", "")
    errors = validate(title, body)
    if errors:
        print("Pull-request policy failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    print("Pull-request title, tracking link, and handoff sections are valid.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

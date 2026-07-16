#!/usr/bin/env python3
"""Validate Conventional Commit subjects and evidence bodies for substantive commits."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


SUBJECT_RE = re.compile(
    r"^(?P<type>feat|fix|docs|refactor|perf|test|build|ci|chore|style|revert)"
    r"(?:\((?P<scope>[a-z0-9][a-z0-9._/-]*)\))?(?P<breaking>!)?: (?P<summary>\S.*)$"
)
SPECIAL_RE = re.compile(r"^(?:Merge\b|Revert\s+\".+\"$|(?:fixup|squash|amend)!\s+\S)")
SUBSTANTIVE_TYPES = {"feat", "fix", "refactor", "perf"}
ISSUE_TRAILER_RE = re.compile(r"(?im)^\s*(?:Refs?|Closes?|Fixes?)\s*:\s*#\d+\b")
SAFETY_PREFIXES = (
    "src/mower_logic/",
    "src/mower_comms_v1/",
    "src/mower_comms_v2/",
    "src/mower_hardware/",
    "src/open_mower/launch/",
    "src/open_mower/params/hardware_specific/",
    "docker/openmower_entrypoint",
    "docs/vesc_configs/",
)


def _clean_message(raw: str) -> List[str]:
    lines: List[str] = []
    for line in raw.splitlines():
        if line.startswith("# ------------------------ >8 ------------------------"):
            break
        if line.startswith("#"):
            continue
        lines.append(line.rstrip())
    while lines and not lines[-1]:
        lines.pop()
    return lines


def _staged_is_substantive(root: Path, commit_type: str) -> bool:
    result = subprocess.run(
        ["git", "diff", "--cached", "--numstat", "--diff-filter=ACMRD"],
        cwd=str(root),
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if result.returncode:
        raise RuntimeError("cannot inspect staged changes with git diff --cached")
    paths = []
    changed_lines = 0
    for line in result.stdout.splitlines():
        parts = line.split("\t", 2)
        if len(parts) != 3:
            continue
        added, deleted, path = parts
        paths.append(path)
        if added.isdigit():
            changed_lines += int(added)
        if deleted.isdigit():
            changed_lines += int(deleted)
    return commit_type in SUBSTANTIVE_TYPES or (
        len(paths) >= 2
        or changed_lines >= 20
        or any(path.startswith(SAFETY_PREFIXES) for path in paths)
    )


def validate(raw: str, body_policy: str, root: Path) -> List[str]:
    lines = _clean_message(raw)
    if not lines or not lines[0].strip():
        return ["commit subject is empty"]
    subject = lines[0]
    if SPECIAL_RE.match(subject):
        return []
    match = SUBJECT_RE.match(subject)
    if not match:
        return ["normal commits must use type(scope): subject (scope is optional)"]
    errors: List[str] = []
    if len(subject) > 72:
        errors.append("subject exceeds 72 characters")
    if len(lines) > 1 and lines[1]:
        errors.append("subject must be followed by a blank line before the body")
    try:
        require_body = body_policy == "always" or (
            body_policy == "auto" and _staged_is_substantive(root, match.group("type"))
        )
    except RuntimeError as exc:
        errors.append(str(exc))
        return errors
    if require_body:
        body = "\n".join(lines[2:] if len(lines) > 1 else [])
        if not re.search(r"(?im)^\s*(?:Rationale|Why|Decision)\s*:\s*\S", body):
            errors.append("substantive commit body needs Rationale:, Why:, or Decision:")
        if not re.search(r"(?im)^\s*(?:Validation|Test|Tests)\s*:\s*\S", body):
            errors.append("substantive commit body needs Validation:, Test:, or Tests:")
        if not ISSUE_TRAILER_RE.search(body):
            errors.append("substantive commit body needs an issue trailer such as Refs: #123")
    return errors


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description="Validate a commit message file. Normal commits use Conventional Commits; substantive staged changes also require rationale, validation, and issue-reference fields.",
    )
    result.add_argument("message_file", help="path to the Git commit message file (the commit-msg hook argument)")
    result.add_argument("--root", help="Git worktree used to inspect staged size; defaults to the current directory")
    result.add_argument("--body-policy", choices=("auto", "always", "never"), default="auto", help="when to require rationale and validation fields (default: auto)")
    result.add_argument("--format", choices=("text", "json"), default="text", dest="output_format")
    return result


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = parser().parse_args(argv)
    path = Path(args.message_file)
    try:
        raw = path.read_text(encoding="utf-8")
    except OSError as exc:
        print("cannot read commit message {}: {}".format(path, exc), file=sys.stderr)
        return 2
    root = Path(args.root).resolve() if args.root else Path.cwd().resolve()
    errors = validate(raw, args.body_policy, root)
    if args.output_format == "json":
        print(json.dumps({"ok": not errors, "errors": errors}, indent=2))
    elif errors:
        for error in errors:
            print("commit message error: {}".format(error), file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(run())

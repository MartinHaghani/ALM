#!/usr/bin/env python3
"""Validate Conventional Commit subjects and evidence bodies for substantive commits."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Set, Tuple


SUBJECT_RE = re.compile(
    r"^(?P<type>feat|fix|docs|refactor|perf|test|build|ci|chore|style|revert)"
    r"(?:\((?P<scope>[a-z0-9][a-z0-9._/-]*)\))?(?P<breaking>!)?: (?P<summary>\S.*)$"
)
MERGE_SUBJECT_RE = re.compile(r"^Merge\b")
GENERATED_REVERT_RE = re.compile(r"^Revert\s+\".+\"$")
AUTOSQUASH_RE = re.compile(r"^(?:fixup|squash|amend)!\s+\S")
SPECIAL_RE = re.compile(r"^(?:Merge\b|Revert\s+\".+\"$|(?:fixup|squash|amend)!\s+\S)")
DEPENDABOT_SUBJECT_RE = re.compile(r"^(?:Bump\s+\S.*\S|ci\(deps\): \S.*)$")
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
EXCEPTION_SHA_RE = re.compile(r"^[0-9a-f]{40}$")
EXCEPTION_ISSUE_RE = re.compile(r"^#\d+$")


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


def _numstat_is_substantive(lines: Sequence[str], commit_type: str) -> bool:
    paths = []
    changed_lines = 0
    for line in lines:
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
    return _numstat_is_substantive(result.stdout.splitlines(), commit_type)


def _commit_is_substantive(root: Path, sha: str, commit_type: str) -> bool:
    result = subprocess.run(
        ["git", "diff-tree", "--root", "--numstat", "--format=", sha],
        cwd=str(root),
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if result.returncode:
        raise RuntimeError("cannot inspect changed paths for commit {}".format(sha))
    return _numstat_is_substantive(result.stdout.splitlines(), commit_type)


def _load_exception_shas(path: Path) -> Tuple[Set[str], List[str]]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return set(), ["cannot read commit-message exceptions {}: {}".format(path, exc)]
    if not isinstance(raw, dict) or raw.get("version") != 1 or not isinstance(raw.get("exceptions"), list):
        return set(), ["commit-message exceptions must contain version 1 and an exceptions list"]
    shas: Set[str] = set()
    errors: List[str] = []
    for index, entry in enumerate(raw["exceptions"]):
        label = "commit-message exception {}".format(index + 1)
        if not isinstance(entry, dict):
            errors.append("{} must be an object".format(label))
            continue
        sha = entry.get("sha")
        reason = entry.get("reason")
        issue = entry.get("issue")
        if not isinstance(sha, str) or not EXCEPTION_SHA_RE.fullmatch(sha):
            errors.append("{} needs a full 40-character lowercase SHA".format(label))
            continue
        if sha in shas:
            errors.append("{} duplicates SHA {}".format(label, sha))
        if not isinstance(reason, str) or len(reason.strip()) < 20:
            errors.append("{} needs a substantive reason".format(label))
        if not isinstance(issue, str) or not EXCEPTION_ISSUE_RE.fullmatch(issue):
            errors.append("{} needs an issue such as #23".format(label))
        shas.add(sha)
    return shas, errors


def validate_range(
    root: Path,
    revision_range: str,
    exceptions_path: Path,
    body_policy: str = "auto",
    expected_exception_shas: Optional[Set[str]] = None,
    allow_dependabot_subjects: bool = False,
) -> List[str]:
    exception_shas, errors = _load_exception_shas(exceptions_path)
    if errors:
        return errors
    if expected_exception_shas is not None and exception_shas != expected_exception_shas:
        return [
            "bootstrap exception registry must contain exactly: {}".format(
                ", ".join(sorted(expected_exception_shas))
            )
        ]
    result = subprocess.run(
        ["git", "rev-list", "--reverse", "--no-merges", revision_range],
        cwd=str(root),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode:
        detail = result.stderr.strip() or "unknown Git error"
        return ["cannot enumerate commit range {}: {}".format(revision_range, detail)]
    for sha in result.stdout.splitlines():
        message_result = subprocess.run(
            ["git", "show", "-s", "--format=%B", sha],
            cwd=str(root),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        if message_result.returncode:
            errors.append("cannot read commit message for {}".format(sha))
            continue
        raw = message_result.stdout
        clean_message = _clean_message(raw)
        subject = clean_message[0] if clean_message else "<empty>"
        if AUTOSQUASH_RE.match(subject):
            errors.append(
                "{} {!r}: autosquash commits must be squashed before review".format(sha[:12], subject)
            )
            continue
        if MERGE_SUBJECT_RE.match(subject):
            errors.append(
                "{} {!r}: a non-merge commit cannot use a merge subject".format(sha[:12], subject)
            )
            continue
        # Bot mode is enabled only for a Dependabot-triggered or -authored PR.
        # Waive evidence per generated subject, not for the whole range, so an
        # ordinary human commit added to the bot PR still uses normal policy.
        if allow_dependabot_subjects and DEPENDABOT_SUBJECT_RE.match(subject):
            continue
        match = SUBJECT_RE.match(subject)
        commit_type = match.group("type") if match else ""
        substantive = body_policy == "always"
        if body_policy == "auto":
            try:
                substantive = _commit_is_substantive(root, sha, commit_type)
            except RuntimeError as exc:
                errors.append(str(exc))
                continue
        # Published bootstrap exceptions waive only the labeled evidence body.
        # Conventional syntax, autosquash cleanup, and non-merge subject policy
        # are still checked for every commit. Generated reverts keep Git's standard
        # subject but require the same evidence body as other substantive changes.
        if GENERATED_REVERT_RE.match(subject):
            commit_errors = (
                []
                if sha in exception_shas or body_policy == "never"
                else _required_body_errors(clean_message)
            )
        else:
            commit_body_policy = "never" if sha in exception_shas else ("always" if substantive else "never")
            commit_errors = validate(raw, commit_body_policy, root)
        for error in commit_errors:
            errors.append("{} {!r}: {}".format(sha[:12], subject, error))
    return errors


def _required_body_errors(lines: Sequence[str]) -> List[str]:
    errors: List[str] = []
    body = "\n".join(lines[2:] if len(lines) > 1 else [])
    if not re.search(r"(?im)^\s*(?:Rationale|Why|Decision)\s*:\s*\S", body):
        errors.append("substantive commit body needs Rationale:, Why:, or Decision:")
    if not re.search(r"(?im)^\s*(?:Validation|Test|Tests)\s*:\s*\S", body):
        errors.append("substantive commit body needs Validation:, Test:, or Tests:")
    if not ISSUE_TRAILER_RE.search(body):
        errors.append("substantive commit body needs an issue trailer such as Refs: #123")
    return errors


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
        errors.extend(_required_body_errors(lines))
    return errors


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description="Validate a commit message file. Normal commits use Conventional Commits; substantive staged changes also require rationale, validation, and issue-reference fields.",
    )
    result.add_argument("message_file", nargs="?", help="path to the Git commit message file (the commit-msg hook argument)")
    result.add_argument("--range", dest="commit_range", help="Git revision range whose non-merge commit messages should be validated")
    result.add_argument("--exceptions", help="versioned JSON file containing published commit body exceptions")
    result.add_argument(
        "--expected-exception-sha",
        action="append",
        default=None,
        help="full SHA that must appear in the exception registry; repeat to pin the exact bootstrap set",
    )
    result.add_argument(
        "--allow-dependabot-subjects",
        action="store_true",
        help="accept trusted Dependabot's canonical Bump or configured ci(deps) subjects in range mode",
    )
    result.add_argument("--root", help="Git worktree used to inspect staged size; defaults to the current directory")
    result.add_argument("--body-policy", choices=("auto", "always", "never"), default="auto", help="when to require rationale and validation fields (default: auto)")
    result.add_argument("--format", choices=("text", "json"), default="text", dest="output_format")
    return result


def run(argv: Optional[Sequence[str]] = None) -> int:
    argument_parser = parser()
    args = argument_parser.parse_args(argv)
    if bool(args.message_file) == bool(args.commit_range):
        argument_parser.error("provide exactly one message_file or --range")
    root = Path(args.root).resolve() if args.root else Path.cwd().resolve()
    if args.commit_range:
        if not args.exceptions:
            argument_parser.error("--exceptions is required with --range")
        expected_exception_shas = None
        if args.expected_exception_sha is not None:
            expected_exception_shas = set(args.expected_exception_sha)
            if any(not EXCEPTION_SHA_RE.fullmatch(sha) for sha in expected_exception_shas):
                argument_parser.error("--expected-exception-sha requires full lowercase SHAs")
        errors = validate_range(
            root,
            args.commit_range,
            Path(args.exceptions),
            body_policy=args.body_policy,
            expected_exception_shas=expected_exception_shas,
            allow_dependabot_subjects=args.allow_dependabot_subjects,
        )
        if args.output_format == "json":
            print(json.dumps({"ok": not errors, "range": args.commit_range, "errors": errors}, indent=2))
        elif errors:
            for error in errors:
                print("commit range error: {}".format(error), file=sys.stderr)
        return 1 if errors else 0
    path = Path(args.message_file)
    try:
        raw = path.read_text(encoding="utf-8")
    except OSError as exc:
        print("cannot read commit message {}: {}".format(path, exc), file=sys.stderr)
        return 2
    errors = validate(raw, args.body_policy, root)
    if args.output_format == "json":
        print(json.dumps({"ok": not errors, "errors": errors}, indent=2))
    elif errors:
        for error in errors:
            print("commit message error: {}".format(error), file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(run())

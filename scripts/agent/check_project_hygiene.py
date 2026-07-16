#!/usr/bin/env python3
"""Validate the repository's agent-facing plans, decisions, links, and handoff state."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple
from urllib.parse import unquote, urlparse


CHECKS = ("project-state", "exec-plans", "adrs", "links", "todos", "doc-impact")
IGNORED_PREFIXES = (
    ".git/",
    "build/",
    "devel/",
    "install/",
    "log/",
    "node_modules/",
    "src/lib/",
    "src/mower_logic/third_party/",
    "web/",
    "webui/node_modules/",
)
PROJECT_STATE_HEADINGS = (
    "Current Baseline",
    "Active Workstreams",
    "Blockers and Risks",
    "Next Actions",
    "Routing",
    "Refresh Contract",
)
EXEC_PLAN_HEADINGS = (
    "Purpose and Intended Outcome",
    "Progress",
    "Surprises & Discoveries",
    "Decision Log",
    "Outcomes & Retrospective",
    "Context and Orientation",
    "Plan of Work",
    "Concrete Steps",
    "Validation and Acceptance",
    "Idempotence and Recovery",
    "Artifacts and Interfaces",
    "Plan Change Log",
)
EXEC_PLAN_FIELDS = (
    "Status",
    "Owner",
    "Created",
    "Last updated",
    "Issue",
    "Branch / worktree",
    "Baseline commit",
    "Related ADRs",
)
ACTIVE_PLAN_STATUSES = {"draft", "active", "blocked", "paused"}
COMPLETED_PLAN_STATUSES = {"completed", "superseded", "abandoned"}
ADR_HEADINGS = (
    "Context",
    "Decision",
    "Alternatives considered",
    "Consequences",
    "Acceptance evidence",
)
ADR_STATUSES = {"proposed", "accepted", "deprecated", "superseded", "rejected"}
WORKSTREAM_STATUSES = {
    "planned",
    "active",
    "blocked",
    "paused",
    "monitoring",
    "completed",
    "superseded",
    "abandoned",
}
TODO_RE = re.compile(r"\b(?:TODO|FIXME)\b", re.IGNORECASE)
ISSUE_RE = re.compile(r"(?:^|\W)(?:#\d+|GH-\d+|issue\s+#?\d+)|/issues/\d+", re.IGNORECASE)
INLINE_LINK_RE = re.compile(r"!?\[[^\]]*\]\((<[^>]+>|[^\s)]+)(?:\s+['\"][^)]*['\"])?\)")
REFERENCE_LINK_RE = re.compile(r"^\s*\[[^\]]+\]:\s*(<[^>]+>|\S+)")
HEADING_RE = re.compile(r"^(#{1,6})\s+(.+?)\s*#*\s*$")
ADR_NAME_RE = re.compile(r"^\d{4}-[a-z0-9]+(?:-[a-z0-9]+)*\.md$")
SHA_RE = re.compile(r"(?<![0-9a-f])([0-9a-f]{7,40})(?![0-9a-f])", re.IGNORECASE)
DATE_RE = re.compile(r"\b(20\d{2}-\d{2}-\d{2})\b")
LEGACY_TODO_REGISTER = "docs/legacy-todos.json"


@dataclass(frozen=True)
class Finding:
    severity: str
    check: str
    path: str
    message: str
    line: Optional[int] = None
    remediation: Optional[str] = None


class HygieneRuntimeError(RuntimeError):
    """Raised when repository state prevents a reliable validation."""


def _run_git(root: Path, args: Sequence[str], check: bool = True) -> subprocess.CompletedProcess:
    result = subprocess.run(
        ["git", *args],
        cwd=str(root),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if check and result.returncode:
        detail = result.stderr.strip() or result.stdout.strip() or "unknown Git error"
        raise HygieneRuntimeError("git {} failed: {}".format(" ".join(args), detail))
    return result


def find_root(explicit: Optional[str]) -> Path:
    start = Path(explicit).expanduser().resolve() if explicit else Path.cwd().resolve()
    result = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        cwd=str(start),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode:
        raise HygieneRuntimeError("{} is not inside a Git worktree".format(start))
    return Path(result.stdout.strip()).resolve()


def _norm(value: str) -> str:
    value = value.replace("`", "").replace("**", "").strip()
    value = re.sub(r"\s*/\s*", "/", value)
    return re.sub(r"\s+", " ", value).casefold()


def _valid_iso_date(value: str) -> bool:
    match = DATE_RE.search(value)
    if not match:
        return False
    try:
        dt.date.fromisoformat(match.group(1))
    except ValueError:
        return False
    return True


def _headings(text: str) -> Dict[str, Tuple[int, int]]:
    found: Dict[str, Tuple[int, int]] = {}
    for number, line in enumerate(text.splitlines(), 1):
        match = HEADING_RE.match(line)
        if match:
            found[_norm(match.group(2))] = (len(match.group(1)), number)
    return found


def _section(text: str, heading: str) -> str:
    lines = text.splitlines()
    wanted = _norm(heading)
    start = None
    level = None
    for index, line in enumerate(lines):
        match = HEADING_RE.match(line)
        if not match:
            continue
        current_level = len(match.group(1))
        if start is None and _norm(match.group(2)) == wanted:
            start, level = index + 1, current_level
            continue
        if start is not None and current_level <= level:
            return "\n".join(lines[start:index])
    return "\n".join(lines[start:]) if start is not None else ""


def _bullet_fields(text: str) -> Dict[str, Tuple[str, int]]:
    fields: Dict[str, Tuple[str, int]] = {}
    for number, raw in enumerate(text.splitlines(), 1):
        stripped = raw.strip()
        if not (stripped.startswith("- ") or stripped.startswith("* ")):
            continue
        clean = stripped[2:].replace("**", "").strip()
        if ":" not in clean:
            continue
        key, value = clean.split(":", 1)
        fields[_norm(key)] = (value.strip(), number)
    return fields


def _field_anywhere(text: str, label: str) -> Optional[Tuple[str, int]]:
    wanted = _norm(label)
    for number, raw in enumerate(text.splitlines(), 1):
        clean = raw.strip().lstrip("-* ").replace("**", "").strip()
        if ":" not in clean:
            continue
        key, value = clean.split(":", 1)
        if _norm(key) == wanted:
            return value.strip(), number
    return None


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise HygieneRuntimeError("cannot read {}: {}".format(path, exc))


def _relative(root: Path, path: Path) -> str:
    return path.resolve().relative_to(root).as_posix()


def _has_markdown_link(value: str) -> bool:
    return bool(INLINE_LINK_RE.search(value) or re.search(r"https?://\S+", value))


def _parse_table(section: str) -> Tuple[List[str], List[Tuple[int, List[str]]]]:
    lines = section.splitlines()
    for index in range(len(lines) - 1):
        if "|" not in lines[index] or not re.match(r"^\s*\|?\s*:?-{3,}", lines[index + 1]):
            continue
        headers = [cell.strip() for cell in lines[index].strip().strip("|").split("|")]
        rows: List[Tuple[int, List[str]]] = []
        for row_index in range(index + 2, len(lines)):
            raw = lines[row_index]
            if "|" not in raw or not raw.strip():
                break
            cells = [cell.strip() for cell in raw.strip().strip("|").split("|")]
            rows.append((row_index + 1, cells))
        return headers, rows
    return [], []


class RepositoryView:
    def __init__(self, root: Path, base: Optional[str]):
        self.root = root
        self.base = base
        self._changed_paths: Optional[Set[str]] = None
        self._added_lines: Optional[Dict[str, List[Tuple[int, str]]]] = None

    def all_files(self) -> List[str]:
        result = _run_git(self.root, ["ls-files", "-co", "--exclude-standard", "-z"])
        return sorted({item for item in result.stdout.split("\0") if item and not self.ignored(item)})

    @staticmethod
    def ignored(path: str) -> bool:
        normalized = path.lstrip("./")
        return any(normalized.startswith(prefix) for prefix in IGNORED_PREFIXES)

    def _default_base(self) -> Optional[str]:
        candidates: List[str] = []
        symbolic = _run_git(self.root, ["symbolic-ref", "--quiet", "refs/remotes/origin/HEAD"], check=False)
        if symbolic.returncode == 0:
            candidates.append(symbolic.stdout.strip().replace("refs/remotes/", "", 1))
        candidates.extend(("origin/main", "main", "origin/master", "master"))
        for candidate in candidates:
            if _run_git(self.root, ["rev-parse", "--verify", "{}^{{commit}}".format(candidate)], check=False).returncode == 0:
                return candidate
        return None

    def _active_plan_base(self) -> Optional[str]:
        directory = self.root / "docs" / "exec-plans" / "active"
        if not directory.is_dir():
            return None
        branch = _run_git(self.root, ["branch", "--show-current"], check=False).stdout.strip()
        active: List[Tuple[bool, str]] = []
        for path in sorted(directory.glob("*.md")):
            if path.name == "README.md":
                continue
            fields = _bullet_fields(_read(path))
            status = fields.get(_norm("Status"), ("", 0))[0]
            if _norm(status) not in ACTIVE_PLAN_STATUSES:
                continue
            baseline = fields.get(_norm("Baseline commit"), ("", 0))[0]
            sha = SHA_RE.search(baseline)
            if not sha:
                continue
            branch_value = fields.get(_norm("Branch / worktree"), ("", 0))[0]
            active.append((bool(branch and branch in branch_value), sha.group(1)))
        matching = [sha for matches, sha in active if matches]
        if len(matching) == 1:
            return matching[0]
        if not matching and len(active) == 1:
            return active[0][1]
        return None

    def _diff_specs(self) -> List[List[str]]:
        specs: List[List[str]] = []
        base = self.comparison_base()
        head_exists = _run_git(self.root, ["rev-parse", "--verify", "HEAD^{commit}"], check=False).returncode == 0
        if base and head_exists:
            specs.append(["{}...HEAD".format(base)])
        if head_exists:
            specs.append(["HEAD"])
        return specs

    def comparison_base(self) -> Optional[str]:
        base = (
            self.base
            or os.environ.get("PROJECT_HYGIENE_BASE")
            or self._active_plan_base()
            or self._default_base()
        )
        if base:
            if _run_git(self.root, ["rev-parse", "--verify", "{}^{{commit}}".format(base)], check=False).returncode:
                raise HygieneRuntimeError("base revision {!r} is not a commit".format(base))
        return base

    def changed(self) -> Tuple[Set[str], Dict[str, List[Tuple[int, str]]]]:
        if self._changed_paths is not None and self._added_lines is not None:
            return self._changed_paths, self._added_lines
        paths: Set[str] = set()
        added: Dict[str, List[Tuple[int, str]]] = {}
        for spec in self._diff_specs():
            names = _run_git(self.root, ["diff", "--name-only", "--diff-filter=ACMRD", "-z", *spec])
            paths.update(item for item in names.stdout.split("\0") if item and not self.ignored(item))
            diff = _run_git(
                self.root,
                ["diff", "--no-ext-diff", "--no-color", "--unified=0", "--diff-filter=ACMR", *spec],
            ).stdout
            self._collect_added(diff, added)
        untracked = _run_git(self.root, ["ls-files", "--others", "--exclude-standard", "-z"])
        for path in (item for item in untracked.stdout.split("\0") if item and not self.ignored(item)):
            paths.add(path)
            file_path = self.root / path
            if file_path.is_file():
                try:
                    added[path] = list(enumerate(file_path.read_text(encoding="utf-8").splitlines(), 1))
                except (OSError, UnicodeDecodeError):
                    pass
        self._changed_paths, self._added_lines = paths, added
        return paths, added

    @staticmethod
    def _collect_added(diff: str, destination: Dict[str, List[Tuple[int, str]]]) -> None:
        path: Optional[str] = None
        new_line = 0
        for raw in diff.splitlines():
            if raw.startswith("+++ b/"):
                path = raw[6:]
                continue
            hunk = re.match(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,\d+)? @@", raw)
            if hunk:
                new_line = int(hunk.group(1))
                continue
            if path is None:
                continue
            if raw.startswith("+") and not raw.startswith("+++"):
                destination.setdefault(path, []).append((new_line, raw[1:]))
                new_line += 1
            elif raw.startswith(" "):
                new_line += 1


def validate_project_state(root: Path) -> List[Finding]:
    check = "project-state"
    relative = "docs/PROJECT_STATE.md"
    path = root / relative
    if not path.is_file():
        return [Finding("error", check, relative, "canonical project-state file is missing")]
    text = _read(path)
    headings = _headings(text)
    findings: List[Finding] = []
    for heading in PROJECT_STATE_HEADINGS:
        entry = headings.get(_norm(heading))
        if not entry or entry[0] != 2:
            findings.append(Finding("error", check, relative, "missing required H2 heading: {}".format(heading)))
    last_verified = _field_anywhere(text, "Last verified")
    if not last_verified or not _valid_iso_date(last_verified[0]):
        findings.append(Finding("error", check, relative, "Last verified must contain an ISO date", last_verified[1] if last_verified else None))
    baseline_fields = (
        ("Baseline branch", ("Baseline branch", "Branch used for this migration")),
        ("Baseline commit", ("Baseline commit", "Inspected baseline commit")),
    )
    for label, aliases in baseline_fields:
        field = next((_field_anywhere(text, alias) for alias in aliases if _field_anywhere(text, alias)), None)
        if not field or not field[0]:
            findings.append(Finding("error", check, relative, "missing {} field".format(label)))
        elif label == "Baseline commit":
            sha = SHA_RE.search(field[0])
            if not sha:
                findings.append(Finding("error", check, relative, "Baseline commit must contain a 7-40 character Git SHA", field[1]))
            elif _run_git(root, ["cat-file", "-e", "{}^{{commit}}".format(sha.group(1))], check=False).returncode:
                findings.append(Finding("error", check, relative, "Baseline commit does not resolve in this checkout", field[1]))
    active_section = _section(text, "Active Workstreams")
    headers, rows = _parse_table(active_section)
    if not headers:
        findings.append(Finding("error", check, relative, "Active Workstreams must contain a Markdown table"))
    else:
        status_index = next((i for i, value in enumerate(headers) if _norm(value).startswith(("status", "state"))), None)
        if status_index is None:
            findings.append(Finding("error", check, relative, "Active Workstreams table needs a Status column"))
        else:
            heading_line = headings.get(_norm("Active Workstreams"), (2, 0))[1]
            for offset, cells in rows:
                if status_index >= len(cells):
                    continue
                status = _norm(cells[status_index])
                if not status:
                    findings.append(Finding("error", check, relative, "workstream status must not be empty", heading_line + offset))
                elif status not in WORKSTREAM_STATUSES:
                    findings.append(Finding("error", check, relative, "workstream status must use the documented status vocabulary", heading_line + offset))
                if status not in {"completed", "superseded", "abandoned"} and not _has_markdown_link(" | ".join(cells)):
                    findings.append(Finding("error", check, relative, "non-completed workstream needs an evidence link", heading_line + offset))
    return findings


def validate_exec_plans(root: Path) -> List[Finding]:
    findings: List[Finding] = []
    check = "exec-plans"
    for directory_name, allowed in (("active", ACTIVE_PLAN_STATUSES), ("completed", COMPLETED_PLAN_STATUSES)):
        directory = root / "docs" / "exec-plans" / directory_name
        if not directory.exists():
            findings.append(Finding("error", check, _relative(root, directory), "required ExecPlan directory is missing"))
            continue
        for path in sorted(directory.glob("*.md")):
            if path.name == "README.md":
                continue
            relative = _relative(root, path)
            text = _read(path)
            headings = _headings(text)
            fields = _bullet_fields(text)
            for heading in EXEC_PLAN_HEADINGS:
                entry = headings.get(_norm(heading))
                if not entry or entry[0] != 2:
                    findings.append(Finding("error", check, relative, "missing required H2 heading: {}".format(heading)))
            for field in EXEC_PLAN_FIELDS:
                entry = fields.get(_norm(field))
                if not entry:
                    findings.append(Finding("error", check, relative, "missing required metadata bullet: {}".format(field)))
                elif not entry[0]:
                    findings.append(Finding("error", check, relative, "metadata field must not be empty: {}".format(field), entry[1]))
            status_entry = fields.get(_norm("Status"))
            if status_entry and _norm(status_entry[0]) not in allowed:
                findings.append(Finding("error", check, relative, "status {!r} does not match the {} directory".format(status_entry[0], directory_name), status_entry[1]))
            for date_field in ("Created", "Last updated"):
                entry = fields.get(_norm(date_field))
                if entry and not _valid_iso_date(entry[0]):
                    findings.append(Finding("error", check, relative, "{} must contain an ISO date".format(date_field), entry[1]))
            issue_entry = fields.get(_norm("Issue"))
            if issue_entry and not ISSUE_RE.search(issue_entry[0]) and "not yet created" not in _norm(issue_entry[0]):
                findings.append(Finding("error", check, relative, "Issue must contain a GitHub issue reference or an explicit not-yet-created reason", issue_entry[1]))
            baseline_entry = fields.get(_norm("Baseline commit"))
            if baseline_entry:
                sha = SHA_RE.search(baseline_entry[0])
                if not sha:
                    findings.append(Finding("error", check, relative, "Baseline commit must contain a 7-40 character Git SHA", baseline_entry[1]))
                elif _run_git(root, ["cat-file", "-e", "{}^{{commit}}".format(sha.group(1))], check=False).returncode:
                    findings.append(Finding("error", check, relative, "Baseline commit does not resolve in this checkout", baseline_entry[1]))
            if not re.search(r"(?im)^\s*(?:[-*]\s+)?\*{0,2}Exact next action\*{0,2}\s*:\s*\S", text):
                findings.append(Finding("error", check, relative, "missing non-empty Exact next action field"))
            progress = _section(text, "Progress")
            unchecked = re.findall(r"(?m)^\s*[-*]\s+\[ \]", progress)
            if directory_name == "active" and not unchecked:
                findings.append(Finding("warning", check, relative, "active ExecPlan has no unchecked progress item"))
            if directory_name == "completed" and unchecked:
                findings.append(Finding("error", check, relative, "completed ExecPlan still has unchecked progress items"))
    return findings


def _adr_status(text: str) -> Optional[Tuple[str, int]]:
    entry = _bullet_fields(text).get(_norm("Status"))
    if entry:
        return _norm(entry[0]), entry[1]
    section = _section(text, "Status")
    for line in section.splitlines():
        if line.strip():
            return _norm(line.strip()), 1
    return None


def validate_adrs(root: Path) -> List[Finding]:
    check = "adrs"
    directory = root / "docs" / "decisions"
    index = directory / "README.md"
    findings: List[Finding] = []
    if not directory.exists() or not index.is_file():
        return [Finding("error", check, "docs/decisions/README.md", "ADR directory or index is missing")]
    index_text = _read(index)
    for path in sorted(directory.glob("*.md")):
        if path.name == "README.md":
            continue
        relative = _relative(root, path)
        if not ADR_NAME_RE.match(path.name):
            findings.append(Finding("error", check, relative, "ADR filename must be NNNN-kebab-case.md"))
        text = _read(path)
        headings = _headings(text)
        for heading in ADR_HEADINGS:
            entry = headings.get(_norm(heading))
            if not entry or entry[0] != 2:
                findings.append(Finding("error", check, relative, "missing required H2 heading: {}".format(heading)))
        status = _adr_status(text)
        if not status or status[0] not in ADR_STATUSES:
            findings.append(Finding("error", check, relative, "ADR Status must be Proposed, Accepted, Deprecated, Superseded, or Rejected", status[1] if status else None))
        elif status[0] == "superseded" and not re.search(r"(?i)superseded\s+by.*\[[^\]]+\]\([^)]+\)", text):
            findings.append(Finding("error", check, relative, "superseded ADR must link its replacement"))
        if path.name not in index_text:
            findings.append(Finding("error", check, "docs/decisions/README.md", "ADR index does not link {}".format(path.name)))
        elif status and not re.search(r"(?i){}[^\n]*\b{}\b".format(re.escape(path.name), re.escape(status[0])), index_text):
            findings.append(Finding("error", check, "docs/decisions/README.md", "ADR index does not display the matching status for {}".format(path.name)))
    return findings


def _markdown_links(text: str) -> Iterable[Tuple[int, str]]:
    fenced = False
    for number, line in enumerate(text.splitlines(), 1):
        if re.match(r"^\s*(```|~~~)", line):
            fenced = not fenced
            continue
        if fenced:
            continue
        scrubbed = re.sub(r"`[^`]*`", "", line)
        for match in INLINE_LINK_RE.finditer(scrubbed):
            yield number, match.group(1)
        reference = REFERENCE_LINK_RE.match(scrubbed)
        if reference:
            yield number, reference.group(1)


def validate_links(root: Path, paths: Iterable[str]) -> List[Finding]:
    findings: List[Finding] = []
    for relative in sorted(set(paths)):
        if not relative.lower().endswith(".md") or RepositoryView.ignored(relative):
            continue
        path = root / relative
        if not path.is_file():
            continue
        for line, raw_target in _markdown_links(_read(path)):
            target = raw_target.strip("<>")
            if not target or target.startswith("#") or any(mark in target for mark in ("{", "}")):
                continue
            parsed = urlparse(target)
            if parsed.scheme or target.startswith("//") or target.startswith("/"):
                continue
            local = unquote(target.split("#", 1)[0].split("?", 1)[0])
            if not local:
                continue
            resolved = (path.parent / local).resolve()
            try:
                resolved.relative_to(root)
            except ValueError:
                findings.append(Finding("error", "links", relative, "local link escapes the repository: {}".format(target), line))
                continue
            if not resolved.exists():
                findings.append(Finding("error", "links", relative, "broken local link: {}".format(target), line))
    return findings


def _todo_candidate(path: str, line: str) -> bool:
    if not TODO_RE.search(line):
        return False
    if path.lower().endswith(".md"):
        return bool(re.search(r"\b(?:TODO|FIXME)\s*[:(]", line, re.IGNORECASE))
    return True


def _legacy_todo_allowlist(root: Path) -> Tuple[Set[Tuple[str, int]], List[Finding]]:
    """Load and validate the ratcheted legacy TODO register.

    New markers must carry an inline issue reference. Existing markers may live in
    this exact-content register so adopting the policy does not force unrelated
    source files through whole-file formatters. A stale or ambiguous entry fails
    validation instead of silently becoming a permanent exception.
    """

    path = root / LEGACY_TODO_REGISTER
    if not path.is_file():
        return set(), []

    findings: List[Finding] = []
    allowed: Set[Tuple[str, int]] = set()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        return set(), [Finding("error", "todos", LEGACY_TODO_REGISTER, "invalid legacy TODO register: {}".format(exc))]

    entries = payload.get("entries") if isinstance(payload, dict) else None
    if not isinstance(entries, list):
        return set(), [Finding("error", "todos", LEGACY_TODO_REGISTER, "legacy TODO register must contain an entries array")]

    seen: Set[Tuple[str, str]] = set()
    for index, entry in enumerate(entries, 1):
        if not isinstance(entry, dict):
            findings.append(Finding("error", "todos", LEGACY_TODO_REGISTER, "entry {} must be an object".format(index)))
            continue
        relative = entry.get("path")
        marker = entry.get("marker")
        issue = entry.get("issue")
        if not isinstance(relative, str) or not relative or not isinstance(marker, str) or not marker:
            findings.append(Finding("error", "todos", LEGACY_TODO_REGISTER, "entry {} needs non-empty path and marker strings".format(index)))
            continue
        if not isinstance(issue, int) or issue <= 0:
            findings.append(Finding("error", "todos", LEGACY_TODO_REGISTER, "entry {} needs a positive issue number".format(index)))
            continue
        key = (relative, marker)
        if key in seen:
            findings.append(Finding("error", "todos", LEGACY_TODO_REGISTER, "duplicate legacy TODO entry for {}".format(relative)))
            continue
        seen.add(key)
        source = (root / relative).resolve()
        try:
            source.relative_to(root)
        except ValueError:
            findings.append(Finding("error", "todos", LEGACY_TODO_REGISTER, "entry {} escapes the repository".format(index)))
            continue
        try:
            matches = [
                (number, line)
                for number, line in enumerate(source.read_text(encoding="utf-8").splitlines(), 1)
                if marker in line
            ]
        except (OSError, UnicodeDecodeError):
            matches = []
        if len(matches) != 1:
            findings.append(
                Finding(
                    "error",
                    "todos",
                    LEGACY_TODO_REGISTER,
                    "entry {} must match exactly one current line in {} (found {})".format(index, relative, len(matches)),
                )
            )
            continue
        number, line = matches[0]
        if not _todo_candidate(relative, line):
            findings.append(Finding("error", "todos", LEGACY_TODO_REGISTER, "entry {} no longer identifies a TODO/FIXME".format(index)))
            continue
        if ISSUE_RE.search(line):
            findings.append(Finding("error", "todos", LEGACY_TODO_REGISTER, "entry {} is now linked inline and must be removed".format(index)))
            continue
        allowed.add((relative, number))
    return allowed, findings


def validate_todos(root: Path, view: RepositoryView, scope: str) -> List[Finding]:
    allowed, findings = _legacy_todo_allowlist(root)
    if scope == "changed":
        sources = view.changed()[1].items()
        severity = "error"
    else:
        collected: Dict[str, List[Tuple[int, str]]] = {}
        for relative in view.all_files():
            path = root / relative
            if not path.is_file():
                continue
            try:
                collected[relative] = list(enumerate(path.read_text(encoding="utf-8").splitlines(), 1))
            except (OSError, UnicodeDecodeError):
                continue
        sources = collected.items()
        severity = "warning"
    for relative, lines in sources:
        if RepositoryView.ignored(relative) or relative in {
            LEGACY_TODO_REGISTER,
            "scripts/agent/check_project_hygiene.py",
            "scripts/agent/tests/test_project_hygiene.py",
        }:
            continue
        for number, line in lines:
            if _todo_candidate(relative, line) and not ISSUE_RE.search(line) and (relative, number) not in allowed:
                findings.append(Finding(severity, "todos", relative, "TODO/FIXME needs a GitHub issue reference", number, "Use TODO(#123), GH-123, or an issue URL."))
    return findings


def validate_doc_impact(view: RepositoryView) -> List[Finding]:
    changed = view.changed()[0]
    findings: List[Finding] = []
    markdown = {path for path in changed if path.lower().endswith(".md")}

    def touched(*prefixes: str) -> bool:
        return any(any(path == prefix or path.startswith(prefix.rstrip("/") + "/") for prefix in prefixes) for path in changed)

    def require(label: str, expected: Set[str]) -> None:
        missing = expected - changed
        for path in sorted(missing):
            findings.append(Finding("warning", "doc-impact", path, "{} changed without this expected companion".format(label)))

    if touched("config/mower_config.schema.json", "config/mower_config.sh.example"):
        require("configuration contract", {"config/mower_config.schema.json", "config/mower_config.sh.example", "docs/CONFIGURATION.md"})
    if any(path.startswith("docs/vesc_configs/") and path.endswith(".xml") for path in changed):
        require("VESC snapshot", {"docs/vesc_configs/README.md", "docs/VESC_MAINTENANCE.md"})
    if touched("docker/openmower_entrypoint.sh", "docker/openmower_entrypoint.legacy.sh", "docker/openmower_entrypoint.pi.sh"):
        require("runtime entrypoint", {"docs/DOCKER.md"})
    safety_prefixes = (
        "src/mower_logic/",
        "src/mower_comms_v1/",
        "src/mower_comms_v2/",
        "src/mower_hardware/",
        "src/open_mower/launch/",
        "src/open_mower/params/hardware_specific/",
    )
    if any(path.startswith(safety_prefixes) for path in changed) and not markdown:
        findings.append(Finding("warning", "doc-impact", "docs/", "safety-sensitive behavior changed without any Markdown documentation change"))
    source_suffixes = (".c", ".cc", ".cpp", ".h", ".hpp", ".py", ".ts", ".tsx", ".sh", ".launch", ".json", ".yaml", ".yml")
    product_change = any(path.startswith(("src/", "webui/", "docker/", "config/", "utils/")) and path.endswith(source_suffixes) for path in changed)
    if product_change and not markdown:
        findings.append(Finding("warning", "doc-impact", "docs/", "implementation changed without documentation or an explicit reviewed no-docs decision"))
    return findings


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description="Validate project state, ExecPlans, ADRs, Markdown links, issue-backed TODOs, and changed-path documentation impact.",
    )
    result.add_argument("--root", help="repository path; defaults to the Git root containing the current directory")
    result.add_argument("--scope", choices=("all", "changed"), default="changed", help="validate all tracked files or only branch/worktree changes (default: changed)")
    result.add_argument("--base", help="Git base revision for changed scope; defaults to PROJECT_HYGIENE_BASE, a uniquely matching active ExecPlan baseline, then the default branch")
    result.add_argument("--format", choices=("text", "json"), default="text", dest="output_format", help="diagnostic output format")
    result.add_argument("--strict", action="store_true", help="treat warnings as failures")
    result.add_argument("--check", action="append", choices=CHECKS, help="run only this check; repeat for multiple checks")
    result.add_argument("--print-base", action="store_true", help="print the resolved changed-scope comparison base and exit")
    return result


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = parser().parse_args(argv)
    try:
        root = find_root(args.root)
        view = RepositoryView(root, args.base)
        if args.print_base:
            base = view.comparison_base()
            if not base:
                raise HygieneRuntimeError("no changed-scope comparison base could be resolved")
            print(base)
            return 0
        selected = args.check or list(CHECKS)
        findings: List[Finding] = []
        if "project-state" in selected:
            findings.extend(validate_project_state(root))
        if "exec-plans" in selected:
            findings.extend(validate_exec_plans(root))
        if "adrs" in selected:
            findings.extend(validate_adrs(root))
        if "links" in selected:
            link_paths = view.all_files() if args.scope == "all" else view.changed()[0]
            findings.extend(validate_links(root, link_paths))
        if "todos" in selected:
            findings.extend(validate_todos(root, view, args.scope))
        if "doc-impact" in selected:
            findings.extend(validate_doc_impact(view))
    except HygieneRuntimeError as exc:
        if args.output_format == "json":
            print(json.dumps({"ok": False, "runtime_error": str(exc), "findings": []}, indent=2))
        else:
            print("hygiene runtime error: {}".format(exc), file=sys.stderr)
        return 2

    findings.sort(key=lambda item: (item.severity != "error", item.check, item.path, item.line or 0, item.message))
    errors = sum(item.severity == "error" for item in findings)
    warnings = len(findings) - errors
    failed = bool(errors or (args.strict and warnings))
    if args.output_format == "json":
        print(json.dumps({"ok": not failed, "errors": errors, "warnings": warnings, "findings": [asdict(item) for item in findings]}, indent=2))
    else:
        for item in findings:
            location = item.path + (":" + str(item.line) if item.line else "")
            print("{} [{}] {}: {}".format(item.severity.upper(), item.check, location, item.message))
            if item.remediation:
                print("  hint: {}".format(item.remediation))
        print("Project hygiene: {} error(s), {} warning(s).".format(errors, warnings))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(run())

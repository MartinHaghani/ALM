#!/usr/bin/env python3
"""Run deterministic fresh-agent routing assertions without invoking a model."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence


@dataclass(frozen=True)
class Result:
    case: str
    path: str
    kind: str
    ok: bool
    detail: str


def _root(explicit: Optional[str]) -> Path:
    start = Path(explicit).resolve() if explicit else Path.cwd().resolve()
    result = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"], cwd=str(start), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    if result.returncode:
        raise RuntimeError("{} is not in a Git worktree".format(start))
    return Path(result.stdout.strip()).resolve()


def _matches(root: Path, pattern: str) -> List[Path]:
    if any(token in pattern for token in ("*", "?", "[")):
        return sorted(path for path in root.glob(pattern) if path.is_file())
    path = root / pattern
    return [path] if path.is_file() else []


def evaluate(root: Path, spec: Dict[str, Any]) -> List[Result]:
    results: List[Result] = []
    for case in spec.get("cases", []):
        case_id = str(case.get("id", "unnamed"))
        for assertion in case.get("assertions", []):
            pattern = str(assertion.get("path", ""))
            kind = str(assertion.get("kind", "contains"))
            paths = _matches(root, pattern)
            if kind == "exists":
                results.append(Result(case_id, pattern, kind, bool(paths), "path exists" if paths else "path is missing"))
                continue
            if not paths:
                results.append(Result(case_id, pattern, kind, False, "no files matched"))
                continue
            values = [str(value) for value in assertion.get("values", [])]
            case_sensitive = bool(assertion.get("case_sensitive", False))
            for path in paths:
                text = path.read_text(encoding="utf-8")
                comparable = text if case_sensitive else text.casefold()
                if kind == "contains":
                    missing = [value for value in values if (value if case_sensitive else value.casefold()) not in comparable]
                elif kind == "headings":
                    headings = {
                        match.group(1).strip().casefold()
                        for line in text.splitlines()
                        if (match := re.match(r"^#{1,6}\s+(.+?)\s*#*\s*$", line))
                    }
                    missing = [value for value in values if value.casefold() not in headings]
                elif kind == "regex":
                    flags = 0 if case_sensitive else re.IGNORECASE
                    missing = [value for value in values if not re.search(value, text, flags)]
                else:
                    results.append(Result(case_id, path.relative_to(root).as_posix(), kind, False, "unknown assertion kind"))
                    continue
                results.append(
                    Result(
                        case_id,
                        path.relative_to(root).as_posix(),
                        kind,
                        not missing,
                        "all assertions present" if not missing else "missing: {}".format(", ".join(missing)),
                    )
                )
    return results


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description="Run machine-readable structural assertions that a fresh agent can find canonical startup, safety, handoff, tracking, and delegation routes. This command never calls a model or external service.",
    )
    result.add_argument("--root", help="repository path; defaults to the current Git root")
    result.add_argument("--spec", default="scripts/agent/evals/fresh_agent_context.json", help="JSON assertion spec relative to the repository root")
    result.add_argument("--format", choices=("text", "json"), default="text", dest="output_format")
    return result


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = parser().parse_args(argv)
    try:
        root = _root(args.root)
        spec_path = Path(args.spec)
        if not spec_path.is_absolute():
            spec_path = root / spec_path
        spec = json.loads(spec_path.read_text(encoding="utf-8"))
        results = evaluate(root, spec)
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
        print("context evaluation error: {}".format(exc), file=sys.stderr)
        return 2
    failed = [result for result in results if not result.ok]
    if args.output_format == "json":
        print(json.dumps({"ok": not failed, "assertions": len(results), "failures": len(failed), "results": [asdict(result) for result in results]}, indent=2))
    else:
        for result in failed:
            print("FAIL [{}] {} ({}): {}".format(result.case, result.path, result.kind, result.detail))
        print("Fresh-agent context: {}/{} assertion(s) passed.".format(len(results) - len(failed), len(results)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(run())

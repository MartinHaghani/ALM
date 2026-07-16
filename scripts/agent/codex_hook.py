#!/usr/bin/env python3
"""Provide concise Codex lifecycle context and a non-mutating stop audit."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Sequence


def _git(root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=str(root), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
    )
    return result.stdout.strip() if result.returncode == 0 else ""


def _root(explicit: Optional[str], payload: Dict[str, Any]) -> Path:
    if explicit:
        return Path(explicit).expanduser().resolve()
    cwd = Path(str(payload.get("cwd") or Path.cwd())).resolve()
    result = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if result.returncode == 0:
        return Path(result.stdout.strip()).resolve()
    return Path(__file__).resolve().parents[2]


def _payload() -> Dict[str, Any]:
    raw = sys.stdin.read()
    if not raw.strip():
        return {}
    try:
        value = json.loads(raw)
        return value if isinstance(value, dict) else {}
    except json.JSONDecodeError:
        return {}


def context(root: Path, payload: Dict[str, Any]) -> int:
    event = str(payload.get("hook_event_name") or "SessionStart")
    branch = _git(root, "branch", "--show-current") or "detached HEAD"
    dirty = len([line for line in _git(root, "status", "--porcelain").splitlines() if line])
    active_dir = root / "docs" / "exec-plans" / "active"
    active = sorted(path.name for path in active_dir.glob("*.md")) if active_dir.exists() else []
    plans = ", ".join(active[:5]) if active else "none"
    lines = [
        "Repository operating context (generated; canonical files remain authoritative):",
        "- Read the applicable AGENTS.md files, docs/PROJECT_STATE.md, and the relevant active ExecPlan before acting.",
        "- Read PLANS.md before creating or changing an ExecPlan; use the project-operations skill for repository work.",
        "- Inspect and preserve unrelated work. Current branch: {}; changed/untracked paths: {}.".format(branch, dirty),
        "- Active ExecPlans: {}.".format(plans),
    ]
    if event == "SubagentStart":
        lines.extend(
            [
                "- Stay within the delegated scope. Do not write unless the parent assigned explicit path ownership.",
                "- One writer owns each file/shared worktree. Return distilled findings with exact evidence, uncertainty, and next steps.",
                "- Do not spawn another agent.",
            ]
        )
    output = {
        "hookSpecificOutput": {
            "hookEventName": event,
            "additionalContext": "\n".join(lines),
        }
    }
    print(json.dumps(output))
    return 0


def stop(root: Path) -> int:
    checker = root / "scripts" / "agent" / "check_project_hygiene.py"
    result = subprocess.run(
        [sys.executable, str(checker), "--root", str(root), "--scope", "changed", "--format", "json"],
        cwd=str(root),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        report = json.loads(result.stdout)
    except json.JSONDecodeError:
        report = {"runtime_error": result.stderr.strip() or "hygiene checker returned invalid output", "findings": []}
    findings = report.get("findings") or []
    runtime_error = report.get("runtime_error")
    if not findings and not runtime_error:
        return 0
    summary = []
    for item in findings[:8]:
        location = item.get("path", "repository")
        if item.get("line"):
            location += ":" + str(item["line"])
        summary.append("- {} [{}] {}: {}".format(item.get("severity", "warning"), item.get("check", "hygiene"), location, item.get("message", "review required")))
    if len(findings) > 8:
        summary.append("- ...and {} more finding(s)".format(len(findings) - 8))
    if runtime_error:
        summary.insert(0, "- checker runtime error: {}".format(runtime_error))
    message = "Project handoff hygiene needs review before declaring completion:\n{}\nRun `python3 scripts/agent/check_project_hygiene.py --scope changed` for the full report. The hook did not edit, commit, or block work.".format("\n".join(summary))
    print(json.dumps({"continue": True, "systemMessage": message}))
    return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="Codex lifecycle hook: inject repository routes or report non-mutating stop-time hygiene findings.")
    result.add_argument("mode", choices=("context", "stop"), help="context emits additional developer context; stop emits hygiene warnings")
    result.add_argument("--root", help="repository root; defaults to the hook payload cwd or this script's repository")
    return result


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = parser().parse_args(argv)
    payload = _payload()
    root = _root(args.root, payload)
    return context(root, payload) if args.mode == "context" else stop(root)


if __name__ == "__main__":
    sys.exit(run())

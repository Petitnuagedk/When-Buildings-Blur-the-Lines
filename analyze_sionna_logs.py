#!/usr/bin/env python3
"""Analyze Sionna log files and classify them as complete / error / timeout / unknown."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Iterable

COMPLETE_PATTERNS = [
    "Simulation completed",
    "Completed successfully",
    "Run finished",
    "All done",
    "Finished executing",
    "Simulation ended",
    "successfully",
    "Sontek?",  # placeholder, not used but to maintain ordering
]
TIMEOUT_PATTERNS = [
    "timed out",
    "timeout",
    "TIMEOUT",
    "time limit",
    "walltime",
    "exceeded time",
    "deadline",
    "killed by wall-time",
]
ERROR_PATTERNS = [
    "Traceback",
    "Exception",
    "Assertion",
    "Failed",
    "ERROR",
    "error",
    "Killed",
    "Aborted",
    "Segmentation fault",
    "core dumped",
    "terminate called",
    "std::bad_alloc",
    "std::exception",
    "AssertionError",
    "Unhandled exception",
]


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""


def find_match(text: str, patterns: Iterable[str]) -> list[str]:
    text_lower = text.lower()
    return [p for p in patterns if p.lower() in text_lower]


def classify_log(out_text: str, err_text: str) -> str:
    timeout_hits = find_match(out_text + "\n" + err_text, TIMEOUT_PATTERNS)
    if timeout_hits:
        return "timeout"

    err_hits = find_match(out_text + "\n" + err_text, ERROR_PATTERNS)
    if err_hits:
        return "error"

    complete_hits = find_match(out_text, COMPLETE_PATTERNS)
    if complete_hits:
        return "complete"

    if err_text.strip():
        return "error"

    # Detect likely incomplete build/run from a final progress line or an abrupt ending.
    if out_text.strip().endswith("]") or out_text.strip().endswith("..."):
        return "unknown"

    return "unknown"


def summarize(directory: Path, show_details: bool) -> dict[str, list[Path]]:
    results: dict[str, list[Path]] = {"complete": [], "error": [], "timeout": [], "unknown": []}
    if not directory.exists():
        raise FileNotFoundError(f"Directory not found: {directory}")
    for err_path in sorted(directory.glob("*.err")):
        if not err_path.is_file():
            continue
        base_name = err_path.stem
        out_path = directory / f"{base_name}.out"
        out_text = read_text(out_path) if out_path.exists() else ""
        err_text = read_text(err_path)
        status = classify_log(out_text, err_text)
        results[status].append(out_path if out_path.exists() else err_path)
    if not results["complete"] and not results["error"] and not results["timeout"] and not results["unknown"]:
        # fallback: scan out files if no err files were present
        for out_path in sorted(directory.glob("*.out")):
            if not out_path.is_file():
                continue
            out_text = read_text(out_path)
            err_text = ""
            status = classify_log(out_text, err_text)
            results[status].append(out_path)
    if show_details:
        return results
    return results


def print_summary(results: dict[str, list[Path]]) -> None:
    total = sum(len(files) for files in results.values())
    print(f"Found {total} log entries")
    for status in ("complete", "error", "timeout", "unknown"):
        files = results[status]
        print(f"  {status:8}: {len(files)}")
    print()
    for status in ("error", "timeout", "unknown", "complete"):
        files = results[status]
        if not files:
            continue
        print(f"{status.upper()} files ({len(files)}):")
        for path in files[:20]:
            print(f"  {path.name}")
        if len(files) > 20:
            print(f"  ... plus {len(files)-20} more")
        print()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze Sionna log files for completion, errors, and timeouts."
    )
    parser.add_argument(
        "log_dir",
        help="Path to the log directory containing .out and .err files",
        nargs="?",
        default=r"C:\Users\hugol\Documents\Firenze\B\j1\SASB-data\sionna-data\log",
    )
    parser.add_argument(
        "--details",
        action="store_true",
        help="Print the list of files for each status category.",
    )
    args = parser.parse_args()
    directory = Path(args.log_dir).expanduser()
    results = summarize(directory, args.details)
    print_summary(results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

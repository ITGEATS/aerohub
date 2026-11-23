"""Check declared migration cases against a recorded result index."""

import argparse
import json
from pathlib import Path
import sys


def read_object(path):
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return document


def check(plan, results, evidence_root):
    issues = []
    for case in sorted(results.keys() - plan.keys()):
        issues.append(f"{case}: undeclared result")

    for case, requirement in plan.items():
        if not isinstance(requirement, str) or not requirement.strip():
            raise ValueError(f"{case}: expected a non-empty command/scenario and pass criterion")

        result = results.get(case, {})
        if not isinstance(result, dict):
            raise ValueError(f"{case}: expected a result object")
        status = result.get("status", "pending")
        if status not in ("passed", "failed", "skipped", "pending"):
            raise ValueError(f"{case}: unknown status {status!r}")
        if status != "passed":
            issues.append(f"{case}: {status}")

        evidence = result.get("evidence", [])
        if not isinstance(evidence, list):
            raise ValueError(f"{case}: evidence must be a list")
        if status == "passed" and not evidence:
            issues.append(f"{case}: missing evidence")
        for reference in evidence:
            if not isinstance(reference, str) or not reference.strip():
                raise ValueError(f"{case}: expected a relative evidence path")
            path = (evidence_root / reference).resolve()
            if Path(reference).is_absolute() or not path.is_relative_to(evidence_root):
                issues.append(f"{case}: evidence outside evidence root: {reference}")
            elif not path.is_file() or path.stat().st_size == 0:
                issues.append(f"{case}: missing or empty evidence file: {reference}")
    return issues


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("results", type=Path)
    parser.add_argument("evidence_root", type=Path)
    args = parser.parse_args()

    try:
        plan = read_object(args.plan)
        if not plan:
            raise ValueError(f"{args.plan}: expected at least one declared case")
        results = read_object(args.results) if args.results.exists() else {}
        issues = check(plan, results, args.evidence_root.resolve())
    except (ValueError, OSError, RuntimeError) as error:
        print(error, file=sys.stderr)
        return 2

    if issues:
        print("\n".join(issues), file=sys.stderr)
        return 1
    print(f"{len(plan)} declared cases passed with evidence files referenced.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

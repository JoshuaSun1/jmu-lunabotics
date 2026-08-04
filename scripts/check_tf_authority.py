#!/usr/bin/env python3
"""Check the documented TF authority contract until runtime launch tests exist."""

from __future__ import annotations

import argparse
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def check_contract() -> int:
    """Ensure the sole-authority rules remain present in the architecture record."""
    architecture = (REPOSITORY_ROOT / "docs" / "architecture.md").read_text(encoding="utf-8")
    expected_phrases = (
        "`map -> odom` | Global `robot_localization` EKF",
        "`odom -> base_link` | Local `robot_localization` EKF",
        "`base_link -> rigid sensor/wheel frames` | `robot_state_publisher`",
    )
    missing = [phrase for phrase in expected_phrases if phrase not in architecture]
    if missing:
        for phrase in missing:
            print(f"missing TF authority contract: {phrase}")
        return 1
    print("TF authority contract is documented; runtime publisher verification is Phase 1.")
    return 0


def main() -> int:
    """Parse command-line options and run only the Phase 0 static check."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--runtime",
        action="store_true",
        help="request runtime verification; unavailable until the Phase 1 launch test exists",
    )
    arguments = parser.parse_args()
    if arguments.runtime:
        parser.error("runtime TF verification is intentionally deferred to Phase 1")
    return check_contract()


if __name__ == "__main__":
    raise SystemExit(main())

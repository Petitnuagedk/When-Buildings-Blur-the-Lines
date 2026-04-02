#!/usr/bin/env python3
"""Run UrbanCompConnectivity for every combination of loss model and node count.

Loss models: FOBA, Friis, ItuR1411LosPropagationLossModel, TwoRayGroundPropagationLossModel
Node counts: 10, 20, 30, 40, 50, 60, 70, 80, 90, 100

Usage:
    python run_all_connectivity_SA.py
    python run_all_connectivity_SA.py --nodes 10 20 --result-dir my-results
    python run_all_connectivity_SA.py --ns3-runner /path/to/ns3
"""

import argparse
import subprocess
import sys
from pathlib import Path

LOSS_MODELS = [
    "FOBA",
    "Friis",
    "ItuR1411LosPropagationLossModel",
    "TwoRayGroundPropagationLossModel",
]

NODE_COUNTS = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]


def run(cmd, **kwargs):
    print("Running:", " ".join(str(c) for c in cmd))
    subprocess.run(cmd, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(
        description="Run UrbanCompConnectivity for all loss models and node counts"
    )
    parser.add_argument(
        "--nodes",
        type=int,
        nargs="*",
        default=NODE_COUNTS,
        help="Node counts to run (default: 10 20 30 40 50 60 70 80 90 100)",
    )
    parser.add_argument(
        "--loss-models",
        nargs="*",
        default=LOSS_MODELS,
        help="Loss models to run (default: all four)",
    )
    parser.add_argument(
        "--result-dir",
        default="connectivity-results-SA",
        help="Base directory for outputs (default: connectivity-results-SA)",
    )
    parser.add_argument(
        "--ns3-runner",
        default="./ns3",
        help="Path to the ns3 runner executable (default: ./ns3)",
    )
    parser.add_argument(
        "--build-first",
        action="store_true",
        help="Build the ns3 program before running",
    )
    parser.add_argument(
        "--extra",
        nargs=argparse.REMAINDER,
        help="Extra arguments forwarded to the simulator",
    )

    args = parser.parse_args()

    ns3_runner = Path(args.ns3_runner).resolve()
    if not ns3_runner.exists():
        print(f"ERROR: ns3 runner not found: {ns3_runner}")
        sys.exit(1)

    if args.build_first:
        run([str(ns3_runner), "build", "scratch/UrbanCompConnectivity.cc"])

    total = len(args.loss_models) * len(args.nodes)
    done = 0

    for lm in args.loss_models:
        for n in args.nodes:
            result_path = f"{args.result_dir}/{lm}/numNodes/{n}"
            cmd = [
                str(ns3_runner),
                "run",
                "scratch/UrbanCompConnectivity.cc",
                "--",
                f"--numNodes={n}",
                f"--lossModel={lm}",
                f"--resultPath={result_path}",
            ]
            if args.extra:
                cmd.extend(args.extra)
            done += 1
            print(f"\n[{done}/{total}] lossModel={lm}  numNodes={n}")
            run(cmd)

    print(f"\nAll {total} runs completed. Results in: {args.result_dir}/")


if __name__ == "__main__":
    main()

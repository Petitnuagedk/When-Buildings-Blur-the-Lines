#!/usr/bin/env python3
"""Run UrbanCompConnectivity-v2 over all epochs and node counts.

This script builds the NS-3 program (if necessary) and then runs it for each
combination of epoch and node count, placing outputs in per-run directories.

By default it uses the same configuration as the layout generator:
- epochs: 1..10
- nodes: 50, 100, 150, 200, 300, 500

Usage:
    python run_all_connectivity.py
    python run_all_connectivity.py --epochs 1 2 3 --nodes 50 100

Note: This assumes the `ns3` runner is available as `./ns3` in the workspace root.
"""

import argparse
import subprocess
import sys
from pathlib import Path


def run(cmd, **kwargs):
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description="Run UrbanCompConnectivity-v2 for multiple epochs and node counts")
    parser.add_argument("--epochs", type=int, nargs="*", default=[0],
                        help="Epoch numbers to run (default: 0)")
    parser.add_argument("--nodes", type=int, nargs="*", default=[50, 100, 150, 200, 300, 500],
                        help="Node counts to run (default: 50 100 150 200 300 500)")
    parser.add_argument("--loss-model", default="FOBA", help="Propagation loss model (default: FOBA)")
    parser.add_argument("--ra", default="aodv", help="Routing algorithm (default: aodv)")
    parser.add_argument("--layout-dir", default="scratch", help="Directory containing layout CSV files")
    parser.add_argument("--result-dir", default="results-con", help="Base directory for outputs")
    parser.add_argument("--ns3-runner", default="./ns3", help="Path to the ns3 runner executable")
    parser.add_argument("--build-first", action="store_true", help="Build the ns3 program before running")
    parser.add_argument("--extra", nargs=argparse.REMAINDER,
                        help="Extra arguments to pass to the simulator executable")

    args = parser.parse_args()

    # Ensure the ns3 runner is available (use an absolute path so it doesn't rely on PATH)
    ns3_runner = Path(args.ns3_runner).resolve()
    if not ns3_runner.exists():
        print(f"ERROR: ns3 runner not found: {ns3_runner}")
        sys.exit(1)

    if args.build_first:
        build_cmd = [str(ns3_runner), "build", "scratch/UrbanCompConnectivity-v3.cc"]
        run(build_cmd)

    for epoch in args.epochs:
        for n in args.nodes:
            cmd = [
                str(ns3_runner),
                "run",
                "scratch/UrbanCompConnectivity-v2.cc",
                "--",
                f"--epoch={epoch}",
                f"--numNodes={n}",
                f"--layoutDir={args.layout_dir}",
                f"--lossModel={args.loss_model}",
                f"--RA={args.ra}",
                f"--resultPath={args.result_dir}",
            ]
            if args.extra:
                cmd.extend(args.extra)
            run(cmd)


if __name__ == "__main__":
    main()

"""
run_all_plots.py
----------------
Master runner that orchestrates all figure-generation scripts with a shared,
paper-ready matplotlib style.

Figures produced (in order):
  §1  Loss comparison – scenario 1           → plots/comparison/
  §2  Loss comparison – scenario 2           → plots/comparison/
      (Both produced in one call; the script splits them automatically.)
  §3  Scenario B – three-figure dashboard    → plots-SB-dashboard-alt/
      Fig 1 · Line plots (metric vs num_nodes, one line per model)
      Fig 2 · Model comparison heatmaps (row-normalised)
      Fig 3 · Radar / spider chart (model fingerprint)
  §4  Routing metrics – Scenario A and B     → plots-routing-alt/
  §5  Drop reasons                           → plots-drops/
  §6a Graph-metric pre-processing            → (various *-output/ dirs)
      Computes pair metrics (lifetime, stability, persistency …) from raw
      connectivity matrices and saves per-pair CSVs.
  §6b Scenario A – connectivity dashboard    → plots-SA-dashboard/

Usage:
    python run_all_plots.py                  # full run
    python run_all_plots.py --skip-processing  # skip §6a (heavy precompute step)
"""

import argparse
import os
import subprocess
import sys
import tempfile
import textwrap

# ─────────────────────────────────────────────────────────────────────────────
# Path configuration
# ─────────────────────────────────────────────────────────────────────────────
ROOT         = os.path.dirname(os.path.abspath(__file__))
PLOT_SCRIPTS = os.path.join(ROOT, "plot_scripts")
GRAPH_TOOLS  = os.path.join(ROOT, "graph-metric-tools")

# Root of the shared simulation-data tree  ← adjust if the data lives elsewhere
SASB_DATA    = r"C:\Users\hugol\Documents\Firenze\B\j1\SASB-data"
CONE_DATA    = os.path.join(SASB_DATA, "cone")
FOSIO_CSV    = os.path.join(SASB_DATA, "fosio", "foba-sionna-comparison")

# Connectivity source dirs consumed by graph-metric-exta.py (§6a).
# Each entry: tag → path that contains the raw connectivity_matrices.csv tree.
# Output is written to {path}-output/ (which PlotSA/SB dashboards read from).
GRAPH_SRCS = {
    "SA-all":  os.path.join(CONE_DATA, "results-con-SA-all"),
    "SB-FOBA": os.path.join(CONE_DATA, "results-con-SB-FOBA-alt"),
    "SB-Friis":os.path.join(CONE_DATA, "results-con-SB-Frii"),
    "SB-ITU":  os.path.join(CONE_DATA, "results-con-SB-ItuR"),
    "SB-TwoR": os.path.join(CONE_DATA, "results-con-SB-TwoR"),
}

# ─────────────────────────────────────────────────────────────────────────────
# Uniform matplotlib style
# Injected into every subprocess via the MATPLOTLIBRC environment variable so
# that no existing script needs to be modified.
# ─────────────────────────────────────────────────────────────────────────────
STYLE = textwrap.dedent("""\
    # Paper-ready uniform style - shared by all plot scripts
    font.family          : DejaVu Sans
    font.size            : 12
    axes.titlesize       : 14
    axes.titleweight     : bold
    axes.labelsize       : 12
    xtick.labelsize      : 11
    ytick.labelsize      : 11
    legend.fontsize      : 11
    legend.framealpha    : 0.85
    legend.edgecolor     : 0.7
    legend.borderpad     : 0.5
    legend.labelspacing  : 0.4
    figure.dpi           : 150
    savefig.dpi          : 150
    savefig.bbox         : tight
    savefig.pad_inches   : 0.05
    axes.grid            : True
    grid.alpha           : 0.3
    grid.linestyle       : --
    grid.color           : 0.75
    lines.linewidth      : 1.8
    lines.markersize     : 6
    errorbar.capsize     : 3
    axes.spines.top      : False
    axes.spines.right    : False
""")

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _build_env(rc_path: str) -> dict:
    """Return os.environ copy with the shared style and non-interactive backend."""
    env = os.environ.copy()
    env["MATPLOTLIBRC"] = rc_path   # matplotlib reads this as the rc file path
    env["MPLBACKEND"]   = "Agg"     # non-interactive; safe for batch runs
    return env


def _run(script: str, extra_args: list, env: dict, label: str) -> bool:
    """
    Execute *script* as a subprocess and print a section banner.
    Returns True on success.
    """
    bar = "─" * 68
    print(f"\n{bar}\n  {label}\n{bar}", flush=True)
    result = subprocess.run(
        [sys.executable, script] + extra_args,
        env=env,
        cwd=ROOT,
    )
    ok = result.returncode == 0
    if not ok:
        print(f"  [WARN] exited with code {result.returncode}", flush=True)
    return ok


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--skip-processing",
        action="store_true",
        help=(
            "Skip §6a graph-metric-exta.py pre-processing. "
            "Use this when pair-metrics CSVs are already up to date."
        ),
    )
    cli = ap.parse_args()

    # Write the shared style to a temporary matplotlibrc file.
    # All subprocesses inherit the MATPLOTLIBRC env var that points to it.
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".matplotlibrc", delete=False, prefix="plot_style_",
        encoding="utf-8",
    ) as fh:
        fh.write(STYLE)
        rc_path = fh.name

    print(f"Shared style : {rc_path}")
    print(f"Output root  : {ROOT}")

    # All output lives under a single top-level directory.
    ALL_PLOTS = os.path.join(ROOT, "allplot")
    os.makedirs(ALL_PLOTS, exist_ok=True)

    results: dict[str, bool] = {}

    try:
        env = _build_env(rc_path)

        # ── §1 + §2  Loss comparison, scenario 1 & 2 ────────────────────────
        # PlotFobaSionnaComparison.py iterates over all scenarios found in the
        # CSV and saves a separate figure for each one (scenario1, scenario2 …).
        results["loss_comparison_sc1_sc2"] = _run(
            os.path.join(PLOT_SCRIPTS, "PlotFobaSionnaComparison.py"),
            ["--input",  FOSIO_CSV,
             "--output", os.path.join(ALL_PLOTS, "loss_comparison")],
            env,
            "§1 + §2  Loss comparison  (scenario 1 & 2 — FOBA / Friis / ITU-R / Two-Ray / Sionna RT)",
        )

        # ── §3  Scenario B – three-figure dashboard ──────────────────────────
        # Produces:
        #   Fig 1 · Line plots  (metric vs num_nodes, shaded IQR band)
        #   Fig 2 · Heatmaps    (row-normalised model comparison)
        #   Fig 3 · Radar chart (model fingerprint at selected node counts)
        results["sb_dashboard"] = _run(
            os.path.join(PLOT_SCRIPTS, "PlotSB_CompactDashboard.py"),
            ["--output", os.path.join(ALL_PLOTS, "scenario_B_dashboard")],
            env,
            "§3  Scenario B – three-figure dashboard  [line · heatmap · radar]",
        )

        # ── §4  Routing metrics – Scenario A and B ───────────────────────────
        # Produces per-scenario / per-algorithm panels:
        #   PDR · End-to-End Delay · Goodput · Route Acquisition · Routing OH
        results["routing_metrics"] = _run(
            os.path.join(PLOT_SCRIPTS, "PlotRoutingMetrics.py"),
            ["--output", os.path.join(ALL_PLOTS, "routing_metrics")],
            env,
            "§4  Routing metrics – Scenario A and B  (PDR · EED · Goodput · RouteAcq · RoutingOH)",
        )

        # ── §5  Drop reasons ─────────────────────────────────────────────────
        # Stacked-bar (proportional) and absolute congestion-drops line charts.
        results["drop_reasons"] = _run(
            os.path.join(PLOT_SCRIPTS, "PlotDropData.py"),
            ["--output", os.path.join(ALL_PLOTS, "drop_reasons")],
            env,
            "§5  Drop reasons – congestion vs propagation vs MAC  (Scenario B)",
        )

        # ── §6a  Graph-metric pre-processing ────────────────────────────────
        # Reads raw connectivity_matrices.csv trees and outputs
        # {model}_numNodes_{n}_pair_metrics.csv files consumed by the dashboards.
        # Skip with --skip-processing if those CSVs already exist.
        if not cli.skip_processing:
            gm_script = os.path.join(GRAPH_TOOLS, "graph-metric-exta.py")
            for tag, src_dir in GRAPH_SRCS.items():
                out_dir = src_dir.rstrip("\\/") + "-output"
                results[f"gm_preproc_{tag}"] = _run(
                    gm_script,
                    ["--results-dir", src_dir, "--out-dir", out_dir],
                    env,
                    f"§6a Graph metrics – pre-process  [{tag}]",
                )
        else:
            print("\n  [INFO] Skipping §6a (--skip-processing set).", flush=True)

        # ── §6b  Scenario A – connectivity dashboard ─────────────────────────
        # Visualises the pair_metrics CSVs produced by §6a:
        #   Fig 1 · Line plots  (metric vs num_nodes, shaded IQR band)
        #   Fig 2 · Heatmaps    (row-normalised model comparison)
        results["sa_dashboard"] = _run(
            os.path.join(PLOT_SCRIPTS, "PlotSA_CompactDashboard.py"),
            ["--output", os.path.join(ALL_PLOTS, "scenario_A_dashboard")],
            env,
            "§6b Graph metrics – Scenario A dashboard  [line · heatmap]",
        )

    finally:
        os.unlink(rc_path)

    # ── Summary ───────────────────────────────────────────────────────────────
    bar_d = "═" * 68
    print(f"\n{bar_d}\n  Run summary\n{bar_d}")
    for name, ok in results.items():
        status = "OK  " if ok else "WARN"
        print(f"  [{status}]  {name}")
    n_warn = sum(1 for ok in results.values() if not ok)
    print(bar_d)
    if n_warn:
        print(f"  {n_warn} script(s) exited with a non-zero code – check output above.")
    else:
        print("  All scripts completed successfully.")
    print()


if __name__ == "__main__":
    main()

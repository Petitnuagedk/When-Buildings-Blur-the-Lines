"""
PlotRoutingMetrics.py
---------------------
Compact multi-panel dashboard focused on routing performance metrics for both
Scenario A (v1, UrbanRaCompDir-SA) and Scenario B (v2, UrbanRaCompDir-v2).

Five routing metrics:
  - PDR        – Packet Delivery Ratio              (all algorithms)
  - EED        – End-to-End Delay (s)               (all algorithms)
  - Goodput    – Goodput (kbps)                     (all algorithms)
  - RouteAcq   – Route Acquisition Time (s)         [AODV only]
  - RoutingOH  – Routing Overhead (RouteSignalizationPacketsSent, absolute count)  (all algorithms)

Three figures saved in plots-routing/:
  Figure 1 – SA Routing Metrics Dashboard   (5 rows = metrics × 3 cols = algorithms)
  Figure 2 – SB Routing Metrics Dashboard   (same layout)
  Figure 3 – SA vs SB side-by-side for AODV (5 rows × 2 cols)

Run from the repository root:
    python plot_scripts/PlotRoutingMetrics.py
"""

import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D

try:
    from scipy.stats import t as student_t
except ImportError:
    student_t = None

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Allow output directory to be overridden from the command line so that a
# master runner can collect all figures into a single tree.
import argparse as _ap
_ap_parser = _ap.ArgumentParser(add_help=False)
_ap_parser.add_argument("--output", default=os.path.join(SCRIPT_DIR, "..", "plots-routing-alt"))
_known, _unknown = _ap_parser.parse_known_args()
OUT_DIR = _known.output

SOURCE_SA = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "..", "SASB-data", "UrbanRaCompDir-SA-alt"))
SOURCE_SB = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "..", "SASB-data", "UrbanRaCompDir-SB-alt"))

MAX_GOODPUT_KBPS = 150.0

LOSS_MODELS = [
    "FOBA",
    "Friis",
    "TwoRayGroundPropagationLossModel",
    "ItuR1411LosPropagationLossModel",
]

MODEL_LABELS = {
    "FOBA":                             "FOBA",
    "Friis":                            "Friis",
    "TwoRayGroundPropagationLossModel": "Two-Ray",
    "ItuR1411LosPropagationLossModel":  "ITU-R 1411",
}

MODEL_COLORS = {
    "FOBA":                             "#e6194b",
    "Friis":                            "#3cb44b",
    "TwoRayGroundPropagationLossModel": "#f58231",
    "ItuR1411LosPropagationLossModel":  "#4363d8",
}

MODEL_MARKERS = {
    "FOBA":                             "o",
    "Friis":                            "s",
    "TwoRayGroundPropagationLossModel": "D",
    "ItuR1411LosPropagationLossModel":  "^",
}

ALGORITHMS = ["aodv", "dsdv", "olsr"]

# (key, short_label, y_label, aodv_only)
METRICS = [
    ("pdr",       "PDR",                  "Packet Delivery Ratio",    False),
    ("eed",       "EED (s)",              "End-to-End Delay (s)",     False),
    ("goodput",   "Goodput (kbps)",       "Goodput (kbps)",           False),
    ("routeacq",  "Route Acq. Time (s)",  "Route Acq. Time (s)",      True),
    ("routingoh", "Routing Overhead",     "Route Signalization Packets Sent", False),
]

# Metrics used in the per-scenario traffic dashboards (Figs 1 & 2)
TRAFFIC_METRICS = [
    ("pdr",     "PDR",            "Packet Delivery Ratio", False),
    ("goodput", "Goodput (kbps)", "Goodput (kbps)",        False),
    ("eed",     "EED (s)",        "End-to-End Delay (s)",  False),
]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def compute_ci95(values):
    arr = np.array([v for v in values if v is not None and not np.isnan(v)], dtype=np.float64)
    n = len(arr)
    if n < 2:
        return np.nan
    s = np.std(arr, ddof=1)
    if student_t is not None:
        tval = float(student_t.ppf(0.975, n - 1))
    else:
        tval = 1.96
    return tval * s / np.sqrt(n)


# ---------------------------------------------------------------------------
# CSV extractors
# ---------------------------------------------------------------------------
def extract_pdr(path):
    try:
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            total, count = 0.0, 0
            for row in reader:
                tx = int(row["TxPackets"])
                rx = int(row["RxPackets"])
                if tx > 0:
                    total += rx / tx
                    count += 1
            return total / count if count > 0 else np.nan
    except Exception:
        return np.nan


def extract_eed(path):
    try:
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            total, count = 0.0, 0
            for row in reader:
                rx = int(row.get("RxPackets", 0))
                delay = float(row.get("sumDelay", 0))
                if rx > 0:
                    total += delay / rx
                    count += 1
            return total / count if count > 0 else np.nan
    except Exception:
        return np.nan


def extract_goodput(path):
    try:
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            total, count = 0.0, 0
            for row in reader:
                rx = int(row["RxPackets"])
                first_tx = float(row["FirstTxTime"])
                last_rx  = float(row["LastRxTime"])
                dur = last_rx - first_tx
                if dur <= 0 or rx <= 0:
                    continue
                gp = (rx * 1024 * 8) / dur / 1000  # kbps
                if gp > MAX_GOODPUT_KBPS:
                    continue
                total += gp
                count += 1
            return total / count if count > 0 else np.nan
    except Exception:
        return np.nan


def extract_route_acq(path):
    try:
        with open(path, newline='') as f:
            headers = [h.strip() for h in f.readline().strip().split(',')]
            reader = csv.DictReader(f, fieldnames=headers)
            times = []
            for row in reader:
                row = {k.strip(): v.strip() for k, v in row.items()}
                try:
                    if int(row["routeFound"]) == 1:
                        acq = float(row["TimeRREP"]) - float(row["TimeRREQ"])
                        if acq >= 0:
                            times.append(acq)
                except Exception:
                    continue
            return float(np.mean(times)) if times else np.nan
    except Exception:
        return np.nan


def extract_routing_overhead(path):
    """Return total RouteSignalizationPacketsSent (absolute count)."""
    try:
        with open(path, newline='') as f:
            headers = [h.strip() for h in f.readline().strip().split(',')]
            reader = csv.DictReader(f, fieldnames=headers)
            total = 0.0
            found = False
            for row in reader:
                row = {k.strip(): v.strip() for k, v in row.items()}
                try:
                    txsig = float(row.get("RouteSignalizationPacketsSent", 0))
                    total += txsig
                    found = True
                except Exception:
                    continue
            return total if found else np.nan
    except Exception:
        return np.nan


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def _discover_epochs(source):
    if not os.path.isdir(source):
        return []
    return sorted(
        d for d in os.listdir(source)
        if os.path.isdir(os.path.join(source, d)) and d.startswith("Epoch_")
    )


def _discover_num_nodes(source, epochs):
    node_set = set()
    for epoch in epochs:
        for lm in LOSS_MODELS:
            for algo in ALGORITHMS:
                nd = os.path.join(source, epoch, lm, algo, "numNodes")
                if os.path.isdir(nd):
                    for entry in os.listdir(nd):
                        if entry.isdigit():
                            node_set.add(int(entry))
    return sorted(node_set)


def load_scenario(source):
    """
    Load and aggregate routing metrics for all epochs in `source`.

    Returns:
        stats          dict[metric_key][lm][algo] = {"mean": list, "ci95": list}
                       where routeacq only has algo "aodv".
        num_nodes_list sorted list of discovered node counts.
    Returns (None, []) if source is missing or empty.
    """
    epochs = _discover_epochs(source)
    if not epochs:
        print(f"  [warn] No epoch folders in {source}")
        return None, []

    num_nodes_list = _discover_num_nodes(source, epochs)
    if not num_nodes_list:
        print(f"  [warn] No node-count folders in {source}")
        return None, []

    N = len(num_nodes_list)
    nidx = {n: i for i, n in enumerate(num_nodes_list)}

    # raw[metric][lm][algo][i] = list of per-epoch floats
    raw = {
        mk: {
            lm: {algo: [[] for _ in range(N)] for algo in ALGORITHMS}
            for lm in LOSS_MODELS
        }
        for mk in ("pdr", "eed", "goodput", "routingoh")
    }
    raw_routeacq = {lm: [[] for _ in range(N)] for lm in LOSS_MODELS}

    for epoch in epochs:
        for lm in LOSS_MODELS:
            for algo in ALGORITHMS:
                for n in num_nodes_list:
                    folder = os.path.join(source, epoch, lm, algo, "numNodes", str(n))
                    if not os.path.isdir(folder):
                        continue
                    i = nidx[n]

                    flow_csv = os.path.join(folder, "flow_information.csv")
                    if os.path.isfile(flow_csv):
                        raw["pdr"][lm][algo][i].append(extract_pdr(flow_csv))
                        raw["eed"][lm][algo][i].append(extract_eed(flow_csv))
                        raw["goodput"][lm][algo][i].append(extract_goodput(flow_csv))

                    ro_csv = os.path.join(folder, "Network_traffic_mapping.csv")
                    if os.path.isfile(ro_csv):
                        raw["routingoh"][lm][algo][i].append(
                            extract_routing_overhead(ro_csv)
                        )

                    if algo == "aodv":
                        rt_csv = os.path.join(folder, "Route_mapping.csv")
                        if os.path.isfile(rt_csv):
                            raw_routeacq[lm][i].append(extract_route_acq(rt_csv))

    def _agg_raw(raw_dict):
        out = {}
        for lm in LOSS_MODELS:
            out[lm] = {}
            for algo in ALGORITHMS:
                means, cis = [], []
                for i in range(N):
                    vals = [v for v in raw_dict[lm][algo][i]
                            if v is not None and not np.isnan(v)]
                    means.append(float(np.mean(vals)) if vals else np.nan)
                    cis.append(compute_ci95(vals))
                out[lm][algo] = {"mean": means, "ci95": cis}
        return out

    stats = {mk: _agg_raw(raw[mk]) for mk in ("pdr", "eed", "goodput", "routingoh")}

    # Route acquisition (AODV only); wrap in same structure for uniform access
    stats["routeacq"] = {}
    for lm in LOSS_MODELS:
        means, cis = [], []
        for i in range(N):
            vals = [v for v in raw_routeacq[lm][i]
                    if v is not None and not np.isnan(v)]
            means.append(float(np.mean(vals)) if vals else np.nan)
            cis.append(compute_ci95(vals))
        stats["routeacq"][lm] = {"aodv": {"mean": means, "ci95": cis}}

    return stats, num_nodes_list


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------
def _draw_panel(ax, x_arr, stats, metric_key, algo, log_scale=False):
    """Plot one metric/algorithm panel. Returns True if any data was drawn."""
    has_data = False
    for lm in LOSS_MODELS:
        try:
            s = stats[metric_key][lm][algo]
        except KeyError:
            continue
        mean = np.array(s["mean"], dtype=np.float64)
        ci   = np.array(s["ci95"],  dtype=np.float64)
        mask = ~np.isnan(mean)
        if log_scale:
            mask &= (mean > 0)   # log scale requires strictly positive values
        if not mask.any():
            continue
        has_data = True
        ci_safe = np.where(np.isnan(ci), 0.0, ci)
        ax.plot(x_arr[mask], mean[mask],
                color=MODEL_COLORS[lm], marker=MODEL_MARKERS[lm],
                linewidth=1.8, markersize=5, label=MODEL_LABELS[lm])
        if log_scale:
            # fill_between crosses zero on log axes — use error bars instead,
            # clipping the lower bound so it never goes below 1 % of the mean.
            lo = np.minimum(ci_safe[mask], mean[mask] * 0.99)
            ax.errorbar(
                x_arr[mask], mean[mask],
                yerr=[lo, ci_safe[mask]],
                fmt="none", color=MODEL_COLORS[lm], alpha=0.4,
                capsize=2, linewidth=0.8, capthick=0.8,
            )
        else:
            ax.fill_between(x_arr[mask],
                            mean[mask] - ci_safe[mask],
                            mean[mask] + ci_safe[mask],
                            color=MODEL_COLORS[lm], alpha=0.15)
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(labelsize=9)
    return has_data


def _format_xticks(ax, x_arr, show_xlabel=False):
    if show_xlabel:
        ax.set_xlabel("Num. nodes", fontsize=10, fontweight="bold")
    else:
        ax.set_xlabel("")
    ax.set_xticks(x_arr)
    ax.set_xticklabels([str(v) for v in x_arr], fontsize=8, rotation=45)


def _legend_handles():
    return [
        Line2D([0], [0],
               color=MODEL_COLORS[lm], marker=MODEL_MARKERS[lm],
               linewidth=1.8, markersize=7, label=MODEL_LABELS[lm])
        for lm in LOSS_MODELS
    ]


def _shared_ylim(stats_list, metric_key, algos, lower_cap=None, upper_cap=None, margin=0.05):
    """Return (y_min, y_max) covering mean ± ci95 for *metric_key* across all
    stats dicts in *stats_list* and all *algos*.  Returns None if no data."""
    vals = []
    for stats in stats_list:
        if stats is None:
            continue
        for lm in LOSS_MODELS:
            for algo in algos:
                try:
                    s = stats[metric_key][lm][algo]
                except KeyError:
                    continue
                mean = np.array(s["mean"], dtype=np.float64)
                ci   = np.array(s["ci95"],  dtype=np.float64)
                ci_safe = np.where(np.isnan(ci), 0.0, ci)
                valid = ~np.isnan(mean)
                if valid.any():
                    vals.extend((mean[valid] + ci_safe[valid]).tolist())
                    vals.extend((mean[valid] - ci_safe[valid]).tolist())
    if not vals:
        return None
    y_min = min(vals)
    y_max = max(vals)
    if lower_cap is not None:
        y_min = max(y_min, lower_cap)
    if upper_cap is not None:
        y_max = min(y_max, upper_cap)
    if y_min == y_max:
        y_min -= 0.5
        y_max += 0.5
    span = y_max - y_min
    return (y_min - span * margin, y_max + span * margin)


def _shared_log_ylim(stats_list, metric_key, algos):
    """Return (y_min, y_max) as rounded log-decade bounds covering all positive
    mean values for *metric_key* across every stats dict and algo.
    Returns None if no positive data found."""
    pos_vals = []
    for stats in stats_list:
        if stats is None:
            continue
        for lm in LOSS_MODELS:
            for algo in algos:
                try:
                    s = stats[metric_key][lm][algo]
                except KeyError:
                    continue
                mean = np.array(s["mean"], dtype=np.float64)
                pos = mean[~np.isnan(mean) & (mean > 0)]
                if pos.size:
                    pos_vals.extend(pos.tolist())
    if not pos_vals:
        return None
    return (
        10 ** (np.floor(np.log10(min(pos_vals)))),
        10 ** (np.ceil(np.log10(max(pos_vals))) + 0.1),
    )


# ---------------------------------------------------------------------------
# Figure 1 & 2 – Per-scenario traffic dashboard (PDR / Goodput / EED)
# ---------------------------------------------------------------------------
def build_dashboard(stats, num_nodes_list, scenario_label, fig_num):
    """3-row (PDR, Goodput, EED) × 3-col (algorithms). Legend embedded in top-right panel."""
    n_metrics = len(TRAFFIC_METRICS)
    n_algos   = len(ALGORITHMS)
    x_arr = np.array(num_nodes_list)

    # Shared y-limits per metric row (same scale across all algo columns)
    row_ylims = {}
    for mk, _sl, _yl, _ao in TRAFFIC_METRICS:
        if mk == "pdr":
            row_ylims[mk] = (0.0, 1.05)
        else:
            row_ylims[mk] = _shared_ylim([stats], mk, ALGORITHMS, lower_cap=0.0)

    fig = plt.figure(figsize=(5.5 * n_algos, 3.8 * n_metrics))
    fig.suptitle(
        f"Traffic Performance – {scenario_label}",
        fontsize=17, fontweight="bold", y=0.995,
    )

    gs = gridspec.GridSpec(
        n_metrics, n_algos,
        figure=fig,
        hspace=0.18, wspace=0.20,
        top=0.93, bottom=0.07, left=0.08, right=0.98,
    )

    for row, (mk, short_label, y_label, _ao) in enumerate(TRAFFIC_METRICS):
        for col, algo in enumerate(ALGORITHMS):
            ax = fig.add_subplot(gs[row, col])

            if row == 0:
                ax.set_title(algo.upper(), fontsize=13, fontweight="bold", pad=6)

            _draw_panel(ax, x_arr, stats, mk, algo)
            if col == 0:
                ax.set_ylabel(y_label, fontsize=11, fontweight="bold")
            ylim = row_ylims.get(mk)
            if ylim is not None:
                ax.set_ylim(ylim)
            if mk == "goodput":
                ax.axhline(y=81.92, color="grey", linestyle=":",
                           linewidth=1.0, alpha=0.6)
            _format_xticks(ax, x_arr, show_xlabel=(row == n_metrics - 1))

            # Embed legend in top-right panel
            if row == 0 and col == n_algos - 1:
                ax.legend(
                    handles=_legend_handles(),
                    title="Prop. model",
                    title_fontsize=10,
                    fontsize=9,
                    loc="best",
                    frameon=True,
                    framealpha=0.85,
                )

    os.makedirs(OUT_DIR, exist_ok=True)
    sc_tag = scenario_label.replace(" ", "_").replace("(", "").replace(")", "")
    fname = os.path.join(OUT_DIR, f"fig{fig_num}_traffic_{sc_tag}.png")
    fig.savefig(fname, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {fname}")


# ---------------------------------------------------------------------------
# Figure 3 – Routing Overhead: SA (row 0) vs SB (row 1), all algorithms
# ---------------------------------------------------------------------------
def build_routing_overhead_fig(stats_sa, nodes_sa, stats_sb, nodes_sb):
    """
    2 rows (SA, SB) × 3 cols (AODV, DSDV, OLSR).
    All 6 panels share the same Y axis. Legend embedded in top-right panel.
    """
    scenario_rows = [
        (stats_sa, nodes_sa, "Scenario A"),
        (stats_sb, nodes_sb, "Scenario B"),
    ]

    # Single shared log-scale y-limit across SA + SB, all algorithms
    ylim = _shared_log_ylim([stats_sa, stats_sb], "routingoh", ALGORITHMS)

    fig = plt.figure(figsize=(5.5 * len(ALGORITHMS), 3.8 * 2))
    fig.suptitle(
        "Routing Overhead – Scenario A vs Scenario B",
        fontsize=17, fontweight="bold", y=0.995,
    )

    gs = gridspec.GridSpec(
        2, len(ALGORITHMS),
        figure=fig,
        hspace=0.15, wspace=0.15,
        top=0.90, bottom=0.09, left=0.08, right=0.98,
    )

    for row, (stats, nodes, sc_label) in enumerate(scenario_rows):
        for col, algo in enumerate(ALGORITHMS):
            ax = fig.add_subplot(gs[row, col])

            # Column header on first row only
            if row == 0:
                ax.set_title(algo.upper(), fontsize=13, fontweight="bold", pad=6)

            # Row label on leftmost column
            if col == 0:
                ax.set_ylabel(f"{sc_label}\nRouting Overhead", fontsize=11, fontweight="bold")

            if stats is None or not nodes:
                ax.text(0.5, 0.5, "No data",
                        ha="center", va="center", transform=ax.transAxes,
                        fontsize=9, color="grey", style="italic")
                ax.set_xticks([])
                ax.set_yticks([])
            else:
                x_arr = np.array(nodes)
                _draw_panel(ax, x_arr, stats, "routingoh", algo, log_scale=True)
                ax.set_yscale("log")
                if ylim is not None:
                    ax.set_ylim(ylim)
                _format_xticks(ax, x_arr, show_xlabel=(row == 1))

            # Embed legend in top-right panel
            if row == 0 and col == len(ALGORITHMS) - 1:
                ax.legend(
                    handles=_legend_handles(),
                    title="Prop. model",
                    title_fontsize=10,
                    fontsize=9,
                    loc="best",
                    frameon=True,
                    framealpha=0.85,
                )

    os.makedirs(OUT_DIR, exist_ok=True)
    fname = os.path.join(OUT_DIR, "fig3_routing_overhead_SA_vs_SB.png")
    fig.savefig(fname, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {fname}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Loading SA data …")
    stats_sa, nodes_sa = load_scenario(SOURCE_SA)
    print(f"  SA node counts: {nodes_sa}")

    print("Loading SB data …")
    stats_sb, nodes_sb = load_scenario(SOURCE_SB)
    print(f"  SB node counts: {nodes_sb}")

    if stats_sa and nodes_sa:
        print("Building Figure 1 – SA Traffic Dashboard (PDR / Goodput / EED) …")
        build_dashboard(stats_sa, nodes_sa, "Scenario A", fig_num=1)
    else:
        print("[skip] No SA data – Figure 1 skipped")

    if stats_sb and nodes_sb:
        print("Building Figure 2 – SB Traffic Dashboard (PDR / Goodput / EED) …")
        build_dashboard(stats_sb, nodes_sb, "Scenario B", fig_num=2)
    else:
        print("[skip] No SB data – Figure 2 skipped")

    print("Building Figure 3 – Routing Overhead SA vs SB …")
    build_routing_overhead_fig(stats_sa, nodes_sa, stats_sb, nodes_sb)

    print(f"\nDone. All figures saved to: {os.path.abspath(OUT_DIR)}")

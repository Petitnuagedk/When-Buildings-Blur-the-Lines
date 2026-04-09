"""PlotDropData.py
Visualise how packet-drop reasons evolve with node density, revealing congestion.

Drop reasons are grouped into three families:
  - Congestion/Channel-Busy : dropRxing, dropTxing, dropBusyDecodingPreamble,
                              dropPreambleDetectionPacketSwitch
  - Signal/Propagation      : dropPreambleDetectFailure, dropLSigFailure,
                              dropReceptionAbortedByTx, and other *Failure variants
  - Other/MAC               : macTxDrop, macRxDrop, dropFiltered, dropUnknown, rest

Two plot families are produced per algorithm × loss-model:
  1. Stacked proportional bar chart  – share of each group per node count (one bar each).
     Congestion slice expanding toward the right demonstrates channel saturation.
  2. Absolute congestion-drops line chart – one line per loss model (mean ± 95 % CI)
     showing super-linear growth with node density.

Output: plots-drops/
"""

import argparse
import os
import csv
import numpy as np
import matplotlib.pyplot as plt

try:
    from scipy.stats import t as student_t
except ImportError:
    student_t = None

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Plot drop-reason evolution vs node count")
parser.add_argument("--epochs", "-e", type=int, default=None,
                    help="Number of epochs to use (first N sorted). Default: all detected.")
args = parser.parse_args()

# ---------------------------------------------------------------------------
# Paths — mirror PlotUrbanComp-v2.py convention
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "..", "SASB-data", "UrbanRaCompDir-v2"))
PLOTS_DIR = os.path.join(SCRIPT_DIR, "..", "plots-drops")
os.makedirs(PLOTS_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------
LOSS_MODELS = ["Friis", "TwoRayGroundPropagationLossModel", "ItuR1411LosPropagationLossModel", "FOBA"]
ALGORITHMS  = ["aodv", "olsr", "dsdv"]

# Discover epoch folders
BASE_DIRS = sorted(
    d for d in os.listdir(SOURCE)
    if os.path.isdir(os.path.join(SOURCE, d)) and d.startswith("Epoch_")
)
if args.epochs is not None:
    if args.epochs <= 0:
        raise SystemExit("--epochs must be a positive integer")
    BASE_DIRS = BASE_DIRS[: args.epochs]

# Discover node-count folders present in the dataset
num_nodes_set = set()
for base in BASE_DIRS:
    for lm in LOSS_MODELS:
        for algo in ALGORITHMS:
            nd = os.path.join(SOURCE, base, lm, algo, "numNodes")
            if os.path.isdir(nd):
                for entry in os.listdir(nd):
                    if entry.isdigit():
                        num_nodes_set.add(int(entry))
NUM_NODES = sorted(num_nodes_set)

# ---------------------------------------------------------------------------
# Drop-reason classification
# ---------------------------------------------------------------------------
CONGESTION_METRICS = {
    "dropRxing",
    "dropTxing",
    "dropBusyDecodingPreamble",
    "dropPreambleDetectionPacketSwitch",
    "dropFrameCapturePacketSwitch",
}

SIGNAL_METRICS = {
    "dropPreambleDetectFailure",
    "dropLSigFailure",
    "dropHtSigFailure",
    "dropSigAFailure",
    "dropSigBFailure",
    "dropUSigFailure",
    "dropEhtSigFailure",
    "dropDmgHeaderFailure",
    "dropReceptionAbortedByTx",
    "dropTruncatedTx",
    "dropObssPdCcaReset",
    "dropPpduTooLate",
}

# Everything else falls into OTHER (dropFiltered, dropUnknown, …)
# macTxDrop / macRxDrop are aggregate counters (macRxDrop == sum of all specific
# PhyRxDrop reasons), so they must be EXCLUDED to avoid double-counting.
AGGREGATE_COUNTERS = {"macTxDrop", "macRxDrop"}

GROUP_NAMES  = ["Congestion / Channel Busy", "Signal / Propagation", "Other / MAC"]
GROUP_COLORS = ["#d62728", "#1f77b4", "#9467bd"]


def classify_row(metric_name: str) -> str:
    if metric_name in AGGREGATE_COUNTERS:
        return "skip"
    if metric_name in CONGESTION_METRICS:
        return "congestion"
    if metric_name in SIGNAL_METRICS:
        return "signal"
    return "other"


# ---------------------------------------------------------------------------
# Reader
# ---------------------------------------------------------------------------
def read_drop_data(csv_path: str) -> dict[str, float]:
    """Return {metric_name: value} dict from a dropData.csv file."""
    result = {}
    try:
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                metric = row.get("Metric", "").strip()
                try:
                    value = float(row.get("Value", 0))
                except ValueError:
                    value = 0.0
                if metric:
                    result[metric] = value
    except Exception as e:
        print(f"  [WARN] Could not read {csv_path}: {e}")
    return result


def sum_groups(drop_dict: dict[str, float]) -> tuple[float, float, float]:
    """Return (congestion_sum, signal_sum, other_sum)."""
    cong = sig = other = 0.0
    for k, v in drop_dict.items():
        grp = classify_row(k)
        if grp == "skip":
            continue  # aggregate counter — don't count
        elif grp == "congestion":
            cong += v
        elif grp == "signal":
            sig += v
        else:
            other += v
    return cong, sig, other


def read_node_traffic_drops(csv_path: str) -> tuple[float, float]:
    """Return (total_PacketsDroppedNoRoute, total_PacketsDroppedRerr) summed over all nodes."""
    no_route = 0.0
    rerr = 0.0
    try:
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            # Header has spaces after commas — strip them
            if reader.fieldnames:
                reader.fieldnames = [h.strip() for h in reader.fieldnames]
            for row in reader:
                try:
                    no_route += float((row.get("PacketsDroppedNoRoute") or "0").strip())
                    rerr     += float((row.get("PacketsDroppedRerr")     or "0").strip())
                except (ValueError, TypeError):
                    pass
    except Exception as e:
        print(f"  [WARN] Could not read {csv_path}: {e}")
    return no_route, rerr


# ---------------------------------------------------------------------------
# CI helper
# ---------------------------------------------------------------------------
def ci95_halfwidth(values):
    arr = np.array(values, dtype=np.float64)
    arr = arr[~np.isnan(arr)]
    n = len(arr)
    if n < 2:
        return np.nan
    s = np.std(arr, ddof=1)
    tval = float(student_t.ppf(0.975, n - 1)) if student_t is not None else 1.96
    return tval * s / np.sqrt(n)


# ---------------------------------------------------------------------------
# Data collection
# Raw structure: raw[epoch][lm][algo][node_idx] = (cong, sig, other)
# ---------------------------------------------------------------------------
print("Collecting drop data …")
raw: dict = {}
for epoch in BASE_DIRS:
    raw[epoch] = {}
    for lm in LOSS_MODELS:
        raw[epoch][lm] = {}
        for algo in ALGORITHMS:
            raw[epoch][lm][algo] = []
            for nodes in NUM_NODES:
                folder = os.path.join(SOURCE, epoch, lm, algo, "numNodes", str(nodes))
                csv_path = os.path.join(folder, "dropData.csv")
                if os.path.isfile(csv_path):
                    d = read_drop_data(csv_path)
                    raw[epoch][lm][algo].append(sum_groups(d))
                else:
                    raw[epoch][lm][algo].append((np.nan, np.nan, np.nan))

# ---------------------------------------------------------------------------
# Average across epochs per (lm, algo, node_idx)
# avg_groups[lm][algo][node_idx] = (mean_cong, mean_sig, mean_other)
# ci95_cong[lm][algo][node_idx]  = ci95 half-width for congestion
# ---------------------------------------------------------------------------
avg_groups: dict = {lm: {algo: [] for algo in ALGORITHMS} for lm in LOSS_MODELS}
ci95_cong:  dict = {lm: {algo: [] for algo in ALGORITHMS} for lm in LOSS_MODELS}
ci95_sig:   dict = {lm: {algo: [] for algo in ALGORITHMS} for lm in LOSS_MODELS}

for lm in LOSS_MODELS:
    for algo in ALGORITHMS:
        for i in range(len(NUM_NODES)):
            cong_vals, sig_vals, other_vals = [], [], []
            for epoch in BASE_DIRS:
                c, s, o = raw[epoch][lm][algo][i]
                if not np.isnan(c):
                    cong_vals.append(c)
                    sig_vals.append(s)
                    other_vals.append(o)
            if cong_vals:
                avg_groups[lm][algo].append((
                    np.mean(cong_vals),
                    np.mean(sig_vals),
                    np.mean(other_vals),
                ))
                ci95_cong[lm][algo].append(ci95_halfwidth(cong_vals))
                ci95_sig[lm][algo].append(ci95_halfwidth(sig_vals))
            else:
                avg_groups[lm][algo].append((np.nan, np.nan, np.nan))
                ci95_cong[lm][algo].append(np.nan)
                ci95_sig[lm][algo].append(np.nan)

# ---------------------------------------------------------------------------
# Upstream / layer-bottleneck data
# raw_upstream[epoch][lm][algo][node_idx] = (macTxDrop, PacketsDroppedNoRoute, PacketsDroppedRerr)
# ---------------------------------------------------------------------------
print("Collecting upstream drop data (macTxDrop, IP No-Route, IP Route-Error) …")
raw_upstream: dict = {}
for epoch in BASE_DIRS:
    raw_upstream[epoch] = {}
    for lm in LOSS_MODELS:
        raw_upstream[epoch][lm] = {}
        for algo in ALGORITHMS:
            raw_upstream[epoch][lm][algo] = []
            for nodes in NUM_NODES:
                folder    = os.path.join(SOURCE, epoch, lm, algo, "numNodes", str(nodes))
                drop_csv  = os.path.join(folder, "dropData.csv")
                nt_csv    = os.path.join(folder, "node_traffic_mapping.csv")
                mac_tx    = 0.0
                if os.path.isfile(drop_csv):
                    d      = read_drop_data(drop_csv)
                    mac_tx = d.get("macTxDrop", 0.0)
                no_route, rerr = (np.nan, np.nan)
                if os.path.isfile(nt_csv):
                    no_route, rerr = read_node_traffic_drops(nt_csv)
                raw_upstream[epoch][lm][algo].append((mac_tx, no_route, rerr))

# Average upstream metrics across epochs
avg_upstream: dict = {lm: {algo: [] for algo in ALGORITHMS} for lm in LOSS_MODELS}
ci95_up: dict = {
    lm: {algo: {"macTxDrop": [], "noRoute": [], "rerr": []} for algo in ALGORITHMS}
    for lm in LOSS_MODELS
}

for lm in LOSS_MODELS:
    for algo in ALGORITHMS:
        for i in range(len(NUM_NODES)):
            mt_vals, nr_vals, re_vals = [], [], []
            for epoch in BASE_DIRS:
                mt, nr, re = raw_upstream[epoch][lm][algo][i]
                mt_vals.append(mt)
                if not np.isnan(nr):
                    nr_vals.append(nr)
                if not np.isnan(re):
                    re_vals.append(re)
            avg_upstream[lm][algo].append((
                np.nanmean(mt_vals) if mt_vals else np.nan,
                np.mean(nr_vals)    if nr_vals else np.nan,
                np.mean(re_vals)    if re_vals else np.nan,
            ))
            ci95_up[lm][algo]["macTxDrop"].append(ci95_halfwidth(mt_vals))
            ci95_up[lm][algo]["noRoute"].append(ci95_halfwidth(nr_vals) if nr_vals else np.nan)
            ci95_up[lm][algo]["rerr"].append(ci95_halfwidth(re_vals) if re_vals else np.nan)

# ---------------------------------------------------------------------------
# Per-epoch proportion statistics for stacked bar Q25/Q75 whiskers
# Boundaries tracked:
#   cb  = top of congestion segment       = p_cong
#   csb = top of congestion+signal segment = p_cong + p_sig
# ---------------------------------------------------------------------------
prop_mean_cb  = {lm: {algo: [] for algo in ALGORITHMS} for lm in LOSS_MODELS}
prop_q25_cb   = {lm: {algo: [] for algo in ALGORITHMS} for lm in LOSS_MODELS}
prop_q75_cb   = {lm: {algo: [] for algo in ALGORITHMS} for lm in LOSS_MODELS}
prop_mean_csb = {lm: {algo: [] for algo in ALGORITHMS} for lm in LOSS_MODELS}
prop_q25_csb  = {lm: {algo: [] for algo in ALGORITHMS} for lm in LOSS_MODELS}
prop_q75_csb  = {lm: {algo: [] for algo in ALGORITHMS} for lm in LOSS_MODELS}

for lm in LOSS_MODELS:
    for algo in ALGORITHMS:
        for i in range(len(NUM_NODES)):
            ep_p_cong, ep_p_cum = [], []
            for epoch in BASE_DIRS:
                c, s, o = raw[epoch][lm][algo][i]
                total = c + s + o
                if not np.isnan(c) and total > 0:
                    ep_p_cong.append(c / total)
                    ep_p_cum.append((c + s) / total)
            if ep_p_cong:
                arr_c  = np.array(ep_p_cong)
                arr_cs = np.array(ep_p_cum)
                prop_mean_cb[lm][algo].append(np.mean(arr_c))
                prop_q25_cb[lm][algo].append(np.percentile(arr_c, 25))
                prop_q75_cb[lm][algo].append(np.percentile(arr_c, 75))
                prop_mean_csb[lm][algo].append(np.mean(arr_cs))
                prop_q25_csb[lm][algo].append(np.percentile(arr_cs, 25))
                prop_q75_csb[lm][algo].append(np.percentile(arr_cs, 75))
            else:
                for d in [prop_mean_cb, prop_q25_cb, prop_q75_cb,
                          prop_mean_csb, prop_q25_csb, prop_q75_csb]:
                    d[lm][algo].append(np.nan)

print(f"  Epochs: {len(BASE_DIRS)}, Node counts: {NUM_NODES}")

# ---------------------------------------------------------------------------
# Plot 1 — Stacked proportional bar charts
# One figure per algorithm, x-axis = node count, bars split by group share.
# ---------------------------------------------------------------------------
print("Generating stacked proportion plots …")

x = np.arange(len(NUM_NODES))
bar_width = 0.18  # width per loss model (they share a node-count tick)

for algo in ALGORITHMS:
    fig, ax = plt.subplots(figsize=(13, 6))

    for lm_idx, lm in enumerate(LOSS_MODELS):
        offset = (lm_idx - len(LOSS_MODELS) / 2 + 0.5) * bar_width
        x_pos = x + offset

        # Use mean-of-proportions (computed per epoch) so bars and whiskers are consistent
        p_cong  = np.array(prop_mean_cb[lm][algo],  dtype=np.float64)
        p_cum   = np.array(prop_mean_csb[lm][algo], dtype=np.float64)
        p_sig   = p_cum - p_cong
        p_other = 1.0 - p_cum
        mask = ~np.isnan(p_cong)

        ax.bar(x_pos[mask], p_cong[mask],  bar_width,
               color=GROUP_COLORS[0], alpha=0.55 + 0.15 * lm_idx)
        ax.bar(x_pos[mask], p_sig[mask],   bar_width, bottom=p_cong[mask],
               color=GROUP_COLORS[1], alpha=0.55 + 0.15 * lm_idx)
        ax.bar(x_pos[mask], p_other[mask], bar_width, bottom=p_cum[mask],
               color=GROUP_COLORS[2], alpha=0.55 + 0.15 * lm_idx)

        # Q25 / Q75 whiskers at each segment boundary
        cb_q25  = np.array(prop_q25_cb[lm][algo],  dtype=np.float64)
        cb_q75  = np.array(prop_q75_cb[lm][algo],  dtype=np.float64)
        csb_q25 = np.array(prop_q25_csb[lm][algo], dtype=np.float64)
        csb_q75 = np.array(prop_q75_csb[lm][algo], dtype=np.float64)
        ax.errorbar(
            x_pos[mask], p_cong[mask],
            yerr=[np.maximum(0, p_cong[mask] - cb_q25[mask]),
                  np.maximum(0, cb_q75[mask] - p_cong[mask])],
            fmt='none', color='black', capsize=2, linewidth=0.8, capthick=0.8,
        )
        ax.errorbar(
            x_pos[mask], p_cum[mask],
            yerr=[np.maximum(0, p_cum[mask] - csb_q25[mask]),
                  np.maximum(0, csb_q75[mask] - p_cum[mask])],
            fmt='none', color='black', capsize=2, linewidth=0.8, capthick=0.8,
        )

    # Custom legend
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    legend_elements = [
        Patch(facecolor=GROUP_COLORS[0], label="Congestion / Channel Busy"),
        Patch(facecolor=GROUP_COLORS[1], label="Signal / Propagation"),
        Patch(facecolor=GROUP_COLORS[2], label="Other / MAC"),
        Line2D([0], [0], color='black', linewidth=0.8, marker='_', markersize=6,
               label="Q25 / Q75 (segment boundary, across epochs)"),
    ]
    ax.legend(handles=legend_elements, loc="upper left")

    ax.set_xticks(x)
    ax.set_xticklabels(NUM_NODES)
    ax.set_xlabel("Number of Nodes")
    ax.set_ylabel("Share of Total Drops")
    ax.set_title(f"Drop-Reason Composition vs. Node Density — {algo.upper()}\n"
                 f"(stacked = normalised; congestion share grows with density)")
    ax.set_ylim(0, 1)
    ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda y, _: f"{y:.0%}"))
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    plt.tight_layout()
    out = os.path.join(PLOTS_DIR, f"drop_proportion_stacked_{algo}.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Saved {out}")

# ---------------------------------------------------------------------------
# Plot 2 — Absolute congestion drops, one line per loss model (mean ± CI 95%)
# One figure per algorithm.
# ---------------------------------------------------------------------------
print("Generating absolute congestion-drop line plots …")

for algo in ALGORITHMS:
    fig, ax = plt.subplots(figsize=(11, 6))
    for lm in LOSS_MODELS:
        y = np.array([avg_groups[lm][algo][i][0] for i in range(len(NUM_NODES))], dtype=np.float64)
        yerr = np.array(ci95_cong[lm][algo], dtype=np.float64)
        mask = ~np.isnan(y)
        line, = ax.plot(np.array(NUM_NODES)[mask], y[mask],
                        marker="o", markersize=6, label=lm)
        valid_err = yerr[mask]
        if not np.all(np.isnan(valid_err)):
            ax.fill_between(
                np.array(NUM_NODES)[mask],
                y[mask] - np.nan_to_num(valid_err),
                y[mask] + np.nan_to_num(valid_err),
                color=line.get_color(), alpha=0.2,
            )
    ax.set_xlabel("Number of Nodes")
    ax.set_ylabel("Dropped Packets (Congestion Group)")
    ax.set_title(f"Congestion-Related Drops vs. Node Density — {algo.upper()}\n"
                 f"(mean ± 95 % CI across epochs; includes: dropRxing, dropTxing,\n"
                 f" dropBusyDecodingPreamble, dropPreambleDetectionPacketSwitch)")
    ax.legend()
    ax.grid(False)
    plt.tight_layout()
    out = os.path.join(PLOTS_DIR, f"congestion_drops_abs_{algo}.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Saved {out}")

# ---------------------------------------------------------------------------
# Plot 3 — Congestion-drop share vs node count (line chart, no stacking)
# Cleaner alternative to the bars when comparing across loss models.
# ---------------------------------------------------------------------------
print("Generating congestion-share line plots …")

for algo in ALGORITHMS:
    fig, ax = plt.subplots(figsize=(11, 6))
    for lm in LOSS_MODELS:
        shares = []
        for i in range(len(NUM_NODES)):
            c, s, o = avg_groups[lm][algo][i]
            total = c + s + o
            shares.append(c / total if (not np.isnan(c) and total > 0) else np.nan)
        y = np.array(shares, dtype=np.float64)
        mask = ~np.isnan(y)
        ax.plot(np.array(NUM_NODES)[mask], y[mask] * 100,
                marker="o", markersize=6, label=lm)
    ax.set_xlabel("Number of Nodes")
    ax.set_ylabel("Congestion Drops (% of total drops)")
    ax.set_title(f"Congestion Share of All Drops vs. Node Density — {algo.upper()}\n"
                 f"(rising share = emerging channel saturation)")
    ax.set_ylim(0, 100)
    ax.legend()
    ax.grid(False)
    plt.tight_layout()
    out = os.path.join(PLOTS_DIR, f"congestion_share_pct_{algo}.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Saved {out}")

# ---------------------------------------------------------------------------
# Plot 4 — Full drop-bottleneck picture per algorithm
# One figure per algorithm, 2×2 subplots (one per loss model).
# Shows all five drop categories so AODV/DSDV PHY plateau can be understood:
# as density rises, MAC TX queue and IP routing-layer absorb traffic upstream.
# Y axis is shared across ALL algorithms so figures are directly comparable.
# ---------------------------------------------------------------------------
print("Generating full bottleneck drop plots …")

# Pre-compute global Y bounds across every algo × lm × series so all figures
# share the same log-scale axis range.
_all_pos_vals: list[float] = []
for _algo in ALGORITHMS:
    for _lm in LOSS_MODELS:
        for _i in range(len(NUM_NODES)):
            for _v in [
                avg_groups[_lm][_algo][_i][0],   # cong
                avg_groups[_lm][_algo][_i][1],   # sig
                avg_upstream[_lm][_algo][_i][0], # macTxDrop
                avg_upstream[_lm][_algo][_i][1], # no_route
                avg_upstream[_lm][_algo][_i][2], # rerr
            ]:
                if not (np.isnan(_v) or _v <= 0):
                    _all_pos_vals.append(_v)

if _all_pos_vals:
    _global_ymin = 10 ** (np.floor(np.log10(min(_all_pos_vals))) - 0.1)
    _global_ymax = 10 ** (np.ceil(np.log10(max(_all_pos_vals)))  + 0.1)
else:
    _global_ymin, _global_ymax = 1, 1e7

# (label, color, marker, linestyle)
BOTTLENECK_SERIES = [
    ("PHY Congestion",  "#d62728", "o",  "-"),
    ("PHY Signal",      "#1f77b4", "s",  "-"),
    ("MAC TX Drop",     "#ff7f0e", "^",  "--"),
    ("IP No-Route",     "#2ca02c", "D",  "-."),
    ("IP Route Error",  "#9467bd", "v",  ":"),
]

for algo in ALGORITHMS:
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), sharey=False)
    axes = axes.flatten()

    for lm_idx, lm in enumerate(LOSS_MODELS):
        ax = axes[lm_idx]
        x_nodes = np.array(NUM_NODES)

        y_cong = np.array([avg_groups[lm][algo][i][0]    for i in range(len(NUM_NODES))], dtype=np.float64)
        y_sig  = np.array([avg_groups[lm][algo][i][1]    for i in range(len(NUM_NODES))], dtype=np.float64)
        y_mact = np.array([avg_upstream[lm][algo][i][0]  for i in range(len(NUM_NODES))], dtype=np.float64)
        y_nr   = np.array([avg_upstream[lm][algo][i][1]  for i in range(len(NUM_NODES))], dtype=np.float64)
        y_re   = np.array([avg_upstream[lm][algo][i][2]  for i in range(len(NUM_NODES))], dtype=np.float64)
        e_cong = np.array(ci95_cong[lm][algo],              dtype=np.float64)
        e_sig  = np.array(ci95_sig[lm][algo],               dtype=np.float64)
        e_mact = np.array(ci95_up[lm][algo]["macTxDrop"],   dtype=np.float64)
        e_nr   = np.array(ci95_up[lm][algo]["noRoute"],     dtype=np.float64)
        e_re   = np.array(ci95_up[lm][algo]["rerr"],        dtype=np.float64)

        datasets = [
            (y_cong, e_cong, "PHY Congestion",  "#d62728", "o",  "-"),
            (y_sig,  e_sig,  "PHY Signal",      "#1f77b4", "s",  "-"),
            (y_mact, e_mact, "MAC TX Drop",     "#ff7f0e", "^",  "--"),
            (y_nr,   e_nr,   "IP No-Route",     "#2ca02c", "D",  "-."),
            (y_re,   e_re,   "IP Route Error",  "#9467bd", "v",  ":"),
        ]

        for y, yerr, label, color, marker, ls in datasets:
            # Mask NaN and true zeros (zeros are undefined on log scale)
            mask = ~np.isnan(y) & (y > 0)
            if not mask.any():
                continue
            line, = ax.plot(
                x_nodes[mask], y[mask],
                marker=marker, markersize=5, linestyle=ls, color=color, label=label,
            )
            # Use asymmetric error bars clipped to stay positive (required for log scale)
            valid_err = np.where(np.isnan(yerr), 0.0, yerr)[mask]
            if np.any(valid_err > 0):
                lo = np.minimum(valid_err, y[mask] * 0.99)  # never go below 1% of value
                ax.errorbar(
                    x_nodes[mask], y[mask],
                    yerr=[lo, valid_err],
                    fmt="none", color=color, alpha=0.5, capsize=2,
                    linewidth=0.8, capthick=0.8,
                )

        ax.set_yscale("log")
        ax.set_ylim(_global_ymin, _global_ymax)
        ax.set_title(lm, fontsize=9)
        ax.set_xlabel("Number of Nodes", fontsize=8)
        ax.set_ylabel("Dropped Packets — log scale (mean ± 95 % CI)", fontsize=8)
        ax.legend(fontsize=7, loc="upper left")
        ax.grid(axis="y", linestyle="--", alpha=0.3)

    fig.suptitle(
        f"Full Drop-Bottleneck Picture — {algo.upper()}\n"
        f"PHY Congestion plateau = channel saturated upstream by MAC TX queue (orange) "
        f"or IP routing layer (green/purple)",
        fontsize=11,
    )
    plt.tight_layout()
    out = os.path.join(PLOTS_DIR, f"full_bottleneck_{algo}.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Saved {out}")

print("Done. All plots saved to:", os.path.abspath(PLOTS_DIR))

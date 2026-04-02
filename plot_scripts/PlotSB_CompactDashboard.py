"""
PlotSB_CompactDashboard.py
--------------------------
Compact multi-panel visualisation of all results-con-SB-*-output data.

Data is read from four per-model output directories:
  results-con-SB-foba-output/     conn_aodv_FOBA_e1_n{N}_pair_metrics.csv
  results-con-SB-friis-output/    conn_aodv_Friis_e1_n{N}_pair_metrics.csv
  results-con-SB-itu-output/      conn_aodv_ItuR1411LosPropagationLossModel_e1_n{N}_pair_metrics.csv
  results-con-SB-2rg-output/      conn_aodv_TwoRayGroundPropagationLossModel_e1_n{N}_pair_metrics.csv

Three figures are produced and saved to plots-SB-dashboard/:

  Figure 1 – Line plots (metric vs num_nodes, one line per propagation model).
              Metric sub-panels + 1 legend panel in a grid.
              Shaded band = interquartile range across node pairs.

  Figure 2 – Model comparison heatmaps.
              One heatmap per propagation model (4 side-by-side), rows = metrics,
              columns = num_nodes.  Values are row-normalised (0–1) so every
              metric is on the same colour scale for quick visual ranking.

  Figure 3 – Radar / spider chart: model fingerprint at selected node counts.

Run from the repository root:
    python plot_scripts/PlotSB_CompactDashboard.py
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR  = os.path.join(BASE_DIR, "..", "plots-SB-dashboard")

# Map: model_key -> (friendly label, per-model data folder, filename prefix)
MODELS = {
    "FOBA":                               "FOBA",
    "Friis":                              "Friis",
    "ItuR1411LosPropagationLossModel":    "ITU-R 1411",
    "TwoRayGroundPropagationLossModel":   "Two-Ray",
}

MODEL_DATA_DIRS = {
    "FOBA":                               os.path.join(BASE_DIR, "..", "results-con-SB-Foba-output"),
    "Friis":                              os.path.join(BASE_DIR, "..", "results-con-SB-Frii-output"),
    "ItuR1411LosPropagationLossModel":    os.path.join(BASE_DIR, "..", "results-con-SB-ItuR-output"),
    "TwoRayGroundPropagationLossModel":   os.path.join(BASE_DIR, "..", "results-con-SB-TwoR-output"),
}

NUM_NODES_LIST = [50, 100, 150, 200, 300, 500]

# Metrics displayed (column name → friendly label)
METRICS = {
    "lifetime":                 "Lifetime",
    "stability":                "Stability",
    #"switches_rate":            "Switches Rate",
    #"avail_to_unavail_rate":    "Avail→Unavail Rate",
    "persistency_exact":        "Persistency (Exact)",
    "persistency_jaccard":      "Persistency (Jaccard)",
    "avg_path_length_norm":     "Avg Path Length (norm)",
    #"avg_inverse_path_length":  "Avg Inverse Path Length",
    "avg_path_length":          "Avg Path Length",
}

MODEL_COLORS = {
    "FOBA":                             "#e6194b",
    "Friis":                            "#3cb44b",
    "ItuR1411LosPropagationLossModel":  "#4363d8",
    "TwoRayGroundPropagationLossModel": "#f58231",
}

MODEL_MARKERS = {
    "FOBA":                             "o",
    "Friis":                            "s",
    "ItuR1411LosPropagationLossModel":  "^",
    "TwoRayGroundPropagationLossModel": "D",
}

# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def load_data() -> dict:
    """Return data[model_key][num_nodes] = DataFrame of reachable pairs."""
    data: dict = {m: {} for m in MODELS}
    for model_key in MODELS:
        data_dir = MODEL_DATA_DIRS[model_key]
        for n in NUM_NODES_LIST:
            path = os.path.join(data_dir, f"conn_aodv_{model_key}_e1_n{n}_pair_metrics.csv")
            if os.path.isfile(path):
                df = pd.read_csv(path)
                # Drop pairs that were never reachable (lifetime == 0)
                df = df[df["lifetime"] > 0].copy()
                data[model_key][n] = df
            else:
                print(f"  [warn] missing: {path}")
    return data


def compute_stats(data: dict) -> dict:
    """
    Returns stats[model_key][metric] = {
        "mean": [...], "q25": [...], "q75": [...]
    }
    one value per entry in NUM_NODES_LIST.
    """
    stats: dict = {m: {metric: {"mean": [], "q25": [], "q75": []}
                       for metric in METRICS}
                   for m in MODELS}
    for model_key in MODELS:
        for metric in METRICS:
            for n in NUM_NODES_LIST:
                df = data[model_key].get(n)
                if df is not None and metric in df.columns and len(df) > 0:
                    vals = df[metric].dropna().values
                    stats[model_key][metric]["mean"].append(float(np.mean(vals)))
                    stats[model_key][metric]["q25"].append(float(np.percentile(vals, 25)))
                    stats[model_key][metric]["q75"].append(float(np.percentile(vals, 75)))
                else:
                    stats[model_key][metric]["mean"].append(np.nan)
                    stats[model_key][metric]["q25"].append(np.nan)
                    stats[model_key][metric]["q75"].append(np.nan)
    return stats


def build_mean_array(stats: dict, model_key: str) -> np.ndarray:
    """(n_metrics × n_nodes) array of mean values for one model."""
    metric_keys = list(METRICS.keys())
    return np.array([stats[model_key][m]["mean"] for m in metric_keys],
                    dtype=float)


def row_normalise(arr: np.ndarray) -> np.ndarray:
    """Normalise each row to [0, 1] for cross-metric heatmap comparison."""
    out = np.full_like(arr, np.nan)
    for i in range(arr.shape[0]):
        row = arr[i]
        valid = row[~np.isnan(row)]
        if len(valid) < 2:
            continue
        lo, hi = valid.min(), valid.max()
        if hi > lo:
            out[i] = (row - lo) / (hi - lo)
        else:
            out[i] = np.where(np.isnan(row), np.nan, 0.5)
    return out

# ---------------------------------------------------------------------------
# Figure 1 – Line plots (all metrics, all models)
# ---------------------------------------------------------------------------
def plot_line_dashboard(stats: dict, out_dir: str) -> None:
    metric_keys   = list(METRICS.keys())
    metric_labels = list(METRICS.values())

    n_metrics = len(metric_keys)
    n_cols    = 2
    n_rows    = -(-( n_metrics + 1) // n_cols)   # ceil division, +1 for legend panel

    fig = plt.figure(figsize=(18, 3.5 * n_rows))
    fig.suptitle("Propagation Model Comparison – All Metrics vs Node Count (Scenario B)",
                 fontsize=15, fontweight="bold", y=0.98)

    gs = gridspec.GridSpec(n_rows, n_cols, figure=fig,
                           hspace=0.55, wspace=0.30,
                           top=0.93, bottom=0.06, left=0.07, right=0.97)

    axes = [fig.add_subplot(gs[r, c])
            for r in range(n_rows) for c in range(n_cols)
            if r * n_cols + c < n_metrics + 1]
    x = np.array(NUM_NODES_LIST)

    for idx, (metric_key, metric_label) in enumerate(zip(metric_keys, metric_labels)):
        ax = axes[idx]
        for model_key, model_label in MODELS.items():
            color  = MODEL_COLORS[model_key]
            marker = MODEL_MARKERS[model_key]
            s      = stats[model_key][metric_key]
            mean   = np.array(s["mean"])
            q25    = np.array(s["q25"])
            q75    = np.array(s["q75"])

            ax.plot(x, mean, color=color, marker=marker,
                    linewidth=1.8, markersize=5, label=model_label)
            ax.fill_between(x, q25, q75, color=color, alpha=0.12)

        ax.set_title(metric_label, fontsize=10, fontweight="bold", pad=4)
        ax.set_xlabel("Number of nodes", fontsize=8)
        ax.set_xticks(x)
        ax.set_xticklabels(x, fontsize=7)
        ax.tick_params(axis="y", labelsize=7)
        ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    # Last panel → legend
    ax_leg = axes[-1]
    ax_leg.axis("off")
    legend_handles = [
        Line2D([0], [0],
               color=MODEL_COLORS[mk], marker=MODEL_MARKERS[mk],
               linewidth=1.8, markersize=7, label=ml)
        for mk, ml in MODELS.items()
    ]
    ax_leg.legend(handles=legend_handles,
                  title="Propagation model",
                  title_fontsize=10,
                  fontsize=10,
                  loc="center",
                  frameon=True,
                  framealpha=0.9)
    note = ("Solid line = mean over all reachable pairs\n"
            "Shaded band = interquartile range (Q25–Q75)")
    ax_leg.text(0.5, 0.15, note,
                transform=ax_leg.transAxes,
                ha="center", va="center",
                fontsize=8, color="#555555",
                bbox=dict(boxstyle="round,pad=0.4", fc="#f5f5f5", ec="#cccccc"))

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "fig1_line_dashboard.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"Saved → {path}")
    return fig


# ---------------------------------------------------------------------------
# Figure 2 – Per-model normalised heatmaps
# ---------------------------------------------------------------------------
def plot_heatmap_comparison(stats: dict, out_dir: str) -> None:
    metric_labels = list(METRICS.values())
    node_labels   = [str(n) for n in NUM_NODES_LIST]
    model_keys    = list(MODELS.keys())
    model_labels  = list(MODELS.values())

    fig, axes = plt.subplots(1, 4, figsize=(20, 5.5),
                             gridspec_kw={"wspace": 0.30})
    fig.suptitle(
        "Metric Overview by Propagation Model  (row-normalised, 0 = worst · 1 = best)  –  Scenario B",
        fontsize=13, fontweight="bold", y=1.01
    )

    for col_idx, (model_key, model_label) in enumerate(zip(model_keys, model_labels)):
        ax       = axes[col_idx]
        arr      = build_mean_array(stats, model_key)
        arr_norm = row_normalise(arr)

        im = ax.imshow(arr_norm, aspect="auto", cmap="RdYlGn",
                       vmin=0, vmax=1, interpolation="nearest")

        # Annotate cells with raw mean values
        for r in range(arr.shape[0]):
            for c in range(arr.shape[1]):
                raw = arr[r, c]
                if not np.isnan(raw):
                    txt = f"{raw:.2f}"
                    brightness = arr_norm[r, c] if not np.isnan(arr_norm[r, c]) else 0.5
                    text_color = "black" if 0.25 < brightness < 0.85 else "white"
                    ax.text(c, r, txt, ha="center", va="center",
                            fontsize=6.5, color=text_color)

        ax.set_xticks(range(len(node_labels)))
        ax.set_xticklabels(node_labels, fontsize=8)
        ax.set_xlabel("Number of nodes", fontsize=9)

        if col_idx == 0:
            ax.set_yticks(range(len(metric_labels)))
            ax.set_yticklabels(metric_labels, fontsize=8)
        else:
            ax.set_yticks(range(len(metric_labels)))
            ax.set_yticklabels([], fontsize=8)

        ax.set_title(model_label, fontsize=11, fontweight="bold", pad=6)

        # Thin grid between cells
        ax.set_xticks(np.arange(-0.5, len(node_labels), 1), minor=True)
        ax.set_yticks(np.arange(-0.5, len(metric_labels), 1), minor=True)
        ax.grid(which="minor", color="white", linewidth=0.8)
        ax.tick_params(which="minor", bottom=False, left=False)

    # Shared colour bar
    cbar = fig.colorbar(im, ax=axes, orientation="vertical",
                        fraction=0.012, pad=0.01,
                        label="Row-normalised mean (0 = min, 1 = max)")
    cbar.ax.tick_params(labelsize=8)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "fig2_heatmap_comparison.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"Saved → {path}")
    return fig


# ---------------------------------------------------------------------------
# Figure 3 – Radar / spider chart: model fingerprint at selected node counts
# ---------------------------------------------------------------------------
def plot_radar(stats: dict, out_dir: str,
               node_counts: list = None) -> None:
    """Spider chart: one subplot per node count, 4 model lines each."""
    if node_counts is None:
        node_counts = [100, 200, 500]

    metric_keys   = list(METRICS.keys())
    metric_labels = list(METRICS.values())
    n_metrics     = len(metric_keys)
    angles        = np.linspace(0, 2 * np.pi, n_metrics, endpoint=False).tolist()
    angles        += angles[:1]   # close the polygon

    fig, axes = plt.subplots(1, len(node_counts),
                             figsize=(6 * len(node_counts), 6),
                             subplot_kw={"polar": True})
    if len(node_counts) == 1:
        axes = [axes]

    fig.suptitle("Model 'Fingerprint' Radar – mean metric values (globally normalised)  –  Scenario B",
                 fontsize=13, fontweight="bold", y=1.02)

    # Pre-compute global per-metric min/max for consistent normalisation
    global_min = {}
    global_max = {}
    for metric in metric_keys:
        all_vals = []
        for mk in MODELS:
            all_vals.extend(stats[mk][metric]["mean"])
        valid = [v for v in all_vals if not np.isnan(v)]
        global_min[metric] = min(valid) if valid else 0
        global_max[metric] = max(valid) if valid else 1

    for ax_idx, n in enumerate(node_counts):
        ax    = axes[ax_idx]
        n_idx = NUM_NODES_LIST.index(n) if n in NUM_NODES_LIST else None

        for model_key, model_label in MODELS.items():
            if n_idx is None:
                continue
            raw = [stats[model_key][m]["mean"][n_idx] for m in metric_keys]
            # Normalise using global range
            normed = []
            for i, metric in enumerate(metric_keys):
                lo, hi = global_min[metric], global_max[metric]
                v = raw[i]
                normed.append((v - lo) / (hi - lo) if (hi > lo and not np.isnan(v)) else 0.5)
            normed += normed[:1]

            ax.plot(angles, normed,
                    color=MODEL_COLORS[model_key],
                    marker=MODEL_MARKERS[model_key],
                    linewidth=1.8, markersize=5, label=model_label)
            ax.fill(angles, normed, color=MODEL_COLORS[model_key], alpha=0.08)

        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(metric_labels, size=8)
        ax.set_yticks([0.25, 0.5, 0.75, 1.0])
        ax.set_yticklabels(["0.25", "0.5", "0.75", "1.0"], size=7, color="grey")
        ax.set_ylim(0, 1)
        ax.set_title(f"N = {n}", size=11, fontweight="bold", pad=14)
        ax.grid(color="grey", linestyle="--", linewidth=0.5, alpha=0.7)

    # Single shared legend
    legend_handles = [
        Line2D([0], [0],
               color=MODEL_COLORS[mk], marker=MODEL_MARKERS[mk],
               linewidth=1.8, markersize=7, label=ml)
        for mk, ml in MODELS.items()
    ]
    fig.legend(handles=legend_handles,
               title="Propagation model",
               loc="lower center",
               ncol=4,
               fontsize=9,
               title_fontsize=9,
               frameon=True,
               bbox_to_anchor=(0.5, -0.04))

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "fig3_radar.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"Saved → {path}")
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Loading data …")
    data  = load_data()
    stats = compute_stats(data)

    print("Generating Figure 1 – line dashboard …")
    fig1 = plot_line_dashboard(stats, OUT_DIR)

    print("Generating Figure 2 – heatmap comparison …")
    fig2 = plot_heatmap_comparison(stats, OUT_DIR)

    print("Generating Figure 3 – radar chart …")
    fig3 = plot_radar(stats, OUT_DIR, node_counts=[100, 200, 500])

    print("\nAll figures saved to:", OUT_DIR)
    #plt.show()

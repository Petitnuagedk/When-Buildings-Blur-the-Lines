"""
Plot FOBA vs Sionna received-power comparison.

Reads loss_comparison.csv and produces figures showing multi-run summaries
using the median together with q25/q75 bands.

Usage:
    python plot_scripts/PlotFobaSionnaComparison.py [--input DIR] [--output DIR]
"""

import argparse
import os

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np


MODEL_SPECS = {
    "foba": {
        "column": "foba_rxPow_dBm",
        "label": "FOBA",
        "color": "#1f77b4",
        "marker": "o",
    },
    "friis": {
        "column": "friis_rxPow_dBm",
        "label": "Friis",
        "color": "#ff7f0e",
        "marker": "^",
    },
    "itur": {
        "column": "itur_rxPow_dBm",
        "label": "ITU-R 1411",
        "color": "#2ca02c",
        "marker": "D",
    },
    "two_ray": {
        "column": "two_ray_rxPow_dBm",
        "label": "Two-Ray",
        "color": "#9467bd",
        "marker": "v",
    },
    "sionna": {
        "column": "sionna_rxPow_dBm",
        "label": "Sionna RT",
        "color": "#d62728",
        "marker": "s",
    },
}


SCENARIO_GEOMETRY = {
    "scenario1": {
        "tx": (50.0, 50.0),
        "buildings": [
            (40.0, 60.0, -10.0, 10.0),
        ],
    },
    "scenario2": {
        "tx": (50.0, 40.0),
        "buildings": [
            (45.0, 55.0, -5.0, 5.0),
            (-5.0, 5.0, -5.0, 5.0),
            (95.0, 105.0, -5.0, 5.0),
        ],
    },
    "scenario3": {
        "tx": (0.0, 0.0),
        "buildings": [
            (x_center - 10.0, x_center + 10.0, y_center - 10.0, y_center + 10.0)
            for x_center in range(-100, 101, 50)
            for y_center in range(-100, 101, 50)
            if not (x_center == 0 and y_center == 0)
        ],
    },
    "scenario4": {
        "tx": (50.0, 50.0),
        "buildings": [
            (0.0, 100.0, -10.0, 10.0),
        ],
    },
}


def summarize_by_position(rx_x, values):
    unique_x = np.unique(rx_x)
    q25 = np.empty_like(unique_x, dtype=float)
    median = np.empty_like(unique_x, dtype=float)
    q75 = np.empty_like(unique_x, dtype=float)

    for idx, pos_x in enumerate(unique_x):
        samples = values[np.isclose(rx_x, pos_x, rtol=0.0, atol=1e-9)]
        samples = samples[~np.isnan(samples)]
        if samples.size == 0:
            q25[idx] = np.nan
            median[idx] = np.nan
            q75[idx] = np.nan
            continue
        q25[idx], median[idx], q75[idx] = np.percentile(samples, [25, 50, 75])

    return unique_x, q25, median, q75


def summarize_by_axis(axis_values, values):
    unique_axis = np.unique(axis_values)
    q25 = np.empty_like(unique_axis, dtype=float)
    median = np.empty_like(unique_axis, dtype=float)
    q75 = np.empty_like(unique_axis, dtype=float)

    for idx, axis_value in enumerate(unique_axis):
        samples = values[np.isclose(axis_values, axis_value, rtol=0.0, atol=1e-9)]
        samples = samples[~np.isnan(samples)]
        if samples.size == 0:
            q25[idx] = np.nan
            median[idx] = np.nan
            q75[idx] = np.nan
            continue
        q25[idx], median[idx], q75[idx] = np.percentile(samples, [25, 50, 75])

    return unique_axis, q25, median, q75


def normalize_scenarios(data):
    if "scenario" not in data.dtype.names:
        return np.full(data.shape, "scenario1", dtype=object)

    scenarios = np.asarray(data["scenario"], dtype=str)
    scenarios = np.char.strip(scenarios)
    lower = np.char.lower(scenarios)

    invalid_mask = (lower == "") | (lower == "nan") | (lower == "none")
    unknown_mask = ~np.isin(scenarios, list(SCENARIO_GEOMETRY.keys()))
    scenarios[invalid_mask | unknown_mask] = "scenario1"
    return scenarios


def get_model_values(data, model_key):
    column = MODEL_SPECS[model_key]["column"]
    if column not in data.dtype.names:
        return np.full(data.shape, np.nan, dtype=float)

    values = np.asarray(data[column], dtype=float)
    return np.where(values <= -998, np.nan, values)


def main():
    parser = argparse.ArgumentParser(description="Plot FOBA vs Sionna loss comparison")
    parser.add_argument("--input", default="scratch/foba-sionna-comparison",
                        help="Directory containing loss_comparison.csv")
    parser.add_argument("--output", default="plots/comparison",
                        help="Directory for output plots")
    args = parser.parse_args()

    csv_path = os.path.join(args.input, r"C:\Users\hugol\Documents\Firenze\B\j1\SASB-data\fosio\foba-sionna-comparison\loss_comparison.csv")
    if not os.path.isfile(csv_path):
        print(f"ERROR: {csv_path} not found. Run FobaSionnaComparison first.")
        return

    os.makedirs(args.output, exist_ok=True)

    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding=None, ndmin=1)
    scenarios = normalize_scenarios(data)
    runs = data["run"] if "run" in data.dtype.names else np.ones_like(data["rx_x"])
    time_values = data["time"] if "time" in data.dtype.names else np.arange(len(data["rx_x"]))
    rx_x = data["rx_x"]
    rx_y = data["rx_y"]
    model_values = {
        model_key: get_model_values(data, model_key)
        for model_key in MODEL_SPECS
    }

    scenario_names = np.unique(scenarios)
    for scenario_name in scenario_names:
        scenario_mask = scenarios == scenario_name
        if not np.any(scenario_mask):
            continue
        scenario_runs = runs[scenario_mask]
        scenario_time = time_values[scenario_mask]
        scenario_rx_x = rx_x[scenario_mask]
        scenario_rx_y = rx_y[scenario_mask]
        scenario_model_values = {
            model_key: values[scenario_mask]
            for model_key, values in model_values.items()
        }

        if scenario_name == "scenario3":
            plot_axis = scenario_time
            plot_axis_label = "Time (s)"
            plot_axis_suffix = "vs_time"
            shade_buildings = False
        else:
            plot_axis = scenario_rx_x
            plot_axis_label = "RX x-position (m)"
            plot_axis_suffix = "vs_position"
            shade_buildings = True

        model_summaries = {}
        for model_key, values in scenario_model_values.items():
            model_summaries[model_key] = summarize_by_axis(plot_axis, values)

        geometry = SCENARIO_GEOMETRY.get(scenario_name, SCENARIO_GEOMETRY["scenario1"])
        tx_x, tx_y = geometry["tx"]
        building_boxes = geometry["buildings"]

        fig, ax = plt.subplots(figsize=(12, 5))
        if shade_buildings:
            for index, (x_min, x_max, _, _) in enumerate(building_boxes):
                label = "Building shadow" if index == 0 else None
                ax.axvspan(x_min, x_max, alpha=0.12, color="gray", label=label)
        for model_key, (axis_vals, q25, median, q75) in model_summaries.items():
            spec = MODEL_SPECS[model_key]
            ax.fill_between(axis_vals, q25, q75, color=spec["color"], alpha=0.10)
            ax.plot(axis_vals, median,
                f'{spec["marker"]}-',
                color=spec["color"],
                markersize=3,
                linewidth=1.8,
                label=spec["label"])
        ax.set_xlabel(plot_axis_label, fontsize=12)
        ax.set_ylabel("Received Power (dBm)", fontsize=12)
        ax.set_title(f"{scenario_name}: Median Received Power",
                     fontsize=13)
        ax.legend(fontsize=11)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        out_path = os.path.join(args.output, f"{scenario_name}_rxpow_{plot_axis_suffix}.png")
        fig.savefig(out_path, dpi=150)
        print(f"Saved: {out_path}")
        plt.close(fig)

        fig2, ax2 = plt.subplots(figsize=(12, 4))
        if shade_buildings:
            for index, (x_min, x_max, _, _) in enumerate(building_boxes):
                label = "Building shadow" if index == 0 else None
                ax2.axvspan(x_min, x_max, alpha=0.12, color="gray", label=label)
        ax2.axhline(0, color="black", linewidth=0.5)
        sionna_axis, _, sionna_med, _ = model_summaries["sionna"]
        for model_key in ("foba", "friis", "itur", "two_ray"):
            spec = MODEL_SPECS[model_key]
            diff_values = scenario_model_values[model_key] - scenario_model_values["sionna"]
            axis_vals, q25, median, q75 = summarize_by_axis(plot_axis, diff_values)
            ax2.fill_between(axis_vals, q25, q75, color=spec["color"], alpha=0.10)
            ax2.plot(axis_vals, median,
                     f'{spec["marker"]}-',
                     color=spec["color"],
                     markersize=3,
                     linewidth=1.8,
                     label=f'{spec["label"]} - Sionna')
        ax2.set_xlabel(plot_axis_label, fontsize=12)
        ax2.set_ylabel("Model - Sionna (dB)", fontsize=12)
        ax2.set_title(f"{scenario_name}: Difference To Sionna",
                      fontsize=13)
        ax2.legend(fontsize=10)
        ax2.grid(True, alpha=0.3)
        fig2.tight_layout()
        out_path2 = os.path.join(args.output, f"{scenario_name}_model_difference_{plot_axis_suffix}.png")
        fig2.savefig(out_path2, dpi=150)
        print(f"Saved: {out_path2}")
        plt.close(fig2)

        fig3, ax3 = plt.subplots(figsize=(10, 4))
        first_run = np.min(scenario_runs)
        run_mask = scenario_runs == first_run
        geom_x = scenario_rx_x[run_mask]
        geom_y = scenario_rx_y[run_mask]

        for index, (x_min, x_max, y_min, y_max) in enumerate(building_boxes):
            label = "Building" if index == 0 else None
            building = mpatches.Rectangle((x_min, y_min),
                                          x_max - x_min,
                                          y_max - y_min,
                                          linewidth=2,
                                          edgecolor="gray",
                                          facecolor="lightgray",
                                          label=label)
            ax3.add_patch(building)

        ax3.plot(tx_x, tx_y, "^", color="blue", markersize=12, label="TX (static)")
        ax3.plot(geom_x, geom_y, "-", color="red", linewidth=2, label="RX trajectory")
        ax3.plot(geom_x[0], geom_y[0], "o", color="red", markersize=8, label="RX start")
        ax3.plot(geom_x[-1], geom_y[-1], "s", color="darkred", markersize=8, label="RX end")
        if len(geom_x) >= 2:
            ax3.annotate("", xy=(geom_x[-1], geom_y[-1]), xytext=(geom_x[-2], geom_y[-2]),
                         arrowprops=dict(arrowstyle="->", color="red", lw=2))

        y_span = max(np.ptp(geom_y), 1.0)
        label_offset = max(2.0, 0.08 * y_span)
        ax3.text(geom_x[0], geom_y[0] - label_offset, "RX start",
                 ha="center", fontsize=9, color="red")
        ax3.text(geom_x[-1], geom_y[-1] - label_offset, "RX end",
                 ha="center", fontsize=9, color="darkred")
        ax3.text(tx_x, tx_y + label_offset, "TX",
                 ha="center", fontsize=9, color="blue")

        x_building = np.array([coord for x_min, x_max, _, _ in building_boxes for coord in (x_min, x_max)])
        y_building = np.array([coord for _, _, y_min, y_max in building_boxes for coord in (y_min, y_max)])
        x_all = np.concatenate((np.array([np.min(geom_x), np.max(geom_x), tx_x]), x_building))
        y_all = np.concatenate((np.array([np.min(geom_y), np.max(geom_y), tx_y]), y_building))
        x_span = max(np.ptp(x_all), 1.0)
        y_span_full = max(np.ptp(y_all), 1.0)
        x_pad = max(5.0, 0.08 * x_span)
        y_pad = max(5.0, 0.08 * y_span_full)

        ax3.set_xlabel("x (m)", fontsize=12)
        ax3.set_ylabel("y (m)", fontsize=12)
        ax3.set_title(f"{scenario_name}: Geometry (representative run)", fontsize=13)
        ax3.set_xlim(np.min(x_all) - x_pad, np.max(x_all) + x_pad)
        ax3.set_ylim(np.min(y_all) - y_pad, np.max(y_all) + y_pad)
        ax3.set_aspect("equal")
        ax3.legend(fontsize=10, loc="upper left")
        ax3.grid(True, alpha=0.3)
        fig3.tight_layout()
        out_path3 = os.path.join(args.output, f"{scenario_name}_geometry.png")
        fig3.savefig(out_path3, dpi=150)
        print(f"Saved: {out_path3}")
        plt.close(fig3)

    print(f"Done. Aggregated {len(np.unique(runs))} runs across {len(scenario_names)} scenarios.")


if __name__ == "__main__":
    main()

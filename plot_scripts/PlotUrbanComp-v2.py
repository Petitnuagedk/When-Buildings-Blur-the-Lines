import argparse
import os
import csv
import matplotlib.pyplot as plt
import numpy as np

try:
    # Use Student-t critical values for 95% CIs when sample size (epochs) is small.
    from scipy.stats import t as student_t
except ImportError:
    student_t = None


def compute_95ci_halfwidth(values):
    """Compute 95% confidence interval half-width for a list of numeric values.

    Uses Student's t distribution when available, falling back to the normal
    approximation (z=1.96) if scipy is not installed.

    This matches the formula described in the request:
        CI95 = mean +/- t_{0.025,n-1} * (s / sqrt(n))
    """

    arr = np.array(values, dtype=np.float64)
    arr = arr[~np.isnan(arr)]
    n = len(arr)
    if n < 2:
        return None

    s = np.std(arr, ddof=1)
    if student_t is not None:
        tval = float(student_t.ppf(0.975, n - 1))
    else:
        # Fallback: normal approximation when scipy is not available.
        tval = 1.96

    return tval * s / np.sqrt(n)

# === Config ===
# Location where the simulation output is stored (as produced by UrbanCompMain-v2.cc)
# This script is in plot_scripts/, so we resolve the data directory relative to it.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
source = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "..", "SASB-data", "UrbanRaCompDir-v2"))

# --- CLI ---
parser = argparse.ArgumentParser(description="Plot urban comp metrics across epochs")
parser.add_argument(
    "--epochs",
    "-e",
    type=int,
    default=None,
    help="Number of epochs to use (first N sorted epoch folders). If unset, uses all detected epochs.",
)
parser.add_argument(
    "--max-goodput-kbps",
    type=float,
    default=150.0,
    help="Maximum goodput (kbps) to include in plots; values above this are treated as invalid/outliers.",
)
args = parser.parse_args()

MAX_GOODPUT_KBPS = args.max_goodput_kbps

# Discover which epoch folders are present (Epoch_1, Epoch_2, ...)
BASE_DIRS = sorted(
    [d for d in os.listdir(source) if os.path.isdir(os.path.join(source, d)) and d.startswith("Epoch_")]
)

# Allow limiting the number of epochs that are used for plotting
if args.epochs is not None:
    if args.epochs <= 0:
        raise SystemExit("--epochs must be a positive integer")
    BASE_DIRS = BASE_DIRS[: args.epochs]

# === Plotting ===
PLOTS_DIR = os.path.join("plots-v2-f")
os.makedirs(PLOTS_DIR, exist_ok=True)

loss_models = ["FOBA", "Friis", "TwoRayGroundPropagationLossModel", "ItuR1411LosPropagationLossModel"]
algorithms = ["aodv", "olsr", "dsdv"]

# Discover which node-count folders are present in the dataset
num_nodes_set = set()
for base in BASE_DIRS:
    for lm in loss_models:
        for algo in algorithms:
            num_nodes_dir = os.path.join(source, base, lm, algo, "numNodes")
            if os.path.isdir(num_nodes_dir):
                for entry in os.listdir(num_nodes_dir):
                    if entry.isdigit():
                        num_nodes_set.add(int(entry))

num_nodes_list = sorted(num_nodes_set)

# === Metrics Storage ===
epochs = {
    epoch: {
        "pdr_vs_nodes": [],
        "eed_vs_nodes": [],
        "goodput_vs_nodes": [],
        "execution_time_vs_nodes": [],
    }
    for epoch in BASE_DIRS
}

route_acquisition_across_epochs = {
    epoch: {lm: [] for lm in loss_models} for epoch in BASE_DIRS
}

routing_overhead_across_epochs = {
    epoch: {lm: {algo: [] for algo in algorithms} for lm in loss_models}
    for epoch in BASE_DIRS
}

# === Extractors ===
def extract_pdr_from_csv(csv_path):
    try:
        with open(csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            total_pdr = 0
            flow_count = 0

            for row in reader:
                tx = int(row["TxPackets"])
                rx = int(row["RxPackets"])
                if tx > 0:
                    total_pdr += rx / tx
                    flow_count += 1
            return total_pdr / flow_count if flow_count > 0 else None
    except:
        return None

def extract_eed_from_csv(csv_path):
    try:
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            total_eed = 0
            flow_count = 0
            for row in reader:
                app_rx = int(row.get("RxPackets", 0))
                sum_delay = float(row.get("sumDelay", 0))
                if app_rx > 0:
                    total_eed += sum_delay / app_rx
                    flow_count += 1
            return total_eed / flow_count if flow_count > 0 else None
    except:
        return None

def extract_goodput_from_csv(csv_path):
    try:
        with open(csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            total_goodput_kbps = 0.0
            flow_count = 0
            excluded_flows = 0

            for row in reader:
                rx = int(row["RxPackets"])
                first_tx = float(row["FirstTxTime"])
                last_rx = float(row["LastRxTime"])
                duration = last_rx - first_tx
                if duration <= 0 or rx <= 0:
                    continue

                goodput_kbps = (rx * 1024 * 8) / duration / 1000
                if goodput_kbps > MAX_GOODPUT_KBPS:
                    excluded_flows += 1
                    continue

                total_goodput_kbps += goodput_kbps
                flow_count += 1

            if excluded_flows > 0:
                print(f"Excluded {excluded_flows} flow(s) from {csv_path} because goodput exceeded {MAX_GOODPUT_KBPS} kbps")

            return total_goodput_kbps / flow_count if flow_count > 0 else None
    except Exception as e:
        print(f"Goodput extraction error for {csv_path}: {e}")
        return None

def extract_route_acquisition_time(csv_path):
    try:
        with open(csv_path, newline='') as csvfile:
            raw_headers = csvfile.readline().strip().split(',')
            headers = [h.strip() for h in raw_headers]
            reader = csv.DictReader(csvfile, fieldnames=headers, delimiter=',')
            acquisition_times = []
            for row in reader:
                try:
                    row = {k.strip(): v.strip() for k, v in row.items()}
                    if int(row["routeFound"]) == 1:
                        time_rrep = float(row["TimeRREP"])
                        time_rreq = float(row["TimeRREQ"])
                        acq = time_rrep - time_rreq
                        if acq >= 0:
                            acquisition_times.append(acq)
                except:
                    continue
            return np.mean(acquisition_times) if acquisition_times else None
    except:
        return None

def extract_routing_overhead(tsv_path):
    try:
        with open(tsv_path, newline='') as tsvfile:
            # Read and clean the header manually
            raw_headers = tsvfile.readline().strip().split(',')
            headers = [h.strip() for h in raw_headers]
            reader = csv.DictReader(tsvfile, fieldnames=headers, delimiter=',')
            ROs = []
            for row in reader:
                try:
                    row = {k.strip(): v.strip() for k, v in row.items()}
                    Rxapp = float(row.get("AppPacketsReceived", 0))
                    Txsig = float(row.get("RouteSignalizationPacketsSent", 0))
                    #print("Rxapp:", Rxapp, "Txsig:", Txsig)
                    if Rxapp < 0 or Txsig < 0:
                        continue
                    if Rxapp/Txsig < 0:
                        print("Negative routing overhead found in", tsv_path, "for row:", row)
                        continue
                    if Txsig > 0:
                        ROs.append(Rxapp / Txsig)
                except Exception as e:
                    print("Row parse error:", e)
                    continue
            return np.mean(ROs) if ROs else None
    except Exception as e:
        print("File read error:", e)
        return None


# === Collect Data ===
for BASE_DIR in BASE_DIRS:
    pdr_vs_nodes = {lm: {algo: [] for algo in algorithms} for lm in loss_models}
    eed_vs_nodes = {lm: {algo: [] for algo in algorithms} for lm in loss_models}
    goodput_vs_nodes = {lm: {algo: [] for algo in algorithms} for lm in loss_models}
    execution_time_vs_nodes = {lm: {algo: [] for algo in algorithms} for lm in loss_models}

    for lm in loss_models:
        for algo in algorithms:
            for nodes in num_nodes_list:
                folder = os.path.join(source, BASE_DIR, lm, algo, "numNodes", str(nodes))

                # Keep the output lists aligned with num_nodes_list so that x values match y values.
                # When data is missing, append np.nan and print why it was missing.
                if not os.path.isdir(folder):
                    print(f"MISSING FOLDER: {BASE_DIR}/{lm}/{algo}/numNodes/{nodes} (skipping point)")
                    execution_time_vs_nodes[lm][algo].append(np.nan)
                    pdr_vs_nodes[lm][algo].append(np.nan)
                    eed_vs_nodes[lm][algo].append(np.nan)
                    goodput_vs_nodes[lm][algo].append(np.nan)
                    continue

                time_path = os.path.join(folder, "simulation_time.csv")
                try:
                    with open(time_path, "r") as f:
                        exec_time = float(f.readline().strip())
                except Exception as e:
                    exec_time = None
                    print(f"MISSING/INVALID simulation_time.csv for {BASE_DIR}/{lm}/{algo}/numNodes/{nodes}: {e}")
                execution_time_vs_nodes[lm][algo].append(exec_time if exec_time is not None else np.nan)

                flow_csv = os.path.join(folder, "flow_information.csv")
                if os.path.isfile(flow_csv):
                    pdr = extract_pdr_from_csv(flow_csv)
                    eed = extract_eed_from_csv(flow_csv)
                    goodput = extract_goodput_from_csv(flow_csv)

                    if pdr is None:
                        print(f"MISSING/INVALID PDR data for {BASE_DIR}/{lm}/{algo}/numNodes/{nodes}")
                    if eed is None:
                        print(f"MISSING/INVALID EED data for {BASE_DIR}/{lm}/{algo}/numNodes/{nodes}")
                    if goodput is None:
                        print(f"MISSING/INVALID Goodput data for {BASE_DIR}/{lm}/{algo}/numNodes/{nodes}")

                    pdr_vs_nodes[lm][algo].append(pdr if pdr is not None else np.nan)
                    eed_vs_nodes[lm][algo].append(eed if eed is not None else np.nan)
                    goodput_vs_nodes[lm][algo].append(goodput if goodput is not None else np.nan)
                else:
                    print(f"MISSING FILE: {flow_csv} (no pdr/eed/goodput for {BASE_DIR}/{lm}/{algo}/numNodes/{nodes})")
                    pdr_vs_nodes[lm][algo].append(np.nan)
                    eed_vs_nodes[lm][algo].append(np.nan)
                    goodput_vs_nodes[lm][algo].append(np.nan)

                ro_csv = os.path.join(folder, "Network_traffic_mapping.csv")
                if os.path.isfile(ro_csv):
                    routing_overhead = extract_routing_overhead(ro_csv)
                    if routing_overhead is not None:
                        routing_overhead_across_epochs[BASE_DIR][lm][algo].append(routing_overhead)
                    else:
                        print("Routing overhead extraction failed for", ro_csv, " most likely due to empty file")
                else:
                    # Keep list aligned even if missing; still allow averaging/masking later.
                    routing_overhead_across_epochs[BASE_DIR][lm][algo].append(np.nan)

                if algo == "aodv":
                    route_csv = os.path.join(folder, "Route_mapping.csv")
                    if os.path.isfile(route_csv):
                        route_time = extract_route_acquisition_time(route_csv)
                        if route_time is None:
                            print(f"MISSING/INVALID route acquisition data for {BASE_DIR}/{lm}/{algo}/numNodes/{nodes}")
                        route_acquisition_across_epochs[BASE_DIR][lm].append(route_time if route_time is not None else np.nan)
                    else:
                        print(f"MISSING FILE: {route_csv} (no route acquisition for {BASE_DIR}/{lm}/{algo}/numNodes/{nodes})")
                        route_acquisition_across_epochs[BASE_DIR][lm].append(np.nan)

    epochs[BASE_DIR]["pdr_vs_nodes"].append(pdr_vs_nodes)
    epochs[BASE_DIR]["eed_vs_nodes"].append(eed_vs_nodes)
    epochs[BASE_DIR]["goodput_vs_nodes"].append(goodput_vs_nodes)
    epochs[BASE_DIR]["execution_time_vs_nodes"].append(execution_time_vs_nodes)

# === Averaging ===
average_metrics_across_epochs = {
    metric: {lm: {algo: [None] * len(num_nodes_list) for algo in algorithms} for lm in loss_models}
    for metric in ["pdr_vs_nodes", "eed_vs_nodes", "goodput_vs_nodes"]
}

average_metrics_ci95_across_epochs = {
    metric: {lm: {algo: [None] * len(num_nodes_list) for algo in algorithms} for lm in loss_models}
    for metric in ["pdr_vs_nodes", "eed_vs_nodes", "goodput_vs_nodes"]
}

average_routing_overhead = {
    lm: {algo: [None] * len(num_nodes_list) for algo in algorithms}
    for lm in loss_models
}

min_routing_overhead = {
    lm: {algo: [None] * len(num_nodes_list) for algo in algorithms}
    for lm in loss_models
}

max_routing_overhead = {
    lm: {algo: [None] * len(num_nodes_list) for algo in algorithms}
    for lm in loss_models
}

for metric in average_metrics_across_epochs:
    for lm in loss_models:
        for algo in algorithms:
            for i in range(len(num_nodes_list)):
                values = []
                for BASE_DIR in BASE_DIRS:
                    try:
                        vals_list = epochs[BASE_DIR][metric][0][lm][algo]
                        if i < len(vals_list):
                            v = vals_list[i]
                            if v is not None and not np.isnan(v):
                                values.append(v)
                    except:
                        continue
                if values:
                    average_metrics_across_epochs[metric][lm][algo][i] = np.mean(values)
                    average_metrics_ci95_across_epochs[metric][lm][algo][i] = compute_95ci_halfwidth(values)

# Route Acquisition
average_route_acquisition = {lm: [None]*len(num_nodes_list) for lm in loss_models}
route_acquisition_ci95 = {lm: [None]*len(num_nodes_list) for lm in loss_models}

for lm in loss_models:
    for algo in algorithms:
        for i in range(len(num_nodes_list)):
            values = []
            for epoch in BASE_DIRS:
                try:
                    vals_list = routing_overhead_across_epochs[epoch][lm][algo]
                    if i < len(vals_list):
                        v = vals_list[i]
                        if v is not None and not np.isnan(v):
                            values.append(v)
                except:
                    continue
            if values:
                average_routing_overhead[lm][algo][i] = np.mean(values)
                min_routing_overhead[lm][algo][i] = np.min(values)
                max_routing_overhead[lm][algo][i] = np.max(values)

for lm in loss_models:
    for i in range(len(num_nodes_list)):
        values = []
        for epoch in BASE_DIRS:
            try:
                vals_list = route_acquisition_across_epochs[epoch][lm]
                if i < len(vals_list):
                    v = vals_list[i]
                    if v is not None and not np.isnan(v):
                        values.append(v)
            except:
                continue
        if values:
            average_route_acquisition[lm][i] = np.mean(values)
            route_acquisition_ci95[lm][i] = compute_95ci_halfwidth(values)



def compute_ylim_from_metric_data(metric_data, lower_cap=None, upper_cap=None, margin=0.05):
    """Compute common y-limits across all algorithms and loss models for a metric."""
    all_vals = []
    for lm_data in metric_data.values():
        for algo_vals in lm_data.values():
            all_vals.extend([v for v in algo_vals if v is not None and not np.isnan(v)])
    if not all_vals:
        return (0, 1)

    y_min = min(all_vals)
    y_max = max(all_vals)

    if lower_cap is not None:
        y_min = max(y_min, lower_cap)
    if upper_cap is not None:
        y_max = min(y_max, upper_cap)

    if y_min == y_max:
        y_min -= 0.5
        y_max += 0.5

    span = y_max - y_min
    y_min -= span * margin
    y_max += span * margin

    if lower_cap is not None:
        y_min = max(y_min, lower_cap)
    if upper_cap is not None:
        y_max = min(y_max, upper_cap)

    return (y_min, y_max)


def plot_metric_by_loss_model(x_vals, metric_data, ylabel, title_prefix, xlabel, filename_prefix, yerr_data=None, ylim=None):
    for algo in algorithms:
        plt.figure(figsize=(10, 6))
        for lm in loss_models:
            y_vals = np.array(metric_data[lm][algo], dtype=np.float64)
            yerr = np.array(yerr_data[lm][algo], dtype=np.float64) if yerr_data else None
            mask = ~np.isnan(y_vals)
            line, = plt.plot(
                np.array(x_vals)[mask],
                y_vals[mask],
                label=f"{lm}",
                marker='o',
                markersize=6,
                linestyle='-'
            )
            if yerr_data is not None:
                plt.fill_between(
                    np.array(x_vals)[mask],
                    y_vals[mask] - yerr[mask],
                    y_vals[mask] + yerr[mask],
                    color=line.get_color(),
                    alpha=0.2,
                )

        if ylim is not None:
            plt.ylim(ylim)

        if "goodput" in filename_prefix.lower():
            plt.axhline(y=81.92, color='red', linestyle=':', linewidth=2, label='Average Theoretical Goodput')

        plt.title(f"{title_prefix} for {algo.upper()}")
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(False)
        plt.tight_layout()
        plt.savefig(os.path.join(PLOTS_DIR, f"{filename_prefix}_{algo}.png"))
        plt.close()

def plot_route_acquisition_time(x_vals, y_data, y_err, ylabel, title, xlabel, filename, ylim=None):
    plt.figure(figsize=(10, 6))
    for lm in loss_models:
        y_vals = np.array(y_data[lm], dtype=np.float64)
        yerr = np.array(y_err[lm], dtype=np.float64)
        mask = ~np.isnan(y_vals)
        line, = plt.plot(np.array(x_vals)[mask], y_vals[mask], label=lm, marker='o')
        plt.fill_between(np.array(x_vals)[mask], y_vals[mask] - yerr[mask], y_vals[mask] + yerr[mask], alpha=0.2, color=line.get_color())

    if ylim is not None:
        plt.ylim(ylim)

    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(False)
    plt.tight_layout()
    plt.savefig(os.path.join(PLOTS_DIR, f"{filename}.png"))
    plt.close()

def plot_routing_overhead_min_max(x_vals, avg_data, min_data, max_data, ylabel, title_prefix, xlabel, filename_prefix, ylim=None):
    for algo in algorithms:
        plt.figure(figsize=(10, 6))
        for lm in loss_models:
            avg_vals = np.array(avg_data[lm][algo], dtype=np.float64)
            min_vals = np.array(min_data[lm][algo], dtype=np.float64)
            max_vals = np.array(max_data[lm][algo], dtype=np.float64)

            mask = ~np.isnan(avg_vals)
            x = np.array(x_vals)[mask]
            y = avg_vals[mask]
            y_min = min_vals[mask]
            y_max = max_vals[mask]

            line, = plt.plot(x, y, label=lm, marker='o')
            plt.fill_between(x, y_min, y_max, color=line.get_color(), alpha=0.2)

        if ylim is not None:
            plt.ylim(ylim)

        plt.title(f"{title_prefix} for {algo.upper()}")
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(False)
        plt.tight_layout()
        plt.savefig(os.path.join(PLOTS_DIR, f"{filename_prefix}_{algo}.png"))
        plt.close()

# === Final Plot Calls ===
# Compute shared y-limits per metric so each algorithm plot uses the same scale.
pdr_ylim = compute_ylim_from_metric_data(average_metrics_across_epochs["pdr_vs_nodes"], lower_cap=0.0, upper_cap=1.0)
eed_ylim = compute_ylim_from_metric_data(average_metrics_across_epochs["eed_vs_nodes"], lower_cap=0.0, upper_cap=10.0)
goodput_ylim = compute_ylim_from_metric_data(average_metrics_across_epochs["goodput_vs_nodes"], lower_cap=0.0)

plot_metric_by_loss_model(num_nodes_list, average_metrics_across_epochs["pdr_vs_nodes"], "Packet Delivery Ratio", "PDR vs. Number of Nodes", "Number of Nodes", "pdr_vs_nodes", yerr_data=average_metrics_ci95_across_epochs["pdr_vs_nodes"], ylim=pdr_ylim)
plot_metric_by_loss_model(num_nodes_list, average_metrics_across_epochs["eed_vs_nodes"], "End-to-End Delay", "EED vs. Number of Nodes", "Number of Nodes", "eed_vs_nodes", yerr_data=average_metrics_ci95_across_epochs["eed_vs_nodes"], ylim=eed_ylim)
plot_metric_by_loss_model(num_nodes_list, average_metrics_across_epochs["goodput_vs_nodes"], "Goodput (kb/s)", "Goodput vs. Number of Nodes", "Number of Nodes", "goodput_vs_nodes", yerr_data=average_metrics_ci95_across_epochs["goodput_vs_nodes"], ylim=goodput_ylim)

# Route acquisition and routing overhead should also share consistent y-limits across algorithm plots.
route_acq_ylim = compute_ylim_from_metric_data({lm: {"aodv": average_route_acquisition[lm]} for lm in loss_models}, lower_cap=0.0)
routing_overhead_ylim = compute_ylim_from_metric_data(average_routing_overhead, lower_cap=0.0)

plot_route_acquisition_time(num_nodes_list, average_route_acquisition, route_acquisition_ci95, "Route Acquisition Time (s)", "Route Acquisition Time vs Number of Nodes (AODV)", "Number of Nodes", "route_acquisition_time_aodv", ylim=route_acq_ylim)
plot_routing_overhead_min_max(num_nodes_list, average_routing_overhead, min_routing_overhead, max_routing_overhead, "Routing Overhead", "Routing Overhead vs Number of Nodes", "Number of Nodes", "routing_overhead", ylim=routing_overhead_ylim)

import os
import csv
import matplotlib.pyplot as plt

# === Config ===
source = "UrbanRaComp-7.0"
BASE_DIRS = ["Epoch_1","Epoch_2", "Epoch_3", "Epoch_4", "Epoch_5", "Epoch_6", "Epoch_7", "Epoch_8", "Epoch_9", "Epoch_10"]
loss_models = ["FOBA", "Friis", "TwoRayGroundPropagationLossModel", "ItuR1411LosPropagationLossModel"]
algorithms = ["aodv", "olsr", "dsdv"]
num_nodes_list = [20, 30, 40, 50, 60, 70, 80, 90]

epochs = {
    epoch: {
        "pdr_vs_nodes": [],
        "eed_vs_nodes": [],
        "goodput_vs_nodes": [],
        "execution_time_vs_nodes": [],
    }
    for epoch in BASE_DIRS
}


# === CSV-based Metrics Extraction ===
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

            return total_pdr / flow_count if flow_count > 0 else 0
    except Exception as e:
        print(f"Error reading PDR from {csv_path}: {e}")
        return 0


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

            if flow_count > 0:
                return total_eed / flow_count
            return 0
    except Exception as e:
        print(f"Error reading {csv_path}: {e}")
        return 0



def extract_goodput_from_csv(csv_path):
    try:
        with open(csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            total_goodput = 0
            flow_count = 0

            for row in reader:
                rx = int(row["RxPackets"])
                first_tx = float(row["FirstTxTime"])
                last_rx = float(row["LastRxTime"])

                duration = last_rx - first_tx
                if duration > 0 and rx > 0:
                    # Assume packet size = 1024 bytes (customize as needed)
                    goodput = (rx * 1024 * 8) / duration  # bits per second
                    total_goodput += goodput
                    flow_count += 1

            return (total_goodput / 1000) / flow_count if flow_count > 0 else 0  # kbps
    except Exception as e:
        print(f"Error reading Goodput from {csv_path}: {e}")
        return 0


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
                if not os.path.isdir(folder):
                    for metric in [pdr_vs_nodes, eed_vs_nodes, goodput_vs_nodes, execution_time_vs_nodes]:
                        metric[lm][algo].append(0)
                    continue

                # Execution Time
                time_path = os.path.join(folder, "simulation_time.csv")
                try:
                    with open(time_path, "r") as f:
                        exec_time = float(f.readline().strip())
                except:
                    exec_time = 0
                execution_time_vs_nodes[lm][algo].append(exec_time)

                # Metrics CSV
                flow_csv = os.path.join(folder, "flow_information.csv")
                if not os.path.isfile(flow_csv):
                    for metric in [pdr_vs_nodes, eed_vs_nodes, goodput_vs_nodes]:
                        metric[lm][algo].append(0)
                    continue

                pdr = extract_pdr_from_csv(flow_csv)
                eed = extract_eed_from_csv(flow_csv)
                goodput = extract_goodput_from_csv(flow_csv)

                pdr_vs_nodes[lm][algo].append(pdr)
                eed_vs_nodes[lm][algo].append(eed)
                goodput_vs_nodes[lm][algo].append(goodput)

    epochs[BASE_DIR]["pdr_vs_nodes"].append(pdr_vs_nodes)
    epochs[BASE_DIR]["eed_vs_nodes"].append(eed_vs_nodes)
    epochs[BASE_DIR]["goodput_vs_nodes"].append(goodput_vs_nodes)
    epochs[BASE_DIR]["execution_time_vs_nodes"].append(execution_time_vs_nodes)

# === Plotting & Averaging Logic (unchanged) ===

PLOTS_DIR = os.path.join("plots")
os.makedirs(PLOTS_DIR, exist_ok=True)

def plot_metric_by_loss_model(x_vals, metric_data, ylabel, title_prefix, xlabel, filename_prefix):
    for algo in algorithms:
        plt.figure(figsize=(10, 6))
        for lm in loss_models:
            y_vals = metric_data[lm][algo]
            plt.plot(x_vals, y_vals, label=f"{lm}", marker='o')

        plt.title(f"{title_prefix} for {algo.upper()}")
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        
        if "pdr" in filename_prefix.lower():
            plt.ylim(0, 1)
        elif "goodput" in filename_prefix.lower():
            plt.ylim(0, 100)
            plt.axhline(y=81.92, color='red', linestyle=':', linewidth=2, label='Max Goodput (81.92 kb/s)')
        elif "eed" in filename_prefix.lower():
            plt.ylim(0, 1)

        plt.legend()
        plt.grid(False)
        plt.tight_layout()
        plt.savefig(os.path.join(PLOTS_DIR, f"{filename_prefix}_{algo}.png"))
        plt.close()

def plot_time_by_loss_model(x_vals, metric_data, ylabel, title_prefix, xlabel, filename_prefix):
    for algo in algorithms:
        plt.figure(figsize=(10, 6))
        for lm in loss_models:
            y_vals = metric_data[lm][algo]
            plt.plot(x_vals, y_vals, label=f"{lm}", marker='o')

        plt.title(f"{title_prefix}")
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(PLOTS_DIR, f"{filename_prefix}_{algo}.png"))
        plt.close()

average_metrics_across_epochs = {
    "pdr_vs_nodes": {lm: {algo: [0] * len(num_nodes_list) for algo in algorithms} for lm in loss_models},
    "eed_vs_nodes": {lm: {algo: [0] * len(num_nodes_list) for algo in algorithms} for lm in loss_models},
    "goodput_vs_nodes": {lm: {algo: [0] * len(num_nodes_list) for algo in algorithms} for lm in loss_models},
}

for metric in ["pdr_vs_nodes", "eed_vs_nodes", "goodput_vs_nodes"]:
    for lm in loss_models:
        for algo in algorithms:
            for i in range(len(num_nodes_list)):
                total = 0
                count = 0
                for BASE_DIR in BASE_DIRS:
                    try:
                        total += epochs[BASE_DIR][metric][0][lm][algo][i]
                        count += 1
                    except Exception as e:
                        print(f"Error aggregating {metric} for {BASE_DIR} - {lm}/{algo}/{i}: {e}")
                average_metrics_across_epochs[metric][lm][algo][i] = total / count if count > 0 else 0

# === Final Plot Calls ===
plot_metric_by_loss_model(num_nodes_list, average_metrics_across_epochs["pdr_vs_nodes"], "Packet Delivery Ratio", "PDR vs. Number of Nodes", "Number of Nodes", "pdr_vs_nodes")
plot_metric_by_loss_model(num_nodes_list, average_metrics_across_epochs["eed_vs_nodes"], "End-to-End Delay", "EED vs. Number of Nodes", "Number of Nodes", "eed_vs_nodes")
plot_metric_by_loss_model(num_nodes_list, average_metrics_across_epochs["goodput_vs_nodes"], "Goodput (kb/s)", "Goodput vs. Number of Nodes", "Number of Nodes", "goodput_vs_nodes")

plot_time_by_loss_model(
    num_nodes_list,
    execution_time_vs_nodes,
    "Execution Time (s)",
    "Execution Time vs. Number of Nodes for 300 simulated seconds",
    "Number of Nodes",
    "execution_time_vs_nodes"
)
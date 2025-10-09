import os
import csv
import matplotlib.pyplot as plt
import numpy as np

# === Config ===
source = "UrbanRaCompDir"
BASE_DIRS = ["Epoch_1", "Epoch_2", "Epoch_3", "Epoch_4", "Epoch_5", "Epoch_6", "Epoch_7"] # , "Epoch_8", "Epoch_9", "Epoch_10"]
loss_models = ["FOBA", "Friis", "TwoRayGroundPropagationLossModel", "ItuR1411LosPropagationLossModel"] #,"Tuned-Friis (SL=2.5)"] 
algorithms = ["aodv", "olsr", "dsdv"]
num_nodes_list = [20, 30, 40, 50, 60, 70, 80, 90]

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
            total_goodput = 0
            flow_count = 0
            for row in reader:
                rx = int(row["RxPackets"])
                first_tx = float(row["FirstTxTime"])
                last_rx = float(row["LastRxTime"])
                duration = last_rx - first_tx
                if duration > 0 and rx > 0:
                    goodput = (rx * 1024 * 8) / duration
                    total_goodput += goodput
                    flow_count += 1
                # if (goodput / 1000) > 4505.6:
                #     print("Goodput for", csv_path, ":", goodput / 1000)
                #     print("duration:", duration, "s, with FirstTxTime:", first_tx, "and LastRxTime:", last_rx, " and rx:", rx)
            # if (total_goodput / 1000) / flow_count > 81.92:
            #     print("Goodput for", csv_path, ":", total_goodput / 1000)
            #     print(flow_count, "flows, duration:", duration, "s, with FirstTxTime:", first_tx, "and LastRxTime:", last_rx, " and rx:", rx)
            return (total_goodput / 1000) / flow_count if flow_count > 0 else None
    except:
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
                if not os.path.isdir(folder):
                    continue

                time_path = os.path.join(folder, "simulation_time.csv")
                try:
                    with open(time_path, "r") as f:
                        exec_time = float(f.readline().strip())
                except:
                    exec_time = None
                if exec_time is not None:
                    execution_time_vs_nodes[lm][algo].append(exec_time)

                flow_csv = os.path.join(folder, "flow_information.csv")
                if os.path.isfile(flow_csv):
                    pdr = extract_pdr_from_csv(flow_csv)
                    eed = extract_eed_from_csv(flow_csv)
                    goodput = extract_goodput_from_csv(flow_csv)
                    # if goodput != None and goodput > 81.92:
                    #     print("Goodput for", algo, "in", folder, ":", goodput)

                    if pdr is not None:
                        pdr_vs_nodes[lm][algo].append(pdr)
                    if eed is not None:
                        eed_vs_nodes[lm][algo].append(eed)
                    if goodput is not None:
                        goodput_vs_nodes[lm][algo].append(goodput)

                ro_csv = os.path.join(folder, "Network_traffic_mapping.csv")
                if os.path.isfile(ro_csv):
                    routing_overhead = extract_routing_overhead(ro_csv)
                    if routing_overhead is not None:
                        routing_overhead_across_epochs[BASE_DIR][lm][algo].append(routing_overhead)
                    else:
                        print("Routing overhead extraction failed for", ro_csv, " most likely due to empty file")

                if algo == "aodv":
                    route_csv = os.path.join(folder, "Route_mapping.csv")
                    if os.path.isfile(route_csv):
                        route_time = extract_route_acquisition_time(route_csv)
                        if route_time is not None:
                            route_acquisition_across_epochs[BASE_DIR][lm].append(route_time)

    epochs[BASE_DIR]["pdr_vs_nodes"].append(pdr_vs_nodes)
    epochs[BASE_DIR]["eed_vs_nodes"].append(eed_vs_nodes)
    epochs[BASE_DIR]["goodput_vs_nodes"].append(goodput_vs_nodes)
    epochs[BASE_DIR]["execution_time_vs_nodes"].append(execution_time_vs_nodes)

# === Averaging ===
average_metrics_across_epochs = {
    metric: {lm: {algo: [None] * len(num_nodes_list) for algo in algorithms} for lm in loss_models}
    for metric in ["pdr_vs_nodes", "eed_vs_nodes", "goodput_vs_nodes"]
}

average_metrics_std_across_epochs = {
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
                            values.append(vals_list[i])
                    except:
                        continue
                if values:
                    average_metrics_across_epochs[metric][lm][algo][i] = np.mean(values)
                    average_metrics_std_across_epochs[metric][lm][algo][i] = np.std(values)

# Route Acquisition
average_route_acquisition = {lm: [None]*len(num_nodes_list) for lm in loss_models}
stddev_route_acquisition = {lm: [None]*len(num_nodes_list) for lm in loss_models}

for lm in loss_models:
    for algo in algorithms:
        for i in range(len(num_nodes_list)):
            values = []
            for epoch in BASE_DIRS:
                try:
                    vals_list = routing_overhead_across_epochs[epoch][lm][algo]
                    if i < len(vals_list):
                        values.append(vals_list[i])
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
                    values.append(vals_list[i])
            except:
                continue
        if values:
            average_route_acquisition[lm][i] = np.mean(values)
            stddev_route_acquisition[lm][i] = np.std(values)

# === Plotting ===
PLOTS_DIR = os.path.join("plots")
os.makedirs(PLOTS_DIR, exist_ok=True)

def plot_metric_by_loss_model(x_vals, metric_data, ylabel, title_prefix, xlabel, filename_prefix, yerr_data=None):
    for algo in algorithms:
        plt.figure(figsize=(10, 6))
        for lm in loss_models:
            y_vals = np.array(metric_data[lm][algo], dtype=np.float64)
            yerr = np.array(yerr_data[lm][algo], dtype=np.float64) if yerr_data else None
            mask = ~np.isnan(y_vals)
            line, = plt.plot(np.array(x_vals)[mask], y_vals[mask], label=f"{lm}", marker='o')
            if yerr_data is not None:
                plt.fill_between(np.array(x_vals)[mask], y_vals[mask]-yerr[mask], y_vals[mask]+yerr[mask], color=line.get_color(), alpha=0.2)

        if "pdr" in filename_prefix.lower():
            plt.ylim(0, 1)
        elif "goodput" in filename_prefix.lower():
            plt.ylim(0, 90)
            plt.axhline(y=81.92, color='red', linestyle=':', linewidth=2, label='Average Theoretical Goodput')
        elif "eed" in filename_prefix.lower():
            plt.ylim(0, 1)
        
        plt.title(f"{title_prefix} for {algo.upper()}")
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(False)
        plt.tight_layout()
        plt.savefig(os.path.join(PLOTS_DIR, f"{filename_prefix}_{algo}.png"))
        plt.close()

def plot_route_acquisition_time(x_vals, y_data, y_err, ylabel, title, xlabel, filename):
    plt.figure(figsize=(10, 6))
    for lm in loss_models:
        y_vals = np.array(y_data[lm], dtype=np.float64)
        yerr = np.array(y_err[lm], dtype=np.float64)
        mask = ~np.isnan(y_vals)
        line, = plt.plot(np.array(x_vals)[mask], y_vals[mask], label=lm, marker='o')
        plt.fill_between(np.array(x_vals)[mask], y_vals[mask] - yerr[mask], y_vals[mask] + yerr[mask], alpha=0.2, color=line.get_color())
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.ylim(4, 38)
    plt.legend()
    plt.grid(False)
    plt.tight_layout()
    plt.savefig(os.path.join(PLOTS_DIR, f"{filename}.png"))
    plt.close()

def plot_routing_overhead_min_max(x_vals, avg_data, min_data, max_data, ylabel, title_prefix, xlabel, filename_prefix):
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

        plt.title(f"{title_prefix} for {algo.upper()}")
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.legend()
        plt.ylim(0, 5)
        plt.grid(False)
        plt.tight_layout()
        plt.savefig(os.path.join(PLOTS_DIR, f"{filename_prefix}_{algo}.png"))
        plt.close()

# === Final Plot Calls ===
plot_metric_by_loss_model(num_nodes_list, average_metrics_across_epochs["pdr_vs_nodes"], "Packet Delivery Ratio", "PDR vs. Number of Nodes", "Number of Nodes", "pdr_vs_nodes", yerr_data=average_metrics_std_across_epochs["pdr_vs_nodes"])
plot_metric_by_loss_model(num_nodes_list, average_metrics_across_epochs["eed_vs_nodes"], "End-to-End Delay", "EED vs. Number of Nodes", "Number of Nodes", "eed_vs_nodes", yerr_data=average_metrics_std_across_epochs["eed_vs_nodes"])
plot_metric_by_loss_model(num_nodes_list, average_metrics_across_epochs["goodput_vs_nodes"], "Goodput (kb/s)", "Goodput vs. Number of Nodes", "Number of Nodes", "goodput_vs_nodes", yerr_data=average_metrics_std_across_epochs["goodput_vs_nodes"])
plot_route_acquisition_time(num_nodes_list, average_route_acquisition, stddev_route_acquisition, "Route Acquisition Time (s)", "Route Acquisition Time vs Number of Nodes (AODV)", "Number of Nodes", "route_acquisition_time_aodv")
plot_routing_overhead_min_max(num_nodes_list, average_routing_overhead, min_routing_overhead, max_routing_overhead, "Routing Overhead", "Routing Overhead vs Number of Nodes", "Number of Nodes", "routing_overhead")

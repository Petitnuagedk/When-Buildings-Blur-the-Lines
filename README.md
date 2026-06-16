# When Buildings Blur the Lines

A simulation framework for evaluating how building-aware propagation loss models affect wireless ad-hoc network performance, using **ns-3** and **Sionna** ray-tracing.

[![ns-3 Version](https://img.shields.io/badge/ns--3-3.44-blue)](https://www.nsnam.org/)
[![Language](https://img.shields.io/badge/language-C++-orange)](https://isocpp.org/)
[![License](https://img.shields.io/badge/license-GPL--2.0-green)](LICENSE)

---

## Overview

This project compares how different propagation loss models and routing protocols perform in realistic urban environments. Simulations are organized into three scenarios:

| Scenario | Simulation | Batch Runner | Description |
|----------|-----------|--------------|-------------|
| **SA** (Scenario A) | `UrbanCompSub.cc` | `UrbanCompMain.cc` | Baseline ns-3 — single-file urban layout, 4 loss models × 3 routing protocols × 10 node counts × 10 epochs |
| **SB** (Scenario B) | `UrbanCompSub-v2.cc` | `UrbanCompMain-v2.cc` | Improved — separate node/building CSVs, z-range support, routing-aware header parsing, threaded batch runner |
| **Sionna RT** | `SionnaSub.cc` | `SionnaMain.cc` | 3D ray-tracing via Sionna — spectrum PHY, sub-6 GHz/mmWave, HPC-ready |

### Propagation Loss Models

- **FOBA** — First-Order Buildings-Aware (custom ns-3 module)
- **Friis** — Free-space path loss
- **ITU-R P.1411** — Short-range outdoor (street-canyon)
- **Two-Ray Ground** — Ground-reflected multipath

### Routing Protocols

- **AODV** — Ad hoc On-Demand Distance Vector
- **OLSR** — Optimized Link State Routing
- **DSDV** — Destination-Sequenced Distance Vector

---

## Prerequisites

1. **ns-3** (version 3.44+) — [installation guide](https://www.nsnam.org/docs/installation.html)
2. **FOBA module** — [First_Order_Buildings_Aware_PathLoss](https://github.com/Petitnuagedk/First_Order_Buildings_Aware_PathLoss)
3. **Python 3** — `pip install numpy matplotlib pandas scipy`
4. *(Sionna only)* **Sionna** with ray-tracing support, pybind11, cppyy

---

## Installation

```bash
git clone https://github.com/Petitnuagedk/When-Buildings-Blur-the-Lines
cd When-Buildings-Blur-the-Lines

# Copy simulation files into your ns-3 tree
cp scratch/* /path/to/ns-3/scratch/

# For Sionna support, also patch the spectrum module
cp src/* /path/to/ns-3/src/spectrum/
```

---

## Usage

### Running Scenario A (SA)

```bash
# Single run
./ns3 run scratch/UrbanCompSub.cc -- --numNodes=40 --RA=aodv --lossModel=FOBA

# Full batch (10 epochs × 4 models × 3 RAs × 10 node counts = 1,200 runs)
./ns3 run scratch/UrbanCompMain.cc
```

### Running Scenario B (SB)

```bash
# Single run
./ns3 run scratch/UrbanCompSub-v2.cc -- --numNodes=40 --RA=aodv --lossModel=FOBA

# Full batch (parallelized with thread pool)
g++ -std=c++17 -pthread -o UrbanCompMain-v2 scratch/UrbanCompMain-v2.cc
./UrbanCompMain-v2
```

### Running Sionna Ray-Tracing

```bash
# Single run
./ns3 run scratch/SionnaSub.cc -- --maxNodes=40 --routing=aodv

# Batch (sequential)
./ns3 run scratch/SionnaMain.cc

# HPC (SLURM array job — 300 tasks)
sbatch scratch/sbatch_sionna_array.sh
```

### FOBA vs Sionna Direct Comparison

A minimal 2-node scenario (1 static TX, 1 mobile RX walking behind a building)
that logs received power from both FOBA and Sionna RT at each time step:

```bash
./ns3 run scratch/FobaSionnaComparison.cc

# Plot results
python plot_scripts/PlotFobaSionnaComparison.py
```

### Connectivity Analysis

```bash
# SA connectivity sweeps
python scratch/run_all_connectivity_SA.py

# SB connectivity sweeps
python scratch/run_all_connectivity-SB.py
```

### Generating Plots

```bash
python plot_scripts/PlotUrbanComp.py        # SA metrics
python plot_scripts/PlotUrbanComp-v2.py     # SB metrics
python plot_scripts/PlotUrbanComp-sionna.py # Sionna metrics
python plot_scripts/PlotDropData.py         # Packet drop analysis
python plot_scripts/PlotRoutingMetrics.py   # SA vs SB routing comparison
python plot_scripts/PlotSA_CompactDashboard.py  # SA connectivity dashboard
python plot_scripts/PlotSB_CompactDashboard.py  # SB connectivity dashboard
python plot_scripts/PlotFobaSionnaComparison.py # FOBA vs Sionna 2-node comparison
```

---

## Project Structure

```
.
├── scratch/                              # ns-3 simulation files (copy to ns-3/scratch/)
│   ├── UrbanCompSub.cc                  # SA simulation engine
│   ├── UrbanCompSub-v2.cc              # SB simulation engine
│   ├── UrbanCompMain.cc                # SA batch runner
│   ├── UrbanCompMain-v2.cc             # SB batch runner (threaded)
│   ├── UrbanCompConnectivity-SA.cc     # SA connectivity tracker
│   ├── UrbanCompConnectivity-SB.cc     # SB connectivity tracker
│   ├── SionnaSub.cc                     # Sionna RT simulation engine
│   ├── SionnaMain.cc                    # Sionna batch runner
│   ├── sionna-rt-custom.cc             # Sionna reference (single-run example)
│   ├── sionna-rt-custom-variant.cc     # Sionna with connectivity logging
│   ├── sionna-rt-connectivity-v2.cpp   # Sionna connectivity tracker
│   ├── ConnectivityBatchRunner.cc      # Sionna connectivity batch sweeper
│   ├── FobaSionnaComparison.cc        # FOBA vs Sionna 2-node loss comparison
│   ├── scene_comparison.xml           # Mitsuba 3 scene for comparison (1 building)
│   ├── run_all_connectivity_SA.py      # Python batch: SA connectivity sweeps
│   ├── run_all_connectivity-SB.py      # Python batch: SB connectivity sweeps
│   ├── layout-maker.py                 # Generate building layouts (uniform/core/corridor)
│   ├── check_oh_scale.py               # Routing overhead sanity checker
│   ├── tester.py                       # Sionna scene file validator
│   ├── sbatch_sionna_array.sh          # SLURM array job for Sionna
│   ├── sionna-2.sh                     # SLURM job template (alternative)
│   ├── scene_wifi24.xml                # Mitsuba 3 scene for Sionna (2.4 GHz)
│   ├── scene_wifi253.xml               # Mitsuba 3 scene for Sionna (variant)
│   ├── UrbanCompLayout.csv             # Node positions + building bounds
│   ├── buildingLayout.csv              # Minimal 4-building test layout
│   ├── nodes.csv                       # Node coordinate set
│   ├── connectivityM-rt.csv            # Connectivity matrix (ray-tracing)
│   ├── mobility-rt.csv                 # Mobility traces
│   └── CMakeLists.txt                  # ns-3 scratch auto-build rules
│
├── src/                                 # ns-3 spectrum module patches (copy to ns-3/src/spectrum/)
│   ├── sionna-rt-channel-model.cc/.h   # Sionna RT channel model
│   ├── sionna-rt-spectrum-propagation-loss-model.cc/.h  # Sionna spectrum loss
│   ├── half-duplex-ideal-phy.cc/.h     # PHY layer patch
│   ├── phy-entity.cc                   # PHY entity patch
│   ├── phy-entity-0.cc                 # PHY entity (original)
│   └── CMakeLists.txt                  # Module build rules (with Sionna detection)
│
├── plot_scripts/                        # Visualization tools
│   ├── PlotUrbanComp.py                # SA: PDR, EED, Goodput, Overhead
│   ├── PlotUrbanComp-v2.py            # SB: same metrics
│   ├── PlotUrbanComp-sionna.py        # Sionna: same metrics
│   ├── PlotDropData.py                # Packet drop breakdown by reason
│   ├── PlotRoutingMetrics.py          # SA vs SB side-by-side routing KPIs
│   ├── PlotConnectivity.py            # Animated connectivity graphs
│   ├── PlotRtConnectivity.py          # Ray-tracing path-loss visualization
│   ├── PlotUrbanCompConnectivity.py   # Static path-loss heatmap
│   ├── PlotSA_CompactDashboard.py     # SA connectivity dashboard (7 metrics)
│   ├── PlotSB_CompactDashboard.py     # SB connectivity dashboard + radar
│   └── PlotFobaSionnaComparison.py   # FOBA vs Sionna comparison plots
│
├── graph-metric-tools/                  # NetworkX graph analysis
│   ├── graph-metric-exta.py            # Compute centrality, clustering, paths
│   ├── foba-conmat.csv                 # FOBA adjacency matrix
│   └── itu-conmat.csv                  # ITU-R adjacency matrix
│
├── plots/                               # Generated outputs (gitignored)
│   ├── scenario-A/                     # SA metric plots
│   ├── scenario-B/                     # SB metric plots
│   ├── sionna/                         # Sionna metric plots
│   ├── drops/                          # Packet drop plots
│   ├── routing/                        # Routing comparison plots
│   ├── dashboard-SA/                   # SA connectivity dashboards
│   └── dashboard-SB/                   # SB connectivity dashboards
│
├── building_layout.csv                  # Default building positions (1500×1500 m area)
├── analyze_sionna_logs.py              # Classify HPC job logs (complete/timeout/error)
├── deploy-scp-run.bat                  # Windows: SCP + SSH deploy to HPC
├── deploy-scp-run.ps1                  # PowerShell: SCP + SSH deploy to HPC
├── .gitignore
└── README.md
```

---

## Simulation Output

Results are organized hierarchically per scenario:

```
UrbanRaCompDir/
├── Epoch_0/
│   ├── FOBA/
│   │   ├── aodv/numNodes/10/
│   │   ├── olsr/numNodes/10/
│   │   └── dsdv/numNodes/10/
│   ├── Friis/
│   ├── TwoRayGround/
│   └── ITUR/
├── Epoch_1/
└── ...
```

Each leaf folder contains:
- `flow_information.csv` — per-flow TX/RX packets, delay, sequence numbers
- `node_target_mapping.csv` — node ID ↔ IP ↔ target mapping
- `drop_data.csv` — PHY/MAC drop reasons
- `network_level_traffic.csv` — aggregate route signalization vs app traffic
- `node_level_traffic.csv` — per-node TX/RX/drop counters
- `routing_perf.csv` — RREQ/RREP timing, hop counts (AODV only)
- `simulation_time.csv` — wall-clock execution time

**Storage**: ~370 MB for a full SA run (1,200 simulations).

---

## Citation

If you use this code, please cite:

> **Reference paper**:
@inproceedings{le2025buildings,
  title={When Buildings Blur the Lines: Revealing the Hidden Performance Equivalences in MANET Routing Protocols},
  author={Le Dirach, Hugo and Boyer, Marc and Lochin, Emmanuel},
  booktitle={2025 23rd International Symposium on Network Computing and Applications (NCA)},
  pages={123--130},
  year={2025},
  organization={IEEE}
}


## Author

**Hugo Le Dirach**
Office National d'Étude et de Recherche Aérospatiale (ONERA)

## License

```
Copyright (c) 2024 Office National d'Étude et de Recherche Aérospatiale (ONERA)
SPDX-License-Identifier: GPL-2.0-only
```
# When Buildings Blur the Lines

A network simulation framework for comparing propagation loss models and routing protocols in urban environments using ns-3.

[![ns-3 Version](https://img.shields.io/badge/ns--3-3.44-blue)](https://www.nsnam.org/)
[![Language](https://img.shields.io/badge/language-C++-orange)](https://isocpp.org/)
[![License](https://img.shields.io/badge/license-GPL--2.0-green)](LICENSE)

## Overview

This repository contains the source code and tools to reproduce the simulation results presented in our research paper (reference below). The project evaluates the performance of different propagation loss models combined with various routing protocols in urban scenarios using the ns-3 network simulator.

### Key Features

- **4 Propagation Loss Models**: FOBA, Friis, Two-Ray Ground, ITU-R
- **3 Routing Protocols**: AODV, OLSR, DSDV
- **Scalability Testing**: Variable node counts (10-100 nodes)
- **Statistical Validation**: 10-epoch simulation runs
- **Automated Analysis**: Python-based plotting utilities

## Prerequisites

Before getting started, ensure you have:

1. **ns-3 Simulator** (version 3.44 or compatible)
   - Download from [nsnam.org](https://www.nsnam.org/)
   - Follow the [installation guide](https://www.nsnam.org/docs/installation.html)

2. **FOBA Loss Model** (Required custom module)
   - Install from: [First_Order_Buildings_Aware_PathLoss](https://github.com/Petitnuagedk/First_Order_Buildings_Aware_PathLoss)
   - Follow the setup instructions in that repository

3. **Python 3** (for plotting scripts)
   - Required packages: `numpy`, `matplotlib`, `pandas`
   - Install via: `pip install numpy matplotlib pandas`

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/Petitnuagedk/When-Buildings-Blur-the-Lines
   cd When-Buildings-Blur-the-Lines
   ```

2. Copy simulation files to ns-3:
   ```bash
   cp scratch/* /path/to/ns-3/scratch/
   ```

3. Copy plotting scripts:
   ```bash
   cp plot_scripts/* /path/to/ns-3/
   ```

## Usage

### Running Simulations

Execute the main simulation script from your ns-3 directory:

```bash
./ns3 run scratch/UrbanCompMain.cc
```

#### Simulation Parameters

The script runs a comprehensive test suite:
- **Epochs**: 10 repetitions for statistical significance
- **Loss Models**: 4 models (FOBA, Friis, Two-Ray Ground, ITU-R)
- **Routing Protocols**: 3 protocols (AODV, OLSR, DSDV)
- **Node Counts**: 10 configurations (10, 20, 30, ..., 100 nodes)
- **Total Runs**: 1,200 simulations (10 × 4 × 3 × 10)

**⚠️ Note**: Complete execution may take several hours depending on your system specifications. Progress will be displayed in the terminal.

### Output Structure

Results are organized hierarchically:

```
UrbanCompDir/
├── Epoch_0/
│   ├── FOBA/
│   │   ├── aodv/
│   │   │   ├── 10_nodes/
│   │   │   ├── 20_nodes/
│   │   │   └── ...
│   │   ├── olsr/
│   │   └── dsdv/
│   ├── Friis/
│   ├── TwoRayGround/
│   └── ITUR/
├── Epoch_1/
└── ...
```

**Total Storage**: Approximately 370 MB

### Generating Plots

From the ns-3 base directory, run:

```bash
python3 PlotUrbanComp.py
```

This will:
1. Parse all simulation results in `UrbanCompDir/`
2. Generate comparative plots
3. Save outputs to `plots/` directory

**Pre-computed plots** are included in this repository for reference.

## Project Structure

```
.
├── scratch/              # ns-3 simulation scripts
│   └── UrbanCompMain.cc # Main simulation entry point
├── plot_scripts/         # Analysis and visualization tools
│   └── PlotUrbanComp.py # Plotting utility
├── plots/               # Pre-computed visualization results
└── README.md            # This file
```
In the plots/ directory, you can find the computational cost in terms of execution time for each combination of Routin algorithm/number of nodes/loss model. There are some artefacts for AODV due to simulation errors that have been set aside.

## Citation

If you use this code in your research, please cite:

**Reference paper**: *To be added upon publication*

## Contributing

We welcome contributions! Please feel free to:
- Report bugs via GitHub Issues
- Submit pull requests for improvements
- Suggest new features or analysis methods

## Author

**Hugo Le Dirach**  
Office National d'Étude et de Recherche Aérospatiale (ONERA)

## License

```
Copyright (c) 2024 Office National d'Étude et de Recherche Aérospatiale (ONERA)
SPDX-License-Identifier: GPL-2.0-only
```

This project is licensed under the GNU General Public License v2.0. See [LICENSE](LICENSE) for details.

## Acknowledgments

This work was conducted at ONERA, the French Aerospace Lab. We thank the ns-3 community for their excellent simulation framework and documentation.

---

**Version**: 1.0  
**Tested with**: ns-3.44  
**Last Updated**: 2025

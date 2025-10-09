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
In the plot directory, you can find the computational cost in terms of execution time for each combination of Routin algorithm/number of nodes/loss model. There are some artefacts for AODV due to simulation errors that have been set aside.

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

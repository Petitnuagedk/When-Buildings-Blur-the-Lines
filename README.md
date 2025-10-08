# When Buildings Blur the Lines, source code

**Language:** C++
**Library/Framework:** ns-3 (Network Simulator 3)

This version (1.0) has been tested with ns-3.44 release.

## Fair use

If you use this model to in your work (paper, report, thesis, ...), please cite the paper detailed in the "Reference" section.
Thank you :)

## Overview


### Key Features

## Usage

*Option 1*: Download ZIP

1. Download this repository as ZIP
2. Extract the archive
3. Copy the first-order-buildings-aware-path-loss folder to your NS-3 contrib directory:
   
`cp -r first-order-buildings-aware-path-loss/ /path/to/ns-3.xx/contrib/`

*Option 2*: Clone Repository

`cd /path/to/ns-3.xx/contrib/
git clone https://github.com/Petitnuagedk/First_Order_Buildings_Aware_PathLoss.git first-order-buildings-aware-path-loss`

In your code:
* write `#include <ns3/first-order-buildings-aware-propagation-loss-model.h>` along the other header files
* When setting up the channel condition use `.AddPropagationLoss("ns3::FirstOrderBuildingsAwarePropagationLossModel")`

## Notes

## Examples

## Author

Hugo Le Dirach

## Reference


## License
Copyright (c) 2024 Office National d'Etude et de Recherche Aérospatiale (ONERA)
SPDX-License-Identifier: GPL-2.0-only


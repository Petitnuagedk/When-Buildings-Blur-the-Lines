#include <iostream>
#include <filesystem>
#include <vector>
#include <cstdlib>

namespace fs = std::filesystem;

int main() {
    std::string rootDir = "UrbanRaCompDir-v2";
    std::vector<std::string> subDirs = {"numNodes"};
    std::vector<std::string> lossModel = {"FOBA", "Friis", "TwoRayGroundPropagationLossModel", "ItuR1411LosPropagationLossModel"};
    std::vector<std::string> RAs = {"aodv", "olsr", "dsdv"};
    std::vector<std::string> numNodes = {"50", "100", "150", "200", "300", "500"};
    std::vector<std::string> numSrc = {"6"};
    int numEpochs = 10; // Number of epochs

    // paths used by layout-maker script
    std::string layoutScript = "scratch/layout-maker.py";
    std::string nodesCsv     = "scratch/nodes.csv";
    std::string buildingsCsv = "scratch/building_layout.csv";

    int numSim = RAs.size() * numNodes.size() * lossModel.size() * numEpochs * numSrc.size();
    int indexSim = 0;

    try {
        // Create root directory
        fs::create_directories(rootDir);

        for (int epoch = 1; epoch <= numEpochs; ++epoch) {
            std::cout << "Starting Epoch " << epoch << "...\n";

            for (const std::string& nNodes : numNodes) {
            // generate urban layout once per epoch+node count
            int areaSize = 1500; // default
            std::string pyCmd = "python " + layoutScript
                                + " --seed " + std::to_string(epoch) // use epoch as seed for layout generation
                                + " --nodes " + nNodes
                                + " --cell 65 --bsize 35 --spacing 20 --clearance 3"
                                + " --size " + std::to_string(areaSize)
                                + " --points-out " + nodesCsv
                                + " --out " + buildingsCsv;
            std::cout << "Generating layout: " << pyCmd << std::endl;
            int pyRet = std::system(pyCmd.c_str());
            if (pyRet != 0) {
                std::cout << "Layout generation failed (return " << pyRet << ")\n";
                continue; // skip this nNodes
            }

            for (const std::string& lm : lossModel) {
                for (const std::string& RA : RAs) {
                    for (const std::string& nSrc : numSrc) {
                        fs::path dirPath = fs::path(rootDir) / ("Epoch_" + std::to_string(epoch)) / lm / RA / subDirs[0] / nNodes;
                        fs::create_directories(dirPath); // Create directories recursively

                        std::string runCmd = "./ns3 run scratch/UrbanCompSub-v2.cc -- --numNodes=" + nNodes
                                                                    + " --RA=" + RA 
                                                                    + " --lossModel=" + lm
                                                                    + " --resultPath=" + dirPath.string()
                                                                    + " --numSource=" + nSrc
                                                                    + " --Seed=" + std::to_string(epoch)
                                                                    + " --nodesFile=" + nodesCsv
                                                                    + " --buildingsFile=" + buildingsCsv;
                        std::cout << runCmd << std::endl;

                        int retCode1 = std::system(runCmd.c_str());
                        indexSim++;

                        if (retCode1 != 0) {
                            std::cout << "[ " + std::to_string(indexSim) + "/" + std::to_string(numSim) + "] Error: simulation exited with code " << retCode1 << std::endl;
                        } else {
                            std::cout << "[ " + std::to_string(indexSim) + "/" + std::to_string(numSim) + "] Simulation Completed !" << std::endl;
                        }
                    }
                }
            }
        }

            std::cout << "Epoch " << epoch << " completed.\n";
        }

        std::cout << "All epochs completed successfully.\n";

    } catch (const fs::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}

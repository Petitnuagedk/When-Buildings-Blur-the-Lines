#include <iostream>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

int main() {
    std::string rootDir = "UrbanRaComp";
    std::vector<std::string> subDirs = {"numNodes"};
    std::vector<std::string> lossModel = {"FOBA", "Friis", "TwoRayGroundPropagationLossModel", "ItuR1411LosPropagationLossModel"};//, "TwoRayGroundPropagationLossModel", "ItuR1411LosPropagationLossModel"};
    std::vector<std::string> RAs = {"aodv", "olsr", "dsdv"};
    std::vector<std::string> numNodes = {"10", "20", "30", "40", "50", "60", "70", "80", "90", "100"}; // Number of nodes in the network
    std::vector<std::string> numSrc = {"6"};
    int numEpochs = 10; // Number of epochs
    int numSim = RAs.size() * numNodes.size() * lossModel.size() * numEpochs * numSrc.size();
    int indexSim = 0;

    try {
        // Create root directory
        fs::create_directories(rootDir);

        for (int epoch = 1; epoch <= numEpochs; ++epoch) {
            std::cout << "Starting Epoch " << epoch << "...\n";

            for (const std::string& lm : lossModel) {
                for (const std::string& RA : RAs) {
                    for (const std::string& numNode : numNodes) {
                        for (const std::string& numSrc : numSrc) {  
                            fs::path dirPath = fs::path(rootDir) / ("Epoch_" + std::to_string(epoch)) / lm / RA / subDirs[0] / numNode;
                            fs::create_directories(dirPath); // Create directories recursively

                            std::string runCmd = "./ns3 run scratch/urbanRAcomp4.cc -- --numNodes=" + numNode
                                                                                + " --RA=" + RA 
                                                                                + " --lossModel=" + lm
                                                                                + " --resultPath=" + dirPath.string()
                                                                                + " --numSource=" + numSrc
                                                                                + " --Seed=" + std::to_string(epoch);
                            std::cout << runCmd << std::endl;

                            int retCode1 = std::system(runCmd.c_str());
                            indexSim++;

                            if (retCode1 != 0) {
                                std::cout << "[ " + std::to_string(indexSim) + "/" + std::to_string(numSim) + "] Error: called_program exited with code " << retCode1 << std::endl;
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

#include <iostream>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

int main() {
    // root directory where all results will be stored
    std::string rootDir = "RT-batch-Dir";

    // subfolder structure is the same as UrbanCompMain: we put the
    // varying parameter under "numNodes" so it is easy to inspect.
    std::vector<std::string> subDirs = {"numNodes"};

    // routing algorithms that we want to sweep
    std::vector<std::string> RAs = {"aodv", "olsr", "dsdv"};

    // number of nodes values to test (0..100 stepping by 10)
    std::vector<std::string> numNodes = {"10","20","30","40","50","60","70","80","90","100"};

    // we keep an epoch loop to mimic UrbanCompMain, it can be used for seeds
    int numEpochs = 10;
    int numSim = RAs.size() * numNodes.size() * numEpochs;
    int indexSim = 0;

    try {
        fs::create_directories(rootDir);

        for (int epoch = 1; epoch <= numEpochs; ++epoch) {
            std::cout << "Starting Epoch " << epoch << "...\n";

            for (const std::string& RA : RAs) {
                for (const std::string& n : numNodes) {
                    fs::path dirPath = fs::path(rootDir) / ("Epoch_" + std::to_string(epoch)) / RA / subDirs[0] / n;
                    fs::create_directories(dirPath);

                    std::string runCmd = "./ns3 run scratch/sionna-rt-connectivity.cc -- --routing=" + RA
                                         + " --numNodes=" + n
                                         + " --resultPath=" + dirPath.string()
                                         + " --Seed=" + std::to_string(epoch);

                    std::cout << runCmd << std::endl;
                    int ret = std::system(runCmd.c_str());
                    ++indexSim;

                    if (ret != 0) {
                        std::cout << "[ " << indexSim << "/" << numSim << "] Error: program exited with code " << ret << std::endl;
                    } else {
                        std::cout << "[ " << indexSim << "/" << numSim << "] Simulation Completed !" << std::endl;
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

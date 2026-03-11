#include <iostream>
#include <filesystem>
#include <vector>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <functional>
#include <atomic>

namespace fs = std::filesystem;

class ThreadPool {
public:
    explicit ThreadPool(int n) : stop_(false) {
        for (int i = 0; i < n; ++i)
            workers_.emplace_back([this] { workerLoop(); });
    }

    void enqueue(std::function<void()> task) {
        {
            std::lock_guard<std::mutex> lk(mu_);
            tasks_.push(std::move(task));
        }
        cv_.notify_one();
    }

    ~ThreadPool() {
        { std::lock_guard<std::mutex> lk(mu_); stop_ = true; }
        cv_.notify_all();
        for (auto& w : workers_) w.join();
    }

private:
    void workerLoop() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait(lk, [this] { return stop_ || !tasks_.empty(); });
                if (stop_ && tasks_.empty()) return;
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();
        }
    }

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mu_;
    std::condition_variable cv_;
    bool stop_;
};

int main() {
    std::string rootDir = "UrbanRaCompDir-v2";
    std::vector<std::string> subDirs = {"numNodes"};
    std::vector<std::string> lossModel = {"FOBA", "Friis", "TwoRayGroundPropagationLossModel", "ItuR1411LosPropagationLossModel"};
    std::vector<std::string> RAs = {"aodv", "olsr", "dsdv"};
    std::vector<std::string> numNodes = {"50", "100", "150", "200", "300", "500"};
    std::vector<std::string> numSrc = {"6"};
    int numEpochs = 10; // Number of epochs
    const int NUM_WORKERS = 12;

    // paths used by layout-maker script
    std::string layoutScript = "scratch/layout-maker.py";

    int numSim = RAs.size() * numNodes.size() * lossModel.size() * numEpochs * numSrc.size();
    std::atomic<int> indexSim(0);
    std::mutex printMu;

    try {
        // Create root directory
        fs::create_directories(rootDir);

        // ── Phase 1: generate all layouts (sequential) ──
        // Each (epoch, nNodes) gets its own files so parallel sims don't clash.
        for (int epoch = 1; epoch <= numEpochs; ++epoch) {
            for (const std::string& nNodes : numNodes) {
                int areaSize = 1500;
                std::string nodesCsv     = "scratch/nodes_e" + std::to_string(epoch) + "_n" + nNodes + ".csv";
                std::string buildingsCsv = "scratch/buildings_e" + std::to_string(epoch) + "_n" + nNodes + ".csv";

                std::string pyCmd = "python3 " + layoutScript
                                    + " --seed " + std::to_string(epoch)
                                    + " --nodes " + nNodes
                                    + " --cell 65 --bsize 35 --spacing 20 --clearance 3"
                                    + " --size " + std::to_string(areaSize)
                                    + " --points-out " + nodesCsv
                                    + " --out " + buildingsCsv;
                std::cout << "Generating layout: " << pyCmd << std::endl;
                int pyRet = std::system(pyCmd.c_str());
                if (pyRet != 0) {
                    std::cerr << "Layout generation failed (return " << pyRet << ")\n";
                }
            }
        }

        // ── Phase 2: build once, then run simulations in parallel ──
        // Pre-build so that parallel ./ns3 run calls don't race on compilation.
        std::cout << "Pre-building UrbanCompSub-v2 ...\n";
        int buildRet = std::system("./ns3 build scratch/UrbanCompSub-v2.cc");
        if (buildRet != 0) {
            std::cerr << "Build failed (return " << buildRet << "). Aborting.\n";
            return 1;
        }

        {
            ThreadPool pool(NUM_WORKERS);

            for (int epoch = 1; epoch <= numEpochs; ++epoch) {
                for (const std::string& nNodes : numNodes) {
                    std::string nodesCsv     = "scratch/nodes_e" + std::to_string(epoch) + "_n" + nNodes + ".csv";
                    std::string buildingsCsv = "scratch/buildings_e" + std::to_string(epoch) + "_n" + nNodes + ".csv";

                    if (!fs::exists(nodesCsv) || !fs::exists(buildingsCsv)) continue;

                    for (const std::string& lm : lossModel) {
                        for (const std::string& RA : RAs) {
                            for (const std::string& nSrc : numSrc) {
                                fs::path dirPath = fs::path(rootDir) / ("Epoch_" + std::to_string(epoch)) / lm / RA / subDirs[0] / nNodes;
                                fs::create_directories(dirPath);

                                std::string runCmd = "./ns3 run scratch/UrbanCompSub-v2.cc --no-build -- --numNodes=" + nNodes
                                                                            + " --RA=" + RA
                                                                            + " --lossModel=" + lm
                                                                            + " --resultPath=" + dirPath.string()
                                                                            + " --numSource=" + nSrc
                                                                            + " --Seed=" + std::to_string(epoch)
                                                                            + " --nodesFile=" + nodesCsv
                                                                            + " --buildingsFile=" + buildingsCsv;

                                pool.enqueue([runCmd, &indexSim, numSim, &printMu] {
                                    int retCode = std::system(runCmd.c_str());
                                    int idx = ++indexSim;
                                    std::lock_guard<std::mutex> lk(printMu);
                                    if (retCode != 0) {
                                        std::cout << "[" << idx << "/" << numSim << "] Error: simulation exited with code " << retCode << std::endl;
                                    } else {
                                        std::cout << "[" << idx << "/" << numSim << "] Simulation Completed!" << std::endl;
                                    }
                                });
                            }
                        }
                    }
                }
            }
            // ThreadPool destructor waits for all tasks to finish
        }

        std::cout << "All simulations completed.\n";

    } catch (const fs::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}

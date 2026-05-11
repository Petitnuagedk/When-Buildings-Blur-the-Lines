#include <iostream>
#include <filesystem>
#include <vector>
#include <cmath>
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
        for (auto& worker : workers_) worker.join();
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
    std::string rootDir = "UrbanRaCompDir-SA";
    std::vector<std::string> subDirs = {"numNodes"};
    std::vector<std::string> lossModel = {"FOBA", "Friis", "TwoRayGroundPropagationLossModel", "ItuR1411LosPropagationLossModel"};//, "TwoRayGroundPropagationLossModel", "ItuR1411LosPropagationLossModel"};
    std::vector<std::string> RAs = {"aodv", "olsr", "dsdv"};
    std::vector<std::string> numNodes = {"10", "20", "30", "40", "50", "60", "70", "80", "90", "100"}; // Number of nodes in the network
    int numEpochs = 1; // Number of epochs
    const int NUM_WORKERS = 12;
    int numSim = RAs.size() * numNodes.size() * lossModel.size() * numEpochs;
    std::atomic<int> indexSim(0);
    std::mutex printMu;

    try {
        // Create root directory
        fs::create_directories(rootDir);

        std::cout << "Pre-building UrbanCompSub ...\n";
        int buildRet = std::system("./ns3 build scratch/UrbanCompSub.cc");
        if (buildRet != 0) {
            std::cerr << "Build failed (return " << buildRet << "). Aborting.\n";
            return 1;
        }

        {
            ThreadPool pool(NUM_WORKERS);

            for (int epoch = 1; epoch <= numEpochs; ++epoch) {
                std::cout << "Queueing Epoch " << epoch << "...\n";

                for (const std::string& lm : lossModel) {
                    for (const std::string& RA : RAs) {
                        for (const std::string& numNode : numNodes) {
                            std::string numSrc = std::to_string(std::max(1, (int)std::round(std::stoi(numNode) * 0.3)));

                            fs::path dirPath = fs::path(rootDir) / ("Epoch_" + std::to_string(epoch)) / lm / RA / subDirs[0] / numNode;
                            fs::create_directories(dirPath); // Create directories recursively

                            std::string runCmd = "./ns3 run scratch/UrbanCompSub.cc --no-build -- --numNodes=" + numNode
                                                                                + " --RA=" + RA
                                                                                + " --lossModel=" + lm
                                                                                + " --resultPath=" + dirPath.string()
                                                                                + " --numSource=" + numSrc
                                                                                + " --Seed=" + std::to_string(epoch);

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
                std::cout << "Epoch " << epoch << " queued.\n";
            }
        }

        std::cout << "All simulations completed.\n";

    } catch (const fs::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}

#include <cmath>
#include <random>
#include <vector> 
#include <iostream>
#include <utility>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <ctime>
#include <string>
#include <deque>
#include <tuple>
#include <cctype>


//--- Core (Ptr, Time, Creatobject...) ---
#include "ns3/core-module.h"
//--- application (UDP) ---
#include "ns3/applications-module.h"
//--- Communication (Netdevice, Layer3, Layer4) ---
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/udp-client-server-helper.h"
//--- trace output ---
#include "ns3/trace-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/flow-monitor-helper.h"
//--- Routing ---
#include "ns3/dsr-module.h"
#include "ns3/aodv-module.h"
#include "ns3/dsdv-helper.h"
#include "ns3/olsr-helper.h"

//--- mobility (helper) ---
#include "ns3/mobility-module.h"
#include "ns3/building.h"
//---Other---


using namespace ns3;

//NS_LOG_COMPONENT_DEFINE("LOG_LEVEL_DEBUG");

// Global connectivity state
std::vector<std::vector<uint32_t>> receivedProbes;
std::vector<std::vector<std::vector<uint32_t>>> TimereceivedProbes;
std::vector<std::vector<std::tuple<double, double, double>>> nodePositions;

// Global propagation loss model pointer (used for direct loss-based connectivity check)
Ptr<PropagationLossModel> globalLossModel;

void RecordNodePositions(NodeContainer nodes) {
    std::vector<std::tuple<double, double, double>> currentPositions;
    for (NodeContainer::Iterator i = nodes.Begin(); i != nodes.End(); ++i) {
        Ptr<Node> node = *i;
        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();
        currentPositions.emplace_back(pos.x, pos.y, pos.z);
    }
    nodePositions.push_back(currentPositions);
}

void SchedulePositionRecording(NodeContainer nodes) {
    RecordNodePositions(nodes);
    Simulator::Schedule(Seconds(1.0), &SchedulePositionRecording, nodes);
}

// (ReceiveProbe removed: connectivity is now determined by direct loss computation)

/**
 * MeasureConnectivity – called every second.
 * For each ordered pair (i, j) with i != j, the received power is computed
 * from the configured propagation loss model.  If it meets or exceeds
 * rxThresholdDbm the link is considered up (1), otherwise down (0).
 * The resulting matrix is appended to TimereceivedProbes.
 */
void MeasureConnectivity(NodeContainer nodes, double txPowerDbm, double rxThresholdDbm)
{
    uint32_t n = nodes.GetN();

    // Recompute matrix from scratch
    for (auto& row : receivedProbes) {
        std::fill(row.begin(), row.end(), 0);
    }

    for (uint32_t i = 0; i < n; ++i) {
        Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel>();
        for (uint32_t j = 0; j < n; ++j) {
            if (i == j) continue;
            Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel>();
            double rxPower = globalLossModel->CalcRxPower(txPowerDbm, mobI, mobJ);
            if (rxPower >= rxThresholdDbm) {
                receivedProbes[i][j] = 1;
            }
        }
    }

    // Snapshot current matrix
    TimereceivedProbes.push_back(receivedProbes);

    Simulator::Schedule(Seconds(1.0), &MeasureConnectivity, nodes, txPowerDbm, rxThresholdDbm);
}

// Read a CSV file of 2 columns (x,y) and append into `points`.
// Expected header: x,y (case-insensitive).
bool readNodesCsv(const std::string& filename, std::vector<std::pair<double, double>>& points) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open nodes file: " << filename << std::endl;
        return false;
    }
    std::string line;
    if (!std::getline(file, line)) {
        std::cerr << "Nodes file is empty: " << filename << std::endl;
        return false;
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        if (!std::getline(ss, x_str, ',') || !std::getline(ss, y_str, ',')) {
            continue;
        }
        try {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            points.emplace_back(x, y);
        } catch (...) {
            continue;
        }
    }
    file.close();
    return true;
}

// Read a CSV file describing building rectangles.
// Supports headers:
//   xmin,xmax,ymin,ymax,zmin,zmax  (common in generated layout files)
//   x,y,xmin,xmax,ymin,ymax        (older UrbanCompLayout.csv format)
// For the first format, it returns rectangles as [xmin, xmax, ymin, ymax].
// For the second, it uses the last 4 columns.
bool readBuildingsCsv(const std::string& filename, std::vector<std::vector<double>>& rectangles) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open buildings file: " << filename << std::endl;
        return false;
    }
    std::string header;
    if (!std::getline(file, header)) {
        std::cerr << "Buildings file is empty: " << filename << std::endl;
        return false;
    }

    // Determine expected format based on header tokens.
    std::vector<std::string> cols;
    {
        std::stringstream ss(header);
        std::string token;
        while (std::getline(ss, token, ',')) {
            cols.push_back(token);
        }
    }

    bool headerLooksLikeLayout = false;
    bool headerLooksLikeBuildings = false;
    for (auto& c : cols) {
        std::string lc = c;
        std::transform(lc.begin(), lc.end(), lc.begin(), ::tolower);
        if (lc == "x" || lc == "y") {
            headerLooksLikeLayout = true;
        }
        if (lc == "xmin" || lc == "xmax" || lc == "ymin" || lc == "ymax") {
            headerLooksLikeBuildings = true;
        }
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        std::vector<double> values;
        while (std::getline(ss, item, ',')) {
            if (item == "Nan" || item == "nan") {
                // skip
            } else {
                try {
                    values.push_back(std::stod(item));
                } catch (...) {
                    // ignore parse errors
                }
            }
        }

        if (headerLooksLikeLayout && values.size() >= 6) {
            // Old format: x,y,xmin,xmax,ymin,ymax
            rectangles.push_back({values[2], values[3], values[4], values[5]});
        } else if (headerLooksLikeBuildings && values.size() >= 6) {
            // New format: xmin,xmax,ymin,ymax,zmin,zmax
            rectangles.push_back({values[0], values[1], values[2], values[3]});
        } else if (values.size() == 4) {
            // A fallback: assume xmin,xmax,ymin,ymax
            rectangles.push_back({values[0], values[1], values[2], values[3]});
        } else {
            // ignore malformed/unknown rows
            continue;
        }
    }
    file.close();
    return true;
}


void readUrbanData(const std::string& filename,
                   std::vector<std::pair<double, double>>& points,
                   std::vector<std::vector<double>>& rectangles)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << filename << std::endl;
        return;
    }
    std::string line;
    // Skip header line
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        std::vector<double> values;
        while (std::getline(ss, item, ','))
        {
            if (item != "Nan")
            {
                values.push_back(std::stod(item));
            }
        }
        if (values.size() == 6)
        {
            points.emplace_back(values[0], values[1]);
            rectangles.push_back({values[2], values[3], values[4], values[5]});
        } else if (values.size() == 2)
        {
            points.emplace_back(values[0], values[1]);
        }
        else
        {
            std::cerr << "Invalid line (expected 6 values): " << line << std::endl;
        }
    }
    file.close();
}


void makebuildings(const std::vector<std::vector<double>>& rectangles, int numBuildings) {
    std::vector<Ptr<Building>> LBuildings;
    int count;
    if (numBuildings==-1)
    {
        count = rectangles.size();
    } else {
        count = std::min(numBuildings, static_cast<int>(rectangles.size()));
    }
    

    for (int i = 0; i < count; ++i) {
        const auto& rect = rectangles[i];

        double xMin = rect[0];
        double xMax = rect[1];
        double yMin = rect[2];
        double yMax = rect[3];
        std::string type = "concrete with windows";

        Ptr<Building> currBuilding = CreateObject<Building>();
        currBuilding->SetBoundaries(Box(xMin, xMax, yMin, yMax, 0, 25));
        currBuilding->SetBuildingType(Building::Residential);

        if (type == "concrete with windows") {
            currBuilding->SetExtWallsType(Building::ConcreteWithWindows);
        } else if (type == "concrete") {
            currBuilding->SetExtWallsType(Building::ConcreteWithoutWindows);
        } else if (type == "wood") {
            currBuilding->SetExtWallsType(Building::Wood);
        } else if (type == "stone") {
            currBuilding->SetExtWallsType(Building::StoneBlocks);
        } else {
            currBuilding->SetExtWallsType(Building::ConcreteWithWindows); // Default case
        }

        LBuildings.push_back(currBuilding);
    }

    return;
}

void printPointsInsideBuildings(const std::vector<std::pair<double, double>>& points,
                                const std::vector<std::vector<double>>& bounds) {
    for (const auto& point : points) {
        for (const auto& b : bounds) {
            if (b.size() == 4) {
                double x = point.first;
                double y = point.second;

                double xmin = b[0];
                double xmax = b[1];
                double ymin = b[2];
                double ymax = b[3];

                if ((x >= xmin) && (x <= xmax) && (y >= ymin) && (y <= ymax)) {
                    std::cout << "Point inside building: (" << x << ", " << y << ") "
                    << "in bounds [" << xmin << ", " << xmax << ", " << ymin << ", " << ymax << "]\n";
                    break; // Found one match, no need to keep checking
                }
            }
        }
    }
}

// Mobility logging file (initialized in main so it can live inside the results folder)
std::ofstream mobilityLog;

void
printmob(NodeContainer nodes)
{
    double current_time = ns3::Simulator::Now().GetSeconds();

    for (NodeContainer::Iterator i = nodes.Begin(); i != nodes.End(); ++i)
    {
        Ptr<Node> node = *i;
        Ptr<MobilityModel> mobility1 = node->GetObject<MobilityModel>();

        Vector pos = mobility1->GetPosition();
        mobilityLog << current_time << "," << node->GetId() << "," << pos.x << "," << pos.y << "," << pos.z << std::endl;
    }

    Simulator::Schedule(Seconds(0.5),
                        &printmob,
                        nodes);
}

int main (int argc, char *argv[])
{   
    std::string RA = "aodv";
    std::string proto = "UDP";
    int numNodes = 60;
    int numBuildings = -1;
    double ratioSource = 0.1; // % of source per Node in the network. number of source is the same as number of sinks
    double transmissionRate = 50;
    std::string resultPath = "results-con";
    bool verbose = false;
    std::string lossModel = "FOBA"; //FOBA Friis Nakagami LogDistancePropagationLossModel TwoRayGroundPropagationLossModel
    double txPowerDbm    = 16.0206; // Transmit power (dBm) used for loss-based link check
    double rxThresholdDbm = -82.0;  // Receiver sensitivity (dBm); link is up if rxPower >= threshold

    // Layout file selection (buildings/nodes)
    std::string layoutDir = "scratch";
    int layoutEpoch = 1;
    bool useEpochLayoutFiles = true;
    std::string legacyLayoutFile = "scratch/UrbanCompLayout.csv";

    CommandLine cmd;
    cmd.AddValue("RA", "Routing algorithm", RA);
    cmd.AddValue("numNodes", "Number of node to be created (used to select layout files)", numNodes);
    cmd.AddValue("numBuildings", "Number of buildings to be created", numBuildings);
    cmd.AddValue("ratioSource", "Number of Source node per node (min : 0, max : 1)", ratioSource);
    cmd.AddValue("transmissionRate", "Transmission rate (kbps)", transmissionRate);
    cmd.AddValue("resultPath", "Path to store results", resultPath);
    cmd.AddValue("lossModel", "Loss model to use", lossModel);
    cmd.AddValue("txPowerDbm", "Transmit power in dBm for loss-based connectivity check", txPowerDbm);
    cmd.AddValue("rxThresholdDbm", "Receiver sensitivity threshold in dBm (link up if rxPower >= threshold)", rxThresholdDbm);
    cmd.AddValue("layoutDir", "Directory containing layout files (nodes/buildings)", layoutDir);
    cmd.AddValue("epoch", "Layout epoch (used with layoutDir)", layoutEpoch);
    cmd.AddValue("useEpochLayoutFiles", "Use epoch/node layout files instead of legacy single layout file", useEpochLayoutFiles);
    cmd.AddValue("legacyLayoutFile", "Legacy layout file path (old format)", legacyLayoutFile);

    cmd.Parse (argc, argv);

    double EndTime = 300; // To modify to 3600 (s)

    // Load layout data
    std::vector<std::pair<double, double>> points;
    std::vector<std::vector<double>> rectangles;

    int layoutNumNodes = numNodes; // used to select the correct layout files

    std::string buildingsFile;
    std::string nodesFile;
    if (useEpochLayoutFiles) {
        buildingsFile = layoutDir + "/buildings_e" + std::to_string(layoutEpoch) + "_n" + std::to_string(layoutNumNodes) + ".csv";
        nodesFile = layoutDir + "/nodes_e" + std::to_string(layoutEpoch) + "_n" + std::to_string(layoutNumNodes) + ".csv";

        if (!readBuildingsCsv(buildingsFile, rectangles)) {
            std::cerr << "Falling back to legacy layout file due to missing buildings file." << std::endl;
            readBuildingsCsv(legacyLayoutFile, rectangles);
        }
        if (!readNodesCsv(nodesFile, points)) {
            std::cerr << "Failed to read nodes file; falling back to legacy layout file for node points." << std::endl;
            // Legacy file may contain both points and rectangles
            readUrbanData(legacyLayoutFile, points, rectangles);
        }
    } else {
        readUrbanData(legacyLayoutFile, points, rectangles);
    }

    // Use actual number of points from the nodes file as the simulation node count.
    if (!points.empty()) {
        numNodes = static_cast<int>(points.size());
    }

    double cleanup_time = 10;

    // If the user forced a higher numNodes, but the nodes file has fewer points, reduce accordingly.
    if (!points.empty() && points.size() < static_cast<size_t>(numNodes)) {
        std::cerr << "Warning: requested numNodes=" << numNodes << " but only " << points.size() << " points are available. "
                  << "Reducing numNodes to " << points.size() << ".\n";
        numNodes = static_cast<int>(points.size());
    }

    // Build a results directory that encodes key simulation parameters.
    std::string simTag = "conn_" + RA + "_" + lossModel + "_e" + std::to_string(layoutEpoch) + "_n" + std::to_string(numNodes);
    std::filesystem::path outDir = std::filesystem::path(resultPath) / simTag;
    std::filesystem::create_directories(outDir);

    // Initialize output files inside the parameterized output directory.
    std::string connectivityFilePath = (outDir / "connectivity_matrices.csv").string();
    std::string positionsFilePath = (outDir / "node_positions.csv").string();
    std::string mobilityLogPath = (outDir / "mobility.csv").string();

    // Re-open mobility log in append mode (so multiple runs can append without truncating).
    if (mobilityLog.is_open()) {
        mobilityLog.close();
    }
    mobilityLog.open(mobilityLogPath, std::ios::app);
    if (!mobilityLog) {
        std::cerr << "Failed to open mobility log file: " << mobilityLogPath << std::endl;
    }

    NodeContainer nodes;
    nodes.Create(numNodes);

    if (!points.empty() && !rectangles.empty() && verbose) {
        std::cout << "First point: (" << points[0].first << ", " << points[0].second << ")\n";
        std::cout << "First rectangles: [" << rectangles[0][0] << ", " << rectangles[0][1] << ", "
                  << rectangles[0][2] << ", " << rectangles[0][3] << "]\n";
    }

    printPointsInsideBuildings(points, rectangles);
    makebuildings(rectangles, numBuildings);

    float xMaxBound, xMinBound, yMaxBound, yMinBound;
        xMaxBound = 750;
        xMinBound = -750;
        yMaxBound = 750;
        yMinBound = -750;

    int Seed = 42; // your chosen identifier
    std::mt19937 rng(Seed); // deterministic random number generator

    // If we don't have enough layout points, fill remaining positions with random points
    if (points.size() < static_cast<size_t>(numNodes)) {
        std::uniform_real_distribution<double> distX(xMinBound, xMaxBound);
        std::uniform_real_distribution<double> distY(yMinBound, yMaxBound);
        while (points.size() < static_cast<size_t>(numNodes)) {
            points.emplace_back(distX(rng), distY(rng));
        }
    }

    std::shuffle(points.begin(), points.end(), rng);

    size_t idx = 0;
    for (NodeContainer::Iterator i = nodes.Begin(); i != nodes.End(); ++i, ++idx)
    {
        Ptr<Node> node = *i;
    
        // Mobility
        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::RandomWalk2dOutdoorMobilityModel",
                                    "Mode", StringValue("Time"),
                                    "Time", TimeValue(Seconds(20.0)),
                                    "Speed", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
                                    "Bounds", RectangleValue(Rectangle(xMinBound, xMaxBound, yMinBound, yMaxBound)));
        mobility.Install(node);
    
        // Extract shuffled position
        double x = points[idx].first;
        double y = points[idx].second;
        double z = 2.0;
    
        // Bounds checking (optional)
        if (!(xMinBound < x) || !(x < xMaxBound))
            std::cout << "x not inside the bounds (x,y): " << x << " " << y << std::endl;
        if (!(yMinBound < y) || !(y < yMaxBound))
            std::cout << "y not inside the bounds (x,y): " << x << " " << y << std::endl;
    
        // Assign position
        Ptr<MobilityModel> mobility1 = node->GetObject<MobilityModel>();
        mobility1->SetPosition(ns3::Vector(x, y, z));
    }
    
    

    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel;

    // Map short names to full ns3 TypeIds
    std::string lossTypeId;
    if      (lossModel == "FOBA")                               lossTypeId = "ns3::FirstOrderBuildingsAwarePropagationLossModel";
    else if (lossModel == "Friis")                              lossTypeId = "ns3::FriisPropagationLossModel";
    else if (lossModel == "ItuR1411LosPropagationLossModel")    lossTypeId = "ns3::ItuR1411LosPropagationLossModel";
    else if (lossModel == "LogDistancePropagationLossModel")    lossTypeId = "ns3::LogDistancePropagationLossModel";
    else if (lossModel == "TwoRayGroundPropagationLossModel")   lossTypeId = "ns3::TwoRayGroundPropagationLossModel";
    else                                                        lossTypeId = "ns3::FriisPropagationLossModel";

    // Use ObjectFactory (string-based) so no concrete class headers are needed.
    // globalLossModel is used by MeasureConnectivity() for direct CalcRxPower() calls.
    {
        ObjectFactory factory;
        factory.SetTypeId(lossTypeId);
        globalLossModel = factory.Create()->GetObject<PropagationLossModel>();
    }
    wifiChannel.AddPropagationLoss(lossTypeId);

    // WiFi device installation and IP stack are not needed:
    // MeasureConnectivity() calls CalcRxPower() directly on MobilityModel pointers.

    receivedProbes.resize(nodes.GetN(), std::vector<uint32_t>(nodes.GetN(), 0));
    TimereceivedProbes.clear();
    nodePositions.clear();

    Simulator::Schedule(Seconds(2.0), &SchedulePositionRecording, nodes);


    // Schedule direct loss-based connectivity measurement every second
    Simulator::Schedule(Seconds(2.0), &MeasureConnectivity, nodes, txPowerDbm, rxThresholdDbm);


    if (verbose)
    {
        Simulator::Schedule(Seconds(0.5),
            &printmob,
            nodes);
    }
    

    if (verbose)
    {
        std::cout << "Simulation starting..." << std::endl;
    }


    Simulator::Stop(Seconds(EndTime+cleanup_time));
    Simulator::Run ();


    Simulator::Destroy ();

    std::ofstream connectivityFile(connectivityFilePath);
    for (const auto& matrix : TimereceivedProbes) {
        for (const auto& row : matrix) {
            for (const auto& value : row) {
                connectivityFile << value << ",";
            }
            connectivityFile << "\n";
        }
        connectivityFile << "\n";
    }
    connectivityFile.close();

    std::ofstream positionsFile(positionsFilePath);
    for (const auto& frame : nodePositions) {
        for (const auto& pos : frame) {
            positionsFile << std::get<0>(pos) << "," << std::get<1>(pos) << "," << std::get<2>(pos) << "\n";
        }
        positionsFile << "\n";
    }
    positionsFile.close();

    if (verbose)
    {
        std::cout << "Random2Dwalk simulation in urban setting with " + lossModel + " ended successfully" << "\n";
    }
    
}






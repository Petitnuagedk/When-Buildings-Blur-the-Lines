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

// Global vector to track received probe counts
std::vector<std::vector<uint32_t>> receivedProbes;
std::vector<std::vector<std::vector<uint32_t>>> TimereceivedProbes;
std::vector<std::vector<std::tuple<double, double, double>>> nodePositions;
Ipv4InterfaceContainer globalInterfaces;

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

void ReceiveProbe(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
        Ipv4Address senderAddress = address.GetIpv4();

        uint32_t senderId = -1;
        for (uint32_t i = 0; i < globalInterfaces.GetN(); ++i)
        {
            if (globalInterfaces.GetAddress(i) == senderAddress)
            {
                senderId = i;
                break;
            }
        }

        if (senderId != (uint32_t)-1)
        {
            Ptr<Node> receiver = socket->GetNode();
            uint32_t receiverId = receiver->GetId();
            receivedProbes[senderId][receiverId] = 1;
        }
    }

}

//int count = 0;
void SendProbes(Ptr<Node> sender, Ipv4InterfaceContainer interfaces) {
    uint32_t senderId = sender->GetId();

    if (senderId==0)
    {
        //std::cout << count++ << "\n";
        // Append the current connectivity matrix to TimereceivedProbes
        TimereceivedProbes.push_back(receivedProbes);
    }

    // Reset receivedProbes for the current time frame
    for (auto& row : receivedProbes) {
        std::fill(row.begin(), row.end(), 0);
    }

    for (uint32_t j = 0; j < interfaces.GetN(); ++j) {
        if (senderId == j) continue;  // skip self

        Ptr<Socket> sendSocket = Socket::CreateSocket(sender, UdpSocketFactory::GetTypeId());
        InetSocketAddress dest = InetSocketAddress(interfaces.GetAddress(j), 9999);
        sendSocket->Connect(dest);

        Ptr<Packet> p = Create<Packet>((uint8_t*)&senderId, sizeof(senderId));
        sendSocket->Send(p);
        sendSocket->Close();
    }

    // Schedule the next probe round
    Simulator::Schedule(Seconds(1.0), &SendProbes, sender, interfaces);

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
        //std::cout << "size : " << values.size() << std::endl; 
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

std::ofstream mobilityLog("mobility.csv", std::ios::app); // Open in append mode

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
    double ratioSource = 0.1; // % of source per Node in the network. number of source is the same as numebr of sink
    double transmissionRate = 50;
    std::string resultPath = "";
    bool verbose = false;
    std::string lossModel = "FOBA"; //FOBA Friis Nakagami LogDistancePropagationLossModel TwoRayGroundPropagationLossModel

    CommandLine cmd;
    cmd.AddValue("RA", "Routing algorithm", RA);
    cmd.AddValue("numNodes", "Number of node to be created", numNodes);
    cmd.AddValue("numBuildings", "Number of buildings to be created", numBuildings);
    cmd.AddValue("ratioSource", "Number of Source node per node (min : 0, max : 1)", ratioSource);
    cmd.AddValue("transmissionRate", "Transmission rate (kbps)", transmissionRate);
    cmd.AddValue("resultPath", "Path to store results", resultPath);
    cmd.AddValue("lossModel", "Loss model to use", lossModel);

    cmd.Parse (argc, argv);

    double EndTime = 300; // To modify to 3600 (s)

    // extract.csv stuff
    std::vector<std::pair<double, double>> points;
    std::vector<std::vector<double>> rectangles;

    readUrbanData("scratch/urban_data_2.csv", points, rectangles);

    double cleanup_time = 10;


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

    if (lossModel == "FOBA")
    {
        //std::cout << "setting up loss model : " << lossModel << std::endl;
        wifiChannel.AddPropagationLoss("ns3::FirstOrderBuildingsAwarePropagationLossModel");
    }
    else if (lossModel == "Friis")
    {
        //std::cout << "setting up loss model : " << lossModel << std::endl;
        wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    }
    else if (lossModel == "Nakagami")
    {
        //std::cout << "setting up loss model : " << lossModel << std::endl;
        wifiChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel");
    }
    else if (lossModel == "LogDistancePropagationLossModel")
    {
        //std::cout << "setting up loss model : " << lossModel << std::endl;
        wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    }
    else if (lossModel == "TwoRayGroundPropagationLossModel")
    {
        //std::cout << "setting up loss model : " << lossModel << std::endl;
        wifiChannel.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel");
    }
    else
    {
        //std::cout << "setting up loss model : Friis (default, check value of lossModel) " << std::endl;
        wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    }    

    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    //wifiPhy.Set("TxPowerStart", DoubleValue(18.0));  // Starting transmission power (dBm)
    //wifiPhy.Set("TxPowerEnd", DoubleValue(18.0));    // Ending transmission power (dBm)
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac"
                    //"Ssid", SsidValue(ssid),
                    //"QosSupported", BooleanValue(true)
                    //"BeaconGeneration", BooleanValue(true),
                    //"BeaconInterval", TimeValue(Seconds(0.1024))
                    );
    NetDeviceContainer devices;   // Container for network devices
    devices = wifi.Install(wifiPhy, wifiMac, nodes);

    // Assign IP addresses
    InternetStackHelper stack;
    stack.Install (nodes);
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);
    globalInterfaces = interfaces;

    receivedProbes.resize(nodes.GetN(), std::vector<uint32_t>(nodes.GetN(), 0));
    TimereceivedProbes.clear();
    nodePositions.clear();

    Simulator::Schedule(Seconds(2.0), &SchedulePositionRecording, nodes);


    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<Node> node = nodes.Get(i);
        Ptr<Socket> recvSocket = Socket::CreateSocket(node, UdpSocketFactory::GetTypeId());
        InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 9999);
        recvSocket->Bind(local);
        recvSocket->SetRecvCallback(MakeBoundCallback(&ReceiveProbe));
    }
    
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Simulator::Schedule(Seconds(2.0), &SendProbes, nodes.Get(i), interfaces);
    }


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

    std::ofstream connectivityFile("connectivity_matrices.csv");
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

    std::ofstream positionsFile("node_positions.csv");
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
    system("python mob_con.py");
    
}






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
#include <chrono>
#include "tinyxml2.h" // Include the tinyxml2 library

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
#include "ns3/log.h"
//----Tracing-----
#include "ns3/packet.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/ipv4-header.h"
#include "ns3/udp-header.h"
#include "ns3/tcp-header.h"

using namespace ns3;
using namespace tinyxml2;

NS_LOG_COMPONENT_DEFINE("LOG_LEVEL_DEBUG");

struct NetwortkLevelTraffic {
    uint32_t RouteSignalizationPacketsSent = 0;
    uint32_t RouteSignalizationPacketsReceived = 0;
    uint32_t AppPacketsSent = 0;
    uint32_t AppPacketsReceived = 0;
};

NetwortkLevelTraffic networkLevelTraffic;

struct DropData {
    int macTxDrop = 0;
    int macRxDrop = 0;
    int dropUnsupportedSettings = 0;
    int dropChannelSwitching = 0;
    int dropRxing = 0;
    int dropTxing = 0;
    int dropSleeping = 0;
    int dropPoweredOff = 0;
    int dropTruncatedTx = 0;
    int dropBusyDecodingPreamble = 0;
    int dropPreambleDetectFailure = 0;
    int dropReceptionAbortedByTx = 0;
    int dropLSigFailure = 0;
    int dropHtSigFailure = 0;
    int dropSigAFailure = 0;
    int dropSigBFailure = 0;
    int dropUSigFailure = 0;
    int dropEhtSigFailure = 0;
    int dropPreambleDetectionPacketSwitch = 0;
    int dropFrameCapturePacketSwitch = 0;
    int dropObssPdCcaReset = 0;
    int dropPpduTooLate = 0;
    int dropFiltered = 0;
    int dropDmgHeaderFailure = 0;
    int dropDmgAllocationEnded = 0;
    int dropUnknown = 0;
};
DropData dropData;

// will be used to store node information : Node, IP, TargetID, TargetIP
struct NodeInformation {
    uint32_t NodeID;
    Ipv4Address NodeIP = Ipv4Address("127.0.0.1");
    int32_t TargetID = -1;
    Ipv4Address TargetIP = Ipv4Address("127.0.0.1");
};
std::map<uint32_t, NodeInformation> nodeInfoMap;

// will be used to store node traffic information : NodeID, AppPacketsSent, AppPacketsReceived, DevicePacketsSent, DevicePacketsReceived
struct NodeLevelTraffic {
    uint32_t NodeID;
    uint32_t AppPacketsSent = 0;
    uint32_t AppPacketsReceived = 0;
    uint32_t DevicePacketsSent = 0;
    uint32_t DevicePacketsReceived = 0;
    uint32_t PacketsDroppedRerr = 0;
    uint32_t PacketsDroppedNoRoute = 0;
};
std::map<uint32_t, NodeLevelTraffic> nodeleveltrafficMap;

struct FlowInformation {
    std::tuple<Ipv4Address, Ipv4Address, uint16_t> FlowPrint = {Ipv4Address("127.0.0.1"), Ipv4Address("127.0.0.1"), 0}; // Source, Destination, Port
    uint32_t FlowID = -1;
    Ipv4Address SourceIP = Ipv4Address("127.0.0.1");
    Ipv4Address Destination = Ipv4Address("127.0.0.1");
    uint16_t TxPackets = 0;
    uint16_t RxPackets = 0;
    //uint16_t IpTxPackets = 0;
    //uint16_t AppRxPackets = 0;
    //uint16_t IpRxPackets = 0;
    double FirstTxTime = -1;
    double LastTxTime = -1;
    double FirstRxTime = -1;
    double LastRxTime = -1;
    double sumDelay = 0.0; // Average delay in seconds
    std::vector<uint64_t> seqNums = {}; // Sequence numbers of packets in the flow to avoid redudancy
};
std::map<uint32_t, FlowInformation> FlowInformationMap;

struct RoutingPerfInfo {
    std::tuple<Ipv4Address, Ipv4Address, uint16_t> RoutePrint = {Ipv4Address("127.0.0.1"), Ipv4Address("127.0.0.1"), 0}; // Source, Destination, DstSeqNo
    uint32_t RREQId;
    Ipv4Address SourceIP;
    Ipv4Address DestinationIP;
    uint16_t hopCount;
    double TimeRREQ = -1.0;
    double TimeRREP = -1.0;
    bool routeFound = false;
};
std::map<uint32_t, RoutingPerfInfo> RoutingPerfInfoMap; 

// Routing protocol ports
const uint16_t AODV_PORT = 654;
const uint16_t OLSR_PORT = 698;
const uint16_t DSDV_PORT = 269;


void PrintSimulationTime() {
    double currentTime = Simulator::Now().GetSeconds();
    std::cout << "Simulation Time: " << currentTime << " seconds" << std::endl;

    // Schedule the next call to this function after 1 second
    Simulator::Schedule(Seconds(1.0), &PrintSimulationTime);
}

// readUrbanData now accepts separate files for nodes and buildings
// nodesFilename should contain two columns (x,y) with header
// buildingsFilename should contain six columns (xmin,xmax,ymin,ymax,zmin,zmax) with header
void readUrbanData(const std::string& nodesFilename,
                   const std::string& buildingsFilename,
                   std::vector<std::pair<double, double>>& points,
                   std::vector<std::vector<double>>& rectangles)
{
    // read node positions
    std::ifstream nfile(nodesFilename);
    if (!nfile.is_open()) {
        std::cerr << "Could not open nodes file: " << nodesFilename << std::endl;
    } else {
        std::string line;
        // Skip header line
        std::getline(nfile, line);
        while (std::getline(nfile, line)) {
            std::stringstream ss(line);
            double x, y;
            // consume comma separator; variable only for extraction
            char comma;
            if (ss >> x >> comma >> y) {
                points.emplace_back(x, y);
            } else {
                std::cerr << "Invalid node line (expected x,y): " << line << std::endl;
            }
            (void)comma; // suppress unused-variable warning
        }
        nfile.close();
    }

    // read building rectangles, ignore z-range for placement
    std::ifstream bfile(buildingsFilename);
    if (!bfile.is_open()) {
        std::cerr << "Could not open buildings file: " << buildingsFilename << std::endl;
    } else {
        std::string line;
        // Skip header line
        std::getline(bfile, line);
        while (std::getline(bfile, line)) {
            std::stringstream ss(line);
            std::vector<double> values;
            double v;
            char comma;
            while (ss >> v) {
                values.push_back(v);
                if (ss.peek() == ',') ss.ignore();
            }
            if (values.size() >= 4) {
                // store entire line; zmin/zmax may follow but are optional
                rectangles.push_back(values);
            } else {
                std::cerr << "Invalid building line (expected at least 4 values): " << line << std::endl;
            }
        }
        bfile.close();
    }
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
        double zMin = 0.0;
        double zMax = 25.0;
        if (rect.size() >= 6) {
            zMin = rect[4];
            zMax = rect[5];
        }
        std::string type = "concrete with windows";

        Ptr<Building> currBuilding = CreateObject<Building>();
        currBuilding->SetBoundaries(Box(xMin, xMax, yMin, yMax, zMin, zMax));
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
            if (b.size() >= 4) {
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

int32_t FindNodeByAddress(const Ipv4Address& address) {
    for (const auto& entry : nodeInfoMap) {
        if (entry.second.NodeIP == address) {
            return entry.second.NodeID; // Return the NodeID if the address matches
        }
    }
    return -1; // Return -1 if no match is found
}

Ipv4Address FindAddressByNodeId(uint32_t nodeId) {
    for (const auto& entry : nodeInfoMap) {
        if (entry.second.NodeID == nodeId)
        {
            return entry.second.NodeIP; // Return the NodeIP if the NodeID matches
        }
    }
    return nullptr; // Return nullptr if no match is found
}

int32_t FindTargetByNodeId(uint32_t nodeId) {
    for (const auto& entry : nodeInfoMap) {
        if (entry.second.NodeID == nodeId) {
            return entry.second.TargetID; // Return the TargetID if the NodeID matches
        }
    }
    return -1; // Return -1 if no match is found
}

int32_t FindNodeByTargetId(int32_t targetId) {
    for (const auto& entry : nodeInfoMap) {
        if (entry.second.TargetID == targetId) {
            return entry.second.NodeID; // Return the NodeID if the TargetID matches
        }
    }
    return -1; // Return -1 if no match is found
}

void RxTraceWithAddresses(Ptr<const Packet> packet, const Address &from, const Address &to) {
    // Extract the sender's IP address from the "from" address
    Ipv4Address fromAddress = InetSocketAddress::ConvertFrom(from).GetIpv4();
    int32_t senderNodeId = FindNodeByAddress(fromAddress);
    int32_t receiverNodeId = FindTargetByNodeId(senderNodeId);

    if (senderNodeId != -1) {
        //std::cout << "Packet received from Node ID: " << senderNodeId << " with IP: " << fromAddress << std::endl;

        // Update traffic map or perform other actions
        nodeleveltrafficMap[receiverNodeId].NodeID = receiverNodeId; // Ensure the NodeID is set
        nodeleveltrafficMap[receiverNodeId].AppPacketsReceived++; // Increment packets sent by the sender
    } else {
        std::cerr << "No node found for IP address: " << fromAddress << std::endl;
    }
}

void TxTraceWithAddresses(Ptr<const Packet> packet, const Address &from, const Address &to)
{
    // Extract the sender's IP address from the "from" address
    Ipv4Address toAddress = InetSocketAddress::ConvertFrom(to).GetIpv4();
    int32_t receiverNodeId = FindNodeByAddress(toAddress);
    int32_t senderNodeId = FindNodeByTargetId(receiverNodeId);


    if (senderNodeId != -1) {
        //std::cout << "Packet sent from Node ID: " << senderNodeId << " with IP: " << fromAddress << std::endl;

        // Update traffic map or perform other actions
        nodeleveltrafficMap[senderNodeId].NodeID = senderNodeId; // Ensure the NodeID is set
        nodeleveltrafficMap[senderNodeId].AppPacketsSent++; // Increment packets sent by the sender
    } else {
        std::cerr << "No node found for IP address: " << toAddress << std::endl;
    }
}

void IpPacketReceivedCallback(Ptr<const Packet> packet, Ptr<Ipv4> to, uint32_t interface) {
    // Make a copy of the packet to extract headers
    Ptr<Packet> packetCopy = packet->Copy();
    // std::cout << packet->ToString() << std::endl;

    Ipv4Header ipv4Header;
    UdpHeader udpHeader;

    // Extract IPv4 header
    packetCopy->RemoveHeader(ipv4Header);
    Ipv4Address source = ipv4Header.GetSource();
    Ipv4Address destination = ipv4Header.GetDestination();

    // Check for UDP or TCP header
    if (packetCopy->PeekHeader(udpHeader)) {
        packetCopy->RemoveHeader(udpHeader);
        nodeleveltrafficMap[to->GetObject<Node>()->GetId()].NodeID = to->GetObject<Node>()->GetId();
        nodeleveltrafficMap[to->GetObject<Node>()->GetId()].DevicePacketsReceived++; // Increment packets received
    }
    uint16_t destinationPort = udpHeader.GetDestinationPort();

    // Check if destination port is 4000
    if (destinationPort != 4000)
    {
        //std::cout << packetCopy->ToString() << std::endl;
        networkLevelTraffic.RouteSignalizationPacketsReceived++; // Increment route signalization packets received
        // possibly routing packet
        ns3::aodv::TypeHeader typeHeader;
        if (!packetCopy->PeekHeader(typeHeader))
        {
            return;
        }
        packetCopy->RemoveHeader(typeHeader);
        
        // TYPES : 1 --> RREQ, 2--> RREP, 3--> RERR, 4--> RREP_ACK
        if (typeHeader.Get() == 2)
        {
            ns3::aodv::RrepHeader rrepHeader;
            if (!packetCopy->PeekHeader(rrepHeader))
            {
                std::cout << "not suppose to be here : there is a typeHeader, but no RREP header" << std::endl;
                std::exit(1);
            }
            packetCopy->RemoveHeader(rrepHeader);
            Ipv4Address OriginIP = rrepHeader.GetOrigin();
            // Check if the adress of the packet matches the destination address of the node that fire the callback
            if (to->GetAddress(interface, 0).GetLocal() != OriginIP)
            {
                return;
            }

            Ipv4Address DestinationIP = rrepHeader.GetDst();
            std::tuple<Ipv4Address, Ipv4Address, uint16_t> targetRoutePrint = {OriginIP, DestinationIP, rrepHeader.GetDstSeqno()};

            // Check if the Route exists in RoutingPerfInfoMap
            auto it = std::find_if(RoutingPerfInfoMap.begin(), RoutingPerfInfoMap.end(),
                [&targetRoutePrint](const auto& entry) {
                    const auto& currentRoutePrint = entry.second.RoutePrint;
                    return std::get<0>(currentRoutePrint) == std::get<0>(targetRoutePrint) &&  // origin
                        std::get<1>(currentRoutePrint) == std::get<1>(targetRoutePrint) &&  // destination
                        std::get<2>(currentRoutePrint) <= std::get<2>(targetRoutePrint);    // sequence number
                });
            
            if (it != RoutingPerfInfoMap.end())
            {
                // Update the existing RoutingPerfInfo object
                RoutingPerfInfo& existingRoute = it->second;
                existingRoute.TimeRREP = Simulator::Now().GetSeconds();
                existingRoute.hopCount = rrepHeader.GetHopCount();
                existingRoute.routeFound = true;
            }
        }
        return;
    }

    // Check if the adress of the packet matches the destination address of the node that fire the callback
    if (to->GetAddress(interface, 0).GetLocal() != destination)
    {
        return;
    }

    Time timestamp;
    SeqTsHeader seqTsHeader;
    if (packetCopy->PeekHeader(seqTsHeader)) {
        packetCopy->RemoveHeader(seqTsHeader);
        timestamp = seqTsHeader.GetTs();
        //std::cout << "SeqTs header found then removed in the packet." << std::endl;
    } else {
        std::cout << "No SeqTs header found in the packet." << std::endl;
        std::exit(1);
        return;
    }

    // Create the FlowPrint tuple
    std::tuple<Ipv4Address, Ipv4Address, uint16_t> flowPrint = {source, destination, destinationPort};

    // Check if the flow already exists in FlowInformationMap
    auto it = std::find_if(FlowInformationMap.begin(), FlowInformationMap.end(),
                            [&flowPrint](const auto& entry) {
                                return entry.second.FlowPrint == flowPrint;
                            });

    if (it != FlowInformationMap.end())
    {
        networkLevelTraffic.AppPacketsReceived++; // Increment route signalization packets received
        FlowInformation& existingFlow = it->second;
        existingFlow.RxPackets++;
        existingFlow.LastRxTime = Simulator::Now().GetSeconds();
        if (existingFlow.FirstRxTime < 0) {
            existingFlow.FirstRxTime = Simulator::Now().GetSeconds();
        }
        double delay = Simulator::Now().GetSeconds() - timestamp.GetSeconds();
        existingFlow.sumDelay += delay; // Update the sum of delays
    } else {
        std::cerr << "No matching flow found for received packet." << std::endl;
    }
    return;
}

void IpPacketSentCallback(Ptr<const Packet> packet, Ptr<Ipv4> from, uint32_t interface)
{    // Make a copy of the packet to extract headers
    Ptr<Packet> packetCopy = packet->Copy();

    Ipv4Header ipv4Header;
    UdpHeader udpHeader;

    // Extract IPv4 header
    packetCopy->RemoveHeader(ipv4Header);
    Ipv4Address source = ipv4Header.GetSource();
    Ipv4Address destination = ipv4Header.GetDestination();

    // Check for UDP or TCP header
    if (packetCopy->PeekHeader(udpHeader)) {
        packetCopy->RemoveHeader(udpHeader);
        nodeleveltrafficMap[from->GetObject<Node>()->GetId()].NodeID = from->GetObject<Node>()->GetId();
        nodeleveltrafficMap[from->GetObject<Node>()->GetId()].DevicePacketsSent++; // Increment packets Sent

    }

    uint16_t destinationPort = udpHeader.GetDestinationPort();

    // Check if destination port is 4000
    if (destinationPort != 4000)
    {
        networkLevelTraffic.RouteSignalizationPacketsSent++; // Increment route signalization packets received
        // possibly routing packet
        ns3::aodv::TypeHeader typeHeader;
        if (!packetCopy->PeekHeader(typeHeader))
        {
            return;
        }
        packetCopy->RemoveHeader(typeHeader);
        
        // TYPES : 1 --> RREQ, 2--> RREP, 3--> RERR, 4--> RREP_ACK
        if (typeHeader.Get() == 1)
        {
            ns3::aodv::RreqHeader rreqHeader;
            if (packetCopy->PeekHeader(rreqHeader))
            {
                packetCopy->RemoveHeader(rreqHeader);
                // RREQ is being sent
                if (rreqHeader.GetHopCount() == 0)
                {
                    // Create a new RoutingPerfInfo object
                    RoutingPerfInfo routingInfo;
                    uint16_t RouteId = RoutingPerfInfoMap.size();
                    routingInfo.RoutePrint = {rreqHeader.GetOrigin(), rreqHeader.GetDst(), rreqHeader.GetDstSeqno()};
                    routingInfo.RREQId = rreqHeader.GetId();
                    routingInfo.SourceIP = rreqHeader.GetOrigin();
                    routingInfo.DestinationIP = rreqHeader.GetDst();
                    routingInfo.hopCount = rreqHeader.GetHopCount();
                    routingInfo.TimeRREQ = Simulator::Now().GetSeconds();

                    // Add it to the map
                    RoutingPerfInfoMap[RouteId] = routingInfo;
                }
                // RREQ is being forwarded (we don't care)
            } 
            // not suppose to be here : there is a typeHeader, but no RREQ header
        }
        return;
    }

    int16_t sequenceNumber;
    SeqTsHeader seqTsHeader;
    if (packetCopy->PeekHeader(seqTsHeader)) {
        packetCopy->RemoveHeader(seqTsHeader);
        sequenceNumber = seqTsHeader.GetSeq();
        //std::cout << "SeqTs header found then removed in the packet." << std::endl;
    } else {
        std::cout << "No SeqTs header found in the packet." << std::endl;
        std::exit(1);
        return;
    }

    if (from->GetAddress(interface, 0).GetLocal() == source)
    {
        networkLevelTraffic.AppPacketsSent++; // Increment route signalization packets received
        // Create the FlowPrint tuple
        std::tuple<Ipv4Address, Ipv4Address, uint16_t> flowPrint = {source, destination, destinationPort};

        // Check if the flow already exists in FlowInformationMap
        auto it = std::find_if(FlowInformationMap.begin(), FlowInformationMap.end(),
                            [&flowPrint](const auto& entry) {
                                return entry.second.FlowPrint == flowPrint;
                            });

        if (it == FlowInformationMap.end()) {
            // Flow does not exist, create a new FlowInformation object
            FlowInformation newFlow;
            newFlow.FlowPrint = flowPrint;
            newFlow.FlowID = FlowInformationMap.size(); // Assign a unique FlowID
            newFlow.SourceIP = source;
            newFlow.Destination = destination;
            newFlow.TxPackets = 1; // First transmission
            newFlow.FirstTxTime = Simulator::Now().GetSeconds();
            newFlow.LastTxTime = Simulator::Now().GetSeconds();
            newFlow.seqNums.push_back(sequenceNumber);

            // Add the new flow to the map
            FlowInformationMap[newFlow.FlowID] = newFlow;
        } else {
            // Flow exists, update the existing FlowInformation object
            FlowInformation& existingFlow = it->second;

            // Check if the sequence number is already in the flow
            if (std::find(existingFlow.seqNums.begin(), existingFlow.seqNums.end(), sequenceNumber) == existingFlow.seqNums.end()) {
                // Sequence number is not present, treat this as a new transmission
                existingFlow.TxPackets++;
                existingFlow.LastTxTime = Simulator::Now().GetSeconds();
                existingFlow.seqNums.push_back(sequenceNumber);
            }
        }
    }
}

void IpPacketDropCallback(const Ipv4Header &header, Ptr< const Packet > packet, Ipv4L3Protocol::DropReason reason, Ptr< Ipv4 > ipv4, uint32_t interface) {
    // Make a copy of the packet to extract headers
    Ptr<Packet> packetCopy = packet->Copy();

    Ipv4Header ipv4Header;
    UdpHeader udpHeader;

    // Extract IPv4 header
    packetCopy->RemoveHeader(ipv4Header);
    // Ipv4Address source = ipv4Header.GetSource();
    // Ipv4Address destination = ipv4Header.GetDestination();

    // Check for UDP header
    if (!packetCopy->PeekHeader(udpHeader)) {
        return; // Not a UDP packet, ignore
    }

    packetCopy->RemoveHeader(udpHeader);
    uint16_t destinationPort = udpHeader.GetDestinationPort();

    // Only interested in packets going to port 4000
    if (destinationPort != 4000) {
        return;
    }

    // Print basic drop information
    // std::cout << "Packet dropped from Node ID: " 
    //           << ipv4->GetObject<Node>()->GetId() 
    //           << std::endl;

    // Print drop reason
    switch (reason) {
        case Ipv4L3Protocol::DROP_TTL_EXPIRED:
            break;
        case Ipv4L3Protocol::DROP_NO_ROUTE:
            nodeleveltrafficMap[ipv4->GetObject<Node>()->GetId()].PacketsDroppedNoRoute++; // Increment packets dropped due to route error
            break;
        case Ipv4L3Protocol::DROP_BAD_CHECKSUM:
            break;
        case Ipv4L3Protocol::DROP_INTERFACE_DOWN:
            break;
        case Ipv4L3Protocol::DROP_ROUTE_ERROR:
            nodeleveltrafficMap[ipv4->GetObject<Node>()->GetId()].PacketsDroppedRerr++; // Increment packets dropped due to route error
            break;
        case Ipv4L3Protocol::DROP_FRAGMENT_TIMEOUT:
            break;
        default:
            std::cout << "Reason: Unknown" << std::endl;
            break;
    }
}

void PhyTxDrop(Ptr<const Packet> packet)
{
    Ptr<Packet> pktCopy = packet->Copy();  // Make a non-const copy

    WifiMacHeader wifiMac;
    if (pktCopy->PeekHeader(wifiMac)) {
        pktCopy->RemoveHeader(wifiMac);  // Remove MAC header
    }

    LlcSnapHeader llc;
    if (pktCopy->PeekHeader(llc)) {
        pktCopy->RemoveHeader(llc);  // Remove LLC/SNAP header (optional, depending on MAC type)
    }

    Ipv4Header ipHeader;
    if (pktCopy->PeekHeader(ipHeader)) {
        pktCopy->RemoveHeader(ipHeader);  // Remove IP header
    }

    UdpHeader udpHeader;
    if (pktCopy->PeekHeader(udpHeader)) {
        pktCopy->RemoveHeader(udpHeader);  // Remove UDP header
    }

    uint16_t dstPort = udpHeader.GetDestinationPort();
    if (dstPort == 4000) {
        dropData.macTxDrop++;
    }

    return;
}
void PhyRxDrop(Ptr<const Packet> packet, WifiPhyRxfailureReason reason)
{
    Ptr<Packet> pktCopy = packet->Copy();

    WifiMacHeader wifiMac;
    if (pktCopy->PeekHeader(wifiMac)) {
        pktCopy->RemoveHeader(wifiMac);
    }

    if (pktCopy->GetSize() == 4) {
        // Only WifiMacTrailer present
        return;
    }

    LlcSnapHeader llc;
    if (pktCopy->PeekHeader(llc)) {
        pktCopy->RemoveHeader(llc);
    }

    Ipv4Header ipHeader;
    if (!pktCopy->PeekHeader(ipHeader)) {
        return;
    }
    pktCopy->RemoveHeader(ipHeader);

    UdpHeader udpHeader;
    if (!pktCopy->PeekHeader(udpHeader)) {
        return;
    }
    pktCopy->RemoveHeader(udpHeader);

    uint16_t dstPort = udpHeader.GetDestinationPort();
    if (dstPort != 4000) {
        return;
    }

    // Packet matches port 4000; log drop
    dropData.macRxDrop++;

    switch (reason)
    {
        case UNSUPPORTED_SETTINGS: dropData.dropUnsupportedSettings++; break;
        case CHANNEL_SWITCHING: dropData.dropChannelSwitching++; break;
        case RXING: dropData.dropRxing++; break;
        case TXING: dropData.dropTxing++; break;
        case SLEEPING: dropData.dropSleeping++; break;
        case POWERED_OFF: dropData.dropPoweredOff++; break;
        case TRUNCATED_TX: dropData.dropTruncatedTx++; break;
        case BUSY_DECODING_PREAMBLE: dropData.dropBusyDecodingPreamble++; break;
        case PREAMBLE_DETECT_FAILURE: dropData.dropPreambleDetectFailure++; break;
        case RECEPTION_ABORTED_BY_TX: dropData.dropReceptionAbortedByTx++; break;
        case L_SIG_FAILURE: dropData.dropLSigFailure++; break;
        case HT_SIG_FAILURE: dropData.dropHtSigFailure++; break;
        case SIG_A_FAILURE: dropData.dropSigAFailure++; break;
        case SIG_B_FAILURE: dropData.dropSigBFailure++; break;
        case U_SIG_FAILURE: dropData.dropUSigFailure++; break;
        case EHT_SIG_FAILURE: dropData.dropEhtSigFailure++; break;
        case PREAMBLE_DETECTION_PACKET_SWITCH: dropData.dropPreambleDetectionPacketSwitch++; break;
        case FRAME_CAPTURE_PACKET_SWITCH: dropData.dropFrameCapturePacketSwitch++; break;
        case OBSS_PD_CCA_RESET: dropData.dropObssPdCcaReset++; break;
        case PPDU_TOO_LATE: dropData.dropPpduTooLate++; break;
        case FILTERED: dropData.dropFiltered++; break;
        case DMG_HEADER_FAILURE: dropData.dropDmgHeaderFailure++; break;
        case DMG_ALLOCATION_ENDED: dropData.dropDmgAllocationEnded++; break;
        case UNKNOWN:
        default:
            dropData.dropUnknown++;
            break;
    }
}

void WriteFlowInformationToCsv(const std::string& filePath) {
    std::ofstream csvFile(filePath);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to create flow_information.csv file.\n";
        return;
    }

    // Write CSV header
    csvFile << "FlowID,SourceIP,DestinationIP,Port,TxPackets,RxPackets,FirstTxTime,LastTxTime,FirstRxTime,LastRxTime,sumDelay,SequenceNumbers\n";

    // Write data for each flow
    for (const auto& entry : FlowInformationMap) {
        const FlowInformation& flow = entry.second;

        // Convert sequence numbers vector to a comma-separated string
        std::ostringstream seqNumsStream;
        for (size_t i = 0; i < flow.seqNums.size(); ++i) {
            seqNumsStream << flow.seqNums[i];
            if (i != flow.seqNums.size() - 1) {
                seqNumsStream << ",";
            }
        }

        csvFile << flow.FlowID << ","
                << flow.SourceIP << ","
                << flow.Destination << ","
                << std::get<2>(flow.FlowPrint) << ","
                << flow.TxPackets << ","
                << flow.RxPackets << ","
                << flow.FirstTxTime << ","
                << flow.LastTxTime << ","
                << flow.FirstRxTime << ","
                << flow.LastRxTime << ","
                << flow.sumDelay << ","
                << seqNumsStream.str() << "\n";
    }

    csvFile.close();
    //std::cout << "Flow information CSV generated successfully.\n";
}

int main (int argc, char *argv[])
{   
    int Seed = 444; // Seed of randomness

    //LogComponentEnable("WifiPhy", LOG_LEVEL_DEBUG);
    PacketMetadata::Enable ();

    std::string RA = "aodv";
    std::string proto = "UDP";
    int numNodes = 40;
    int numBuildings = -1; // -1 : all buildings activated
    double numSource = 6; // Number of Source nodes
    double transmissionRate = 10; // packets per second
    std::string lossModel = "Friis"; //Friis FOBA TwoRayGroundPropagationLossModel

    std::string resultPath = "scratch/test-urban";
    bool verbose = false; // Enable verbose output
    bool Pcap = false; // Enable Pcap output
    bool bappcallback = true; // Enable callback for applicative packets
    bool nodeinfo = true; // Enable node information output
    bool deviceTracing = true; // Enable device tracing

    // layout input files
    std::string nodesFile = "scratch/nodes.csv";
    std::string buildingsFile = "scratch/building_layout.csv";

    CommandLine cmd;
    cmd.AddValue("RA", "Routing algorithm", RA);
    cmd.AddValue("numNodes", "Number of node to be created", numNodes);
    cmd.AddValue("numBuildings", "Number of buildings to be created", numBuildings);
    cmd.AddValue("numSource", "Number of Source node", numSource);
    cmd.AddValue("transmissionRate", "Transmission rate (kbps)", transmissionRate);
    cmd.AddValue("resultPath", "Path to store results", resultPath);
    cmd.AddValue("lossModel", "Loss model to use", lossModel);
    cmd.AddValue("Seed", "Set randomness seed", Seed);
    cmd.AddValue("nodesFile", "CSV file containing node coordinates", nodesFile);
    cmd.AddValue("buildingsFile", "CSV file containing building layout", buildingsFile);

    cmd.Parse (argc, argv);

    if (numSource > numNodes)
    {
        std::cout << "Warning: numSource is greater than numNodes. "
                  << "This may lead to some miss configuration." << std::endl;
    }

    double StartTime = 5;
    double EndTime = 300;

    // Set randomness
    RngSeedManager::SetSeed(Seed);
    RngSeedManager::SetRun(1);

    // make sure output directory exists before creating files
    if (!resultPath.empty()) {
        std::filesystem::create_directories(resultPath);
    }

    // load layout from CSVs (nodes + building rectangles)
    std::vector<std::pair<double, double>> points;
    std::vector<std::vector<double>> rectangles;

    // filenames are expected relative to working directory or can be overwritten via command line later
    readUrbanData(nodesFile, buildingsFile, points, rectangles);

    double cleanup_time = 20;


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

    std::mt19937 rng(Seed); // deterministic random number generator
    std::uniform_int_distribution<size_t> dist(0, points.size() - 1);
    u_int k = 0;
    for (NodeContainer::Iterator i = nodes.Begin(); i != nodes.End(); ++i)
    {
        size_t index = dist(rng);
        Ptr<Node> node = *i;
        // Mobility
        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::RandomWalk2dOutdoorMobilityModel",
                                    "Mode",
                                    StringValue("Time"),
                                    "Time",
                                    TimeValue(Seconds(20.0)),
                                    "Speed",
                                    StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
                                    "Bounds",
                                    RectangleValue(Rectangle(xMinBound, xMaxBound, yMinBound, yMaxBound)));
        mobility.Install(node);
        double x = points[index].first;
        double y = points[index].second;
        double z = 2;
        //std::cout << "(" << x <<","<<y<<","<<z<<")"<<std::endl;
        if (!(xMinBound<x)||!(x<xMaxBound))
        {
            std::cout << "x not inside the bounds (x,y): " << x << " " << y << std::endl;
        }
        if (!(yMinBound<y)||!(y<yMaxBound))
        {
            std::cout << "y not inside the bounds (x,y): " << x << " " << y << std::endl;
        }
        Ptr<MobilityModel> mobility1 = node->GetObject<MobilityModel> ();
        mobility1->SetPosition (ns3::Vector(x,y,z));
        k++;
        points.erase(points.begin() + index);
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
        wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel",
                                       "SystemLoss", DoubleValue(2.6));

    }
    else if (lossModel == "Cost231PropagationLossModel")
    {
        //std::cout << "setting up loss model : " << lossModel << std::endl;
        wifiChannel.AddPropagationLoss("ns3::Cost231PropagationLossModel");
    }
    else if (lossModel == "TwoRayGroundPropagationLossModel")
    {
        //std::cout << "setting up loss model : " << lossModel << std::endl;
        wifiChannel.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel");
    }
    else if (lossModel == "ItuR1411LosPropagationLossModel")
    {
        //std::cout << "setting up loss model : " << lossModel << std::endl;
        wifiChannel.AddPropagationLoss("ns3::ItuR1411LosPropagationLossModel");
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

    
    if (RA == "olsr")
    {
        InternetStackHelper stack;
        Ipv4StaticRoutingHelper staticRouting;
        OlsrHelper olsr;    
        Ipv4ListRoutingHelper list;
        list.Add(staticRouting, 0);
        list.Add(olsr, 10);
        stack.SetRoutingHelper(list);
        stack.Install (nodes);
    }

    if (RA == "aodv")
    {
        InternetStackHelper stack;
        AodvHelper aodv;
        stack.SetRoutingHelper(aodv);
        if (true)
        {
            Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(resultPath+"/aodv.routes", std::ios::out);
            Ipv4RoutingHelper::PrintRoutingTableAllAt(Seconds(5), routingStream);
        }
        stack.Install (nodes);
    }
    if (RA == "dsdv")
    {
        InternetStackHelper stack;
        DsdvHelper dsdv;
        dsdv.Set("PeriodicUpdateInterval", TimeValue(Seconds(5)));
        dsdv.Set("SettlingTime", TimeValue(Seconds(5)));
        stack.SetRoutingHelper(dsdv); // has effect on the next Install ()

        if (true)
        {
            Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>((resultPath+"/dsdv.routes"), std::ios::out);
            Ipv4RoutingHelper::PrintRoutingTableAllAt(Seconds(5), routingStream);
            Ipv4RoutingHelper::PrintRoutingTableAllAt(Seconds(35), routingStream);

        }
        stack.Install (nodes);
    }
    

    // Assign IP addresses
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // Populate the nodeInfoMap with NodeID and NodeIP
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<Node> node = nodes.Get(i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        Ipv4Address nodeIp = ipv4->GetAddress(1, 0).GetLocal();

        NodeInformation info;
        info.NodeID = i;
        info.NodeIP = nodeIp;
        nodeInfoMap[i] = info;
    }

    // Setup the message maker nodes and their communication
    std::mt19937 gen(Seed);
    std::uniform_int_distribution<> dis(4, nodes.GetN() - 1); // Don't launch simulation with less than 5 nodes
    std::set<uint32_t> selectedNodes; // To track already-selected nodes
    //std::cout << "Setting up message makers and their communication..." << std::endl;

    for (uint32_t i = 0; i < numSource; ++i) {
        Ptr<Node> node = nodes.Get(i);
    
        // Ensure unique selection for nextNode
        uint32_t nextNodeIndex;
        do {
            nextNodeIndex = dis(gen);
        } while (nextNodeIndex == i || selectedNodes.find(nextNodeIndex) != selectedNodes.end());
        selectedNodes.insert(nextNodeIndex);
    
        Ptr<Node> nextNode = nodes.Get(nextNodeIndex);
        //std::cout << "Node " << node->GetId() << " will try to communicate with Node " << nextNode->GetId() << std::endl;
        Ipv4Address nextNodeAddress = nextNode->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    
        uint16_t port = (proto == "UDP") ? 4000 : 50000;
    
        if (proto == "UDP") {
            // Setup the UDP server on the next node
            UdpServerHelper server(port);
            ApplicationContainer apps = server.Install(nextNode);
            apps.Start(Seconds(1.0));
            apps.Stop(Seconds(EndTime));

            // Setup the UDP client on the message maker node
            UdpClientHelper client(nextNodeAddress, port);
            client.SetAttribute("MaxPackets", UintegerValue(1000000)); // 1.000.000 packets max per client
            client.SetAttribute("Interval", TimeValue(Seconds(1 / transmissionRate))); // Set transmission rate
            client.SetAttribute("PacketSize", UintegerValue(1024)); // Packet size in bytes
            apps = client.Install(node);
            apps.Start(Seconds(StartTime));
            apps.Stop(Seconds(EndTime));

            // Update the nodeInfoMap for source
            nodeInfoMap[i].TargetID = static_cast<int32_t>(nextNodeIndex); // Set the target ID
            nodeInfoMap[i].TargetIP = nextNodeAddress; // Set the target IP

            if (bappcallback)
            {
                // Search for UdpServer on the nextNode
                Ptr<UdpServer> udpServer = nullptr;
                for (uint32_t i = 0; i < nextNode->GetNApplications(); ++i)
                {
                    udpServer = DynamicCast<UdpServer>(nextNode->GetApplication(i));
                    if (udpServer)
                    {
                        udpServer->TraceConnectWithoutContext("RxWithAddresses", MakeBoundCallback(&RxTraceWithAddresses));
                        break;
                    }
                }

                if (!udpServer)
                {
                    std::cerr << "Error: UdpServer not found on node " << nextNode->GetId() << std::endl;
                }

                // Search for UdpClient on the source node
                Ptr<UdpClient> udpClient = nullptr;
                for (uint32_t i = 0; i < node->GetNApplications(); ++i)
                {
                    udpClient = DynamicCast<UdpClient>(node->GetApplication(i));
                    if (udpClient)
                    {
                        udpClient->TraceConnectWithoutContext("TxWithAddresses", MakeBoundCallback(&TxTraceWithAddresses));
                        break;
                    }
                }

                if (!udpClient)
                {
                    std::cerr << "Error: UdpClient not found on node " << node->GetId() << std::endl;
                }
            }
        }

        if (proto == "TCP") {
            // Setup the TCP server on the next node
            Address hubLocalAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
            PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", hubLocalAddress);
            ApplicationContainer hubApp = packetSinkHelper.Install(nextNode);
            hubApp.Start(Seconds(1.0));
            hubApp.Stop(Seconds(EndTime));
    
            // Setup the TCP client on the message maker node
            OnOffHelper onOffHelper("ns3::TcpSocketFactory", Address());
            onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
            AddressValue remoteAddress(InetSocketAddress(nextNodeAddress, port));
            onOffHelper.SetAttribute("Remote", remoteAddress);
    
            ApplicationContainer spokeApps = onOffHelper.Install(node);
            spokeApps.Start(Seconds(StartTime));
            spokeApps.Stop(Seconds(EndTime));
        }
    }

    // Attach tracing callbacks to all nodes
    if (deviceTracing)
    {
        for (NodeContainer::Iterator i = nodes.Begin(); i != nodes.End(); ++i)
        {
            Ptr<Node> node = *i;
            Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();

            ipv4->TraceConnectWithoutContext("Tx", MakeCallback(&IpPacketSentCallback));
            ipv4->TraceConnectWithoutContext("Rx", MakeCallback(&IpPacketReceivedCallback));
            ipv4->TraceConnectWithoutContext("Drop", MakeCallback(&IpPacketDropCallback));

            // Iterate through the devices of the node
            for (uint32_t j = 0; j < node->GetNDevices(); ++j) {
                Ptr<NetDevice> device = node->GetDevice(j);

                // Check if the device is a WifiNetDevice
                Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(device);
                if (wifiDevice) {
                    // Get the WifiPhy object from the WifiNetDevice
                    Ptr<WifiPhy> wifiPhy = wifiDevice->GetPhy();

                    // Connect the MacTx function to the m_phyTxEndTrace callback
                    wifiPhy->TraceConnectWithoutContext("PhyRxDrop", MakeCallback(&PhyRxDrop));
                    wifiPhy->TraceConnectWithoutContext("PhyTxDrop", MakeCallback(&PhyTxDrop));
                }
            }

            for (uint32_t j = 0; j < node->GetNApplications(); ++j) {
                Ptr<Application> app = node->GetApplication(j);
                Ptr<UdpServer> server = DynamicCast<UdpServer>(app);
                Ptr<UdpClient> client = DynamicCast<UdpClient>(app);
            
                if (server) {
                    //std::cout << "UDP Server found on node " << node->GetId() << std::endl;
                    //server->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&AppRx));
                }
                if (client) {
                    //std::cout << "UDP Client found on node " << node->GetId() << std::endl;
                    //client->TraceConnectWithoutContext("TxWithAddresses", MakeCallback(&AppTx));
                }
            }
        }
    }

    // Tracing
    if (Pcap)
    {
        for (uint32_t i = 0; i < nodes.GetN(); ++i)
        {
            std::string filename = resultPath + "/Node-" + std::to_string(i) + "-" + RA + ".pcap"; 
            wifiPhy.EnablePcap(filename, nodes.Get(i)->GetDevice(0), false, true);
        }
    }



    if (verbose)
    {
        std::cout << "Positions will be recorded \n";
        Simulator::Schedule(Seconds(0.5),
            &printmob,
            nodes);
    }
    

    if (verbose)
    {
        std::cout << "Simulation starting..." << std::endl;
    }

    if (nodeinfo)
    {
        std::ofstream csvFile(resultPath + "/node_target_mapping.csv");
        if (!csvFile.is_open()) {
            std::cerr << "Failed to create node_target_mapping.csv file.\n";
            return 1;
        }
        
        // Write CSV header
        csvFile << "Node,Node IP,Node Target,IP Target\n";
        
        // Write data for each node
        for (const auto& entry : nodeInfoMap) {
            const NodeInformation& info = entry.second;
            csvFile << info.NodeID << "," << info.NodeIP << "," << info.TargetID << "," << info.TargetIP << "\n";
        }
        
        csvFile.close();
        //std::cout << "Node target mapping CSV generated successfully.\n";
    }


    Simulator::Stop(Seconds(EndTime+cleanup_time));

    auto simulationStart = std::chrono::high_resolution_clock::now();
    //Simulator::Schedule(Seconds(1.0), &PrintSimulationTime);

    Simulator::Run ();

    auto simulationEnd = std::chrono::high_resolution_clock::now(); // End time measurement
    std::chrono::duration<double> simulationDuration = simulationEnd - simulationStart;
    
    // Record simulation time
    std::ofstream timeLog(resultPath + "/simulation_time.csv");
    if (timeLog.is_open())
    {
        timeLog << simulationDuration.count() << "\n"; // Write only the simulation time value
        timeLog.close();
    } else
    {
        std::cerr << "Failed to write simulation time to file.\n";
    }


    // Log packet statistics
    if (bappcallback)
    {
        WriteFlowInformationToCsv(resultPath + "/flow_information.csv");

        std::ofstream csvFileNT(resultPath + "/Network_traffic_mapping.csv");
        if (!csvFileNT.is_open())
        {
            std::cerr << "Failed to create Network_traffic_mapping.csv file.\n";
            return 1;
        }
        
        // Write CSV header
        csvFileNT << "RouteSignalizationPacketsSent, RouteSignalizationPacketsReceived, AppPacketsSent, AppPacketsReceived\n";
        csvFileNT << networkLevelTraffic.RouteSignalizationPacketsSent << ","
            << networkLevelTraffic.RouteSignalizationPacketsReceived << ","
            << networkLevelTraffic.AppPacketsSent << ","
            << networkLevelTraffic.AppPacketsReceived << "\n";
        csvFileNT.close();

        std::ofstream csvFile(resultPath + "/node_traffic_mapping.csv");
        if (!csvFile.is_open())
        {
            std::cerr << "Failed to create node_traffic_mapping.csv file.\n";
            return 1;
        }
        
        // Write CSV header
        csvFile << "Node, AppTx, AppRx, DeviceTx, DeviceRx, PacketsDroppedRerr, PacketsDroppedNoRoute\n";
        
        // Write data for each node
        for (const auto& entry : nodeleveltrafficMap)
        {
            const NodeLevelTraffic& info = entry.second;
            csvFile << info.NodeID << "," << info.AppPacketsSent << "," << info.AppPacketsReceived
                << "," << info.DevicePacketsReceived << "," << info.DevicePacketsSent << "," << info.PacketsDroppedRerr
                << "," << info.PacketsDroppedNoRoute <<"\n";
        }
        csvFile.close();

        std::ofstream csvFileR(resultPath + "/Route_mapping.csv");
        if (!csvFileR.is_open())
        {
            std::cerr << "Failed to create Route_mapping.csv file.\n";
            return 1;
        }
        
        // Write CSV header
        csvFileR << "OriginIP,DestIP,DestSeqNo,RREQId,SourceIP,DestinationIP,hopCount,TimeRREQ,TimeRREP,routeFound\n";

        for (const auto& entry : RoutingPerfInfoMap)
        {
            const RoutingPerfInfo& routingInfo = entry.second;
        
            csvFileR 
                << std::get<0>(routingInfo.RoutePrint) << ","  // SrcIP
                << std::get<1>(routingInfo.RoutePrint) << ","  // DstIP
                << std::get<2>(routingInfo.RoutePrint) << ","  // DestSeqNo
                << routingInfo.RREQId << "," 
                << routingInfo.SourceIP << "," 
                << routingInfo.DestinationIP << "," 
                << routingInfo.hopCount << "," 
                << routingInfo.TimeRREQ << "," 
                << routingInfo.TimeRREP << "," 
                << routingInfo.routeFound << "\n";
        }
        
        csvFileR.close();

        std::ofstream csvFileD(resultPath + "/dropData.csv");
        if (!csvFileD.is_open())
        {
            std::cerr << "Failed to create dropData.csv file.\n";
            return 1;
        }
        
        // Write CSV header
        csvFileD << "Metric,Value\n";

        csvFileD << "macTxDrop," << dropData.macTxDrop << "\n";
        csvFileD << "macRxDrop," << dropData.macRxDrop << "\n";
        csvFileD << "dropUnsupportedSettings," << dropData.dropUnsupportedSettings << "\n";
        csvFileD << "dropChannelSwitching," << dropData.dropChannelSwitching << "\n";
        csvFileD << "dropRxing," << dropData.dropRxing << "\n";
        csvFileD << "dropTxing," << dropData.dropTxing << "\n";
        csvFileD << "dropSleeping," << dropData.dropSleeping << "\n";
        csvFileD << "dropPoweredOff," << dropData.dropPoweredOff << "\n";
        csvFileD << "dropTruncatedTx," << dropData.dropTruncatedTx << "\n";
        csvFileD << "dropBusyDecodingPreamble," << dropData.dropBusyDecodingPreamble << "\n";
        csvFileD << "dropPreambleDetectFailure," << dropData.dropPreambleDetectFailure << "\n";
        csvFileD << "dropReceptionAbortedByTx," << dropData.dropReceptionAbortedByTx << "\n";
        csvFileD << "dropLSigFailure," << dropData.dropLSigFailure << "\n";
        csvFileD << "dropHtSigFailure," << dropData.dropHtSigFailure << "\n";
        csvFileD << "dropSigAFailure," << dropData.dropSigAFailure << "\n";
        csvFileD << "dropSigBFailure," << dropData.dropSigBFailure << "\n";
        csvFileD << "dropUSigFailure," << dropData.dropUSigFailure << "\n";
        csvFileD << "dropEhtSigFailure," << dropData.dropEhtSigFailure << "\n";
        csvFileD << "dropPreambleDetectionPacketSwitch," << dropData.dropPreambleDetectionPacketSwitch << "\n";
        csvFileD << "dropFrameCapturePacketSwitch," << dropData.dropFrameCapturePacketSwitch << "\n";
        csvFileD << "dropObssPdCcaReset," << dropData.dropObssPdCcaReset << "\n";
        csvFileD << "dropPpduTooLate," << dropData.dropPpduTooLate << "\n";
        csvFileD << "dropFiltered," << dropData.dropFiltered << "\n";
        csvFileD << "dropDmgHeaderFailure," << dropData.dropDmgHeaderFailure << "\n";
        csvFileD << "dropDmgAllocationEnded," << dropData.dropDmgAllocationEnded << "\n";
        csvFileD << "dropUnknown," << dropData.dropUnknown << "\n";
        
        csvFileD.close();
        //std::cout << "Node target mapping CSV generated successfully.\n";
    }

    Simulator::Destroy ();
    if (verbose)
    {
        std::cout << "Random2Dwalk simulation in urban setting with " + RA + " ended successfully" << "\n";
    }
    
}



// variant of sionna-rt-custom with connectivity/mobility logging

#include "pybind11/pybind11.h"

#include "ns3/channel-condition-model.h"
#include "ns3/core-module.h"
#include "ns3/lte-spectrum-value-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/applications-module.h"
#include "ns3/adhoc-aloha-noack-ideal-phy-helper.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/spectrum-helper.h"
#include "ns3/single-model-spectrum-channel.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/node.h"
#include "ns3/sionna-rt-channel-model.h"
#include "ns3/sionna-rt-spectrum-propagation-loss-model.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/three-gpp-antenna-model.h"
#include "ns3/uniform-planar-array.h"
#include "ns3/ipv4-l3-protocol.h" // for drop reasons

#include <fstream>
#include <sstream>
#include <numeric>
#include <map>
#include <tuple>
#include <vector>
#include <algorithm>
#include <iostream>

using namespace ns3;
namespace py = pybind11;

// global guard against scheduling beyond stop time
static double g_simStopTime = 0.0;

// Use the concrete class to avoid illegal Ptr conversions.
static Ptr<SionnaRtSpectrumPropagationLossModel>
    m_spectrumLossModel; //!< the SpectrumPropagationLossModel object

// connectivity/mobility globals from UrbanCompConnectivity
static Ipv4InterfaceContainer g_ifaces;
static std::vector<std::vector<uint32_t>> receivedProbes;
static std::vector<std::vector<std::vector<uint32_t>>> timeReceivedProbes;
static std::vector<std::vector<std::tuple<double,double,double>>> nodePositions;

// additional globals for sionna metrics may be defined later

NS_LOG_COMPONENT_DEFINE("SionnaRTConnectExample");

// forward declarations for connectivity helpers
void RecordNodePositions(NodeContainer nodes);
void SchedulePositionRecording(NodeContainer nodes);
void ReceiveProbe(Ptr<Socket> socket);
void SendProbes(Ptr<Node> sender, Ipv4InterfaceContainer ifaces);

// traffic metric definitions and helper routines taken from sionna-rt-custom

// declare parameters for scheduled SNR computation
struct ComputeSnrParams
{
    Ptr<MobilityModel> txMob;        //!< the tx mobility model
    Ptr<MobilityModel> rxMob;        //!< the rx mobility model
    double txPow;                    //!< the tx power in dBm
    double noiseFigure;              //!< the noise figure in dB
    Ptr<PhasedArrayModel> txAntenna; //!< the tx antenna array
    Ptr<PhasedArrayModel> rxAntenna; //!< the rx antenna array
};

// per‑level traffic counters
struct NetwortkLevelTraffic {
    uint32_t RouteSignalizationPacketsSent = 0;
    uint32_t RouteSignalizationPacketsReceived = 0;
    uint32_t AppPacketsSent = 0;
    uint32_t AppPacketsReceived = 0;
};

NetwortkLevelTraffic networkLevelTraffic;

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
    std::tuple<Ipv4Address, Ipv4Address, uint16_t> FlowPrint = {Ipv4Address("127.0.0.1"), Ipv4Address("127.0.0.1"), 0};
    uint32_t FlowID = -1;
    Ipv4Address SourceIP = Ipv4Address("127.0.0.1");
    Ipv4Address Destination = Ipv4Address("127.0.0.1");
    uint16_t TxPackets = 0;
    uint16_t RxPackets = 0;
    double FirstTxTime = -1;
    double LastTxTime = -1;
    double FirstRxTime = -1;
    double LastRxTime = -1;
    double sumDelay = 0.0;
    std::vector<uint64_t> seqNums = {};
};

std::map<uint32_t, FlowInformation> FlowInformationMap;

static void
IpPacketSentCallback(Ptr<const Packet> packet, Ptr<Ipv4> from, uint32_t interface)
{
    Ptr<Packet> packetCopy = packet->Copy();

    Ipv4Header ipv4Header;
    UdpHeader udpHeader;

    packetCopy->RemoveHeader(ipv4Header);
    Ipv4Address source = ipv4Header.GetSource();
    Ipv4Address destination = ipv4Header.GetDestination();

    uint16_t destinationPort = 0;
    if (packetCopy->PeekHeader(udpHeader)) {
        packetCopy->RemoveHeader(udpHeader);
        nodeleveltrafficMap[from->GetObject<Node>()->GetId()].NodeID = from->GetObject<Node>()->GetId();
        nodeleveltrafficMap[from->GetObject<Node>()->GetId()].DevicePacketsSent++;
        destinationPort = udpHeader.GetDestinationPort();
    } else {
        return;
    }

    if (destinationPort != 4000)
    {
        networkLevelTraffic.RouteSignalizationPacketsSent++;
        return;
    }

    if (from->GetAddress(interface, 0).GetLocal() == source)
    {
        int16_t sequenceNumber = -1;
        SeqTsHeader seqTsHeader;
        if (packetCopy->PeekHeader(seqTsHeader)) {
            packetCopy->RemoveHeader(seqTsHeader);
            sequenceNumber = seqTsHeader.GetSeq();
        }

        networkLevelTraffic.AppPacketsSent++;
        std::tuple<Ipv4Address, Ipv4Address, uint16_t> flowPrint = {source, destination, destinationPort};
        auto it = std::find_if(FlowInformationMap.begin(), FlowInformationMap.end(),
                            [&flowPrint](const auto& entry) {
                                return entry.second.FlowPrint == flowPrint;
                            });

        if (it == FlowInformationMap.end()) {
            FlowInformation newFlow;
            newFlow.FlowPrint = flowPrint;
            newFlow.FlowID = FlowInformationMap.size();
            newFlow.SourceIP = source;
            newFlow.Destination = destination;
            newFlow.TxPackets = 1;
            newFlow.FirstTxTime = Simulator::Now().GetSeconds();
            newFlow.LastTxTime = Simulator::Now().GetSeconds();
            if (sequenceNumber >= 0) {
                newFlow.seqNums.push_back(sequenceNumber);
            }
            FlowInformationMap[newFlow.FlowID] = newFlow;
        } else {
            FlowInformation& existingFlow = it->second;
            if (sequenceNumber >= 0 &&
                std::find(existingFlow.seqNums.begin(), existingFlow.seqNums.end(), sequenceNumber) == existingFlow.seqNums.end()) {
                existingFlow.TxPackets++;
                existingFlow.LastTxTime = Simulator::Now().GetSeconds();
                existingFlow.seqNums.push_back(sequenceNumber);
            }
        }
    }
}

static void
IpPacketReceivedCallback(Ptr<const Packet> packet, Ptr<Ipv4> to, uint32_t interface)
{
    Ptr<Packet> packetCopy = packet->Copy();

    Ipv4Header ipv4Header;
    UdpHeader udpHeader;

    packetCopy->RemoveHeader(ipv4Header);
    Ipv4Address source = ipv4Header.GetSource();
    Ipv4Address destination = ipv4Header.GetDestination();

    uint16_t destinationPort = 0;
    if (packetCopy->PeekHeader(udpHeader)) {
        packetCopy->RemoveHeader(udpHeader);
        nodeleveltrafficMap[to->GetObject<Node>()->GetId()].NodeID = to->GetObject<Node>()->GetId();
        nodeleveltrafficMap[to->GetObject<Node>()->GetId()].DevicePacketsReceived++;
        destinationPort = udpHeader.GetDestinationPort();
    } else {
        return;
    }

    if (destinationPort != 4000)
    {
        networkLevelTraffic.RouteSignalizationPacketsReceived++;
        return;
    }

    if (to->GetAddress(interface, 0).GetLocal() != destination)
    {
        return;
    }

    Time timestamp;
    SeqTsHeader seqTsHeader;
    if (packetCopy->PeekHeader(seqTsHeader)) {
        packetCopy->RemoveHeader(seqTsHeader);
        timestamp = seqTsHeader.GetTs();
    }

    std::tuple<Ipv4Address, Ipv4Address, uint16_t> flowPrint = {source, destination, destinationPort};
    auto it = std::find_if(FlowInformationMap.begin(), FlowInformationMap.end(),
                            [&flowPrint](const auto& entry) {
                                return entry.second.FlowPrint == flowPrint;
                            });

    if (it != FlowInformationMap.end())
    {
        networkLevelTraffic.AppPacketsReceived++;
        FlowInformation& existingFlow = it->second;
        existingFlow.RxPackets++;
        existingFlow.LastRxTime = Simulator::Now().GetSeconds();
        if (existingFlow.FirstRxTime < 0) {
            existingFlow.FirstRxTime = Simulator::Now().GetSeconds();
        }
        double delay = Simulator::Now().GetSeconds() - timestamp.GetSeconds();
        existingFlow.sumDelay += delay;
    }
}

static void
IpPacketDropCallback(const Ipv4Header &header, Ptr< const Packet > packet, Ipv4L3Protocol::DropReason reason, Ptr< Ipv4 > ipv4, uint32_t interface)
{
    Ptr<Packet> packetCopy = packet->Copy();
    Ipv4Header ipv4Header;
    UdpHeader udpHeader;

    packetCopy->RemoveHeader(ipv4Header);
    if (!packetCopy->PeekHeader(udpHeader)) {
        return;
    }
    packetCopy->RemoveHeader(udpHeader);
    uint16_t destinationPort = udpHeader.GetDestinationPort();
    if (destinationPort != 4000) {
        return;
    }
    switch (reason) {
        case Ipv4L3Protocol::DROP_NO_ROUTE:
            nodeleveltrafficMap[ipv4->GetObject<Node>()->GetId()].PacketsDroppedNoRoute++;
            break;
        case Ipv4L3Protocol::DROP_ROUTE_ERROR:
            nodeleveltrafficMap[ipv4->GetObject<Node>()->GetId()].PacketsDroppedRerr++;
            break;
        default:
            break;
    }
}

static void
WriteFlowInformationToCsv(const std::string& filePath) {
    std::ofstream csvFile(filePath);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to create flow_information.csv file.\n";
        return;
    }

    csvFile << "FlowID,SourceIP,DestinationIP,Port,TxPackets,RxPackets,FirstTxTime,LastTxTime,FirstRxTime,LastRxTime,sumDelay,SequenceNumbers\n";
    for (const auto& entry : FlowInformationMap) {
        const FlowInformation& flow = entry.second;
        std::ostringstream seqNumsStream;
        for (size_t i = 0; i < flow.seqNums.size(); ++i) {
            seqNumsStream << flow.seqNums[i];
            if (i != flow.seqNums.size() - 1) seqNumsStream << ",";
        }
        csvFile << flow.FlowID << "," << flow.SourceIP << "," << flow.Destination << ","
                << std::get<2>(flow.FlowPrint) << "," << flow.TxPackets << ","
                << flow.RxPackets << "," << flow.FirstTxTime << "," << flow.LastTxTime << ","
                << flow.FirstRxTime << "," << flow.LastRxTime << "," << flow.sumDelay << ","
                << seqNumsStream.str() << "\n";
    }
    csvFile.close();
}

static void
WriteAllMetricsToCsv()
{
    WriteFlowInformationToCsv("flow_information.csv");
    std::ofstream csvFileNT("Network_traffic_mapping.csv");
    if (csvFileNT.is_open()) {
        csvFileNT << "RouteSignalizationPacketsSent,RouteSignalizationPacketsReceived,AppPacketsSent,AppPacketsReceived\n";
        csvFileNT << networkLevelTraffic.RouteSignalizationPacketsSent << ","
                  << networkLevelTraffic.RouteSignalizationPacketsReceived << ","
                  << networkLevelTraffic.AppPacketsSent << ","
                  << networkLevelTraffic.AppPacketsReceived << "\n";
        csvFileNT.close();
    }
    std::ofstream csvNode("node_traffic_mapping.csv");
    if (csvNode.is_open()) {
        csvNode << "Node,AppTx,AppRx,DeviceTx,DeviceRx,PacketsDroppedRerr,PacketsDroppedNoRoute\n";
        for (const auto& entry : nodeleveltrafficMap) {
            const NodeLevelTraffic& info = entry.second;
            csvNode << info.NodeID << "," << info.AppPacketsSent << "," << info.AppPacketsReceived
                    << "," << info.DevicePacketsReceived << "," << info.DevicePacketsSent << ","
                    << info.PacketsDroppedRerr << "," << info.PacketsDroppedNoRoute << "\n";
        }
        csvNode.close();
    }
}

static void
LogSimTime()
{
    double t = Simulator::Now().GetSeconds();
    NS_LOG_INFO("SimTime: " << t << " s");
    std::ofstream lf;
    lf.open("simtime-trace.txt", std::ios::out | std::ios::app);
    lf << t << std::endl;
    lf.close();
    // schedule next invocation only if it won't exceed the configured stop time
    if (t + 5.0 < g_simStopTime) {
        Simulator::Schedule(Seconds(5), &LogSimTime);
    }
}

static void
DoBeamforming(Ptr<NetDevice> thisDevice,
              Ptr<PhasedArrayModel> thisAntenna,
              Ptr<NetDevice> otherDevice)
{
    // retrieve the position of the two devices
    Vector aPos = thisDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();
    Vector bPos = otherDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();

    // compute the azimuth and the elevation angles
    Angles completeAngle(bPos, aPos);
    double hAngleRadian = completeAngle.GetAzimuth();

    double vAngleRadian = completeAngle.GetInclination(); // the elevation angle

    // retrieve the number of antenna elements and resize the vector
    uint64_t totNoArrayElements = thisAntenna->GetNumElems();
    PhasedArrayModel::ComplexVector antennaWeights(totNoArrayElements);

    // the total power is divided equally among the antenna elements
    double power = 1.0 / sqrt(totNoArrayElements);

    // compute the antenna weights
    const double sinVAngleRadian = sin(vAngleRadian);
    const double cosVAngleRadian = cos(vAngleRadian);
    const double sinHAngleRadian = sin(hAngleRadian);
    const double cosHAngleRadian = cos(hAngleRadian);

    for (uint64_t ind = 0; ind < totNoArrayElements; ind++)
    {
        Vector loc = thisAntenna->GetElementLocation(ind);
        double phase = -2 * M_PI *
                       (sinVAngleRadian * cosHAngleRadian * loc.x +
                        sinVAngleRadian * sinHAngleRadian * loc.y + cosVAngleRadian * loc.z);
        antennaWeights[ind] = exp(std::complex<double>(0, phase)) * power;
    }

    // store the antenna weights
    thisAntenna->SetBeamformingVector(antennaWeights);
}

static void
PrintPythonExecutable()
{
    py::object sys = py::module_::import("sys");
    py::print("Python executable:", sys.attr("executable"));
    py::print("Python version:", sys.attr("version"));
}

static void
ComputeSnr(ComputeSnrParams params)
{
    // Create the tx PSD.  LTE helper shown here; if you prefer ISM:
    //
    //     IsmSpectrumValueHelper ism;
    //     Ptr<SpectrumValue> txPsd = ism.CreateTxPowerSpectralDensity(txPower,
    //                                                                  channelNumber);
    //
    std::vector<int> activeRbs0(100);
    std::iota(activeRbs0.begin(), activeRbs0.end(), 0);
    Ptr<SpectrumValue> txPsd =
        LteSpectrumValueHelper::CreateTxPowerSpectralDensity(2100, 100,
                                                              params.txPow,
                                                              activeRbs0);
    Ptr<SpectrumSignalParameters> txParams = Create<SpectrumSignalParameters>();
    txParams->psd = txPsd->Copy();
    NS_LOG_DEBUG("Average tx power " << 10 * log10(Sum(*txPsd) * 180e3) << " dB");

    // create the noise PSD (LTE helper shown; use a SpectrumValueHelper
    // instance if you go the ISM route).
    Ptr<SpectrumValue> noisePsd =
        LteSpectrumValueHelper::CreateNoisePowerSpectralDensity(2100, 100,
                                                                params.noiseFigure);

    NS_LOG_DEBUG("Average noise power " << 10 * log10(Sum(*noisePsd) * 180e3) << " dB");

    // Zero pathloss :: already included in the SionnaRtSpectrumPropagationLossModel
    // so no need to add it here

    NS_ASSERT_MSG(params.txAntenna, "params.txAntenna is nullptr!");
    NS_ASSERT_MSG(params.rxAntenna, "params.rxAntenna is nullptr!");

    // apply the fast fading and the beamforming gain
    auto rxParams = m_spectrumLossModel->CalcRxPowerSpectralDensity(txParams,
                                                                    params.txMob,
                                                                    params.rxMob,
                                                                    params.txAntenna,
                                                                    params.rxAntenna);
    auto rxPsd = rxParams->psd;
    NS_LOG_DEBUG("Average rx power " << 10 * log10(Sum(*rxPsd) * 180e3) << " dB");
    // compute the SNR
    NS_LOG_DEBUG("Average SNR " << 10 * log10(Sum(*rxPsd) / Sum(*noisePsd)) << " dB");
    // print the SNR and pathloss values in the snr-trace.txt file
    std::ofstream f;
    f.open("snr-trace.txt", std::ios::out | std::ios::app);
    f << Simulator::Now().GetSeconds() << " " << 10 * log10(Sum(*rxPsd) / Sum(*noisePsd))
      << std::endl;
    f.close();
}


// connectivity helpers implementation

void
RecordNodePositions(NodeContainer nodes)
{
    std::vector<std::tuple<double,double,double>> current;
    for (auto it = nodes.Begin(); it != nodes.End(); ++it) {
        Ptr<Node> n = *it;
        Ptr<MobilityModel> m = n->GetObject<MobilityModel>();
        Vector p = m->GetPosition();
        current.emplace_back(p.x,p.y,p.z);
    }
    nodePositions.push_back(current);
}

void
SchedulePositionRecording(NodeContainer nodes)
{
    RecordNodePositions(nodes);
    if (Simulator::Now().GetSeconds() + 1.0 < g_simStopTime) {
        Simulator::Schedule(Seconds(1.0), &SchedulePositionRecording, nodes);
    }
}

void
ReceiveProbe(Ptr<Socket> socket)
{
    Ptr<Packet> pkt;
    Address from;
    while ((pkt = socket->RecvFrom(from))) {
        Ipv4Address addr = InetSocketAddress::ConvertFrom(from).GetIpv4();
        for (uint32_t i = 0; i < g_ifaces.GetN(); ++i) {
            if (g_ifaces.GetAddress(i) == addr) {
                Ptr<Node> recv = socket->GetNode();
                uint32_t rid = recv->GetId();
                receivedProbes[i][rid] = 1;
                break;
            }
        }
    }
}

void
SendProbes(Ptr<Node> sender, Ipv4InterfaceContainer ifaces)
{
    uint32_t sid = sender->GetId();
    if (sid == 0) {
        timeReceivedProbes.push_back(receivedProbes);
    }
    for (auto &row : receivedProbes) std::fill(row.begin(), row.end(), 0);
    for (uint32_t j = 0; j < ifaces.GetN(); ++j) {
        if (j == sid) continue;
        Ptr<Socket> sock = Socket::CreateSocket(sender, UdpSocketFactory::GetTypeId());
        InetSocketAddress dest(ifaces.GetAddress(j), 9999);
        sock->Connect(dest);
        Ptr<Packet> p = Create<Packet>((uint8_t*)&sid, sizeof(sid));
        sock->Send(p);
        sock->Close();
    }
    if (Simulator::Now().GetSeconds() + 1.0 < g_simStopTime) {
        Simulator::Schedule(Seconds(1.0), &SendProbes, sender, ifaces);
    }
}

int
main(int argc, char* argv[])
{
    py::scoped_interpreter guard{}; // Python stays alive for whole program

    // to check the python executable and version
    PrintPythonExecutable();

    double frequency = 2.4e9; // operating frequency in Hz (2.4 GHz)
    double txPow = 49.0;      // tx power in dBm
    double noiseFigure = 9.0; // noise figure in dB
    double distance = 50.0;   // distance between tx and rx nodes in meters
    double startTime = 2.0;     // simulation start time (for apps) in seconds
    double endTime = 30.0;  // simulation time in seconds
    double timeRes = 0.01;    // time resolution in seconds
    bool verbose = true;    // enable verbose logging

    std::string Scenario = "simple_street_canyon_with_cars"; // propagation scenario
    std::string SceneFile = "scratch/scene_wifi24.xml"; // Mitsuba scene XML file (relative or absolute path)
    std::string LayoutFile = "scratch/UrbanCompLayout.csv"; // CSV with node positions
    int numSource = 6; // default number of source nodes for traffic
    std::string routing = "olsr"; // routing protocol: olsr or aodv

    bool enableGnbIso = false;          // enable isotropic elements at gNB
    bool enableUeIso = false;           // enable isotropic elements at UE
    bool enableGnbDualPolarized = true; // enable dual-polarized elements at gNB
    bool enableUeDualPolarized = true;  // enable dual-polarized elements at UE

    bool IsImageRenderedEnabled = false;               // enable rendering of scene images to file
    Vector CameraPosition(Vector(70.0, -20.0, 190.0)); // Camera position
    Vector CameraLookAt(Vector(0.0, 0.0, 4.0));        // Camera look-at point
    std::string filename = "sionna-rt-scene3-";        // output file name for scene images
    std::string filedirectory = "sionna-rt-images2";   // output file directory for scene images

    // Sionna RT path solver configuration defaults
    SionnaRtChannelModel::RtPathSolverConfig RtPathSolverConfig;
    RtPathSolverConfig.maxDepth = 2;              // Maximum reflection/refraction depth
    RtPathSolverConfig.los = true;                // Include line-of-sight path
    RtPathSolverConfig.specularReflection = true; // Enable specular reflections
    RtPathSolverConfig.diffuseReflection = true;  // Enable diffuse reflections
    RtPathSolverConfig.diffraction = false;       // Disable diffraction
    RtPathSolverConfig.edge_diffraction = false;  // Disable edge diffraction
    RtPathSolverConfig.refraction = true;         // Enable refractions
    RtPathSolverConfig.syntheticArray = true;     // Use synthetic array processing
    RtPathSolverConfig.seed = 41;                 // Random seed

    CommandLine cmd;
    cmd.AddValue("Scenario", "Propagation scenario", Scenario);

    cmd.AddValue("frequency", "Operating frequency in Hz", frequency);

    cmd.AddValue("enableGnbIso", "Enable isotropic elements at gNB", enableGnbIso);
    cmd.AddValue("enableUeIso", "Enable isotropic elements at UE", enableUeIso);
    cmd.AddValue("enableGnbDualPolarized",
                 "Enable dual-polarized elements at gNB",
                 enableGnbDualPolarized);
    cmd.AddValue("enableUeDualPolarized",
                 "Enable dual-polarized elements at UE",
                 enableUeDualPolarized);

    cmd.AddValue("IsImageRenderedEnabled",
                 "Enable rendering of scene images to file",
                 IsImageRenderedEnabled);
    cmd.AddValue("outputFileName", "Output file name for scene images", filename);
    cmd.AddValue("outputFileDirectory", "Output file directory for scene images", filedirectory);
    cmd.AddValue("cameraPosition", "Camera position for scene rendering", CameraPosition);
    cmd.AddValue("cameraLookAt", "Camera look-at point for scene rendering", CameraLookAt);
    cmd.AddValue("SceneFile", "Path to Mitsuba scene XML file", SceneFile);
    cmd.AddValue("layoutFile", "Path to node layout CSV file", LayoutFile);
    cmd.AddValue("numSource", "Number of source nodes to create traffic", numSource);
    cmd.AddValue("routing", "Routing protocol: olsr or aodv", routing);

    cmd.AddValue("txPow", "Tx power in dBm", txPow);
    cmd.AddValue("noiseFigure", "Noise figure in dB", noiseFigure);
    cmd.AddValue("distance", "Distance between tx and rx nodes in meters", distance);

    cmd.AddValue("startTime", "Simulation start time in seconds", startTime);
    cmd.AddValue("endTime", "Simulation end time in seconds", endTime);
    cmd.AddValue("timeRes", "Time resolution in seconds", timeRes);

    cmd.AddValue("maxDepth", "Maximum reflection/refraction depth", RtPathSolverConfig.maxDepth);
    cmd.AddValue("los", "Include line-of-sight path", RtPathSolverConfig.los);
    cmd.AddValue("specularReflection",
                 "Enable specular reflections",
                 RtPathSolverConfig.specularReflection);
    cmd.AddValue("diffuseReflection",
                 "Enable diffuse reflections",
                 RtPathSolverConfig.diffuseReflection);
    cmd.AddValue("refraction", "Enable refractions", RtPathSolverConfig.refraction);
    cmd.AddValue("syntheticArray",
                 "Use synthetic array processing",
                 RtPathSolverConfig.syntheticArray);
    cmd.AddValue("diffraction", "Enable diffraction", RtPathSolverConfig.diffraction);
    cmd.AddValue("edge_diffraction",
                 "Enable edge diffraction",
                 RtPathSolverConfig.edge_diffraction);
    cmd.AddValue("seed", "Random seed", RtPathSolverConfig.seed);

    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("SionnaRTConnectExample", LOG_LEVEL_INFO);
    }

    // set the channel update period used by the Sionna RT channel model
    Config::SetDefault("ns3::SionnaRtChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(5))); // update the channel at each iteration

    RngSeedManager::SetSeed(RtPathSolverConfig.seed);
    RngSeedManager::SetRun(RtPathSolverConfig.seed);

    // create and configure the Sionna-RT spectrum propagation loss model
    m_spectrumLossModel = CreateObject<SionnaRtSpectrumPropagationLossModel>();
    m_spectrumLossModel->SetChannelModelAttribute("Frequency", DoubleValue(frequency));
    // If a Mitsuba scene XML file is provided, pass it to the channel model
    m_spectrumLossModel->SetChannelModelAttribute(
        "SceneFile",
        StringValue(SceneFile));
    // Also set the Scenario attribute for backward compatibility
    m_spectrumLossModel->SetChannelModelAttribute(
        "Scenario",
        StringValue(Scenario));

    // enable image rendering and set output filenames/paths if desired
    m_spectrumLossModel->SetChannelModelAttribute(
        "IsImageRenderedEnabled",
        BooleanValue(IsImageRenderedEnabled)); // enable rendering of scene images to file
    m_spectrumLossModel->SetChannelModelAttribute(
        "OutputImageDirectory",
        StringValue(filedirectory)); // set output directory for scene images
    m_spectrumLossModel->SetChannelModelAttribute(
        "OutputImageName",
        StringValue(filename)); // set the output file name for scene images

    // set up camera parameters used for scene rendering
    m_spectrumLossModel->SetChannelModelAttribute("CameraPosition", VectorValue(CameraPosition));
    m_spectrumLossModel->SetChannelModelAttribute("CameraLookAt", VectorValue(CameraLookAt));

    // apply the solver configuration
    m_spectrumLossModel->SetRtPathSolverConfig(RtPathSolverConfig);

    // Load node positions from CSV and create nodes accordingly
    std::vector<Vector> positions;
    {
        std::ifstream csv(LayoutFile);
        if (!csv.is_open())
        {
            std::cerr << "Warning: could not open layout file " << LayoutFile << ". Falling back to two-node default.\n";
            positions.push_back(Vector(-20.0, 0.0, 5.0));
            positions.push_back(Vector(distance - 20.0, 0.0, 5.0));
        }
        else
        {
            std::string line;
            // skip header
            std::getline(csv, line);
            while (std::getline(csv, line))
            {
                if (line.empty())
                    continue;
                std::stringstream ss(line);
                std::string item;
                double x = 0.0, y = 0.0;
                if (!std::getline(ss, item, ','))
                    continue;
                try { x = std::stod(item); } catch (...) { continue; }
                if (!std::getline(ss, item, ','))
                    continue;
                try { y = std::stod(item); } catch (...) { continue; }
                positions.push_back(Vector(x, y, 5.0)); // 5 m AGL
            }
            if (positions.empty())
            {
                positions.push_back(Vector(-20.0, 0.0, 5.0));
                positions.push_back(Vector(distance - 20.0, 0.0, 5.0));
            }
        }
    }

    NodeContainer nodes;
    nodes.Create(positions.size());

    // Configure Spectrum channel + Adhoc Aloha Ideal PHY (spectrum-based)
    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    // The SionnaRt model no longer derives from SpectrumPropagationLossModel in
    // recent ns-3 versions, so adding it to the spectrum channel results in a
    // hard compile error.  We keep the object around for the explicit SNR
    // computation later, but do not attach it to the channel; the channel will
    // fall back to the default (free‑space) behaviour.
    // channel->AddSpectrumPropagationLossModel(m_spectrumLossModel);
    //
    // build tx/noise PSDs using the LTE helper (default fallback).  This
    // mirrors the logic used in ComputeSnr(); we could also expose command
    // line options here if desired.
    std::vector<int> activeRbs0(100);
    std::iota(activeRbs0.begin(), activeRbs0.end(), 0);
    Ptr<SpectrumValue> txPsd =
        LteSpectrumValueHelper::CreateTxPowerSpectralDensity(2100, 100,
                                                              txPow,
                                                              activeRbs0);
    Ptr<SpectrumValue> noisePsd =
        LteSpectrumValueHelper::CreateNoisePowerSpectralDensity(2100, 100,
                                                                noiseFigure);

    AdhocAlohaNoackIdealPhyHelper deviceHelper;
    deviceHelper.SetChannel(channel);
    deviceHelper.SetTxPowerSpectralDensity(txPsd);
    deviceHelper.SetNoisePowerSpectralDensity(noisePsd);
    deviceHelper.SetPhyAttribute("Rate", DataRateValue(DataRate("1Mbps")));
    NetDeviceContainer devices = deviceHelper.Install(nodes);

    // Install Internet stack with selected routing
    InternetStackHelper stack;
    if (routing == "olsr")
    {
        Ipv4StaticRoutingHelper staticRouting;
        OlsrHelper olsr;
        Ipv4ListRoutingHelper list;
        list.Add(staticRouting, 0);
        list.Add(olsr, 10);
        stack.SetRoutingHelper(list);
    }
    else if (routing == "aodv")
    {
        AodvHelper aodv;
        stack.SetRoutingHelper(aodv);
    }
    else
    {
        NS_LOG_WARN("Unknown routing protocol, defaulting to OLSR");
        Ipv4StaticRoutingHelper staticRouting;
        OlsrHelper olsr;
        Ipv4ListRoutingHelper list;
        list.Add(staticRouting, 0);
        list.Add(olsr, 10);
        stack.SetRoutingHelper(list);
    }
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // --- connectivity addition ---
    g_ifaces = interfaces;
    receivedProbes.assign(nodes.GetN(), std::vector<uint32_t>(nodes.GetN(), 0));
    timeReceivedProbes.clear();
    nodePositions.clear();
    Simulator::Schedule(Seconds(2.0), &SchedulePositionRecording, nodes);
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<Node> n = nodes.Get(i);
        Ptr<Socket> sock = Socket::CreateSocket(n, UdpSocketFactory::GetTypeId());
        sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9999));
        sock->SetRecvCallback(MakeCallback(&ReceiveProbe));
    }
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Simulator::Schedule(Seconds(2.0), &SendProbes, nodes.Get(i), interfaces);
    }

    // --- attach IP layer tracing for metrics ---
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<Ipv4> ipv4 = nodes.Get(i)->GetObject<Ipv4>();
        ipv4->TraceConnectWithoutContext("Tx", MakeCallback(&IpPacketSentCallback));
        ipv4->TraceConnectWithoutContext("Rx", MakeCallback(&IpPacketReceivedCallback));
        ipv4->TraceConnectWithoutContext("Drop", MakeCallback(&IpPacketDropCallback));
    }

    // set mobility model and pattern using RandomWalk2dOutdoorMobilityModel
    double xMinBound = -750.0, xMaxBound = 750.0, yMinBound = -750.0, yMaxBound = 750.0;
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::RandomWalk2dOutdoorMobilityModel",
                              "Mode", StringValue("Time"),
                              "Time", TimeValue(Seconds(20.0)),
                              "Speed", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
                              "Bounds", RectangleValue(Rectangle(xMinBound, xMaxBound, yMinBound, yMaxBound)));
    mobility.Install(nodes);

    // set initial positions from CSV
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        nodes.Get(i)->GetObject<MobilityModel>()->SetPosition(positions[i]);
    }

    // select devices 0 and 1 as tx/rx for Sionna SNR computation
    Ptr<NetDevice> txDev = devices.Get(0);
    Ptr<NetDevice> rxDev = devices.Get( std::min<uint32_t>(1, devices.GetN() - 1) );

    // set up UDP client/server pairs for scaled connectivity
    int sources = std::min<int>(numSource, nodes.GetN() / 2);
    uint16_t port = 4000;
    for (int i = 0; i < sources; ++i) {
        uint32_t clientIdx = i;
        uint32_t serverIdx = (i + sources) % nodes.GetN();

        UdpServerHelper server(port);
        ApplicationContainer serverApp = server.Install(nodes.Get(serverIdx));
        serverApp.Start(Seconds(startTime));
        serverApp.Stop(Seconds(endTime));

        UdpClientHelper client(interfaces.GetAddress(serverIdx), port);
        client.SetAttribute("MaxPackets", UintegerValue(1000000));
        client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
        client.SetAttribute("PacketSize", UintegerValue(1024));
        ApplicationContainer clientApp = client.Install(nodes.Get(clientIdx));
        clientApp.Start(Seconds(startTime));
        clientApp.Stop(Seconds(endTime));
    }

    // retrieve mobility pointers for use in ComputeSnr (use nodes 0 and 1)
    Ptr<MobilityModel> txMob = nodes.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> rxMob = nodes.Get( std::min<uint32_t>(1, nodes.GetN() - 1) )->GetObject<MobilityModel>();

    // create the antenna objects and set their dimensions
    Ptr<PhasedArrayModel> txAntenna =
        CreateObjectWithAttributes<UniformPlanarArray>("NumColumns",
                                                       UintegerValue(1),
                                                       "NumRows",
                                                       UintegerValue(2));
    if (!enableGnbIso) {
        txAntenna->SetAttribute("AntennaElement",
                                PointerValue(CreateObject<ThreeGppAntennaModel>()));
    }

    if (enableGnbDualPolarized) {
        txAntenna->SetAttribute("IsDualPolarized", BooleanValue(true));
    }
    Ptr<PhasedArrayModel> rxAntenna =
        CreateObjectWithAttributes<UniformPlanarArray>("NumColumns",
                                                       UintegerValue(2),
                                                       "NumRows",
                                                       UintegerValue(2));
    if (!enableUeIso) {
        rxAntenna->SetAttribute("AntennaElement",
                                PointerValue(CreateObject<ThreeGppAntennaModel>()));
    }

    if (enableUeDualPolarized) {
        rxAntenna->SetAttribute("IsDualPolarized", BooleanValue(true));
    }
    // set the beamforming vectors
    DoBeamforming(txDev, txAntenna, rxDev);
    DoBeamforming(rxDev, rxAntenna, txDev);

    // schedule periodic SNR computations; track maximum scheduled time
    double lastSnrTime = 0.0;
    for (int i = 0; i < floor(endTime / timeRes); i++) {
        double t = timeRes * i;
        ComputeSnrParams params{txMob, rxMob, txPow, noiseFigure, txAntenna, rxAntenna};
        Simulator::Schedule(Seconds(t), &ComputeSnr, params);
        if (t > lastSnrTime) lastSnrTime = t;
    }
    if (lastSnrTime > (endTime + 5.0)) {
        std::cerr << "[warning] last Snr event (" << lastSnrTime << "s) > stop time (" << endTime + 5.0 << "s)" << std::endl;
    }

    // compute global stop time and install it
    g_simStopTime = endTime + 5.0;
    std::cout << "[info] simulation will stop at " << g_simStopTime << " seconds" << std::endl;
    Simulator::Schedule(Seconds(5), &LogSimTime);
    Simulator::Stop(Seconds(g_simStopTime));

    // check SNR scheduling boundaries
    {
        double lastSnr = 0.0;
        for (int i = 0; i < floor(endTime / timeRes); i++) {
            double t = timeRes * i;
            if (t > lastSnr) lastSnr = t;
        }
        if (lastSnr > g_simStopTime) {
            std::cerr << "[warning] last ComputeSnr event (" << lastSnr << "s) exceeds stop time (" << g_simStopTime << "s)" << std::endl;
        }
    }

    Simulator::Run();

    // write collected traffic metrics after simulation finishes
    WriteAllMetricsToCsv();

    // write connectivity & mobility result files
    std::ofstream connFile("connectivityM-rt.csv");
    for (auto &mat : timeReceivedProbes) {
        for (auto &row : mat) {
            for (auto v: row) connFile << v << ",";
            connFile << "\n";
        }
        connFile << "\n";
    }
    connFile.close();

    std::ofstream mobFile("mobility-rt.csv");
    for (auto &frame : nodePositions) {
        for (auto &pos : frame) {
            mobFile << std::get<0>(pos) << "," << std::get<1>(pos) << "," << std::get<2>(pos) << "\n";
        }
        mobFile << "\n";
    }
    mobFile.close();

    Simulator::Destroy();
    return 0;
}

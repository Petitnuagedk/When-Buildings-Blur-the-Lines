/**
 * FOBA-vs-Sionna Loss Comparison
 *
 * Two-node scenario: one static transmitter, one mobile receiver that walks
 * along a straight line passing behind a single building.  At each time step
 * we log the received power (or path loss) from both FOBA and Sionna-RT so
 * the two models can be compared on exactly the same geometry.
 *
 * Building layout (top view, y-axis up):
 *
 *     TX (static)                        Building                  
 *     (0,0,2)            [40,60] x [-10,10] x [0,15]
 *
 *                    RX moves along y=0 from x=-50 to x=+150
 *                    => LOS for x<40, NLOS for 40<x<60, LOS again for x>60
 *
 * Usage:
 *   ./ns3 run scratch/FobaSionnaComparison.cc -- [options]
 *
 * Outputs:
 *   <resultPath>/loss_comparison.csv
 *   Columns: time,rx_x,rx_y,rx_z,distance,foba_rxPow_dBm,sionna_rxPow_dBm
 */

#include "pybind11/pybind11.h"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/building.h"
#include "ns3/buildings-helper.h"
#include "ns3/mobility-building-info.h"

// FOBA (standard propagation loss model)
#include "ns3/propagation-loss-model.h"

// Sionna RT (spectrum-based)
#include "ns3/sionna-rt-channel-model.h"
#include "ns3/sionna-rt-spectrum-propagation-loss-model.h"
#include "ns3/lte-spectrum-value-helper.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/uniform-planar-array.h"

#include <fstream>
#include <numeric>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <vector>
#include <string>

NS_LOG_COMPONENT_DEFINE("FobaSionnaComparison");
namespace py = pybind11;
using namespace ns3;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static Ptr<SionnaRtSpectrumPropagationLossModel> g_sionnaLoss;
static Ptr<PropagationLossModel>                 g_fobaLoss;
static Ptr<PropagationLossModel>                 g_friisLoss;
static Ptr<PropagationLossModel>                 g_iturLoss;
static Ptr<PropagationLossModel>                 g_twoRayLoss;

static Ptr<MobilityModel> g_txMob;
static Ptr<MobilityModel> g_rxMob;

// Sionna antenna objects (needed for CalcRxPowerSpectralDensity)
static Ptr<PhasedArrayModel> g_txAntenna;
static Ptr<PhasedArrayModel> g_rxAntenna;

// Sionna PSD helpers
static Ptr<SpectrumValue> g_txPsd;
static Ptr<SpectrumValue> g_noisePsd;

static std::ofstream g_csv;

static std::string g_scenarioName;
static uint32_t g_runId   = 0;
static uint32_t g_seedId  = 0;
static double g_txPow   = 20.0;  // dBm
static double g_endTime  = 0.0;

struct ScenarioBuilding
{
    double xMin;
    double xMax;
    double yMin;
    double yMax;
    double height;
};

struct ScenarioConfig
{
    std::string name;
    std::string sceneFile;
    double txX;
    double txY;
    double txZ;
    std::vector<Vector> rxWaypoints;
    std::vector<ScenarioBuilding> buildings;
};

// ---------------------------------------------------------------------------
// Periodic measurement callback
// ---------------------------------------------------------------------------
static void
MeasureLoss()
{
    double t = Simulator::Now().GetSeconds();
    Vector rxPos = g_rxMob->GetPosition();
    double dist = g_txMob->GetDistanceFrom(g_rxMob);

    std::cout << "[measure] t=" << t << " pos=(" << rxPos.x << ","
              << rxPos.y << "," << rxPos.z << ") d=" << dist << std::endl;

    // Skip when TX and RX overlap — log10(0) causes SIGABRT in loss models
    if (dist < 0.1)
    {
        std::cout << "[measure] skipping (distance < 0.1 m)" << std::endl;
        if (t + 0.5 < g_endTime)
        {
            Simulator::Schedule(Seconds(0.5), &MeasureLoss);
        }
        return;
    }

    // ---- FOBA loss ----
    std::cout << "[measure] calling FOBA CalcRxPower ..." << std::flush;
    double fobaRxPow = g_fobaLoss->CalcRxPower(g_txPow, g_txMob, g_rxMob);
    std::cout << " OK -> " << fobaRxPow << " dBm" << std::endl;

    double friisRxPow = g_friisLoss->CalcRxPower(g_txPow, g_txMob, g_rxMob);
    double iturRxPow = g_iturLoss->CalcRxPower(g_txPow, g_txMob, g_rxMob);
    double twoRayRxPow = g_twoRayLoss->CalcRxPower(g_txPow, g_txMob, g_rxMob);

    // ---- Sionna RT loss ----
    double sionnaRxPow_dBm = -999.0;
    {
        std::cout << "[measure] creating Sionna txParams ..." << std::flush;
        Ptr<SpectrumSignalParameters> txParams =
            Create<SpectrumSignalParameters>();
        txParams->psd = g_txPsd->Copy();
        std::cout << " OK" << std::endl;

        std::cout << "[measure] calling Sionna CalcRxPowerSpectralDensity ..." << std::flush;
        auto rxParams = g_sionnaLoss->CalcRxPowerSpectralDensity(
            txParams, g_txMob, g_rxMob, g_txAntenna, g_rxAntenna);
        std::cout << " OK" << std::endl;

        std::cout << "[measure] integrating Sionna PSD ..." << std::flush;
        double rxPow_W = Sum(*rxParams->psd) * 180e3; // integrate over BW
        if (rxPow_W > 0.0)
        {
            sionnaRxPow_dBm = 10.0 * std::log10(rxPow_W) + 30.0; // W → dBm
        }
        std::cout << " OK -> " << sionnaRxPow_dBm << " dBm" << std::endl;
    }

    // ---- Log ----
    g_csv << std::fixed << std::setprecision(4)
            << g_scenarioName << ","
            << g_runId << ","
            << g_seedId << ","
          << t << ","
          << rxPos.x << "," << rxPos.y << "," << rxPos.z << ","
          << dist << ","
          << fobaRxPow << ","
          << friisRxPow << ","
          << iturRxPow << ","
          << twoRayRxPow << ","
          << sionnaRxPow_dBm << "\n";

    std::cout << "t=" << std::setw(6) << t
              << "  rx=(" << rxPos.x << "," << rxPos.y << ")"
              << "  d=" << std::setw(7) << dist
              << "  FOBA=" << std::setw(8) << fobaRxPow
              << "  Friis=" << std::setw(8) << friisRxPow
              << "  ITUR=" << std::setw(8) << iturRxPow
              << "  TwoRay=" << std::setw(8) << twoRayRxPow
              << "  Sionna=" << std::setw(8) << sionnaRxPow_dBm
              << " dBm\n";

    // schedule next measurement
    if (t + 0.5 < g_endTime)
    {
        Simulator::Schedule(Seconds(0.5), &MeasureLoss);
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int
main(int argc, char* argv[])
{
    py::scoped_interpreter guard{}; // Python stays alive for Sionna

    double frequency   = 2.4e9;   // Hz
    double txPow       = 20.0;    // dBm
    double noiseFigure  = 9.0;    // dB
    double speed        = 5.0;    // m/s
    double timeRes      = 0.5;    // seconds between samples
    double endTime      = 40.0;   // seconds
    int    seed         = 42;
    uint32_t numRuns    = 10;
    bool singleRun      = true;

    // Building geometry (single rectangular building)
    double bldgXmin = 40.0,  bldgXmax = 60.0;
    double bldgYmin = -10.0, bldgYmax = 10.0;
    double bldgH    = 15.0;

    // Node positions
    double txX = 50.0, txY = 50.0, txZ = 2.0;
    // RX starts at (-50, -40, 2) and walks in +x direction
    double rxStartX = -50.0, rxY = -40.0, rxZ = 2.0;

    std::string resultPath   = "scratch/foba-sionna-comparison";
    std::string sceneFile    = "scratch/scene_comparison.xml";
    std::string sceneFileScenario2 = "scratch/scene_comparison_scenario2.xml";
    std::string sceneFileScenario3 = "scratch/scene_comparison_scenario3.xml";
    std::string sceneFileScenario4 = "scratch/scene_comparison_scenario4.xml";

    CommandLine cmd;
    cmd.AddValue("frequency",   "Operating frequency (Hz)",     frequency);
    cmd.AddValue("txPow",       "TX power (dBm)",               txPow);
    cmd.AddValue("noiseFigure", "Noise figure (dB)",            noiseFigure);
    cmd.AddValue("speed",       "RX walking speed (m/s)",       speed);
    cmd.AddValue("timeRes",     "Measurement interval (s)",     timeRes);
    cmd.AddValue("endTime",     "Simulation duration (s)",      endTime);
    cmd.AddValue("seed",        "Random seed",                  seed);
    cmd.AddValue("numRuns",     "Number of independent runs",   numRuns);
    cmd.AddValue("singleRun",   "Run only one repetition per scenario", singleRun);
    cmd.AddValue("resultPath",  "Output directory",             resultPath);
    cmd.AddValue("sceneFile",   "Mitsuba XML scene for Sionna", sceneFile);
    cmd.AddValue("sceneFileScenario2",
                 "Mitsuba XML scene for Sionna scenario 2",
                 sceneFileScenario2);
    cmd.AddValue("sceneFileScenario3",
                 "Mitsuba XML scene for Sionna scenario 3",
                 sceneFileScenario3);
    cmd.AddValue("sceneFileScenario4",
                 "Mitsuba XML scene for Sionna scenario 4",
                 sceneFileScenario4);
    cmd.AddValue("bldgXmin",    "Building x-min",               bldgXmin);
    cmd.AddValue("bldgXmax",    "Building x-max",               bldgXmax);
    cmd.AddValue("bldgYmin",    "Building y-min",               bldgYmin);
    cmd.AddValue("bldgYmax",    "Building y-max",               bldgYmax);
    cmd.AddValue("bldgH",      "Building height (m)",           bldgH);
    cmd.AddValue("rxStartX",   "RX starting x position",        rxStartX);
    cmd.Parse(argc, argv);

    uint32_t runsToExecute = singleRun ? 1u : numRuns;

    g_txPow  = txPow;
    g_endTime = endTime;

    RngSeedManager::SetSeed(seed);

    std::vector<ScenarioBuilding> scenario3Buildings;
    for (int xCenter = -100; xCenter <= 100; xCenter += 50)
    {
        for (int yCenter = -100; yCenter <= 100; yCenter += 50)
        {
            if (xCenter == 0 && yCenter == 0)
            {
                continue;
            }

            scenario3Buildings.push_back({xCenter - 10.0,
                                          xCenter + 10.0,
                                          yCenter - 10.0,
                                          yCenter + 10.0,
                                          bldgH});
        }
    }

    std::vector<ScenarioConfig> scenarios = {
        {
            "scenario1",
            sceneFile,
            txX,
            txY,
            txZ,
            {Vector(rxStartX, rxY, rxZ), Vector(150.0, rxY, rxZ)},
            {{bldgXmin, bldgXmax, bldgYmin, bldgYmax, bldgH}},
        },
        {
            "scenario2",
            sceneFileScenario2,
            50.0,
            40.0,
            txZ,
            {Vector(-50.0, -40.0, rxZ), Vector(150.0, -40.0, rxZ)},
            {
                {45.0, 55.0, -5.0, 5.0, bldgH},
                {-5.0, 5.0, -5.0, 5.0, bldgH},
                {95.0, 105.0, -5.0, 5.0, bldgH},
            },
        },
        {
            "scenario3",
            sceneFileScenario3,
            0.0,
            0.0,
            txZ,
            {
                Vector(0.0, 25.0, rxZ),
                Vector(75.0, 25.0, rxZ),
                Vector(75.0, -75.0, rxZ),
                Vector(-125.0, -75.0, rxZ),
                Vector(-125.0, 125.0, rxZ),
            },
            scenario3Buildings,
        },
        {
            "scenario4",
            sceneFileScenario4,
            txX,
            txY,
            txZ,
            {Vector(rxStartX, rxY, rxZ), Vector(150.0, rxY, rxZ)},
            {{0.0, 100.0, bldgYmin, bldgYmax, bldgH}},
        },
    };

    // Create output directory
    std::filesystem::create_directories(resultPath);

    std::cout << "\n=== FOBA vs Sionna Loss Comparison ===\n"
              << "Frequency: " << frequency / 1e9 << " GHz\n"
              << "TX power: " << txPow << " dBm\n"
              << "Runs per scenario: " << runsToExecute << " (seed=" << seed;
    if (singleRun)
    {
        std::cout << ", singleRun=true)\n";
    }
    else
    {
        std::cout << ", run=1.." << runsToExecute << ")\n";
    }
    std::cout
              << "Scenarios: " << scenarios.size() << "\n\n";

    g_csv.open(resultPath + "/loss_comparison.csv");
    g_csv << "scenario,run,seed,time,rx_x,rx_y,rx_z,distance,"
             "foba_rxPow_dBm,friis_rxPow_dBm,itur_rxPow_dBm,two_ray_rxPow_dBm,"
             "sionna_rxPow_dBm\n";

    for (const auto& scenario : scenarios)
    {
        g_scenarioName = scenario.name;
        std::cout << "[" << scenario.name << "] TX=(" << scenario.txX << ","
                  << scenario.txY << "," << scenario.txZ << ") RX start=("
                  << scenario.rxWaypoints.front().x << ","
                  << scenario.rxWaypoints.front().y << ","
                  << scenario.rxWaypoints.front().z << ") buildings=" << scenario.buildings.size()
                  << std::endl;

        for (uint32_t runId = 1; runId <= runsToExecute; ++runId)
        {
            g_runId = runId;
            g_seedId = seed + runId - 1;
            RngSeedManager::SetRun(runId);

            std::cout << "[" << scenario.name << "] run " << runId << "/"
                      << runsToExecute << " setup" << std::endl;

            for (const auto& buildingConfig : scenario.buildings)
            {
                Ptr<Building> building = CreateObject<Building>();
                building->SetBoundaries(Box(buildingConfig.xMin,
                                            buildingConfig.xMax,
                                            buildingConfig.yMin,
                                            buildingConfig.yMax,
                                            0.0,
                                            buildingConfig.height));
                building->SetBuildingType(Building::Residential);
                building->SetExtWallsType(Building::ConcreteWithWindows);
            }

            std::cout << "[step 2] Creating nodes ..." << std::endl;
            NodeContainer nodes;
            nodes.Create(2);

            MobilityHelper txMobility;
            txMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
            txMobility.Install(nodes.Get(0));
            nodes.Get(0)->GetObject<MobilityModel>()->SetPosition(
                Vector(scenario.txX, scenario.txY, scenario.txZ));

            MobilityHelper rxMobility;
            rxMobility.SetMobilityModel("ns3::WaypointMobilityModel");
            rxMobility.Install(nodes.Get(1));
            Ptr<WaypointMobilityModel> rxMob =
                nodes.Get(1)->GetObject<WaypointMobilityModel>();

            double waypointTime = 0.0;
            rxMob->AddWaypoint(Waypoint(Seconds(waypointTime), scenario.rxWaypoints.front()));
            for (size_t waypointIndex = 1; waypointIndex < scenario.rxWaypoints.size(); ++waypointIndex)
            {
                Vector previous = scenario.rxWaypoints[waypointIndex - 1];
                Vector current = scenario.rxWaypoints[waypointIndex];
                double segmentDistance = CalculateDistance(previous, current);
                waypointTime += segmentDistance / speed;
                rxMob->AddWaypoint(Waypoint(Seconds(waypointTime), current));
            }

            double scenarioEndTime = std::max(endTime, waypointTime + timeRes);
            g_endTime = scenarioEndTime;

            g_txMob = nodes.Get(0)->GetObject<MobilityModel>();
            g_rxMob = nodes.Get(1)->GetObject<MobilityModel>();

            std::cout << "[step 2b] BuildingsHelper::Install ..." << std::endl;
            BuildingsHelper::Install(nodes);

            std::cout << "[step 3] FOBA TypeId lookup ..." << std::endl;
            {
                TypeId fobaTypeId;
                bool fobaFound = TypeId::LookupByNameFailSafe(
                    "ns3::FirstOrderBuildingsAwarePropagationLossModel", &fobaTypeId);
                if (!fobaFound)
                {
                    std::cout << "FATAL: TypeId 'ns3::FirstOrderBuildingsAwarePropagationLossModel' "
                                 "not found.  Is the FOBA contrib module compiled?" << std::endl;
                    return 1;
                }
                ObjectFactory fobaFactory;
                fobaFactory.SetTypeId(fobaTypeId);
                g_fobaLoss = DynamicCast<PropagationLossModel>(fobaFactory.Create());
                if (!g_fobaLoss)
                {
                    std::cout << "FATAL: DynamicCast<PropagationLossModel> returned nullptr "
                                 "for FOBA object." << std::endl;
                    return 1;
                }
                std::cout << "[step 3] FOBA OK" << std::endl;
            }

            g_friisLoss = CreateObject<FriisPropagationLossModel>();
            g_friisLoss->SetAttribute("Frequency", DoubleValue(frequency));
            g_friisLoss->SetAttribute("SystemLoss", DoubleValue(2.6));

            {
                TypeId iturTypeId;
                bool iturFound = TypeId::LookupByNameFailSafe(
                    "ns3::ItuR1411LosPropagationLossModel", &iturTypeId);
                if (!iturFound)
                {
                    std::cout << "FATAL: TypeId 'ns3::ItuR1411LosPropagationLossModel' "
                                 "not found." << std::endl;
                    return 1;
                }
                ObjectFactory iturFactory;
                iturFactory.SetTypeId(iturTypeId);
                g_iturLoss = DynamicCast<PropagationLossModel>(iturFactory.Create());
                if (!g_iturLoss)
                {
                    std::cout << "FATAL: DynamicCast<PropagationLossModel> returned nullptr "
                                 "for ITU-R object." << std::endl;
                    return 1;
                }
                g_iturLoss->SetAttribute("Frequency", DoubleValue(frequency));
            }

            g_twoRayLoss = CreateObject<TwoRayGroundPropagationLossModel>();
            g_twoRayLoss->SetAttribute("Frequency", DoubleValue(frequency));

            std::cout << "[step 4] Sionna RT setup ..." << std::endl;
            Config::SetDefault("ns3::SionnaRtChannelModel::UpdatePeriod",
                               TimeValue(MilliSeconds(100)));

            g_sionnaLoss = CreateObject<SionnaRtSpectrumPropagationLossModel>();
            g_sionnaLoss->SetChannelModelAttribute("Frequency", DoubleValue(frequency));
            g_sionnaLoss->SetChannelModelAttribute("SceneFile", StringValue(scenario.sceneFile));
            g_sionnaLoss->SetChannelModelAttribute("Scenario",
                                                   StringValue("simple_street_canyon"));
            g_sionnaLoss->SetChannelModelAttribute("IsImageRenderedEnabled",
                                                   BooleanValue(false));

            SionnaRtChannelModel::RtPathSolverConfig solverCfg;
            solverCfg.maxDepth          = 3;
            solverCfg.los               = true;
            solverCfg.specularReflection = true;
            solverCfg.diffuseReflection  = true;
            solverCfg.diffraction        = true;
            solverCfg.edge_diffraction   = true;
            solverCfg.refraction         = true;
            solverCfg.syntheticArray     = true;
            solverCfg.seed               = g_seedId;
            g_sionnaLoss->SetRtPathSolverConfig(solverCfg);
            std::cout << "[step 4] Sionna RT OK" << std::endl;

            std::cout << "[step 5] Antenna + PSD setup ..." << std::endl;
            PhasedArrayModel::ComplexVector bfWeight(1);
            bfWeight[0] = {1.0, 0.0};

            g_txAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
                "NumColumns", UintegerValue(1),
                "NumRows",    UintegerValue(1));
            g_txAntenna->SetBeamformingVector(bfWeight);

            g_rxAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
                "NumColumns", UintegerValue(1),
                "NumRows",    UintegerValue(1));
            g_rxAntenna->SetBeamformingVector(bfWeight);

            std::vector<int> activeRbs(100);
            std::iota(activeRbs.begin(), activeRbs.end(), 0);
            g_txPsd = LteSpectrumValueHelper::CreateTxPowerSpectralDensity(
                2100, 100, txPow, activeRbs);
            g_noisePsd = LteSpectrumValueHelper::CreateNoisePowerSpectralDensity(
                2100, 100, noiseFigure);

            std::cout << "[step 6] Starting simulation ..." << std::endl;
            Simulator::Schedule(Seconds(0.0), &MeasureLoss);
            Simulator::Stop(Seconds(scenarioEndTime));
            Simulator::Run();
            Simulator::Destroy();
        }
    }

    g_csv.close();
    std::cout << "\nResults written to " << resultPath << "/loss_comparison.csv\n";

    return 0;
}

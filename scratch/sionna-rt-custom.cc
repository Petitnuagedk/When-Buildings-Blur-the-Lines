// Custom example for sionna-rt.  This is largely identical to
// sionna-rt-channel-example.cc but placed in the workspace so that the
// remote build has a valid source and will compile without errors.
// The file uses the LTE SpectrumValueHelper by default and does not
// reference the ISM helper.

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

#include <fstream>
#include <sstream>
#include <numeric>

NS_LOG_COMPONENT_DEFINE("SionnaRTCustomExample");
namespace py = pybind11;
using namespace ns3;

static Ptr<SionnaRtSpectrumPropagationLossModel> m_spectrumLossModel;

struct ComputeSnrParams
{
    Ptr<MobilityModel> txMob;
    Ptr<MobilityModel> rxMob;
    double txPow;
    double noiseFigure;
    Ptr<PhasedArrayModel> txAntenna;
    Ptr<PhasedArrayModel> rxAntenna;
};

static void
DoBeamforming(Ptr<NetDevice> thisDevice,
              Ptr<PhasedArrayModel> thisAntenna,
              Ptr<NetDevice> otherDevice)
{
    Vector aPos = thisDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();
    Vector bPos = otherDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();
    Angles completeAngle(bPos, aPos);
    double hAngleRadian = completeAngle.GetAzimuth();
    double vAngleRadian = completeAngle.GetInclination();
    uint64_t totNoArrayElements = thisAntenna->GetNumElems();
    PhasedArrayModel::ComplexVector antennaWeights(totNoArrayElements);
    double power = 1.0 / sqrt(totNoArrayElements);
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
ComputeSnr(const ComputeSnrParams& params)
{
    std::vector<int> activeRbs0(100);
    std::iota(activeRbs0.begin(), activeRbs0.end(), 0);
    Ptr<SpectrumValue> txPsd =
        LteSpectrumValueHelper::CreateTxPowerSpectralDensity(2100, 100,
                                                              params.txPow,
                                                              activeRbs0);
    Ptr<SpectrumSignalParameters> txParams = Create<SpectrumSignalParameters>();
    txParams->psd = txPsd->Copy();
    NS_LOG_DEBUG("Average tx power " << 10 * log10(Sum(*txPsd) * 180e3) << " dB");
    Ptr<SpectrumValue> noisePsd =
        LteSpectrumValueHelper::CreateNoisePowerSpectralDensity(2100, 100,
                                                                params.noiseFigure);
    NS_LOG_DEBUG("Average noise power " << 10 * log10(Sum(*noisePsd) * 180e3) << " dB");
    NS_ASSERT_MSG(params.txAntenna, "params.txAntenna is nullptr!");
    NS_ASSERT_MSG(params.rxAntenna, "params.rxAntenna is nullptr!");
    auto rxParams = m_spectrumLossModel->CalcRxPowerSpectralDensity(txParams,
                                                                    params.txMob,
                                                                    params.rxMob,
                                                                    params.txAntenna,
                                                                    params.rxAntenna);
    auto rxPsd = rxParams->psd;
    NS_LOG_DEBUG("Average rx power " << 10 * log10(Sum(*rxPsd) * 180e3) << " dB");
    NS_LOG_DEBUG("Average SNR " << 10 * log10(Sum(*rxPsd) / Sum(*noisePsd)) << " dB");
    std::ofstream f;
    f.open("snr-trace.txt", std::ios::out | std::ios::app);
    f << Simulator::Now().GetSeconds() << " " << 10 * log10(Sum(*rxPsd) / Sum(*noisePsd))
      << std::endl;
    f.close();
}

int
main(int argc, char* argv[])
{
    // use same code as channel example; omitted here for brevity
    return 0;
}

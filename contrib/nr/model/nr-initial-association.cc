// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-initial-association.h"

#include "beamforming-vector.h"
#include "nr-gnb-net-device.h"
#include "nr-gnb-phy.h"
#include "nr-spectrum-phy.h"
#include "nr-ue-net-device.h"
#include "nr-ue-phy.h"

#include "ns3/double.h"
#include "ns3/net-device.h"
#include "ns3/nr-spectrum-value-helper.h"
#include "ns3/nr-wraparound-utils.h"
#include "ns3/object.h"
#include "ns3/parse-string-to-vector.h"
#include "ns3/spectrum-model.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <limits>
#include <numeric>

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("NrInitialAssociation");
NS_OBJECT_ENSURE_REGISTERED(NrInitialAssociation);

using LocalSearchParams = NrInitialAssociation::LocalSearchParams;

TypeId
NrInitialAssociation::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrInitialAssociation")
            .SetParent<Object>()
            .SetGroupName("Initial Access")
            .AddConstructor<NrInitialAssociation>()
            .AddAttribute(
                "HandoffMargin",
                "handoff margin (dB); UE attaches to any gNB whose RSRP is within hand off margin",
                DoubleValue(0),
                MakeDoubleAccessor(&NrInitialAssociation::SetHandoffMargin,
                                   &NrInitialAssociation::GetHandoffMargin),
                MakeDoubleChecker<double>())
            .AddAttribute("PrimaryCarrierIndex",
                          "primary carrier index",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrInitialAssociation::SetPrimaryCarrier,
                                               &NrInitialAssociation::GetPrimaryCarrier),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("NumMainInterfererGnb",
                          "Number of main interferer gNBs",
                          UintegerValue(6),
                          MakeUintegerAccessor(&NrInitialAssociation::SetNumMainInterfererGnb,
                                               &NrInitialAssociation::GetNumMainInterfererGnb),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("ColumnAngles",
                          "Column angles separated by |",
                          StringValue("0|90"),
                          MakeStringAccessor(&NrInitialAssociation::ParseColBeamAngles),
                          MakeStringChecker())
            .AddAttribute("RowAngles",
                          "Row angles separated by |",
                          StringValue("0|90"),
                          MakeStringAccessor(&NrInitialAssociation::ParseRowBeamAngles),
                          MakeStringChecker());

    return tid;
}

void
NrInitialAssociation::ParseColBeamAngles(std::string colAngles)
{
    SetColBeamAngles(ParseVBarSeparatedValuesStringToVector(colAngles));
}

void
NrInitialAssociation::ParseRowBeamAngles(std::string rowAngles)
{
    SetRowBeamAngles(ParseVBarSeparatedValuesStringToVector(rowAngles));
}

void
NrInitialAssociation::SetNumMainInterfererGnb(uint8_t numInterfere)
{
    m_numMainInterfererGnb = numInterfere;
}

uint8_t
NrInitialAssociation::GetNumMainInterfererGnb() const
{
    return m_numMainInterfererGnb;
}

Ptr<NetDevice>
NrInitialAssociation::GetAssociatedGnb() const
{
    NS_ASSERT(m_associatedGnb != nullptr);
    return m_associatedGnb;
}

int64_t
NrInitialAssociation::AssignStreams(int64_t stream)
{
    m_assocRng->SetStream(stream);
    return 1;
}

NetDeviceContainer
NrInitialAssociation::GetInterferingGnbs() const
{
    return m_intfGnbDevs;
}

double
NrInitialAssociation::GetMaxRsrp(uint64_t gnbId) const
{
    return 10 * log10(m_bestBeams[gnbId].rsrp);
}

NrAnglePair
NrInitialAssociation::GetBestBfv(uint64_t gnbId) const
{
    return m_bestBeams[gnbId].bfAngles;
}

double
NrInitialAssociation::GetRelativeRsrpRatio() const
{
    return m_rsrpRatio;
}

void
NrInitialAssociation::SetStartSsbRb(uint16_t startSsb)
{
    m_startSsb = startSsb;
}

void
NrInitialAssociation::SetNumSsbRb(uint16_t numSsbRb)
{
    m_numBandsSsb = numSsbRb;
}

bool
NrInitialAssociation::CheckNumBeamsAllowed() const
{
    NS_ASSERT_MSG(m_freq, "Error freq in initial association must set first");

    auto numGnbBeams = m_colBeamAngles.size() * m_rowBeamAngles.size();
    if (m_freq <= 3e9)
    {
        return (numGnbBeams <= 4);
    }
    else if (m_freq <= 6e9)
    {
        return (numGnbBeams <= 8);
    }
    else
    {
        return (numGnbBeams <= 64);
    }
}

void
NrInitialAssociation::SetHandoffMargin(double margin)
{
    m_handoffMargin = margin;
}

double
NrInitialAssociation::GetCarrierFrequency() const
{
    return m_freq;
}

double
NrInitialAssociation::GetHandoffMargin() const
{
    return m_handoffMargin;
}

std::vector<double>
NrInitialAssociation::GetRowBeamAngles() const
{
    return m_rowBeamAngles;
}

void
NrInitialAssociation::SetRowBeamAngles(std::vector<double> rowBfVect)
{
    m_rowBeamAngles = rowBfVect;
}

std::vector<double>
NrInitialAssociation::GetColBeamAngles() const
{
    return m_colBeamAngles;
}

void
NrInitialAssociation::SetColBeamAngles(std::vector<double> colBfVect)
{
    m_colBeamAngles = colBfVect;
}

Ptr<const NetDevice>
NrInitialAssociation::GetUeDevice() const
{
    return m_ueDevice;
}

void
NrInitialAssociation::SetUeDevice(const Ptr<NetDevice>& ueDev)
{
    m_ueDevice = ueDev;
}

void
NrInitialAssociation::SetGnbDevices(const NetDeviceContainer& gnbDevices)
{
    m_gnbDevices = gnbDevices;
    // Force a new search with these gNBs
    m_bestBeams.clear();
}

void
NrInitialAssociation::SetPrimaryCarrier(uint8_t index)
{
    m_primaryCarrierIndex = index;
}

uint8_t
NrInitialAssociation::GetPrimaryCarrier() const
{
    return m_primaryCarrierIndex;
}

LocalSearchParams
NrInitialAssociation::ExtractUeParameters() const
{
    auto ueDev = m_ueDevice->GetObject<NrUeNetDevice>();
    auto phy = ueDev->GetPhy(m_primaryCarrierIndex);
    auto spectrumPhy = phy->GetSpectrumPhy();
    auto spectrumChannel = spectrumPhy->GetSpectrumChannel();

    NS_ASSERT_MSG(spectrumChannel->GetPhasedArraySpectrumPropagationLossModel(),
                  "NrInitialAssociation requires channel fading. "
                  "Check NrChannelHelper or manually setup settings.");
    auto spectrumPropModel = StaticCast<ThreeGppSpectrumPropagationLossModel>(
        spectrumChannel->GetPhasedArraySpectrumPropagationLossModel());
    AntennaArrayModels antModel;
    for (uint8_t i = 0; i < spectrumPhy->GetNumPanels(); i++)
    {
        auto bPhasedArrayModel = spectrumPhy->GetPanelByIndex(i)->GetObject<PhasedArrayModel>();
        antModel.ueArrayModel.push_back(
            Copy<UniformPlanarArray>(DynamicCast<UniformPlanarArray>(bPhasedArrayModel)));
        antModel.ueArrayModel[i]->SetNumVerticalPorts(bPhasedArrayModel->GetNumRows());
        antModel.ueArrayModel[i]->SetNumHorizontalPorts(bPhasedArrayModel->GetNumColumns());
    }

    auto channel =
        StaticCast<ThreeGppSpectrumPropagationLossModel>(spectrumPropModel)->GetChannelModel();
    ChannelParams chParams;
    chParams.channelModel = StaticCast<ThreeGppChannelModel>(channel);
    chParams.spectrumPropModel = spectrumPropModel;
    NS_ASSERT_MSG(chParams.channelModel != nullptr,
                  "Channel Model should be to be ThreeGppChannelModel");

    Mobilities mobility;
    mobility.ueMobility = spectrumPhy->GetMobility();
    chParams.pathLossModel =
        StaticCast<ThreeGppPropagationLossModel>(spectrumChannel->GetPropagationLossModel());
    LocalSearchParams lsps{.chParams = chParams, .mobility = mobility, .antennaArrays = antModel};
    return lsps;
}

Ptr<UniformPlanarArray>
NrInitialAssociation::ExtractGnbParameters(const Ptr<NetDevice>& gnbDevice,
                                           LocalSearchParams& lsps) const
{
    auto& chParams = lsps.chParams;
    auto& mobility = lsps.mobility;
    auto& antenna = lsps.antennaArrays;

    const auto gnbDev = gnbDevice->GetObject<NrGnbNetDevice>();
    const auto phy = gnbDev->GetPhy(m_primaryCarrierIndex);
    const auto spectrumPhy = phy->GetSpectrumPhy();
    chParams.spectralModel = spectrumPhy->GetRxSpectrumModel();
    auto bPhasedArrayModel = spectrumPhy->GetAntenna()->GetObject<PhasedArrayModel>();

    // Local copy of antenna model is modified so actual model used after initial access is not
    // affected
    antenna.gnbArrayModel =
        Copy<UniformPlanarArray>(DynamicCast<UniformPlanarArray>(bPhasedArrayModel));
    mobility.gnbMobility = GetVirtualMobilityModel(spectrumPhy->GetSpectrumChannel(),
                                                   spectrumPhy->GetMobility(),
                                                   mobility.ueMobility);
    const auto rowElemsPerPort = antenna.gnbArrayModel->GetVElemsPerPort();
    const auto colElemsPerPort = antenna.gnbArrayModel->GetHElemsPerPort();

    // For initial access beams typically have wider beams so limit the beams to first port
    //  only.
    // This reduces the complexity of the channel model
    antenna.gnbArrayModel->SetNumVerticalPorts(1);
    antenna.gnbArrayModel->SetNumHorizontalPorts(1);
    antenna.gnbArrayModel->SetNumRows(rowElemsPerPort);
    antenna.gnbArrayModel->SetNumColumns(colElemsPerPort);

    return antenna.gnbArrayModel;
}

PhasedArrayModel::ComplexVector
NrInitialAssociation::GenBeamforming(double angRow,
                                     double angCol,
                                     Ptr<UniformPlanarArray> gnbArrayModel) const
{
    auto bfVector = CreateKroneckerBfv(gnbArrayModel, angRow, angCol);
    return bfVector;
}

double
NrInitialAssociation::ComputeRxPsd(Ptr<const SpectrumSignalParameters> spectrumSigParam) const
{
    auto& spectrumChannelMatrix = spectrumSigParam->spectrumChannelMatrix;
    NS_ASSERT_MSG(spectrumChannelMatrix->GetNumPages() >= m_startSsb + m_numBandsSsb,
                  "The primary carrier bandwidth should have at least 20 PRBs to fit SSBs");
    auto numUePorts = spectrumChannelMatrix->GetNumRows();
    double totalPsd = 0;
    for (size_t iRb = m_startSsb; iRb < m_startSsb + m_numBandsSsb; iRb++)
    {
        auto psdPerRb{0.0};
        // Compute the PSD from the MIMO channel matrix.
        for (size_t idxAnt = 0; idxAnt < numUePorts; idxAnt++)
        {
            psdPerRb += norm(spectrumChannelMatrix->Elem(idxAnt, 0, iRb));
        }
        totalPsd += psdPerRb;
    }
    return totalPsd;
}

NrInitialAssociation::BestBeam
NrInitialAssociation::ComputeMaxRsrp(const Ptr<NetDevice>& gnbDevice, LocalSearchParams& lsps) const
{
    auto& chParams = lsps.chParams;
    auto& mobility = lsps.mobility;
    auto& antennas = lsps.antennaArrays;
    BestBeam best{.panel = GetUeActivePanel()};
    antennas.gnbArrayModel = ExtractGnbParameters(gnbDevice, lsps);

    NS_ASSERT_MSG(chParams.spectralModel->GetNumBands() >= m_startSsb + m_numBandsSsb,
                  "The primary carrier bandwidth should have at least 20 PRBs to fit SSBs");
    std::vector<int> activeRbs;
    for (size_t rbId = m_startSsb; rbId < m_numBandsSsb + m_startSsb; rbId++)
    {
        activeRbs.push_back(rbId);
    }
    Ptr<const SpectrumValue> fakePsd = NrSpectrumValueHelper::CreateTxPowerSpectralDensity(
        DynamicCast<NrGnbNetDevice>(gnbDevice)->GetPhy(m_primaryCarrierIndex)->GetTxPower(),
        activeRbs,
        chParams.spectralModel,
        NrSpectrumValueHelper::UNIFORM_POWER_ALLOCATION_USED);
    auto txParams = Create<SpectrumSignalParameters>();
    for (auto& i : antennas.ueArrayModel)
    {
        PhasedArrayModel::ComplexVector uebfVector(i->GetNumElems());
        uebfVector[0] = 1.0;
        i->SetBeamformingVector(uebfVector);
    }

    // Compute the received power of each UE panel and gNB beam. The strongest pair of this gNB
    // sets its RSRP, best beam and best UE panel.
    for (size_t k = 0; k < antennas.ueArrayModel.size(); k++)
    {
        for (size_t j = 0; j < m_rowBeamAngles.size(); j++)
        {
            for (size_t i = 0; i < m_colBeamAngles.size(); i++)
            {
                auto bf =
                    GenBeamforming(m_rowBeamAngles[j], m_colBeamAngles[i], antennas.gnbArrayModel);
                antennas.gnbArrayModel->SetBeamformingVector(bf);
                txParams->psd = Copy<SpectrumValue>(fakePsd);
                auto rxParam = chParams.spectrumPropModel->DoCalcRxPowerSpectralDensity(
                    txParams,
                    mobility.gnbMobility,
                    mobility.ueMobility,
                    antennas.gnbArrayModel,
                    antennas.ueArrayModel[k],
                    bf,
                    antennas.ueArrayModel[k]->GetBeamformingVector());
                if (!rxParam->spectrumChannelMatrix)
                {
                    // out-of-range (see DistanceBasedThreeGppSpectrumPropagationLossModel)
                    continue;
                }
                // The channel matrix already carries the PSD of the gNB transmit power
                auto eng = ComputeRxPsd(rxParam);
                if (eng > best.rsrp)
                {
                    best.rsrp = eng;
                    best.bfAngles = {m_rowBeamAngles[j], m_colBeamAngles[i]};
                    best.panel = k;
                }
            }
        }
    }
    auto attenuation =
        chParams.pathLossModel->CalcRxPower(0, mobility.gnbMobility, mobility.ueMobility);
    best.rsrp *= pow(10.0, attenuation / 10.0);
    return best;
}

std::map<uint16_t, double>
NrInitialAssociation::GetCellRsrps()
{
    std::map<uint16_t, double> result;
    if (m_gnbDevices.GetN() == 0)
    {
        return result;
    }
    auto localParams = ExtractUeParameters();
    for (size_t i = 0; i < m_gnbDevices.GetN(); i++)
    {
        auto gnbDev = m_gnbDevices.Get(i);
        auto gnbNetDev = DynamicCast<NrGnbNetDevice>(gnbDev);
        if (!gnbNetDev)
        {
            continue;
        }
        double rsrpLin = ComputeMaxRsrp(gnbDev, localParams).rsrp;
        if (rsrpLin > 0)
        {
            result[gnbNetDev->GetCellId()] = 10.0 * std::log10(rsrpLin);
        }
    }
    return result;
}

double
NrInitialAssociation::ComputeRsrpRatio(const double totalRsrp, const std::vector<uint16_t> idxVal)
{
    NS_ASSERT_MSG(m_numIntfGnbs > 0,
                  "Number of main interfering gNBs should not be negative " << m_numIntfGnbs);
    NS_ASSERT_MSG(m_numIntfGnbs < m_bestBeams.size(),
                  "Number of main interfering gNBs should be less than the number of gNB ");
    double intfRsrp = 0;
    auto nIntf = 0;
    auto j = m_bestBeams.size() - nIntf;
    while (nIntf < int(m_numIntfGnbs))
    {
        j--;
        auto gnbDev = m_gnbDevices.Get(idxVal[j]);
        if (gnbDev != m_associatedGnb)
        {
            m_intfGnbDevs.Add(gnbDev);
            intfRsrp += m_bestBeams[idxVal[j]].rsrp;
            nIntf++;
        }
    }
    return (totalRsrp - intfRsrp) / intfRsrp;
}

void
NrInitialAssociation::InitializeIntfSet(uint16_t numIntf, bool useRelRsrp, double relRsrpThreshold)
{
    NS_ASSERT_MSG(!m_bestBeams.empty(), "Populate RSRP values first");
    NS_ASSERT_MSG(m_associatedGnb, "Association should be completed first");
    m_numIntfGnbs = numIntf;

    std::vector<uint16_t> idxVal(m_bestBeams.size());
    std::iota(idxVal.begin(), idxVal.end(), 0); // Initializing the indexes

    // getting the index of the gNBs in increasing order of received power to the UE
    std::sort(idxVal.begin(), idxVal.end(), [&](int i, int j) {
        return (GetMaxRsrp(i) < GetMaxRsrp(j));
    });

    auto cumSumIntf = GetInterference(idxVal);
    auto totalInterference = GetTotalInterference(cumSumIntf);
    NS_ASSERT_MSG(totalInterference > 0,
                  "Initial detected power of interferer should be greater than 0");

    m_numIntfGnbs = useRelRsrp
                        ? GetNumIntfGnbsByRelRsrp(cumSumIntf, relRsrpThreshold, totalInterference)
                        : m_numIntfGnbs;
    m_rsrpRatio = ComputeRsrpRatio(totalInterference, idxVal);
}

std::vector<double>
NrInitialAssociation::GetInterference(const std::vector<uint16_t>& idxVal) const
{
    std::vector<double> cumSumIntf(m_bestBeams.size()); // Cumulative sum of RSRP values from gNBs
    cumSumIntf[0] = m_bestBeams[idxVal[0]].rsrp;
    for (size_t i = 1; i < m_bestBeams.size(); ++i)
    {
        // Getting cumulative sum of received RSRP from gNB wherein RSRP are in increasing order
        cumSumIntf[i] = cumSumIntf[i - 1] + m_bestBeams[idxVal[i]].rsrp;
    }
    return cumSumIntf;
}

double
NrInitialAssociation::GetTotalInterference(const std::vector<double>& cumSumIntf) const
{
    auto totalInterference =
        cumSumIntf[m_bestBeams.size() - 1] -
        std::pow(10.0,
                 m_rsrpAsscGnb / 10.0); // subtract the power of the associated gNB to get
                                        // overall interference. Note that because hand off
                                        // margin, the associated gNB may not be the one with
                                        // highest received power
    return totalInterference;
}

size_t
NrInitialAssociation::GetNumIntfGnbsByRelRsrp(const std::vector<double> cumSumIntf,
                                              const double relRsrpThreshold,
                                              const double totalInterference) const
{
    auto numIntfGnbs = m_bestBeams.size() - 1;
    for (size_t i = 0; i < m_bestBeams.size(); ++i)
    {
        // This line is equivalent to  cumSumIntf[i] < relPsd * Main Interference.
        // Note that main interference is totalInterference - cumSumIntf[i]
        if ((1 + relRsrpThreshold) * cumSumIntf[i] > relRsrpThreshold * totalInterference)
        {
            numIntfGnbs -= i; // -1 for the associated gNB
            break;
        }
    }
    return numIntfGnbs;
}

void
NrInitialAssociation::PopulateRsrps(LocalSearchParams& lsps)
{
    m_bestBeams.resize(m_gnbDevices.GetN());
    for (size_t i = 0; i < m_gnbDevices.GetN(); i++)
    {
        m_bestBeams[i] = ComputeMaxRsrp(m_gnbDevices.Get(i), lsps);
    }
}

std::pair<Ptr<NetDevice>, double>
NrInitialAssociation::FindAssociatedGnb()
{
    auto localParams = ExtractUeParameters();
    m_freq = localParams.chParams.channelModel->GetFrequency();
    if (m_bestBeams.empty())
    {
        PopulateRsrps(localParams);
    }
    // Compute each RSRP once: recomputing it may round differently (e.g., FMA
    // contraction), and the strongest gNB would then not be within a zero margin
    std::vector<double> rsrpDb(m_bestBeams.size());
    double maxVal = -std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < m_bestBeams.size(); i++)
    {
        rsrpDb[i] = GetMaxRsrp(i);
        maxVal = std::max(maxVal, rsrpDb[i]);
    }
    NS_ABORT_MSG_IF(std::isinf(maxVal), "The UE is out of range of all the gNBs");

    // Keep gnbDevs with RSRP values lower than maxVal and handoffMargin
    std::vector<bool> assocFlag(m_bestBeams.size());
    for (size_t i = 0; i < m_bestBeams.size(); i++)
    {
        assocFlag[i] = (maxVal - rsrpDb[i]) <= m_handoffMargin;
    }
    auto numPossibleGnb = std::accumulate(assocFlag.begin(), assocFlag.end(), 0);

    // Choose Randomly gnbDev from possible gNB
    m_assocRng->SetAttribute("Min", DoubleValue(1));
    m_assocRng->SetAttribute("Max", DoubleValue(numPossibleGnb));
    auto value = m_assocRng->GetInteger();
    auto count = 0;
    for (size_t i = 0; i < m_gnbDevices.GetN(); i++)
    {
        count = (assocFlag[i] ? count + 1 : count);
        if (count == int(value))
        {
            m_associatedGnb = m_gnbDevices.Get(i);
            m_rsrpAsscGnb = rsrpDb[i];
            SetUeActivePanel(m_bestBeams[i].panel);
            return std::make_pair(m_associatedGnb, m_rsrpAsscGnb);
        }
    }
    NS_FATAL_ERROR("Method should have returned");
}

double
NrInitialAssociation::GetAssociatedRsrp() const
{
    return m_rsrpAsscGnb;
}

void
NrInitialAssociation::SetUeActivePanel(int8_t panelIndex) const
{
    auto ueDev = m_ueDevice->GetObject<NrUeNetDevice>();
    auto phy = ueDev->GetPhy(m_primaryCarrierIndex);
    auto spectrumPhy = phy->GetSpectrumPhy();
    spectrumPhy->SetActivePanel(panelIndex);
}

uint8_t
NrInitialAssociation::GetUeActivePanel() const
{
    auto ueDev = m_ueDevice->GetObject<NrUeNetDevice>();
    auto phy = ueDev->GetPhy(m_primaryCarrierIndex);
    auto spectrumPhy = phy->GetSpectrumPhy();
    for (uint8_t i = 0; i < spectrumPhy->GetNumPanels(); i++)
    {
        if (spectrumPhy->GetPanelByIndex(i) == spectrumPhy->GetAntenna())
        {
            return i;
        }
    }
    NS_ABORT_MSG("Missed the antenna panel");
}

} // namespace ns3

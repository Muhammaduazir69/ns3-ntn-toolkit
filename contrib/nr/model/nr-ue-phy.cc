// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
// Copyright (c) 2015 NYU WIRELESS, Tandon School of Engineering, New York University
// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#define NS_LOG_APPEND_CONTEXT                                                                      \
    do                                                                                             \
    {                                                                                              \
        std::clog << " [ CellId " << GetCellId() << ", bwpId " << GetBwpId() << "] ";              \
    } while (false);

#include "nr-ue-phy.h"

#include "beam-manager.h"
#include "nr-ch-access-manager.h"
#include "nr-radio-bearer-tag.h"
#include "nr-ue-net-device.h"
#include "nr-ue-power-control.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cfloat>

namespace ns3
{

const Time NR_DEFAULT_PMI_INTERVAL_WB{MilliSeconds(10)}; // Wideband PMI update interval
const Time NR_DEFAULT_PMI_INTERVAL_SB{MilliSeconds(2)};  // Subband PMI update interval

NS_LOG_COMPONENT_DEFINE("NrUePhy");
NS_OBJECT_ENSURE_REGISTERED(NrUePhy);

NrUePhy::NrUePhy()
{
    NS_LOG_FUNCTION(this);
    m_wbCqiLast = Simulator::Now();
    m_ueCphySapProvider = new MemberNrUeCphySapProvider<NrUePhy>(this);
    m_powerControl = CreateObject<NrUePowerControl>(this);
    m_isConnected = false;
    Simulator::Schedule(m_ueMeasurementsFilterPeriod, &NrUePhy::ReportUeMeasurements, this);
}

NrUePhy::~NrUePhy()
{
    NS_LOG_FUNCTION(this);
}

void
NrUePhy::DoDispose()
{
    NS_LOG_FUNCTION(this);
    delete m_ueCphySapProvider;
    if (m_powerControl)
    {
        m_powerControl->Dispose();
        m_powerControl = nullptr;
    }
    if (m_cam)
    {
        m_cam->Dispose();
        m_cam = nullptr;
    }
    NrPhy::DoDispose();
}

TypeId
NrUePhy::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrUePhy")
            .SetParent<NrPhy>()
            .AddConstructor<NrUePhy>()
            .AddAttribute("TxPower",
                          "Transmission power in dBm",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&NrUePhy::m_txPower),
                          MakeDoubleChecker<double>())
            .AddAttribute(
                "NoiseFigure",
                "Loss (dB) in the Signal-to-Noise-Ratio due to non-idealities in the receiver."
                " According to Wikipedia (http://en.wikipedia.org/wiki/Noise_figure), this is "
                "\"the difference in decibels (dB) between"
                " the noise output of the actual receiver to the noise output of an "
                " ideal receiver with the same overall gain and bandwidth when the receivers "
                " are connected to sources at the standard noise temperature T0.\" "
                "In this model, we consider T0 = 290K.",
                DoubleValue(5.0), // nr code from NYU and UniPd assumed in the code the value of
                                  // 5dB, that is why we configure the default value to that
                MakeDoubleAccessor(&NrPhy::SetNoiseFigure, &NrPhy::GetNoiseFigure),
                MakeDoubleChecker<double>())
            .AddAttribute(
                "PowerAllocationType",
                "Defines the type of the power allocation. Currently are supported "
                "two types: \"UniformPowerAllocBw\", which is a uniform power allocation over all "
                "bandwidth (over all RBs), and \"UniformPowerAllocBw\", which is a uniform "
                "power allocation over used (active) RBs. By default is set a uniform power "
                "allocation over used RBs .",
                EnumValue(NrSpectrumValueHelper::UNIFORM_POWER_ALLOCATION_USED),
                MakeEnumAccessor<NrSpectrumValueHelper::PowerAllocationType>(
                    &NrPhy::SetPowerAllocationType,
                    &NrPhy::GetPowerAllocationType),
                MakeEnumChecker(NrSpectrumValueHelper::UNIFORM_POWER_ALLOCATION_BW,
                                "UniformPowerAllocBw",
                                NrSpectrumValueHelper::UNIFORM_POWER_ALLOCATION_USED,
                                "UniformPowerAllocUsed"))
            .AddAttribute("SpectrumPhy",
                          "The SpectrumPhy associated to this NrPhy",
                          TypeId::ATTR_GET,
                          PointerValue(),
                          MakePointerAccessor(&NrPhy::GetSpectrumPhy),
                          MakePointerChecker<NrSpectrumPhy>())
            .AddAttribute("LBTThresholdForCtrl",
                          "After a DL/UL transmission, if we have less than this value to send the "
                          "UL CTRL, we consider the channel as granted",
                          TimeValue(MicroSeconds(25)),
                          MakeTimeAccessor(&NrUePhy::m_lbtThresholdForCtrl),
                          MakeTimeChecker())
            .AddAttribute("TbDecodeLatency",
                          "Transport block decode latency",
                          TimeValue(MicroSeconds(100)),
                          MakeTimeAccessor(&NrPhy::SetTbDecodeLatency, &NrPhy::GetTbDecodeLatency),
                          MakeTimeChecker())
            .AddAttribute("EnableUplinkPowerControl",
                          "If true, Uplink Power Control will be enabled.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrUePhy::SetEnableUplinkPowerControl),
                          MakeBooleanChecker())
            .AddAttribute("WbPmiUpdateInterval",
                          "Wideband PMI update interval",
                          TimeValue(NR_DEFAULT_PMI_INTERVAL_WB),
                          MakeTimeAccessor(&NrUePhy::m_wbPmiUpdateInterval),
                          MakeTimeChecker())
            .AddAttribute("SbPmiUpdateInterval",
                          "Subband PMI update interval",
                          TimeValue(NR_DEFAULT_PMI_INTERVAL_SB),
                          MakeTimeAccessor(&NrUePhy::m_sbPmiUpdateInterval),
                          MakeTimeChecker())
            .AddAttribute("AlphaCovMat",
                          "The alpha parameter for the calculation of the interference covariance "
                          "matrix moving average",
                          DoubleValue(1),
                          MakeDoubleAccessor(&NrUePhy::SetAlphaCovMat, &NrUePhy::GetAlphaCovMat),
                          MakeDoubleChecker<double>(0.0, 1))
            .AddAttribute(
                "CsiImDuration",
                "CSI-IM duration in the number of OFDM symbols",
                UintegerValue(1),
                MakeUintegerAccessor(&NrUePhy::SetCsiImDuration, &NrUePhy::GetCsiImDuration),
                MakeUintegerChecker<uint8_t>(1, 12))
            // The reported value is NrUePhy::ComputeAvgSinr(): a linear SINR (apply 10*log10() for
            // dB) averaged over the RBs of the received PDSCH. Fired only in SISO CQI feedback
            // configurations (CsiFeedbackFlags = CQI_PDSCH_SISO, or a channel without a
            // phased-array loss model): the value is the per-RB C/(I+N) computed by NrInterference
            // (noise included), emitted at the end of each PDSCH data reception, and feeds the SISO
            // wideband CQI. MIMO CSI feedback configurations do not fire this trace; see
            // CqiFeedbackTrace.
            .AddTraceSource("DlDataSinr",
                            "Effective DL data SINR used for CQI/CSI feedback, reported as a "
                            "linear ratio (not dB) averaged over the allocated RBs.",
                            MakeTraceSourceAccessor(&NrUePhy::m_dlDataSinrTrace),
                            "ns3::NrUePhy::DlDataSinrTracedCallback")
            .AddTraceSource("DlCtrlSinr",
                            "Report the SINR computed for DL CTRL",
                            MakeTraceSourceAccessor(&NrUePhy::m_dlCtrlSinrTrace),
                            "ns3::NrUePhy::DlCtrlSinrTracedCallback")
            .AddAttribute("UeMeasurementsFilterPeriod",
                          "Time period for reporting UE measurements, i.e., the"
                          "length of layer-1 filtering.",
                          TimeValue(MilliSeconds(200)),
                          MakeTimeAccessor(&NrUePhy::m_ueMeasurementsFilterPeriod),
                          MakeTimeChecker())
            .AddTraceSource("ReportUplinkTbSize",
                            "Report allocated uplink TB size for trace.",
                            MakeTraceSourceAccessor(&NrUePhy::m_reportUlTbSize),
                            "ns3::UlTbSize::TracedCallback")
            .AddTraceSource("ReportDownlinkTbSize",
                            "Report allocated downlink TB size for trace.",
                            MakeTraceSourceAccessor(&NrUePhy::m_reportDlTbSize),
                            "ns3::DlTbSize::TracedCallback")
            .AddTraceSource("ReportRsrp",
                            "RSRP statistics.",
                            MakeTraceSourceAccessor(&NrUePhy::m_reportRsrpTrace),
                            "ns3::CurrentRsrp::TracedCallback")
            .AddTraceSource("UePhyRxedCtrlMsgsTrace",
                            "Ue PHY Control Messages Traces.",
                            MakeTraceSourceAccessor(&NrUePhy::m_phyRxedCtrlMsgsTrace),
                            "ns3::NrPhyRxTrace::RxedUePhyCtrlMsgsTracedCallback")
            .AddTraceSource("UePhyTxedCtrlMsgsTrace",
                            "Ue PHY Control Messages Traces.",
                            MakeTraceSourceAccessor(&NrUePhy::m_phyTxedCtrlMsgsTrace),
                            "ns3::NrPhyRxTrace::TxedUePhyCtrlMsgsTracedCallback")
            .AddTraceSource("UePhyRxedDlDciTrace",
                            "Ue PHY DL DCI Traces.",
                            MakeTraceSourceAccessor(&NrUePhy::m_phyUeRxedDlDciTrace),
                            "ns3::NrPhyRxTrace::RxedUePhyDlDciTracedCallback")
            .AddTraceSource("UePhyTxedHarqFeedbackTrace",
                            "Ue PHY DL HARQ Feedback Traces.",
                            MakeTraceSourceAccessor(&NrUePhy::m_phyUeTxedHarqFeedbackTrace),
                            "ns3::NrPhyRxTrace::TxedUePhyHarqFeedbackTracedCallback")
            .AddTraceSource("ReportPowerSpectralDensity",
                            "Power Spectral Density data.",
                            MakeTraceSourceAccessor(&NrUePhy::m_reportPowerSpectralDensity),
                            "ns3::NrUePhy::PowerSpectralDensityTracedCallback")
            .AddTraceSource("CqiFeedbackTrace",
                            "Mimo CQI feedback traces containing RNTI, WB CQI, MCS, and RI ",
                            MakeTraceSourceAccessor(&NrUePhy::m_cqiFeedbackTrace),
                            "ns3::NrUePhy::CqiFeedbackTracedCallback")
            .AddTraceSource("ReportUeMeasurements",
                            "Report UE measurements RSRP (dBm) and RSRQ (dB).",
                            MakeTraceSourceAccessor(&NrUePhy::m_reportUeMeasurements),
                            "ns3::NrUePhy::RsrpRsrqTracedCallback")
            .AddAttribute("Qout",
                          "corresponds to 10% block error rate of a hypothetical PDCCH transmission"
                          "taking into account the PCFICH errors with transmission parameters."
                          "see 3GPP TS 36.213 4.2.1 and TS 36.133 7.6",
                          DoubleValue(-5),
                          MakeDoubleAccessor(&NrUePhy::m_qOut),
                          MakeDoubleChecker<double>())
            .AddAttribute("Qin",
                          "corresponds to 2% block error rate of a hypothetical PDCCH transmission"
                          "taking into account the PCFICH errors with transmission parameters."
                          "see 3GPP TS 36.213 4.2.1 and TS 36.133 7.6",
                          DoubleValue(-3.9),
                          MakeDoubleAccessor(&NrUePhy::m_qIn),
                          MakeDoubleChecker<double>())
            .AddAttribute(
                "NumQoutEvalSf",
                "This specifies the total number of consecutive subframes"
                "which corresponds to the Qout evaluation period",
                UintegerValue(200), // see 3GPP 3GPP TS 36.133 7.6.2.1
                MakeUintegerAccessor(&NrUePhy::SetNumQoutEvalSf, &NrUePhy::GetNumQoutEvalSf),
                MakeUintegerChecker<uint16_t>())
            .AddAttribute(
                "NumQinEvalSf",
                "This specifies the total number of consecutive subframes"
                "which corresponds to the Qin evaluation period",
                UintegerValue(100), // see 3GPP 3GPP TS 36.133 7.6.2.1
                MakeUintegerAccessor(&NrUePhy::SetNumQinEvalSf, &NrUePhy::GetNumQinEvalSf),
                MakeUintegerChecker<uint16_t>())
            .AddAttribute("EnableRlfDetection",
                          "If true, RLF detection will be enabled.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrUePhy::m_enableRlfDetection),
                          MakeBooleanChecker())
            .AddAttribute(
                "SsRsrpOnOccupiedRbs",
                "ntn-patch 19 (TS 38.215 5.1.1: SS-RSRP is the linear average over the REs "
                "that carry the SSS): average the PSS power per RB over the RBs that carry the "
                "cell's SSB, i.e. those with a non-zero received PSD (nr's own convention, "
                "NrAmc: 'SINR == 0 means no signal in this RB'). Needed when cells send their "
                "control region in a sub-band (frequency reuse); false (nr default) averages "
                "over every RB of the BWP.",
                BooleanValue(false),
                MakeBooleanAccessor(&NrUePhy::m_ssRsrpOnOccupiedRbs),
                MakeBooleanChecker())
            .AddAttribute(
                "SsbBeamsPerBurst",
                "ntn-patch 19: number of SSB beams the serving cell sweeps (TS 38.213 4.1, "
                "ssb-PositionsInBurst). The path-loss reference of uplink power control is then "
                "the best SSB of the last burst (TS 38.213 7.1.1, the SS/PBCH block the UE "
                "uses as q_d) instead of the last control region received, which is sent on "
                "whichever beam the sweep is at. 0 (default): no sweep, nr behavior.",
                UintegerValue(0),
                MakeUintegerAccessor(&NrUePhy::m_ssbBeamsPerBurst),
                MakeUintegerChecker<uint32_t>(0, 255))
            .AddAttribute(
                "SsbBeamResultLifetime",
                "ntn-patch 21: how long a per-SSB-beam layer-1 result is kept when the beam is "
                "not measured again (a sweep longer than the layer-1 period). The cell quality "
                "is the best of the kept results (TS 38.331 5.5.3.3). 0 (default): only the "
                "beams measured in the current period count (patch 19).",
                TimeValue(Seconds(0)),
                MakeTimeAccessor(&NrUePhy::m_ssbBeamResultLifetime),
                MakeTimeChecker(Seconds(0)))
            .AddAttribute(
                "InitialBandwidthRbs",
                "ntn-patch 19: RBs of the UE's initial (pre-SIB) receive bandwidth, centred on the "
                "carrier. 0 (default) is nr's 6 RBs. Set to the BWP's RB count when cells send "
                "their SSB in reuse sub-bands, so the UE finds its cell's SSB anywhere in the BWP.",
                UintegerValue(0),
                MakeUintegerAccessor(&NrUePhy::m_initialBandwidthRbs),
                MakeUintegerChecker<uint32_t>())
            .AddAttribute("DlHarqFeedbackDisabled",
                          "NTN (TS 38.331 downlinkHARQ-FeedbackDisabled-r17, TS 38.213 9.1.2): "
                          "bitmap over DL HARQ process IDs, bit i set = the UE does not report "
                          "HARQ-ACK on PUCCH for a PDSCH received on process i. Must match the "
                          "gNB scheduler's DlHarqFeedbackDisabled. 0 (default) reports every "
                          "process, the terrestrial behavior.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrUePhy::m_dlHarqFeedbackDisabled),
                          MakeUintegerChecker<uint32_t>())
            .AddTraceSource("DlHarqFeedbackSuppressed",
                            "A DL HARQ-ACK that was not sent on PUCCH because its HARQ process "
                            "has feedback disabled (RNTI, HARQ process ID).",
                            MakeTraceSourceAccessor(&NrUePhy::m_dlHarqFeedbackSuppressedTrace),
                            "ns3::NrUePhy::DlHarqFeedbackSuppressedTracedCallback");
    return tid;
}

void
NrUePhy::ChannelAccessGranted([[maybe_unused]] const Time& time)
{
    NS_LOG_FUNCTION(this);
    // That will be granted only till the end of the slot
    m_channelStatus = GRANTED;
}

void
NrUePhy::ChannelAccessDenied()
{
    NS_LOG_FUNCTION(this);
    m_channelStatus = NONE;
}

void
NrUePhy::SetUeCphySapUser(NrUeCphySapUser* s)
{
    NS_LOG_FUNCTION(this);
    m_ueCphySapUser = s;
}

NrUeCphySapProvider*
NrUePhy::GetUeCphySapProvider()
{
    NS_LOG_FUNCTION(this);
    return (m_ueCphySapProvider);
}

void
NrUePhy::SetEnableUplinkPowerControl(bool enable)
{
    m_enableUplinkPowerControl = enable;
}

void
NrUePhy::SetAlphaCovMat(double alpha)
{
    m_alphaCovMat = alpha;
}

double
NrUePhy::GetAlphaCovMat() const
{
    return m_alphaCovMat;
}

void
NrUePhy::SetCsiImDuration(uint8_t csiImDuration)
{
    m_csiImDuration = csiImDuration;
}

uint8_t
NrUePhy::GetCsiImDuration() const
{
    return m_csiImDuration;
}

void
NrUePhy::SetTxPower(double pow)
{
    m_txPower = pow;
    m_powerControl->SetTxPower(pow);
}

double
NrUePhy::GetTxPower() const
{
    return m_txPower;
}

double
NrUePhy::GetRsrp() const
{
    return m_rsrp;
}

void
NrUePhy::SetNumQoutEvalSf(uint16_t numSubframes)
{
    NS_LOG_FUNCTION(this << numSubframes);
    NS_ABORT_MSG_IF(numSubframes % 10 != 0,
                    "Number of subframes used for Qout "
                    "evaluation must be multiple of 10");
    m_numOfQoutEvalSf = numSubframes;
}

void
NrUePhy::SetNumQinEvalSf(uint16_t numSubframes)
{
    NS_LOG_FUNCTION(this << numSubframes);
    NS_ABORT_MSG_IF(numSubframes % 10 != 0,
                    "Number of subframes used for Qin "
                    "evaluation must be multiple of 10");
    m_numOfQinEvalSf = numSubframes;
}

uint16_t
NrUePhy::GetNumQoutEvalSf() const
{
    return m_numOfQoutEvalSf;
}

uint16_t
NrUePhy::GetNumQinEvalSf() const
{
    return m_numOfQinEvalSf;
}

Ptr<NrUePowerControl>
NrUePhy::GetUplinkPowerControl() const
{
    NS_LOG_FUNCTION(this);
    return m_powerControl;
}

NrPhySapProvider::PuschPowerHeadroom
NrUePhy::GetPuschPowerHeadroom(const std::vector<bool>& rbgBitmask) const
{
    NrPhySapProvider::PuschPowerHeadroom ph;
    if (!m_enableUplinkPowerControl || !m_powerControl ||
        !m_powerControl->HasPathLossEstimate())
    {
        return ph;
    }
    const auto rbs = FromRBGBitmaskToRBAssignment(rbgBitmask);
    if (rbs.empty())
    {
        return ph;
    }
    ph.valid = true;
    ph.numRb = static_cast<uint32_t>(rbs.size());
    ph.phDb = m_powerControl->GetPuschPowerHeadroom(rbs.size());
    ph.pcmaxDbm = m_powerControl->GetPcmax();
    ph.pathlossDb = m_powerControl->GetPathLoss();
    return ph;
}

void
NrUePhy::SetUplinkPowerControl(Ptr<NrUePowerControl> pc)
{
    m_powerControl = pc;
}

void
NrUePhy::SetDlAmc(const Ptr<const NrAmc>& amc)
{
    m_amc = amc;

    if (m_pmSearch)
    {
        m_pmSearch->SetAmc(amc);
    }
}

void
NrUePhy::SetSubChannelsForTransmission(const std::vector<int>& mask, uint32_t numSym)
{
    Ptr<SpectrumValue> txPsd = GetTxPowerSpectralDensity(mask);
    NS_ASSERT(txPsd);

    m_reportPowerSpectralDensity(m_currentSlot,
                                 txPsd,
                                 numSym * GetSymbolPeriod(),
                                 m_rnti,
                                 m_imsi,
                                 GetBwpId(),
                                 GetCellId());
    m_spectrumPhy->SetTxPowerSpectralDensity(txPsd);
}

void
NrUePhy::DoSendControlMessage(Ptr<NrControlMessage> msg)
{
    NS_LOG_FUNCTION(this << msg);
    EnqueueCtrlMessage(msg);
}

void
NrUePhy::DoSendControlMessageNow(Ptr<NrControlMessage> msg)
{
    NS_LOG_FUNCTION(this << msg);
    EnqueueCtrlMsgNow(msg);
}

void
NrUePhy::ProcessDataDci(const SfnSf& ulSfnSf,
                        const std::shared_ptr<DciInfoElementTdma>& dciInfoElem)
{
    NS_LOG_FUNCTION(this);

    NS_LOG_DEBUG("UE" << m_rnti << " UL-DCI received for slot " << ulSfnSf << " symStart "
                      << static_cast<uint32_t>(dciInfoElem->m_symStart) << " numSym "
                      << static_cast<uint32_t>(dciInfoElem->m_numSym) << " tbs "
                      << dciInfoElem->m_tbSize << " harqId "
                      << static_cast<uint32_t>(dciInfoElem->m_harqProcess));

    if (ulSfnSf == m_currentSlot)
    {
        InsertAllocation(dciInfoElem);
    }
    else
    {
        InsertFutureAllocation(ulSfnSf, dciInfoElem);
    }
}

void
NrUePhy::SendRachPreamble(uint32_t PreambleId, uint32_t Rnti)
{
    NS_LOG_FUNCTION(this << PreambleId);
    m_raPreambleId = PreambleId;
    Ptr<NrRachPreambleMessage> msg = Create<NrRachPreambleMessage>();
    msg->SetSourceBwpArfcn(DoGetArfcn());
    msg->SetRapId(PreambleId);
    EnqueueCtrlMsgNow(msg);
}

void
NrUePhy::ProcessSrsDci(const SfnSf& ulSfnSf, const std::shared_ptr<DciInfoElementTdma>& dciInfoElem)
{
    NS_LOG_FUNCTION(this);
    // Instruct PHY for transmitting the SRS
    if (ulSfnSf == m_currentSlot)
    {
        InsertAllocation(dciInfoElem);
    }
    else
    {
        InsertFutureAllocation(ulSfnSf, dciInfoElem);
    }
}

void
NrUePhy::RegisterToGnb(uint16_t cellId)
{
    NS_LOG_FUNCTION(this);

    InitializeMessageList();
    DoSetCellId(cellId);
}

void
NrUePhy::SetUlCtrlSyms(uint8_t ulCtrlSyms)
{
    m_ulCtrlSyms = ulCtrlSyms;
}

void
NrUePhy::SetDlCtrlSyms(uint8_t dlCtrlSyms)
{
    m_dlCtrlSyms = dlCtrlSyms;
}

void
NrUePhy::SetNumRbPerRbg(uint32_t numRbPerRbg)
{
    m_numRbPerRbg = numRbPerRbg;
}

void
NrUePhy::SetPattern(const std::string& pattern)
{
    NS_LOG_FUNCTION(this);

    static std::unordered_map<std::string, LteNrTddSlotType> lookupTable = {
        {"DL", LteNrTddSlotType::DL},
        {"UL", LteNrTddSlotType::UL},
        {"S", LteNrTddSlotType::S},
        {"F", LteNrTddSlotType::F},
    };

    std::vector<LteNrTddSlotType> vector;
    std::stringstream ss(pattern);
    std::string token;
    std::vector<std::string> extracted;

    while (std::getline(ss, token, '|'))
    {
        extracted.push_back(token);
    }

    vector.reserve(extracted.size());
    for (const auto& v : extracted)
    {
        vector.push_back(lookupTable[v]);
    }

    m_tddPattern = vector;
    // Check if pure UL BWP
    const auto ulSlots = std::count(m_tddPattern.begin(), m_tddPattern.end(), LteNrTddSlotType::UL);
    if (static_cast<size_t>(ulSlots) == m_tddPattern.size())
    {
        // In case Downlink CSI feedback is enabled, disable it
        m_csiFeedbackType = 0;
    }
}

uint32_t
NrUePhy::GetNumRbPerRbg() const
{
    return m_numRbPerRbg;
}

void
NrUePhy::SetCurrentSfnSf(const SfnSf& currentSfnSf)
{
    m_currentSlot = currentSfnSf;
}

void
NrUePhy::SetLastSlotStart(Time startTime)
{
    m_lastSlotStart = startTime;
}

Time
NrUePhy::GetLastSlotStart() const
{
    return m_lastSlotStart;
}

NrUePhySapUser*
NrUePhy::GetPhySapUser() const
{
    return m_phySapUser;
}

double
NrUePhy::ComputeAvgSinr(const SpectrumValue& sinr)
{
    // averaged SINR among RBs
    double sum = 0.0;
    uint16_t rbNum = 0;
    Values::const_iterator it;

    for (it = sinr.ConstValuesBegin(); it != sinr.ConstValuesEnd(); it++)
    {
        sum += (*it);
        rbNum++;
    }

    double avrgSinr = (rbNum > 0) ? (sum / rbNum) : DBL_MAX;

    return avrgSinr;
}

void
NrUePhy::InsertAllocation(const std::shared_ptr<DciInfoElementTdma>& dci)
{
    NS_LOG_FUNCTION(this);

    VarTtiAllocInfo varTtiInfo(dci);
    m_currSlotAllocInfo.m_varTtiAllocInfo.push_back(varTtiInfo);
    std::stable_sort(m_currSlotAllocInfo.m_varTtiAllocInfo.begin(),
                     m_currSlotAllocInfo.m_varTtiAllocInfo.end());
}

void
NrUePhy::InsertFutureAllocation(const SfnSf& sfnSf, const std::shared_ptr<DciInfoElementTdma>& dci)
{
    NS_LOG_FUNCTION(this);

    VarTtiAllocInfo varTtiInfo(dci);
    if (SlotAllocInfoExists(sfnSf))
    {
        auto& ulSlot = PeekSlotAllocInfo(sfnSf);
        ulSlot.m_varTtiAllocInfo.push_back(varTtiInfo);
        std::stable_sort(ulSlot.m_varTtiAllocInfo.begin(), ulSlot.m_varTtiAllocInfo.end());
    }
    else
    {
        SlotAllocInfo slotAllocInfo = SlotAllocInfo(sfnSf);
        slotAllocInfo.m_varTtiAllocInfo.push_back(varTtiInfo);
        PushBackSlotAllocInfo(slotAllocInfo);
    }
}

void
NrUePhy::PhyCtrlMessagesReceived(const Ptr<NrControlMessage>& msg)
{
    NS_LOG_FUNCTION(this);

    if (msg->GetMessageType() == NrControlMessage::DL_DCI)
    {
        auto dciMsg = DynamicCast<NrDlDciMessage>(msg);
        auto dciInfoElem = dciMsg->GetDciInfoElement();

        m_phyRxedCtrlMsgsTrace(m_currentSlot, GetCellId(), m_rnti, GetBwpId(), msg);

        if (dciInfoElem->m_rnti != 0 && dciInfoElem->m_rnti != m_rnti)
        {
            return; // DCI not for me
        }

        SfnSf dciSfn = m_currentSlot;
        uint32_t k0Delay = dciMsg->GetKDelay();
        dciSfn.Add(k0Delay);

        NS_LOG_DEBUG("UE" << m_rnti << " DL-DCI received for slot " << dciSfn << " symStart "
                          << static_cast<uint32_t>(dciInfoElem->m_symStart) << " numSym "
                          << static_cast<uint32_t>(dciInfoElem->m_numSym) << " tbs "
                          << dciInfoElem->m_tbSize << " harqId "
                          << static_cast<uint32_t>(dciInfoElem->m_harqProcess));

        /* BIG ASSUMPTION: We assume that K0 is always 0 */

        auto it = m_harqIdToK1Map.find(dciInfoElem->m_harqProcess);
        if (it != m_harqIdToK1Map.end())
        {
            m_harqIdToK1Map.erase(m_harqIdToK1Map.find(dciInfoElem->m_harqProcess));
        }

        m_harqIdToK1Map.insert(std::make_pair(dciInfoElem->m_harqProcess, dciMsg->GetK1Delay()));

        m_phyUeRxedDlDciTrace(m_currentSlot,
                              GetCellId(),
                              m_rnti,
                              GetBwpId(),
                              dciInfoElem->m_harqProcess,
                              dciMsg->GetK1Delay());

        InsertAllocation(dciInfoElem);

        m_phySapUser->ReceiveControlMessage(msg);

        if (m_enableUplinkPowerControl)
        {
            m_powerControl->ReportTpcPusch(dciInfoElem->m_tpc);
            m_powerControl->ReportTpcPucch(dciInfoElem->m_tpc);
        }
    }
    else if (msg->GetMessageType() == NrControlMessage::UL_DCI)
    {
        auto dciMsg = DynamicCast<NrUlDciMessage>(msg);
        auto dciInfoElem = dciMsg->GetDciInfoElement();

        m_phyRxedCtrlMsgsTrace(m_currentSlot, GetCellId(), m_rnti, GetBwpId(), msg);

        if (dciInfoElem->m_rnti != 0 && dciInfoElem->m_rnti != m_rnti)
        {
            return; // DCI not for me
        }

        SfnSf ulSfnSf = m_currentSlot;
        uint32_t k2Delay = dciMsg->GetKDelay();
        ulSfnSf.Add(k2Delay);

        if (dciInfoElem->m_type == DciInfoElementTdma::DATA)
        {
            ProcessDataDci(ulSfnSf, dciInfoElem);
            m_phySapUser->ReceiveControlMessage(msg);
        }
        else if (dciInfoElem->m_type == DciInfoElementTdma::SRS)
        {
            ProcessSrsDci(ulSfnSf, dciInfoElem);
            // Do not pass the DCI to MAC
        }
    }
    else if (msg->GetMessageType() == NrControlMessage::MIB)
    {
        NS_LOG_DEBUG("received MIB");
        Ptr<NrMibMessage> msg2 = DynamicCast<NrMibMessage>(msg);
        m_phyRxedCtrlMsgsTrace(m_currentSlot, GetCellId(), m_rnti, GetBwpId(), msg);
        m_ueCphySapUser->RecvMasterInformationBlock(GetCellId(), DoGetArfcn(), msg2->GetMib());
    }
    else if (msg->GetMessageType() == NrControlMessage::SIB1)
    {
        Ptr<NrSib1Message> msg2 = DynamicCast<NrSib1Message>(msg);
        m_phyRxedCtrlMsgsTrace(m_currentSlot, GetCellId(), m_rnti, GetBwpId(), msg);
        m_ueCphySapUser->RecvSystemInformationBlockType1(
            msg2->GetSib1().cellAccessRelatedInfo.cellIdentity,
            DoGetArfcn(),
            msg2->GetSib1());
    }
    else if (msg->GetMessageType() == NrControlMessage::RAR)
    {
        Ptr<NrRarMessage> rarMsg = DynamicCast<NrRarMessage>(msg);

        ProcessRar(rarMsg);
    }
    else
    {
        NS_LOG_INFO("Message type not recognized " << msg->GetMessageType());
        m_phyRxedCtrlMsgsTrace(m_currentSlot, GetCellId(), m_rnti, GetBwpId(), msg);
        m_phySapUser->ReceiveControlMessage(msg);
    }
}

void
NrUePhy::ProcessRar(const Ptr<NrRarMessage>& rarMsg)
{
    NS_LOG_FUNCTION(this);
    bool myRar = false;
    {
        for (auto it = rarMsg->RarListBegin(); it != rarMsg->RarListEnd(); ++it)
        {
            NS_LOG_INFO("Received RAR in slot" << m_currentSlot << " with RA preamble ID: "
                                               << std::to_string(it->rarPayload.raPreambleId));
            if (it->rarPayload.raPreambleId == m_raPreambleId)
            {
                NS_LOG_INFO("Received RAR with RA preamble ID:" << +it->rarPayload.raPreambleId
                                                                << " current RA preamble ID is :"
                                                                << m_raPreambleId);
                // insert allocation
                SfnSf ulSfnSf = m_currentSlot;
                uint32_t k2Delay = it->rarPayload.k2Delay;
                ulSfnSf.Add(k2Delay);
                NS_LOG_DEBUG("Insert RAR UL DCI allocation for " << ulSfnSf);
                ProcessDataDci(ulSfnSf, (*it).rarPayload.ulMsg3Dci);
                myRar = true;
                // notify MAC and above about transmission opportunity
                m_phySapUser->ReceiveControlMessage(rarMsg);
                // fire CTRL msg trace
                m_phyRxedCtrlMsgsTrace(m_currentSlot, GetCellId(), m_rnti, GetBwpId(), rarMsg);
                // reset RACH variables with out of range values
                m_raPreambleId = 255;
            }
        }
        if (!myRar)
        {
            NS_LOG_DEBUG("Skipping RAR, does not contain preamble ID."
                         << "\n My preamble id: " << std::to_string(m_raPreambleId) << " found:");
            for (auto it = rarMsg->RarListBegin(); it != rarMsg->RarListEnd(); ++it)
            {
                NS_LOG_DEBUG("rapId: " << std::to_string(it->rapId));
            }
        }
    }
}

void
NrUePhy::TryToPerformLbt()
{
    NS_LOG_FUNCTION(this);
    uint8_t ulCtrlSymStart = 0;
    uint8_t ulCtrlNumSym = 0;

    for (const auto& alloc : m_currSlotAllocInfo.m_varTtiAllocInfo)
    {
        if (alloc.m_dci->m_type == DciInfoElementTdma::CTRL &&
            alloc.m_dci->m_format == DciInfoElementTdma::UL)
        {
            ulCtrlSymStart = alloc.m_dci->m_symStart;
            ulCtrlNumSym = alloc.m_dci->m_numSym;
            break;
        }
    }

    if (ulCtrlNumSym != 0)
    {
        // We have an UL CTRL symbol scheduled and we have to transmit CTRLs..
        // .. so we check that we have at least 25 us between the latest DCI,
        // or we have to schedule an LBT event.

        Time limit = m_lastSlotStart + GetSlotPeriod() -
                     ((GetSymbolsPerSlot() - ulCtrlSymStart) * GetSymbolPeriod()) -
                     m_lbtThresholdForCtrl;

        for (const auto& alloc : m_currSlotAllocInfo.m_varTtiAllocInfo)
        {
            int64_t symbolPeriod = GetSymbolPeriod().GetMicroSeconds();
            int64_t dciEndsAt = m_lastSlotStart.GetMicroSeconds() +
                                ((alloc.m_dci->m_numSym + alloc.m_dci->m_symStart) * symbolPeriod);

            if (alloc.m_dci->m_type != DciInfoElementTdma::DATA &&
                alloc.m_dci->m_type != DciInfoElementTdma::MSG3)
            {
                continue;
            }

            if (limit.GetMicroSeconds() < dciEndsAt)
            {
                NS_LOG_INFO("This data DCI ends at "
                            << MicroSeconds(dciEndsAt)
                            << " which is inside the LBT shared COT (the limit is " << limit
                            << "). No need for LBT");
                m_lbtEvent.Cancel(); // Forget any LBT we previously set, because of the new
                // DCI information
                m_channelStatus = GRANTED;
            }
            else
            {
                NS_LOG_INFO("This data DCI starts at "
                            << +alloc.m_dci->m_symStart << " for " << +alloc.m_dci->m_numSym
                            << " ends at " << MicroSeconds(dciEndsAt)
                            << " which is outside the LBT shared COT (the limit is " << limit
                            << ").");
            }
        }
        if (m_channelStatus != GRANTED)
        {
            Time sched = m_lastSlotStart - Simulator::Now() + (GetSymbolPeriod() * ulCtrlSymStart) -
                         MicroSeconds(25);
            NS_LOG_DEBUG("Scheduling an LBT for sending the UL CTRL at "
                         << Simulator::Now() + sched);
            m_lbtEvent.Cancel();
            m_lbtEvent = Simulator::Schedule(sched, &NrUePhy::RequestAccess, this);
        }
        else
        {
            NS_LOG_DEBUG("Not scheduling LBT: the UE has a channel status that is GRANTED");
        }
    }
    else
    {
        NS_LOG_DEBUG("Not scheduling LBT; the UE has no UL CTRL symbols available");
    }
}

void
NrUePhy::RequestAccess()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG("Request access because we have to transmit UL CTRL");
    m_cam->RequestAccess(); // This will put the m_channelStatus to granted when
                            // the channel will be granted.
}

void
NrUePhy::PushCtrlAllocations(const SfnSf currentSfnSf)
{
    NS_LOG_FUNCTION(this);

    // The UE does not know anything from the GNB yet, so listen on the default
    // bandwidth.
    std::vector<bool> rbgBitmask(GetRbNum(), true);

    // The UE still doesn't know the TDD pattern, so just add a DL CTRL
    if (m_tddPattern.empty())
    {
        NS_LOG_INFO("TDD Pattern unknown, insert DL CTRL at the beginning of the slot");
        VarTtiAllocInfo dlCtrlSlot(std::make_shared<DciInfoElementTdma>(0,
                                                                        m_dlCtrlSyms,
                                                                        DciInfoElementTdma::DL,
                                                                        DciInfoElementTdma::CTRL,
                                                                        rbgBitmask));
        m_currSlotAllocInfo.m_varTtiAllocInfo.push_front(dlCtrlSlot);
        return;
    }

    uint64_t currentSlotN = currentSfnSf.Normalize() % m_tddPattern.size();
    const auto overlapsRange = [this](uint8_t start, uint8_t numSym) {
        const uint8_t end = start + numSym;
        return std::any_of(m_currSlotAllocInfo.m_varTtiAllocInfo.begin(),
                           m_currSlotAllocInfo.m_varTtiAllocInfo.end(),
                           [start, end](const auto& alloc) {
                               const uint8_t allocStart = alloc.m_dci->m_symStart;
                               const uint8_t allocEnd = allocStart + alloc.m_dci->m_numSym;
                               return allocStart < end && start < allocEnd;
                           });
    };

    if (m_tddPattern[currentSlotN] < LteNrTddSlotType::UL)
    {
        if (!overlapsRange(0, m_dlCtrlSyms))
        {
            NS_LOG_DEBUG("The current TDD pattern indicates that we are in a "
                         << m_tddPattern[currentSlotN]
                         << " slot, so insert DL CTRL at the beginning of the slot");
            VarTtiAllocInfo dlCtrlSlot(
                std::make_shared<DciInfoElementTdma>(0,
                                                     m_dlCtrlSyms,
                                                     DciInfoElementTdma::DL,
                                                     DciInfoElementTdma::CTRL,
                                                     rbgBitmask));
            m_currSlotAllocInfo.m_varTtiAllocInfo.push_front(dlCtrlSlot);
        }
        else
        {
            NS_LOG_WARN("Skipping DL CTRL insertion because the reserved DL CTRL symbols are "
                        "already occupied");
        }
    }
    if (m_tddPattern[currentSlotN] > LteNrTddSlotType::DL)
    {
        const uint8_t ulCtrlStart = GetSymbolsPerSlot() - m_ulCtrlSyms;
        if (!overlapsRange(ulCtrlStart, m_ulCtrlSyms))
        {
            NS_LOG_DEBUG("The current TDD pattern indicates that we are in a "
                         << m_tddPattern[currentSlotN]
                         << " slot, so insert UL CTRL at the end of the slot");
            VarTtiAllocInfo ulCtrlSlot(
                std::make_shared<DciInfoElementTdma>(ulCtrlStart,
                                                     m_ulCtrlSyms,
                                                     DciInfoElementTdma::UL,
                                                     DciInfoElementTdma::CTRL,
                                                     rbgBitmask));
            m_currSlotAllocInfo.m_varTtiAllocInfo.push_back(ulCtrlSlot);
        }
        else
        {
            NS_LOG_WARN("Skipping UL CTRL insertion because the reserved UL CTRL symbols are "
                        "already occupied");
        }
    }
}

void
NrUePhy::StartSlot(const SfnSf& s)
{
    NS_LOG_FUNCTION(this);

    if (GetNumerology() != s.GetNumerology())
    {
        // The active PHY numerology changed (e.g. the UE re-tuned to a cell on a
        // different numerology during cell (re)selection or an inter-frequency
        // handover). The in-flight slot 's' belongs to the abandoned numerology
        // timeline, so its frame/subframe/slot indices are meaningless here.
        // Drop the stale allocations and re-stamp the slot machine onto the new
        // numerology so that the event loop keeps running instead of stalling
        // forever (which would silently kill the UE PHY, e.g. preventing RACH
        // preambles from ever being transmitted on the target cell).
        NS_LOG_INFO("Numerology changed from " << s.GetNumerology() << " to " << GetNumerology()
                                               << ", restarting slot loop on the new numerology.");
        PurgeStaleSlotAllocInfo();

        // Rebuild a slot index consistent with the new numerology from the
        // current absolute time, and align to the next slot boundary.
        Time slotPeriod = GetSlotPeriod();
        uint64_t slotsElapsed = Simulator::Now().GetTimeStep() / slotPeriod.GetTimeStep();
        Time nextSlotStart = slotPeriod * (slotsElapsed + 1) - Simulator::Now();
        SfnSf restartSlot(0, 0, 0, GetNumerology());
        restartSlot.Add(static_cast<uint32_t>(slotsElapsed + 1));
        Simulator::Schedule(nextSlotStart, &NrUePhy::StartSlot, this, restartSlot);
        return;
    }
    m_currentSlot = s;
    m_lastSlotStart = Simulator::Now();

    // Call MAC before doing anything in PHY
    m_phySapUser->SlotIndication(m_currentSlot); // trigger mac

    // update the current slot object, and insert DL/UL CTRL allocations depending on the TDD
    // pattern
    bool nrAllocationExists = SlotAllocInfoExists(m_currentSlot);
    FinishSlotProcessing(s, nrAllocationExists);
}

void
NrUePhy::FinishSlotProcessing(const SfnSf& s, bool nrAllocationExists)
{
    NS_LOG_FUNCTION(this << s);
    if (nrAllocationExists)
    {
        m_currSlotAllocInfo = RetrieveSlotAllocInfo(m_currentSlot);
    }
    else
    {
        m_currSlotAllocInfo = SlotAllocInfo(m_currentSlot);
    }

    PushCtrlAllocations(m_currentSlot);
    NS_ASSERT(m_currSlotAllocInfo.m_sfnSf == m_currentSlot);

    NS_LOG_DEBUG("UE " << m_rnti << " start slot " << m_currSlotAllocInfo.m_sfnSf
                       << " composed by the following allocations, total "
                       << m_currSlotAllocInfo.m_varTtiAllocInfo.size());
    for (const auto& alloc : m_currSlotAllocInfo.m_varTtiAllocInfo)
    {
        std::string direction;
        std::string type;

        if (alloc.m_dci->m_format == DciInfoElementTdma::UL)
        {
            direction = "UL";
        }
        else
        {
            direction = "DL";
        }

        switch (alloc.m_dci->m_type)
        {
        case DciInfoElementTdma::VarTtiType::SRS:
            type = "SRS";
            NS_LOG_DEBUG("Allocation from sym "
                         << static_cast<uint32_t>(alloc.m_dci->m_symStart) << " to sym "
                         << static_cast<uint32_t>(alloc.m_dci->m_numSym + alloc.m_dci->m_symStart)
                         << " direction " << direction << " type " << type);
            break;
        case DciInfoElementTdma::VarTtiType::DATA:
            type = "DATA";
            NS_LOG_INFO("Allocation from sym "
                        << static_cast<uint32_t>(alloc.m_dci->m_symStart) << " to sym "
                        << static_cast<uint32_t>(alloc.m_dci->m_numSym + alloc.m_dci->m_symStart)
                        << " direction " << direction << " type " << type);
            break;
        case DciInfoElementTdma::VarTtiType::CTRL:
            type = "CTRL";
            NS_LOG_DEBUG("Allocation from sym "
                         << static_cast<uint32_t>(alloc.m_dci->m_symStart) << " to sym "
                         << static_cast<uint32_t>(alloc.m_dci->m_numSym + alloc.m_dci->m_symStart)
                         << " direction " << direction << " type " << type);
            break;
        case DciInfoElementTdma::VarTtiType::MSG3:
            type = "MSG3";
            NS_LOG_DEBUG("Allocation from sym "
                         << static_cast<uint32_t>(alloc.m_dci->m_symStart) << " to sym "
                         << static_cast<uint32_t>(alloc.m_dci->m_numSym + alloc.m_dci->m_symStart)
                         << " direction " << direction << " type " << type);
            break;
        default:
            NS_LOG_ERROR("Unknown type DciInfoElementTdma::VarTtiType " << alloc.m_dci->m_type);
        }
    }

    TryToPerformLbt();

    VarTtiAllocInfo allocation = m_currSlotAllocInfo.m_varTtiAllocInfo.front();
    m_currSlotAllocInfo.m_varTtiAllocInfo.pop_front();

    auto nextVarTtiStart = GetSymbolPeriod() * allocation.m_dci->m_symStart;

    auto ctrlMsgs = PopCurrentSlotCtrlMsgs();
    if (m_netDevice)
    {
        DynamicCast<NrUeNetDevice>(m_netDevice)->RouteOutgoingCtrlMsgs(ctrlMsgs, GetBwpId());
    }
    else
    {
        // No netDevice (that could happen in tests) so just redirect them to us
        for (const auto& msg : ctrlMsgs)
        {
            EncodeCtrlMsg(msg);
        }
    }

    Simulator::Schedule(nextVarTtiStart, &NrUePhy::StartVarTti, this, allocation.m_dci);
}

Time
NrUePhy::DlCtrl(const std::shared_ptr<DciInfoElementTdma>& dci)
{
    NS_LOG_FUNCTION(this);

    Time varTtiDuration = GetSymbolPeriod() * dci->m_numSym;

    NS_LOG_DEBUG("UE" << m_rnti
                      << " RXing DL CTRL frame for"
                         " symbols "
                      << +dci->m_symStart << "-" << +(dci->m_symStart + dci->m_numSym - 1)
                      << "\t start " << Simulator::Now() << " end "
                      << (Simulator::Now() + varTtiDuration));
    if (!m_lastSlotStart.IsZero())
    {
        m_tryToPerformLbt = true;
    }

    m_spectrumPhy->AddExpectedDlCtrlEnd(Simulator::Now() + varTtiDuration);

    return varTtiDuration;
}

Time
NrUePhy::UlSrs(const std::shared_ptr<DciInfoElementTdma>& dci)
{
    NS_LOG_FUNCTION(this);

    std::vector<int> channelRbs(GetRbNum());
    std::iota(channelRbs.begin(), channelRbs.end(), 0);

    if (m_enableUplinkPowerControl)
    {
        m_txPower = m_powerControl->GetSrsTxPower(channelRbs.size());
    }
    SetSubChannelsForTransmission(channelRbs, dci->m_numSym);

    std::list<Ptr<NrControlMessage>> srsMsg;
    Ptr<NrSrsMessage> srs = Create<NrSrsMessage>();
    srs->SetSourceBwpArfcn(DoGetArfcn());
    srsMsg.emplace_back(srs);
    Time varTtiDuration = GetSymbolPeriod() * dci->m_numSym;

    m_phyTxedCtrlMsgsTrace(m_currentSlot, GetCellId(), dci->m_rnti, GetBwpId(), *srsMsg.begin());
    m_spectrumPhy->StartTxUlControlFrames(srsMsg, varTtiDuration - NanoSeconds(1.0));

    NS_LOG_DEBUG("UE" << m_rnti << " TXing UL SRS frame for symbols " << +dci->m_symStart << "-"
                      << +(dci->m_symStart + dci->m_numSym - 1) << "\t start " << Simulator::Now()
                      << " end " << (Simulator::Now() + varTtiDuration - NanoSeconds(1.0)));

    ChannelAccessDenied(); // Reset the channel status
    return varTtiDuration;
}

Time
NrUePhy::UlCtrl(const std::shared_ptr<DciInfoElementTdma>& dci)
{
    NS_LOG_FUNCTION(this);

    Time varTtiDuration = GetSymbolPeriod() * dci->m_numSym;

    if (m_ctrlMsgs.empty())
    {
        NS_LOG_DEBUG("UE" << m_rnti << " reserved space for UL CTRL frame for symbols "
                          << +dci->m_symStart << "-" << +(dci->m_symStart + dci->m_numSym - 1)
                          << "\t start " << Simulator::Now() << " end "
                          << (Simulator::Now() + varTtiDuration - NanoSeconds(1.0))
                          << " but no data to transmit");
        m_cam->Cancel();
        // The grant covers this slot only; left GRANTED, the next slot
        // requests no access and an SRS there leaves its UL CTRL unsent
        ChannelAccessDenied();
        return varTtiDuration;
    }
    else if (m_channelStatus != GRANTED)
    {
        NS_LOG_INFO("UE" << m_rnti << " has to transmit CTRL but channel not granted");
        m_cam->Cancel();
        return varTtiDuration;
    }

    for (const auto& msg : m_ctrlMsgs)
    {
        m_phyTxedCtrlMsgsTrace(m_currentSlot, GetCellId(), dci->m_rnti, GetBwpId(), msg);

        if (msg->GetMessageType() == NrControlMessage::DL_HARQ)
        {
            Ptr<NrDlHarqFeedbackMessage> harqMsg = DynamicCast<NrDlHarqFeedbackMessage>(msg);
            uint8_t harqId = harqMsg->GetDlHarqFeedback().m_harqProcessId;

            auto it = m_harqIdToK1Map.find(harqId);
            if (it != m_harqIdToK1Map.end())
            {
                m_phyUeTxedHarqFeedbackTrace(m_currentSlot,
                                             GetCellId(),
                                             m_rnti,
                                             GetBwpId(),
                                             static_cast<uint32_t>(harqId),
                                             it->second);
            }
        }
    }

    std::vector<int> channelRbs(GetRbNum());
    std::iota(channelRbs.begin(), channelRbs.end(), 0);

    if (m_enableUplinkPowerControl)
    {
        m_txPower = m_powerControl->GetPucchTxPower(channelRbs.size());
    }
    SetSubChannelsForTransmission(channelRbs, dci->m_numSym);

    NS_LOG_DEBUG("UE" << m_rnti << " TXing UL CTRL frame for symbols " << +dci->m_symStart << "-"
                      << +(dci->m_symStart + dci->m_numSym - 1) << "\t start " << Simulator::Now()
                      << " end " << (Simulator::Now() + varTtiDuration - NanoSeconds(1.0)));

    SendCtrlChannels(varTtiDuration - NanoSeconds(1.0));

    ChannelAccessDenied(); // Reset the channel status
    return varTtiDuration;
}

Time
NrUePhy::DlData(const std::shared_ptr<DciInfoElementTdma>& dci)
{
    NS_LOG_FUNCTION(this);

    Time varTtiDuration = GetSymbolPeriod() * dci->m_numSym;

    if (GetRbNum() / GetNumRbPerRbg() != dci->m_rbgBitmask.size())
    {
        NS_LOG_DEBUG("Mismatching number of RBGs and RBG bitmask");
        return varTtiDuration;
    }

    if (dci->m_rnti != m_rnti)
    {
        NS_LOG_DEBUG("Transmission scheduled before the UE disconnected");
        return varTtiDuration;
    }

    m_receptionEnabled = true;
    NS_ASSERT(dci->m_rnti == m_rnti);
    m_spectrumPhy->AddExpectedTb({dci->m_ndi,
                                  dci->m_tbSize,
                                  dci->m_mcs,
                                  dci->m_rank,
                                  dci->m_rnti,
                                  FromRBGBitmaskToRBAssignment(dci->m_rbgBitmask),
                                  dci->m_harqProcess,
                                  dci->m_rv,
                                  true,
                                  dci->m_symStart,
                                  dci->m_numSym,
                                  m_currentSlot});
    m_reportDlTbSize(m_netDevice->GetObject<NrUeNetDevice>()->GetImsi(), dci->m_tbSize);
    NS_LOG_INFO("UE" << m_rnti << " RXing DL DATA frame for symbols " << +dci->m_symStart << "-"
                     << +(dci->m_symStart + dci->m_numSym - 1) << " num of rbg assigned: "
                     << FromRBGBitmaskToRBAssignment(dci->m_rbgBitmask).size()
                     << ". RX will take place for " << varTtiDuration);

    return varTtiDuration;
}

Time
NrUePhy::UlData(const std::shared_ptr<DciInfoElementTdma>& dci)
{
    NS_LOG_FUNCTION(this);
    Time varTtiDuration = GetSymbolPeriod() * dci->m_numSym;

    if (GetRbNum() / GetNumRbPerRbg() != dci->m_rbgBitmask.size())
    {
        return varTtiDuration;
    }
    if (m_enableUplinkPowerControl)
    {
        m_txPower = m_powerControl->GetPuschTxPower(
            (FromRBGBitmaskToRBAssignment(dci->m_rbgBitmask)).size());
    }
    SetSubChannelsForTransmission(FromRBGBitmaskToRBAssignment(dci->m_rbgBitmask), dci->m_numSym);
    std::list<Ptr<NrControlMessage>> ctrlMsg;
    Ptr<PacketBurst> pktBurst = GetPacketBurst(m_currentSlot, dci->m_symStart, dci->m_rnti);
    if (pktBurst && pktBurst->GetNPackets() > 0)
    {
        std::list<Ptr<Packet>> pkts = pktBurst->GetPackets();
        NrRadioBearerTag bearerTag;
        if (!pkts.front()->PeekPacketTag(bearerTag))
        {
            NS_FATAL_ERROR("No radio bearer tag");
        }
    }
    else
    {
        // put an error, as something is wrong. The UE should not be scheduled
        // if there is no data for him...
        if (dci->m_type == DciInfoElementTdma::MSG3)
        {
            NS_LOG_WARN("Not sending MSG3. Probably in RRC IDEAL mode.");
            return varTtiDuration;
        }
        else
        {
            NS_LOG_WARN("UE " << dci->m_rnti << " was scheduled but has nothing to transmit");
            return varTtiDuration;
        }
    }
    m_reportUlTbSize(m_netDevice->GetObject<NrUeNetDevice>()->GetImsi(), dci->m_tbSize);

    NS_LOG_DEBUG("UE" << m_rnti << " TXing UL DATA frame for"
                      << " symbols " << +dci->m_symStart << "-"
                      << +(dci->m_symStart + dci->m_numSym - 1) << "\t start " << Simulator::Now()
                      << " end " << (Simulator::Now() + varTtiDuration));

    Simulator::Schedule(NanoSeconds(1.0),
                        &NrUePhy::SendDataChannels,
                        this,
                        pktBurst,
                        ctrlMsg,
                        dci,
                        varTtiDuration - NanoSeconds(2.0));
    return varTtiDuration;
}

void
NrUePhy::StartVarTti(const std::shared_ptr<DciInfoElementTdma>& dci)
{
    NS_LOG_FUNCTION(this);
    Time varTtiDuration;

    m_currTbs = dci->m_tbSize;
    m_receptionEnabled = false;

    if (dci->m_type == DciInfoElementTdma::CTRL && dci->m_format == DciInfoElementTdma::DL)
    {
        varTtiDuration = DlCtrl(dci);
    }
    else if (dci->m_type == DciInfoElementTdma::CTRL && dci->m_format == DciInfoElementTdma::UL)
    {
        varTtiDuration = UlCtrl(dci);
    }
    else if (dci->m_type == DciInfoElementTdma::SRS && dci->m_format == DciInfoElementTdma::UL)
    {
        varTtiDuration = UlSrs(dci);
    }
    else if (dci->m_type == DciInfoElementTdma::DATA && dci->m_format == DciInfoElementTdma::DL)
    {
        varTtiDuration = DlData(dci);
    }
    else if ((dci->m_type == DciInfoElementTdma::DATA || dci->m_type == DciInfoElementTdma::MSG3) &&
             dci->m_format == DciInfoElementTdma::UL)
    {
        varTtiDuration = UlData(dci);
    }

    Simulator::Schedule(varTtiDuration, &NrUePhy::EndVarTti, this, dci);
}

void
NrUePhy::EndVarTti(const std::shared_ptr<DciInfoElementTdma>& dci)
{
    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG("DCI started at symbol "
                 << static_cast<uint32_t>(dci->m_symStart) << " which lasted for "
                 << static_cast<uint32_t>(dci->m_numSym) << " symbols finished");

    if (m_tryToPerformLbt)
    {
        TryToPerformLbt();
        m_tryToPerformLbt = false;
    }

    if (m_currSlotAllocInfo.m_varTtiAllocInfo.empty())
    {
        // end of slot
        m_currentSlot.Add(1);

        // During an inter-frequency / inter-numerology handover the UE re-tunes
        // to a different BWP PHY. The slot period (which is numerology-dependent)
        // and the m_lastSlotStart reference belong to the abandoned timeline, so
        // the next-slot start can resolve to a time in the past. Clamp such a
        // stale boundary to the current instant so the slot machine keeps
        // running on the target BWP instead of asserting; StartSlot() detects
        // the numerology change and cleanly re-stamps the loop from there.
        Time delay = m_lastSlotStart + GetSlotPeriod() - Simulator::Now();
        if (delay.IsStrictlyNegative())
        {
            NS_LOG_WARN("Clamping next-slot start "
                        << delay << " in the past to now, due to a BWP/numerology switch");
            delay = Time(0);
        }

        Simulator::Schedule(delay, &NrUePhy::StartSlot, this, m_currentSlot);
    }
    else
    {
        VarTtiAllocInfo allocation = m_currSlotAllocInfo.m_varTtiAllocInfo.front();
        m_currSlotAllocInfo.m_varTtiAllocInfo.pop_front();

        Time nextVarTtiStart = GetSymbolPeriod() * allocation.m_dci->m_symStart;
        Time nextVarTtiTime = nextVarTtiStart + m_lastSlotStart;
        Time delay = nextVarTtiTime - Simulator::Now();

        // During an inter-frequency handover the UE re-tunes to a different BWP
        // PHY, whose m_lastSlotStart timeline is re-stamped by StartSlot(). A
        // VarTti that was in flight on the abandoned timeline can then resolve
        // to a start time slightly in the past, yielding a small negative delay.
        // Clamp such a stale allocation to the current instant so that it is
        // still processed (keeping the data plane alive across the handover)
        // without scheduling an event in the past.
        if (delay.IsStrictlyNegative())
        {
            NS_LOG_WARN("Clamping VarTti scheduled "
                        << delay << " in the past to now, due to a BWP/numerology switch");
            delay = Time(0);
        }

        Simulator::Schedule(delay, &NrUePhy::StartVarTti, this, allocation.m_dci);
    }

    m_receptionEnabled = false;
}

void
NrUePhy::PhyDataPacketReceived(const Ptr<Packet>& p)
{
    Simulator::ScheduleWithContext(m_netDevice->GetNode()->GetId(),
                                   GetTbDecodeLatency(),
                                   &NrUePhySapUser::ReceivePhyPdu,
                                   m_phySapUser,
                                   p);
    // m_phySapUser->ReceivePhyPdu (p);
}

void
NrUePhy::SendDataChannels(const Ptr<PacketBurst>& pb,
                          const std::list<Ptr<NrControlMessage>>& ctrlMsg,
                          const std::shared_ptr<DciInfoElementTdma>& dci,
                          const Time& duration)
{
    if (pb->GetNPackets() > 0)
    {
        NrRadioBearerTag tag;
        if (!pb->GetPackets().front()->PeekPacketTag(tag))
        {
            NS_FATAL_ERROR("No radio bearer tag");
        }
    }

    m_spectrumPhy->StartTxDataFrames(pb, ctrlMsg, dci, duration);
}

void
NrUePhy::SendCtrlChannels(Time duration)
{
    m_spectrumPhy->StartTxUlControlFrames(m_ctrlMsgs, duration);
    m_ctrlMsgs.clear();
}

Ptr<NrDlCqiMessage>
NrUePhy::CreateDlCqiFeedbackMessage(const SpectrumValue& sinr)
{
    NS_LOG_FUNCTION(this);
    // Create DL CQI CTRL message
    Ptr<NrDlCqiMessage> msg = Create<NrDlCqiMessage>();
    msg->SetSourceBwpArfcn(DoGetArfcn());
    DlCqiInfo dlcqi;

    dlcqi.m_rnti = m_rnti;
    dlcqi.m_cqiType = DlCqiInfo::WB;

    std::vector<int> cqi;
    dlcqi.m_wbCqi = m_amc->CreateCqiFeedbackSiso(sinr, dlcqi.m_mcs);
    msg->SetDlCqi(dlcqi);

    m_cqiFeedbackTrace(m_rnti, dlcqi.m_wbCqi, dlcqi.m_mcs, 1);
    return msg;
}

void
NrUePhy::GenerateDlCqiReport(const SpectrumValue& sinr)
{
    NS_LOG_FUNCTION(this);
    // Not totally sure what this is about. We have to check.
    if (m_ulConfigured && (m_rnti > 0) && m_receptionEnabled)
    {
        m_dlDataSinrTrace(GetCellId(), m_rnti, ComputeAvgSinr(sinr), GetBwpId());

        if (Simulator::Now() > m_wbCqiLast)
        {
            Ptr<NrDlCqiMessage> msg = CreateDlCqiFeedbackMessage(sinr);

            if (msg)
            {
                DoSendControlMessage(msg);
            }
        }
    }
}

void
NrUePhy::EnqueueDlHarqFeedback(const DlHarqInfo& m)
{
    NS_LOG_FUNCTION(this);
    // NTN downlinkHARQ-FeedbackDisabled-r17: no HARQ-ACK is reported for a
    // feedback-disabled process, so nothing is put on PUCCH for it.
    if (m.m_harqProcessId < 32 && ((m_dlHarqFeedbackDisabled >> m.m_harqProcessId) & 1U))
    {
        ++m_dlHarqFeedbackSuppressed;
        m_dlHarqFeedbackSuppressedTrace(m.m_rnti, m.m_harqProcessId);
        return;
    }
    // get the feedback from NrSpectrumPhy and send it through ideal PUCCH to gNB
    Ptr<NrDlHarqFeedbackMessage> msg = Create<NrDlHarqFeedbackMessage>();
    msg->SetSourceBwpArfcn(DoGetArfcn());
    msg->SetDlHarqFeedback(m);

    auto k1It = m_harqIdToK1Map.find(m.m_harqProcessId);

    NS_LOG_DEBUG("ReceiveNrDlHarqFeedback" << " Harq Process " << static_cast<uint32_t>(k1It->first)
                                           << " K1: " << k1It->second << " Frame "
                                           << m_currentSlot);

    Time event = m_lastSlotStart + (GetSlotPeriod() * k1It->second);
    if (event <= Simulator::Now())
    {
        Simulator::ScheduleNow(&NrUePhy::DoSendControlMessageNow, this, msg);
    }
    else
    {
        Simulator::Schedule(event - Simulator::Now(), &NrUePhy::DoSendControlMessageNow, this, msg);
    }
}

void
NrUePhy::SetCam(const Ptr<NrChAccessManager>& cam)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(cam != nullptr);
    m_cam = cam;
    m_cam->SetAccessGrantedCallback(
        std::bind(&NrUePhy::ChannelAccessGranted, this, std::placeholders::_1));
    m_cam->SetAccessDeniedCallback(std::bind(&NrUePhy::ChannelAccessDenied, this));
}

const SfnSf&
NrUePhy::GetCurrentSfnSf() const
{
    return m_currentSlot;
}

uint16_t
NrUePhy::GetRnti() const
{
    return m_rnti;
}

void
NrUePhy::DoReset()
{
    NS_LOG_FUNCTION(this);
    m_raPreambleId = 255; // value out of range
    m_isConnected = false;
    ResetCsiMeasurements();
    PurgeStaleSlotAllocInfo();
}

void
NrUePhy::ResetCsiMeasurements()
{
    NS_LOG_FUNCTION(this);
    m_csiRsMimoSignal = {};
    m_avgIntCovMat = {};
}

void
NrUePhy::DoStartCellSearch(uint16_t arfcn)
{
    NS_LOG_FUNCTION(this << arfcn);
    InitializeMessageList();
    DoSetInitialBandwidth();
}

void
NrUePhy::DoSynchronizeWithGnb(uint16_t cellId, uint16_t arfcn)
{
    NS_LOG_FUNCTION(this << cellId << arfcn);
    DoSynchronizeWithGnb(cellId);
}

void
NrUePhy::DoSetPa(double pa)
{
    NS_LOG_FUNCTION(this << pa);
}

void
NrUePhy::DoSetRsrpFilterCoefficient(uint8_t rsrpFilterCoefficient)
{
    NS_LOG_FUNCTION(this << +rsrpFilterCoefficient);
}

void
NrUePhy::DoSynchronizeWithGnb(uint16_t cellId)
{
    NS_LOG_FUNCTION(this << cellId);
    DoSetCellId(cellId);
    ResetCsiMeasurements();
    DoSetInitialBandwidth();
}

BeamId
NrUePhy::GetBeamId([[maybe_unused]] uint16_t rnti) const
{
    NS_LOG_FUNCTION(this);
    // That's a bad specification: the UE PHY doesn't know anything about its beam id.
    NS_FATAL_ERROR("ERROR");
}

void
NrUePhy::ScheduleStartEventLoop(uint32_t nodeId, uint16_t frame, uint8_t subframe, uint16_t slot)
{
    NS_LOG_FUNCTION(this);
    Simulator::ScheduleWithContext(nodeId,
                                   MilliSeconds(0),
                                   &NrUePhy::StartEventLoop,
                                   this,
                                   frame,
                                   subframe,
                                   slot);
}

void
NrUePhy::ReportRsReceivedPower(const SpectrumValue& rsReceivedPower)
{
    NS_LOG_FUNCTION(this << rsReceivedPower);
    m_rsrp = 10 * log10(Integral(rsReceivedPower)) + 30;
    // ntn-patch 19: with an SSB beam sweep the control region of each slot
    // goes out on another beam; the downlink path-loss reference is the best
    // SS/PBCH block (TS 38.213 7.1.1 q_d), here the strongest of the last burst.
    if (m_ssbBeamsPerBurst > 0)
    {
        m_rsPowerBurstMw.push_back(std::pow(10.0, m_rsrp / 10.0));
        while (m_rsPowerBurstMw.size() > m_ssbBeamsPerBurst)
        {
            m_rsPowerBurstMw.pop_front();
        }
        const double best = *std::max_element(m_rsPowerBurstMw.begin(), m_rsPowerBurstMw.end());
        m_rsrp = 10.0 * std::log10(best);
    }
    NS_LOG_DEBUG("RSRP value updated: " << m_rsrp << " dBm");

    if (m_enableUplinkPowerControl)
    {
        m_powerControl->SetLoggingInfo(GetCellId(), m_rnti);
        m_powerControl->SetRsrp(m_rsrp);
    }
}

void
NrUePhy::ReceivePss(uint16_t cellId, const Ptr<SpectrumValue>& p)
{
    NS_LOG_FUNCTION(this);

    double sum = 0.0;
    uint16_t nRB = 0;

    uint32_t subcarrierSpacing;
    subcarrierSpacing = 15000 * static_cast<uint32_t>(std::pow(2, GetNumerology()));

    Values::const_iterator itPi;
    for (itPi = p->ConstValuesBegin(); itPi != p->ConstValuesEnd(); itPi++)
    {
        // ntn-patch 19: only the RBs that carry this cell's SSB (TS 38.215 5.1.1)
        if (m_ssRsrpOnOccupiedRbs && (*itPi) == 0.0)
        {
            continue;
        }
        // convert PSD [W/Hz] to linear power [W] for the single RE
        double powerTxW = (*itPi) * subcarrierSpacing;
        sum += powerTxW;
        nRB++;
    }
    if (nRB == 0)
    {
        return; // ntn-patch 19: nothing of this SSB falls in the BWP
    }

    // Instantaneous RSRP in linear mW per RE. A zero-power sample (e.g. a fully
    // nulled beamforming cross-link) carries no detectable PSS: skip it rather
    // than store it, since its dB value (-inf) would poison every average and
    // Layer-3 filter it enters downstream.
    double rsrp = 1000.0 * (sum / static_cast<double>(nRB));
    if (rsrp <= 0.0)
    {
        return;
    }

    NS_LOG_DEBUG("RSRP value updated: " << 10 * log10(rsrp) << " dBm"
                                        << " for Cell Id: " << cellId << " RNTI: " << m_rnti);

    // ntn-patch 19: a cell that sweeps SSB beams is measured per SSB index
    // (TS 38.215 5.1.1 SS-RSRP per SS/PBCH block); its cell quality is derived
    // from the best beam in ReportUeMeasurements (TS 38.331 5.5.3.3).
    const uint8_t ssbIndex = m_spectrumPhy->GetLastRxSsbIndex();
    auto& measMap = (ssbIndex == NrSpectrumPhy::NO_SSB_INDEX) ? m_ueMeasurementsMap
                                                              : m_ueSsbMeasurementsMap[cellId];
    const uint16_t measKey = (ssbIndex == NrSpectrumPhy::NO_SSB_INDEX) ? cellId : ssbIndex;

    // store RSRP measurements
    auto itMeasMap = measMap.find(measKey);
    if (itMeasMap == measMap.end())
    {
        // insert new entry
        UeMeasurementsElement newEl;
        newEl.rsrpSum = rsrp;
        newEl.rsrpNum = 1;
        newEl.rsrqSum = 0;
        newEl.rsrqNum = 0;

        NS_LOG_DEBUG("New RSRP entry for Cell Id: " << cellId << " RNTI: " << m_rnti
                                                    << " RSRP: " << newEl.rsrpSum << " mW"
                                                    << " number of entries: " << +newEl.rsrpNum);

        measMap.insert(std::pair<uint16_t, UeMeasurementsElement>(measKey, newEl));
    }
    else
    {
        (*itMeasMap).second.rsrpSum += rsrp;
        (*itMeasMap).second.rsrpNum++;

        NS_LOG_DEBUG("Update RSRP entry for Cell Id: "
                     << cellId << " RNTI: " << m_rnti
                     << " RSRP Sum: " << (*itMeasMap).second.rsrpSum << " mW"
                     << " number of entries: " << +((*itMeasMap).second.rsrpNum));
    }
}

void
NrUePhy::SetGenieRsrpCallback(Callback<std::map<uint16_t, double>> cb)
{
    NS_LOG_FUNCTION(this);
    m_genieRsrpCallback = cb;
}

uint8_t
NrUePhy::GetBestSsbIndex(uint16_t cellId) const
{
    auto it = m_bestSsbIndex.find(cellId);
    return it == m_bestSsbIndex.end() ? NrSpectrumPhy::NO_SSB_INDEX : it->second;
}

void
NrUePhy::ReportUeMeasurements()
{
    NS_LOG_FUNCTION(this);

    // ntn-patch 19 (v5.1 port): TS 38.331 5.5.3.3 cell quality of a cell that
    // sweeps SSB beams. Without nrofSS-BlocksToAverage the cell quality is the
    // highest beam measurement quantity, so the layer-1 result of the best SSB
    // beam (per-beam linear average over this period, TS 38.215 5.1.1) stands
    // for the cell. Only that beam enters the map, so the RSSI below counts the
    // cell once rather than every SSB sample of its sweep.
    m_bestSsbIndex.clear();
    // ntn-patch 21: a sweep longer than the layer-1 period (an Earth-moving
    // grid of more SSB beams than one period visits) left some periods without
    // a sample of the best beam, and the cell quality fell to the best beam
    // that HAD been visited, 6 to 8 dB lower, about once a second. With
    // SsbBeamResultLifetime > 0 each beam keeps its last layer-1 result until
    // it is measured again or the lifetime expires, so the cell quality is the
    // best of the most recent per-beam results (TS 38.331 5.5.3.3), whichever
    // period they were taken in. 0 keeps the patch-19 behavior.
    std::map<uint16_t, std::map<uint16_t, UeMeasurementsElement>> ssbView;
    if (m_ssbBeamResultLifetime.IsStrictlyPositive())
    {
        const Time now = Simulator::Now();
        for (const auto& [cell, beams] : m_ueSsbMeasurementsMap)
        {
            for (const auto& [idx, el] : beams)
            {
                if (el.rsrpNum != 0)
                {
                    m_ssbBeamLast[cell][idx] = {el, now};
                }
            }
        }
        for (auto cit = m_ssbBeamLast.begin(); cit != m_ssbBeamLast.end();)
        {
            for (auto bit = cit->second.begin(); bit != cit->second.end();)
            {
                if (now - bit->second.second > m_ssbBeamResultLifetime)
                {
                    bit = cit->second.erase(bit);
                }
                else
                {
                    ssbView[cit->first][bit->first] = bit->second.first;
                    ++bit;
                }
            }
            cit = cit->second.empty() ? m_ssbBeamLast.erase(cit) : std::next(cit);
        }
    }
    const auto& ssbSource =
        m_ssbBeamResultLifetime.IsStrictlyPositive() ? ssbView : m_ueSsbMeasurementsMap;
    for (const auto& [cell, beams] : ssbSource)
    {
        double bestMw = -1.0;
        const UeMeasurementsElement* best = nullptr;
        uint16_t bestIdx = NrSpectrumPhy::NO_SSB_INDEX;
        for (const auto& [idx, el] : beams)
        {
            if (el.rsrpNum == 0)
            {
                continue;
            }
            const double mw = el.rsrpSum / static_cast<double>(el.rsrpNum); // linear mW
            if (mw > bestMw)
            {
                bestMw = mw;
                best = &el;
                bestIdx = idx;
            }
        }
        if (best)
        {
            m_ueMeasurementsMap[cell] = *best;
            m_bestSsbIndex[cell] = static_cast<uint8_t>(bestIdx);
        }
    }
    m_ueSsbMeasurementsMap.clear();

    if (!m_genieRsrpCallback.IsNull())
    {
        // Keep the PSS-measured serving cell entry intact (it reflects the
        // locked-beam signal that drives SINR / RLF), and replace neighbor
        // entries with genie best-beam estimates. The genie values are in
        // a different absolute reference than PSS-derived dBm (see
        // NrInitialAssociation::ComputeMaxRsrp), so anchor the scale
        // by computing an offset between PSS-serving and genie-serving and
        // shifting all reported neighbors by that offset. This preserves
        // the genie-computed relative deltas between cells while keeping
        // A3's serving anchor consistent with the real signal magnitude.
        const uint16_t servingCellId = GetCellId();
        auto genie = m_genieRsrpCallback();
        auto servingIt = m_ueMeasurementsMap.find(servingCellId);
        auto genieServingIt = genie.find(servingCellId);
        if (servingIt != m_ueMeasurementsMap.end() && genieServingIt != genie.end() &&
            servingIt->second.rsrpNum > 0)
        {
            const double pssServingRsrp =
                10 *
                log10(servingIt->second.rsrpSum / static_cast<double>(servingIt->second.rsrpNum));
            const double offset = pssServingRsrp - genieServingIt->second;
            for (auto it = m_ueMeasurementsMap.begin(); it != m_ueMeasurementsMap.end();)
            {
                if (it->first != servingCellId)
                {
                    it = m_ueMeasurementsMap.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            for (const auto& kv : genie)
            {
                if (kv.first == servingCellId)
                {
                    continue;
                }
                UeMeasurementsElement el;
                el.rsrpSum = std::pow(10.0, (kv.second + offset) / 10.0); // dBm -> linear mW
                el.rsrpNum = 1;
                el.rsrqSum = 0;
                el.rsrqNum = 0;
                m_ueMeasurementsMap[kv.first] = el;
            }
        }
    }

    NrUeCphySapUser::UeMeasurementsParameters ret{};

    // Carrier RSSI (per resource element, linear W) used to derive a real RSRQ below:
    // the total received power = sum of every measured cell's RS power plus thermal noise.
    // RSRP measures one cell's signal; RSSI captures the inter-cell interference + noise, so
    // RSRQ = RSRP/RSSI is the interference-aware quality the RSRP-only measurement lacked.
    double rssiPerReLin = 0.0;
    const auto scsHz = static_cast<double>(GetSubcarrierSpacing());
    if (scsHz > 0.0)
    {
        // thermal noise per subcarrier: kB*T (=-174 dBm/Hz at 290 K) * SCS * noise figure
        constexpr double kBT = 1.38064852e-23 * 290.0; // W/Hz
        rssiPerReLin += kBT * scsHz * std::pow(10.0, GetNoiseFigure() / 10.0);
    }
    for (const auto& kv : m_ueMeasurementsMap)
    {
        if (kv.second.rsrpNum > 0)
        {
            // rsrpSum is linear mW; convert the average to W
            rssiPerReLin += kv.second.rsrpSum / static_cast<double>(kv.second.rsrpNum) / 1000.0;
        }
    }

    std::map<uint16_t, UeMeasurementsElement>::iterator it;
    for (it = m_ueMeasurementsMap.begin(); it != m_ueMeasurementsMap.end(); it++)
    {
        double avg_rsrp;
        double avg_rsrq = 0;
        if ((*it).second.rsrpNum != 0)
        {
            // Samples are accumulated in linear power so that occasional
            // deep-null sidelobe samples (PSS received while the gNB's analog
            // beam points at another UE) do not dominate the average the way
            // they would in a dB-domain mean.
            avg_rsrp = 10 * log10((*it).second.rsrpSum / static_cast<double>((*it).second.rsrpNum));
            // RSRQ = SUBCARRIERS_PER_RB * RSRP / RSSI (per RE): this cell's RS power relative
            // to the total received power (all cells + noise). Interference-aware -- a strong
            // cell in a crowded/interfered zone scores low. (Previously hardcoded to 0 dB.)
            if (rssiPerReLin > 0.0)
            {
                const double rsrpLin = std::pow(10.0, (avg_rsrp - 30.0) / 10.0);
                avg_rsrq = 10.0 * std::log10(rsrpLin / (NrSpectrumValueHelper::SUBCARRIERS_PER_RB *
                                                        rssiPerReLin));
            }
        }
        else
        {
            NS_LOG_WARN(" RSRP nSamples is zero!");
            avg_rsrp = 0;
        }

        NS_LOG_DEBUG(" Report UE Measurements for CellId "
                     << (*it).first << " Reporting UE " << m_rnti << " Av. RSRP " << avg_rsrp
                     << " (nSamples " << +((*it).second.rsrpNum) << ")"
                     << " BwpID " << GetBwpId());

        m_reportRsrpTrace(GetCellId(), m_imsi, m_rnti, avg_rsrp, GetBwpId());

        NrUeCphySapUser::UeMeasurementsElement newEl;
        newEl.m_cellId = (*it).first;
        newEl.m_rsrp = avg_rsrp;
        newEl.m_rsrq = avg_rsrq; // interference-aware RSRQ (RSRP/RSSI), computed above
        ret.m_ueMeasurementsList.push_back(newEl);
        ret.m_componentCarrierId = GetBwpId();

        m_reportUeMeasurements(m_rnti,
                               (*it).first,
                               avg_rsrp,
                               avg_rsrq,
                               (*it).first == GetCellId(),
                               ret.m_componentCarrierId);
    }

    // report to RRC
    m_ueCphySapUser->ReportUeMeasurements(ret);

    m_ueMeasurementsMap.clear();
    Simulator::Schedule(m_ueMeasurementsFilterPeriod, &NrUePhy::ReportUeMeasurements, this);
}

void
NrUePhy::SetCsiFeedbackType(uint8_t csiFeedbackType)
{
    m_csiFeedbackType = csiFeedbackType;
}

void
NrUePhy::ReportDlCtrlSinr(const SpectrumValue& sinr)
{
    NS_LOG_FUNCTION(this);
    uint32_t rbUsed = 0;
    double sinrSum = 0.0;

    for (uint32_t i = 0; i < sinr.GetValuesN(); i++)
    {
        double currentSinr = sinr.ValuesAt(i);
        if (currentSinr != 0)
        {
            rbUsed++;
            sinrSum += currentSinr;
        }
    }

    NS_ASSERT(rbUsed);
    m_ctrlSinrForRlf = sinr;
    m_dlCtrlSinrTrace(GetCellId(), m_rnti, sinrSum / rbUsed, GetBwpId());

    // trigger RLF detection only when UE has an active RRC connection
    // and RLF detection attribute is set to true
    if (m_isConnected && m_enableRlfDetection)
    {
        RlfDetection(10 * log10(ComputeAvgSinr(m_ctrlSinrForRlf)));
    }
}

uint8_t
NrUePhy::ComputeCqi(const SpectrumValue& sinr)
{
    NS_LOG_FUNCTION(this);
    uint8_t mcs; // it is initialized by AMC in the following call
    uint8_t wbCqi = m_amc->CreateCqiFeedbackSiso(sinr, mcs);
    return wbCqi;
}

void
NrUePhy::StartEventLoop(uint16_t frame, uint8_t subframe, uint16_t slot)
{
    NS_LOG_FUNCTION(this);

    if (GetChannelBandwidth() == 0)
    {
        NS_LOG_INFO("Initial bandwidth not set, configuring the default one for Cell ID: "
                    << GetCellId() << ", RNTI: " << GetRnti() << ", BWP ID: " << GetBwpId());
        if (GetSubcarrierSpacing() == 0)
        {
            NS_LOG_INFO("No numerology was set, assuming numerology 0 for Cell ID: "
                        << GetCellId() << ", RNTI: " << GetRnti() << ", BWP ID: " << GetBwpId());
            SetNumerology(0);
        }
        DoSetInitialBandwidth();
    }

    NS_LOG_INFO("PHY starting. Configuration: "
                << std::endl
                << "\t TxPower: " << m_txPower << " dBm" << std::endl
                << "\t NoiseFigure: " << m_noiseFigure << std::endl
                << "\t TbDecodeLatency: " << GetTbDecodeLatency().GetMicroSeconds() << " us "
                << std::endl
                << "\t Numerology: " << GetNumerology() << std::endl
                << "\t SymbolsPerSlot: " << GetSymbolsPerSlot() << std::endl
                << "\t Pattern: " << NrPhy::GetPattern(m_tddPattern) << std::endl
                << "Attached to physical channel: " << std::endl
                << "\t Channel bandwidth: " << GetChannelBandwidth() << " Hz" << std::endl
                << "\t Channel central freq: " << GetCentralFrequency() << " Hz" << std::endl
                << "\t Num. RB: " << GetRbNum());
    SfnSf startSlot(frame, subframe, slot, GetNumerology());
    StartSlot(startSlot);
}

void
NrUePhy::DoSetInitialBandwidth()
{
    NS_LOG_FUNCTION(this);
    // configure initial bandwidth to 6 RBs, numerology 0. ntn-patch 19: with frequency reuse the
    // cell's SSB / CORESET#0 sit in its own sub-band (TR 38.821 6.1.1.1), which a 6-RB window at
    // the carrier centre misses for every reuse colour but the middle one; InitialBandwidthRbs
    // widens the initial search window (0 keeps the 6 RBs).
    const uint32_t initialRbs = (m_initialBandwidthRbs > 0) ? m_initialBandwidthRbs : 6;
    double initialBandwidthHz =
        initialRbs * GetSubcarrierSpacing() * NrSpectrumValueHelper::SUBCARRIERS_PER_RB;
    // divided by 100*1000 because the parameter should be in 100KHz
    uint16_t initialBandwidthIn100KHz = ceil(initialBandwidthHz / (100 * 1000));
    // account for overhead that will be reduced when determining real BW
    uint16_t initialBandwidthWithOverhead = initialBandwidthIn100KHz / (1 - GetRbOverhead());

    NS_ABORT_MSG_IF(initialBandwidthWithOverhead == 0,
                    " Initial bandwidth could not be set. Parameters provided are: "
                    "\n dlBandwidthInRBNum = "
                        << 6 << "\n m_subcarrierSpacing = " << GetSubcarrierSpacing()
                        << "\n NrSpectrumValueHelper::SUBCARRIERS_PER_RB  = "
                        << (unsigned)NrSpectrumValueHelper::SUBCARRIERS_PER_RB
                        << "\n m_rbOh = " << GetRbOverhead());

    DoSetDlBandwidth(initialBandwidthWithOverhead);
}

uint16_t
NrUePhy::DoGetCellId() const
{
    return GetCellId();
}

void
NrUePhy::DoSetDlBandwidth(uint16_t dlBandwidth)
{
    NS_LOG_FUNCTION(this << +dlBandwidth);

    const auto rbNum = GetRbNum();
    SetChannelBandwidth(dlBandwidth);
    if (GetRbNum() != rbNum)
    {
        ResetCsiMeasurements();
    }

    NS_LOG_DEBUG("PHY reconfiguring. Result: "
                 << std::endl
                 << "\t TxPower: " << m_txPower << " dBm" << std::endl
                 << "\t NoiseFigure: " << m_noiseFigure << std::endl
                 << "\t TbDecodeLatency: " << GetTbDecodeLatency().GetMicroSeconds() << " us "
                 << std::endl
                 << "\t Numerology: " << GetNumerology() << std::endl
                 << "\t SymbolsPerSlot: " << GetSymbolsPerSlot() << std::endl
                 << "\t Pattern: " << NrPhy::GetPattern(m_tddPattern) << std::endl
                 << "Attached to physical channel: " << std::endl
                 << "\t Channel bandwidth: " << GetChannelBandwidth() << " Hz" << std::endl
                 << "\t Channel central freq: " << GetCentralFrequency() << " Hz" << std::endl
                 << "\t Num. RB: " << GetRbNum());
}

void
NrUePhy::DoConfigureUplink(uint16_t arfcn, uint8_t ulBandwidth)
{
    NS_LOG_FUNCTION(this << arfcn << +ulBandwidth);
    // Ignore this; should be equal to dlBandwidth
    m_ulConfigured = true;
}

void
NrUePhy::DoConfigureReferenceSignalPower(int8_t referenceSignalPower)
{
    NS_LOG_FUNCTION(this << referenceSignalPower);
    m_powerControl->ConfigureReferenceSignalPower(referenceSignalPower);
}

void
NrUePhy::DoSetRnti(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    GetSpectrumPhy()->SetRnti(rnti);
    m_rnti = rnti;
}

void
NrUePhy::DoSetTransmissionMode(uint8_t txMode)
{
    NS_LOG_FUNCTION(this << +txMode);
}

void
NrUePhy::DoSetSrsConfigurationIndex(uint16_t srcCi)
{
    NS_LOG_FUNCTION(this << srcCi);
}

void
NrUePhy::SetPhySapUser(NrUePhySapUser* ptr)
{
    m_phySapUser = ptr;
}

void
NrUePhy::NotifyConnectionSuccessful()
{
    m_isConnected = true;
    // Initialize the parameters for radio link failure detection
    InitializeRlfParams();
}

void
NrUePhy::DoResetPhyAfterRlf()
{
    NS_LOG_FUNCTION(this);
    // m_spectrumPhy->m_harqPhyModule->ClearDlHarqBuffer(m_rnti); // flush HARQ buffers
    ClearRntiSlotAllocInfo(m_rnti);
    DoSetCellId(0);
    DoReset();
}

void
NrUePhy::DoResetRlfParams()
{
    NS_LOG_FUNCTION(this);
    InitializeRlfParams();
}

void
NrUePhy::DoStartInSyncDetection()
{
    NS_LOG_FUNCTION(this);
    // indicates that the downlink radio link quality has to be monitored for in-sync indications
    m_downlinkInSync = false;
}

void
NrUePhy::InitializeRlfParams()
{
    NS_LOG_FUNCTION(this);
    m_numOfSubframes = 0;
    m_sinrDbFrame = 0;
    m_numOfFrames = 0;
    m_downlinkInSync = true;
}

void
NrUePhy::RlfDetection(double sinrDb)
{
    NS_LOG_FUNCTION(this << sinrDb);
    m_sinrDbFrame += sinrDb;
    m_numOfSubframes++;
    NS_LOG_LOGIC("No of Subframes: " << m_numOfSubframes
                                     << " UE synchronized: " << m_downlinkInSync);
    // check for out_of_sync indications first when UE is both DL and UL synchronized
    // m_downlinkInSync=true indicates that the evaluation is for out-of-sync indications
    if (m_downlinkInSync && m_numOfSubframes == 10)
    {
        /**
         * For every frame, if the downlink radio link quality(avg SINR)
         * is less than the threshold Qout, then the frame cannot be decoded
         */
        if ((m_sinrDbFrame / m_numOfSubframes) < m_qOut)
        {
            m_numOfFrames++; // increment the counter if a frame cannot be decoded
            NS_LOG_LOGIC("No of Frames which cannot be decoded: " << m_numOfFrames);
        }
        else
        {
            /**
             * If the downlink radio link quality(avg SINR) is greater
             * than the threshold Qout, then the frame counter is reset
             * since only consecutive frames should be considered.
             */
            NS_LOG_INFO("Resetting frame counter at phy. Current value = " << m_numOfFrames);
            m_numOfFrames = 0;
            // Also reset the sync indicator counter at RRC
            m_ueCphySapUser->ResetSyncIndicationCounter();
        }
        m_numOfSubframes = 0;
        m_sinrDbFrame = 0;
    }
    /**
     * Once the number of consecutive frames which cannot be decoded equals
     * the Qout evaluation period (i.e 200ms), then an out-of-sync indication
     * is sent to the RRC layer
     */
    if (m_downlinkInSync && (m_numOfFrames * 10) == m_numOfQoutEvalSf)
    {
        NS_LOG_LOGIC("At " << Simulator::Now().As(Time::MS)
                           << " ms UE PHY sending out of sync indication to UE RRC layer");
        m_ueCphySapUser->NotifyOutOfSync();
        m_numOfFrames = 0;
    }
    // check for in_sync indications when T310 timer is started
    // m_downlinkInSync=false indicates that the evaluation is for in-sync indications
    if (!m_downlinkInSync && m_numOfSubframes == 10)
    {
        /**
         * For every frame, if the downlink radio link quality(avg SINR)
         * is greater than the threshold Qin, then the frame can be
         * successfully decoded.
         */
        if ((m_sinrDbFrame / m_numOfSubframes) > m_qIn)
        {
            m_numOfFrames++; // increment the counter if a frame can be decoded
            NS_LOG_LOGIC("No of Frames successfully decoded: " << m_numOfFrames);
        }
        else
        {
            /**
             * If the downlink radio link quality(avg SINR) is less
             * than the threshold Qin, then the frame counter is reset
             * since only consecutive frames should be considered
             */
            m_numOfFrames = 0;
            // Also reset the sync indicator counter at RRC
            m_ueCphySapUser->ResetSyncIndicationCounter();
        }
        m_numOfSubframes = 0;
        m_sinrDbFrame = 0;
    }
    /**
     * Once the number of consecutive frames which can be decoded equals the Qin evaluation period
     * (i.e 100ms), then an in-sync indication is sent to the RRC layer
     */
    if (!m_downlinkInSync && (m_numOfFrames * 10) == m_numOfQinEvalSf)
    {
        NS_LOG_LOGIC("At " << Simulator::Now().As(Time::MS)
                           << " ms UE PHY sending in sync indication to UE RRC layer");
        m_ueCphySapUser->NotifyInSync();
        m_numOfFrames = 0;
    }
}

void
NrUePhy::DoSetImsi(uint64_t imsi)
{
    NS_LOG_FUNCTION(this);
    m_imsi = imsi;
}

void
NrUePhy::GenerateDlCqiReportMimo(const NrMimoSignal& rxSignal,
                                 NrPmSearch::PmiUpdate pmiUpdateParams)
{
    NS_LOG_FUNCTION(this);
    // Adopted from NrUePhy::GenerateDlCqiReport: CQI feedback requires properly configured UE
    if (!m_ulConfigured || (m_rnti == 0))
    {
        return;
    }

    // Create DL CQI message for CQI, PMI, and RI. PMI values are updated only if specified by
    // pmiUpdateParams, otherwise assume same PMI values as during last CQI feedback
    auto cqi = m_pmSearch->CreateCqiFeedbackMimo(rxSignal, pmiUpdateParams);
    auto dlcqi = DlCqiInfo{
        .m_rnti = m_rnti,
        .m_ri = cqi.m_rank,
        .m_cqiType = cqi.m_cqiType,
        .m_wbCqi = cqi.m_wbCqi,
        .m_wbPmi = cqi.m_wbPmi,
        .m_sbCqis = cqi.m_sbCqis,
        .m_sbPmis = cqi.m_sbPmis,
        .m_mcs = cqi.m_mcs,
        .m_optPrecMat = cqi.m_optPrecMat,
    };

    m_cqiFeedbackTrace(m_rnti, cqi.m_wbCqi, cqi.m_mcs, cqi.m_rank);

    auto msg = Create<NrDlCqiMessage>();
    msg->SetSourceBwpArfcn(DoGetArfcn());
    msg->SetDlCqi(dlcqi);

    DoSendControlMessage(msg);
}

uint8_t
NrUePhy::GetCsiFeedbackType() const
{
    return m_csiFeedbackType;
}

void
NrUePhy::CsiRsReceived(const std::vector<MimoSignalChunk>& csiRsMimoSignal)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(csiRsMimoSignal.size() == 1);
    m_csiRsMimoSignal = NrMimoSignal(csiRsMimoSignal);
    m_lastCsiRsMimoSignalTime = Simulator::Now();
}

void
NrUePhy::GenerateCsiRsCqi()
{
    NS_LOG_FUNCTION(this);

    if (!m_isConnected)
    {
        NS_LOG_DEBUG("UE is disconnected, so not transmitting any CSI feedback.");
        return;
    }
    if (m_csiRsMimoSignal.m_chanMat.GetSize() == 0)
    {
        NS_LOG_DEBUG("The CSI-RS was discarded by a cell or bandwidth change.");
        return;
    }
    NrMimoSignal csiFeedbackSignal = m_csiRsMimoSignal;
    // if there is some old interference information use it,
    // otherwise, just use the plain CSI-RS signal for the CQI feedback
    // (i.e., no interference information). This may happen before any
    // PDSCH for this UE is scheduled, and CSI-IM is disabled.
    if (m_avgIntCovMat.GetSize() != 0)
    {
        csiFeedbackSignal.m_covMat = m_avgIntCovMat;
    }
    TriggerDlCqiGeneration(csiFeedbackSignal, NrPmSearch::PmiUpdate(true, true));
}

void
NrUePhy::CsiImEnded(const std::vector<MimoSignalChunk>& csiImSignalChunks)
{
    NS_LOG_FUNCTION(this);

    if (!m_isConnected)
    {
        NS_LOG_DEBUG("UE is disconnected, so not transmitting any CSI feedback.");
        return;
    }

    // Combine multiple CSI-IM signal chunks into a single channel,
    // and interference covariance
    auto csiFeedbackSignal = NrMimoSignal(csiImSignalChunks);
    CalcAvgIntCovMat(&m_avgIntCovMat, csiFeedbackSignal.m_covMat);
    // CSI-IM does not have RX spectrum channel matrix, because it only contains the interference
    // hence the channel spectrum matrix to be used is from CSI-RS signal
    if (m_csiRsMimoSignal.m_chanMat.GetSize() == 0)
    {
        NS_LOG_DEBUG("The CSI-RS of this CSI-IM was discarded by a cell or bandwidth change.");
        return;
    }
    if (m_alphaCovMat != 1)
    {
        csiFeedbackSignal.m_covMat = m_avgIntCovMat;
    }
    csiFeedbackSignal.m_chanMat = m_csiRsMimoSignal.m_chanMat;
    TriggerDlCqiGeneration(csiFeedbackSignal, NrPmSearch::PmiUpdate(true, true));
}

void
NrUePhy::PdschMimoReceived(const std::vector<MimoSignalChunk>& pdschMimoChunks)
{
    NS_LOG_FUNCTION(this);

    if (!m_isConnected)
    {
        NS_LOG_DEBUG("UE is disconnected, so not transmitting any CSI feedback.");
        return;
    }

    // Combine multiple signal chunks into a single channel matrix and interference covariance
    auto csiFeedbackSignal = NrMimoSignal(pdschMimoChunks);
    // if alpha != 1, calculate the interference covariance moving average
    CalcAvgIntCovMat(&m_avgIntCovMat, csiFeedbackSignal.m_covMat);
    if (m_alphaCovMat != 1)
    {
        csiFeedbackSignal.m_covMat = m_avgIntCovMat;
    }
    // if CSI-RS enabled, use the spectrum channel matrix from CSI-RS signal, once the UE
    // received a CSI-RS since it connected to the cell
    if ((m_csiFeedbackType & CQI_CSI_RS) && m_csiRsMimoSignal.m_chanMat.GetSize() != 0)
    {
        csiFeedbackSignal.m_chanMat = m_csiRsMimoSignal.m_chanMat;
    }

    // CSI-RS slot, or PDSCH only based CQI feedback
    // Determine if an update to wideband or subband PMI is needed and possible
    auto pmiUpdateParams = CheckUpdatePmi();
    TriggerDlCqiGeneration(csiFeedbackSignal, pmiUpdateParams);
}

void
NrUePhy::TriggerDlCqiGeneration(const NrMimoSignal& csiFeedbackSignal,
                                NrPmSearch::PmiUpdate pmiUpdateParams)
{
    NS_LOG_FUNCTION(this);
    if (m_pmSearch)
    {
        GenerateDlCqiReportMimo(csiFeedbackSignal, pmiUpdateParams);
    }
    else
    {
        // Interference whitening: normalize the signal such that interference + noise covariance
        // matrix is the identity matrix
        auto intfNormChanMat =
            csiFeedbackSignal.m_covMat.CalcIntfNormChannel(csiFeedbackSignal.m_chanMat);
        // Create a dummy precoding matrix
        ComplexMatrixArray precMat = ComplexMatrixArray(
            csiFeedbackSignal.m_chanMat.GetNumCols(),
            csiFeedbackSignal.m_chanMat.GetNumRows(),
            csiFeedbackSignal.m_chanMat.GetNumPages(),
            std::valarray<std::complex<double>>(std::complex<double>(1.0, 0.0),
                                                csiFeedbackSignal.m_chanMat.GetSize()));
        ;

        NrSinrMatrix sinrMatrix = intfNormChanMat.ComputeSinrForPrecoding(precMat);
        GenerateDlCqiReport(sinrMatrix.GetVectorizedSpecVal());
    }
}

void
NrUePhy::CalcAvgIntCovMat(NrCovMat* avgIntCovMat, const NrCovMat& newCovMat) const
{
    NS_LOG_FUNCTION(this);
    if (avgIntCovMat->GetSize() == 0)
    {
        *avgIntCovMat = ComplexMatrixArray(newCovMat.GetNumRows(),
                                           newCovMat.GetNumCols(),
                                           newCovMat.GetNumPages());
    }

    *avgIntCovMat = newCovMat * std::complex<double>{m_alphaCovMat, 0.0} +
                    *avgIntCovMat * std::complex<double>{1 - m_alphaCovMat, 0.0};
}

NrPmSearch::PmiUpdate
NrUePhy::CheckUpdatePmi()
{
    // This implementation only checks if sufficient time has passed since the last update.
    // TODO: Improve following logic that defines when to update wideband and/or
    // subband PMIs for two-stage codebooks. The algorithm must allow managing the computational
    // complexity of PMI updates, and take into account availability of PUCCH/PUSCH resources for
    // sending PMI.
    auto pmiUpdate = NrPmSearch::PmiUpdate{};
    auto now = Simulator::Now();
    if (now > m_wbPmiLastUpdate + m_wbPmiUpdateInterval)
    {
        pmiUpdate.updateWb = true;
        m_wbPmiLastUpdate = now;
    }
    if (now > m_sbPmiLastUpdate + m_sbPmiUpdateInterval)
    {
        pmiUpdate.updateSb = true;
        m_sbPmiLastUpdate = now;
    }
    return pmiUpdate;
}

void
NrUePhy::SetPmSearch(Ptr<NrPmSearch> pmSearch)
{
    m_pmSearch = pmSearch;
    NS_ASSERT(m_amc);
    m_pmSearch->SetAmc(m_amc);
}

Ptr<NrPmSearch>
NrUePhy::GetPmSearch() const
{
    return m_pmSearch;
}

} // namespace ns3

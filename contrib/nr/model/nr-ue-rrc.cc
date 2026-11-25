// Copyright (c) 2011, 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
// Copyright (c) 2018 Fraunhofer ESK : RLF extensions
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Nicola Baldo <nbaldo@cttc.es>
//         Budiarto Herman <budiarto.herman@magister.fi>
// Modified by:
//          Danilo Abrignani <danilo.abrignani@unibo.it> (Carrier Aggregation - GSoC 2015)
//          Biljana Bojovic <biljana.bojovic@cttc.es> (Carrier Aggregation)
//          Vignesh Babu <ns3-dev@esk.fraunhofer.de> (RLF extensions)

#include "nr-ue-rrc.h"

#include "nr-common.h"
#include "nr-device-registry.h"
#include "nr-pdcp.h"
#include "nr-radio-bearer-info.h"
#include "nr-rlc-am.h"
#include "nr-rlc-tm.h"
#include "nr-rlc-um.h"
#include "nr-rlc.h"

#include "ns3/double.h"
#include "ns3/fatal-error.h"
#include "ns3/log.h"
#include "ns3/object-factory.h"
#include "ns3/object-map.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <iostream>

namespace ns3
{
const Time NR_UE_MEASUREMENT_REPORT_DELAY = MicroSeconds(1);

NS_LOG_COMPONENT_DEFINE("NrUeRrc");

/////////////////////////////
// CMAC SAP forwarder
/////////////////////////////

/// UeMemberNrUeCmacSapUser class
class UeMemberNrUeCmacSapUser : public NrUeCmacSapUser
{
  public:
    /**
     * Constructor
     *
     * @param rrc the RRC class
     */
    UeMemberNrUeCmacSapUser(NrUeRrc* rrc);

    void SetTemporaryCellRnti(uint16_t rnti) override;
    void NotifyRandomAccessSuccessful() override;
    void NotifyRandomAccessFailed() override;

  private:
    NrUeRrc* m_rrc; ///< the RRC class
};

UeMemberNrUeCmacSapUser::UeMemberNrUeCmacSapUser(NrUeRrc* rrc)
    : m_rrc(rrc)
{
}

void
UeMemberNrUeCmacSapUser::SetTemporaryCellRnti(uint16_t rnti)
{
    m_rrc->DoSetTemporaryCellRnti(rnti);
}

void
UeMemberNrUeCmacSapUser::NotifyRandomAccessSuccessful()
{
    m_rrc->DoNotifyRandomAccessSuccessful();
}

void
UeMemberNrUeCmacSapUser::NotifyRandomAccessFailed()
{
    m_rrc->DoNotifyRandomAccessFailed();
}

/// Map each of UE RRC states to its string representation.
const std::string
ToString(NrUeRrc::State state)
{
    switch (state)
    {
    case NrUeRrc::IDLE_START:
        return "IDLE_START";
    case NrUeRrc::IDLE_CELL_SEARCH:
        return "IDLE_CELL_SEARCH";
    case NrUeRrc::IDLE_WAIT_MIB_SIB1:
        return "IDLE_WAIT_MIB_SIB1";
    case NrUeRrc::IDLE_WAIT_MIB:
        return "IDLE_WAIT_MIB";
    case NrUeRrc::IDLE_WAIT_SIB1:
        return "IDLE_WAIT_SIB1";
    case NrUeRrc::IDLE_CAMPED_NORMALLY:
        return "IDLE_CAMPED_NORMALLY";
    case NrUeRrc::IDLE_WAIT_SIB2:
        return "IDLE_WAIT_SIB2";
    case NrUeRrc::IDLE_RANDOM_ACCESS:
        return "IDLE_RANDOM_ACCESS";
    case NrUeRrc::IDLE_CONNECTING:
        return "IDLE_CONNECTING";
    case NrUeRrc::CONNECTED_NORMALLY:
        return "CONNECTED_NORMALLY";
    case NrUeRrc::CONNECTED_HANDOVER:
        return "CONNECTED_HANDOVER";
    case NrUeRrc::CONNECTED_PHY_PROBLEM:
        return "CONNECTED_PHY_PROBLEM";
    case NrUeRrc::CONNECTED_REESTABLISHING:
        return "CONNECTED_REESTABLISHING";
    default:
        return "UNKNOWN_STATE";
    }
}

std::ostream&
operator<<(std::ostream& os, NrUeRrc::State state)
{
    os << ToString(state);
    return os;
}

/////////////////////////////
// ue RRC methods
/////////////////////////////

NS_OBJECT_ENSURE_REGISTERED(NrUeRrc);

NrUeRrc::NrUeRrc()
    : m_cmacSapProvider(0),
      m_rrcSapUser(nullptr),
      m_macSapProvider(nullptr),
      m_asSapUser(nullptr),
      m_ccmRrcSapProvider(nullptr),
      m_state(IDLE_START),
      m_imsi(0),
      m_rnti(0),
      m_cellId(0),
      m_useRlcSm(true),
      m_useRrcReestablishment(true),
      m_connectionPending(false),
      m_hasReceivedMib(false),
      m_hasReceivedSib1(false),
      m_hasReceivedSib2(false),
      m_csgWhiteList(0),
      m_noOfSyncIndications(0),
      m_leaveConnectedMode(false),
      m_previousCellId(0),
      m_connEstFailCountLimit(0),
      m_connEstFailCount(0),
      m_numberOfComponentCarriers(nr::MIN_NO_CC)
{
    NS_LOG_FUNCTION(this);
    m_cphySapUser.push_back(new MemberNrUeCphySapUser<NrUeRrc>(this));
    m_cmacSapUser.push_back(new UeMemberNrUeCmacSapUser(this));
    m_cphySapProvider.push_back(nullptr);
    m_cmacSapProvider.push_back(nullptr);
    m_rrcSapProvider = new MemberNrUeRrcSapProvider<NrUeRrc>(this);
    m_drbPdcpSapUser = new NrPdcpSpecificNrPdcpSapUser<NrUeRrc>(this);
    m_asSapProvider = new MemberNrAsSapProvider<NrUeRrc>(this);
    m_ccmRrcSapUser = new MemberNrUeCcmRrcSapUser<NrUeRrc>(this);
}

NrUeRrc::~NrUeRrc()
{
    NS_LOG_FUNCTION(this);
}

void
NrUeRrc::NotifyConnectionSuccessfulAllBwps()
{
    // ntn-patch 24: every bandwidth part of the serving cell is connected, not only the primary
    // UL/DL ones. The UE PHY of a BWP gates its MIMO CSI feedback (PDSCH, CSI-RS and CSI-IM
    // based) on its own connection state, so a BWP never told about the connection received
    // PDSCH but sent no CQI/PMI/RI: with per-slice BWPs every slice BWP ran at the starting
    // MCS 0 (measured: MCS 0 at 22 dB SINR, 0.2 Mbit/s; 2.1 Mbit/s with a single BWP).
    for (auto& mac : m_cmacSapProvider)
    {
        if (mac)
        {
            mac->NotifyConnectionSuccessful();
        }
    }
}

void
NrUeRrc::DoDispose()
{
    NS_LOG_FUNCTION(this);
    for (uint16_t i = 0; i < m_numberOfComponentCarriers; i++)
    {
        delete m_cphySapUser.at(i);
        delete m_cmacSapUser.at(i);
    }
    m_cphySapUser.clear();
    m_cmacSapUser.clear();
    delete m_rrcSapProvider;
    delete m_drbPdcpSapUser;
    delete m_asSapProvider;
    delete m_ccmRrcSapUser;
    m_cphySapProvider.erase(m_cphySapProvider.begin(), m_cphySapProvider.end());
    m_cphySapProvider.clear();
    m_cmacSapProvider.erase(m_cmacSapProvider.begin(), m_cmacSapProvider.end());
    m_cmacSapProvider.clear();
    m_drbMap.clear();
    // These callbacks capture a Ptr to the BWP manager, which in turn holds a callback capturing
    // this RRC. Release them to break the reference cycle.
    m_updateBwpOutputLinkFn = nullptr;
    m_updateQosFlowBwpFn = nullptr;
    m_clearBwpOutputLinksFn = nullptr;
}

TypeId
NrUeRrc::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrUeRrc")
            .SetParent<Object>()
            .SetGroupName("Nr")
            .AddConstructor<NrUeRrc>()
            .AddAttribute("DataRadioBearerMap",
                          "List of UE RadioBearerInfo for Data Radio Bearers by LCID.",
                          ObjectMapValue(),
                          MakeObjectMapAccessor(&NrUeRrc::m_drbMap),
                          MakeObjectMapChecker<NrDataRadioBearerInfo>())
            .AddAttribute("BwpSwitchHysteresis",
                          "Hysteresis margin (dB) a non-serving same-cell BWP must exceed the "
                          "serving BWP's RSRP by before the UE switches its primary BWP to it.",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&NrUeRrc::m_bwpSwitchHysteresisDb),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("Srb0",
                          "SignalingRadioBearerInfo for SRB0",
                          PointerValue(),
                          MakePointerAccessor(&NrUeRrc::m_srb0),
                          MakePointerChecker<NrSignalingRadioBearerInfo>())
            .AddAttribute("Srb1",
                          "SignalingRadioBearerInfo for SRB1",
                          PointerValue(),
                          MakePointerAccessor(&NrUeRrc::m_srb1),
                          MakePointerChecker<NrSignalingRadioBearerInfo>())
            .AddAttribute("CellId",
                          "Serving cell identifier",
                          UintegerValue(0), // unused, read-only attribute
                          MakeUintegerAccessor(&NrUeRrc::GetCellId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("C-RNTI",
                          "Cell Radio Network Temporary Identifier",
                          UintegerValue(0), // unused, read-only attribute
                          MakeUintegerAccessor(&NrUeRrc::GetRnti),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute(
                "T300",
                "Timer for the RRC Connection Establishment procedure "
                "(i.e., the procedure is deemed as failed if it takes longer than this). "
                "Standard values: 100ms, 200ms, 300ms, 400ms, 600ms, 1000ms, 1500ms, 2000ms",
                TimeValue(MilliSeconds(
                    100)), // see 3GPP 36.331 UE-TimersAndConstants & RLF-TimersAndConstants
                MakeTimeAccessor(&NrUeRrc::m_t300),
                MakeTimeChecker(MilliSeconds(100), MilliSeconds(2000)))
            .AddAttribute(
                "T310",
                "Timer for detecting the Radio link failure "
                "(i.e., the radio link is deemed as failed if this timer expires). "
                "Standard values: 0ms 50ms, 100ms, 200ms, 500ms, 1000ms, 2000ms",
                TimeValue(MilliSeconds(
                    1000)), // see 3GPP 36.331 UE-TimersAndConstants & RLF-TimersAndConstants
                MakeTimeAccessor(&NrUeRrc::m_t310),
                MakeTimeChecker(MilliSeconds(0), MilliSeconds(2000)))
            .AddAttribute(
                "N310",
                "This specifies the maximum number of out-of-sync indications. "
                "Standard values: 1, 2, 3, 4, 6, 8, 10, 20",
                UintegerValue(6), // see 3GPP 36.331 UE-TimersAndConstants & RLF-TimersAndConstants
                MakeUintegerAccessor(&NrUeRrc::m_n310),
                MakeUintegerChecker<uint8_t>(1, 20))
            .AddAttribute(
                "N311",
                "This specifies the maximum number of in-sync indications. "
                "Standard values: 1, 2, 3, 4, 5, 6, 8, 10",
                UintegerValue(2), // see 3GPP 36.331 UE-TimersAndConstants & RLF-TimersAndConstants
                MakeUintegerAccessor(&NrUeRrc::m_n311),
                MakeUintegerChecker<uint8_t>(1, 10))
            .AddAttribute(
                "UseRrcReestablishment",
                "Whether to use RRC connection reestablishment procedure. "
                "When disabled, the UE directly clears its context upon radio link failure.",
                BooleanValue(true),
                MakeBooleanAccessor(&NrUeRrc::m_useRrcReestablishment),
                MakeBooleanChecker())
            .AddAttribute("RlcMaxRetxTriggersRlf",
                          "If true, an RLC-AM entity reaching maxRetxThreshold declares a radio "
                          "link failure (TS 38.331 5.3.10.3). This lets an undeliverable uplink "
                          "(e.g. a lost RrcConnectionReconfigurationComplete on a degraded link) "
                          "fail and recover via reestablishment instead of retransmitting forever "
                          "while the peer stays stuck. Default false (legacy behaviour: RLF is "
                          "driven only by T310 / handover-command timing).",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrUeRrc::m_rlcMaxRetxTriggersRlf),
                          MakeBooleanChecker())
            .AddAttribute(
                "Tr36839HandoverFailure",
                "Prototype TR 36.839 (5.3.2) handover-failure model: if the handover "
                "command is received while the source radio link is already below Qout "
                "(modelled as the T310 timer running), declare the handover a failure and "
                "trigger radio link failure, instead of letting the late command rescue the "
                "link. Disabled by default to preserve legacy behaviour.",
                BooleanValue(false),
                MakeBooleanAccessor(&NrUeRrc::m_tr36839HandoverFailure),
                MakeBooleanChecker())
            .AddAttribute(
                "Tr36839HoFailureMinT310Elapsed",
                "Graded threshold for the TR 36.839 handover-failure model "
                "(Tr36839HandoverFailure). A handover command that arrives while T310 is "
                "running is declared a too-late failure only if T310 has already been "
                "running for at least this long; a command arriving earlier still rescues "
                "the link (the UE can still receive it). This makes the failure depend on "
                "HOW degraded the source is, not merely on T310 being pending, so short-"
                "TimeToTrigger configurations (whose commands arrive early in T310) escape "
                "while long-TTT ones (deep in T310) fail. 0 ms reproduces the original "
                "binary behaviour (fail on any pending T310).",
                TimeValue(MilliSeconds(0)),
                MakeTimeAccessor(&NrUeRrc::m_tr36839HoFailureMinT310Elapsed),
                MakeTimeChecker())
            .AddAttribute("MseEnable",
                          "Enable Mobility State Estimation (TS 36.331 5.5.6.2 / TS 36.304 "
                          "5.2.4.3): count recent handovers to classify the UE mobility state and "
                          "scale the measurement time-to-trigger, so a fast UE does not chase a "
                          "small cell's transient peak (nor fail to hand over in time).",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrUeRrc::m_mseEnable),
                          MakeBooleanChecker())
            .AddAttribute("MseCountWindow",
                          "Sliding window over which handovers are counted for mobility-state "
                          "estimation.",
                          TimeValue(Seconds(1)),
                          MakeTimeAccessor(&NrUeRrc::m_mseCountWindow),
                          MakeTimeChecker())
            .AddAttribute("MseHystNormal",
                          "Minimum time the UE keeps an elevated (Medium/High) mobility state "
                          "before it may drop back down.",
                          TimeValue(Seconds(1)),
                          MakeTimeAccessor(&NrUeRrc::m_mseHystNormal),
                          MakeTimeChecker())
            .AddAttribute("MseThreshMedium",
                          "Handover count within MseCountWindow at/above which the UE is Medium "
                          "mobility.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&NrUeRrc::m_mseThreshMedium),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("MseThreshHigh",
                          "Handover count within MseCountWindow at/above which the UE is High "
                          "mobility.",
                          UintegerValue(4),
                          MakeUintegerAccessor(&NrUeRrc::m_mseThreshHigh),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("MseSfMedium",
                          "Time-to-trigger scale factor applied in Medium mobility (TS 36.331 "
                          "uses 0.25..1.0; a value >1 lengthens TTT to resist small-cell churn).",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrUeRrc::m_mseSfMedium),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MseSfHigh",
                          "Time-to-trigger scale factor applied in High mobility.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrUeRrc::m_mseSfHigh),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute(
                "MseFixedScale",
                "If >0, apply this fixed time-to-trigger scale factor from t=0 (bypassing "
                "handover-count classification). Models a known-speed cohort: a fast UE "
                "gets the longer TTT immediately, so it does not chase a small cell's "
                "transient peak on its very first handover.",
                DoubleValue(0.0),
                MakeDoubleAccessor(&NrUeRrc::m_mseFixedScale),
                MakeDoubleChecker<double>(0.0))
            .AddAttribute(
                "MibWaitReselectTimeout",
                "Fallback for a force-camped UE that never decodes its target cell's MIB "
                "(e.g. the assigned cell is far and its broadcast is drowned by a closer "
                "co-channel neighbour, leaving the UE stuck in IDLE_WAIT_MIB forever). When "
                "non-zero, if no MIB arrives within this time the UE reselects the strongest "
                "cell it has measured and re-camps, mimicking idle-mode cell reselection. "
                "0 ms disables the fallback (legacy behaviour: wait indefinitely for the "
                "assigned cell).",
                TimeValue(MilliSeconds(0)),
                MakeTimeAccessor(&NrUeRrc::m_mibWaitReselectTimeout),
                MakeTimeChecker())
            .AddAttribute(
                "MibWaitReselectMaxAttempts",
                "Maximum number of cells a UE tries when MibWaitReselectTimeout is enabled, "
                "before giving up initial cell acquisition.",
                UintegerValue(8),
                MakeUintegerAccessor(&NrUeRrc::m_mibWaitReselectMaxAttempts),
                MakeUintegerChecker<uint32_t>(1))
            .AddTraceSource("MibReceived",
                            "trace fired upon reception of Master Information Block",
                            MakeTraceSourceAccessor(&NrUeRrc::m_mibReceivedTrace),
                            "ns3::NrUeRrc::MibSibHandoverTracedCallback")
            .AddTraceSource("Sib1Received",
                            "trace fired upon reception of System Information Block Type 1",
                            MakeTraceSourceAccessor(&NrUeRrc::m_sib1ReceivedTrace),
                            "ns3::NrUeRrc::MibSibHandoverTracedCallback")
            .AddTraceSource("Sib2Received",
                            "trace fired upon reception of System Information Block Type 2",
                            MakeTraceSourceAccessor(&NrUeRrc::m_sib2ReceivedTrace),
                            "ns3::NrUeRrc::ImsiCidRntiTracedCallback")
            .AddTraceSource("StateTransition",
                            "trace fired upon every UE RRC state transition",
                            MakeTraceSourceAccessor(&NrUeRrc::m_stateTransitionTrace),
                            "ns3::NrUeRrc::StateTracedCallback")
            .AddTraceSource("InitialCellSelectionEndOk",
                            "trace fired upon successful initial cell selection procedure",
                            MakeTraceSourceAccessor(&NrUeRrc::m_initialCellSelectionEndOkTrace),
                            "ns3::NrUeRrc::CellSelectionTracedCallback")
            .AddTraceSource("InitialCellSelectionEndError",
                            "trace fired upon failed initial cell selection procedure",
                            MakeTraceSourceAccessor(&NrUeRrc::m_initialCellSelectionEndErrorTrace),
                            "ns3::NrUeRrc::CellSelectionTracedCallback")
            .AddTraceSource("RandomAccessSuccessful",
                            "trace fired upon successful completion of the random access procedure",
                            MakeTraceSourceAccessor(&NrUeRrc::m_randomAccessSuccessfulTrace),
                            "ns3::NrUeRrc::ImsiCidRntiTracedCallback")
            .AddTraceSource("RandomAccessError",
                            "trace fired upon failure of the random access procedure",
                            MakeTraceSourceAccessor(&NrUeRrc::m_randomAccessErrorTrace),
                            "ns3::NrUeRrc::ImsiCidRntiTracedCallback")
            .AddTraceSource("ConnectionEstablished",
                            "trace fired upon successful RRC connection establishment",
                            MakeTraceSourceAccessor(&NrUeRrc::m_connectionEstablishedTrace),
                            "ns3::NrUeRrc::ImsiCidRntiTracedCallback")
            .AddTraceSource("ConnectionTimeout",
                            "trace fired upon timeout RRC connection establishment because of T300",
                            MakeTraceSourceAccessor(&NrUeRrc::m_connectionTimeoutTrace),
                            "ns3::NrUeRrc::ImsiCidRntiCountTracedCallback")
            .AddTraceSource("ConnectionReconfiguration",
                            "trace fired upon RRC connection reconfiguration",
                            MakeTraceSourceAccessor(&NrUeRrc::m_connectionReconfigurationTrace),
                            "ns3::NrUeRrc::ImsiCidRntiTracedCallback")
            .AddTraceSource("HandoverStart",
                            "trace fired upon start of a handover procedure",
                            MakeTraceSourceAccessor(&NrUeRrc::m_handoverStartTrace),
                            "ns3::NrUeRrc::MibSibHandoverTracedCallback")
            .AddTraceSource("HandoverEndOk",
                            "trace fired upon successful termination of a handover procedure",
                            MakeTraceSourceAccessor(&NrUeRrc::m_handoverEndOkTrace),
                            "ns3::NrUeRrc::ImsiCidRntiTracedCallback")
            .AddTraceSource("HandoverEndError",
                            "trace fired upon failure of a handover procedure",
                            MakeTraceSourceAccessor(&NrUeRrc::m_handoverEndErrorTrace),
                            "ns3::NrUeRrc::ImsiCidRntiTracedCallback")
            .AddTraceSource("SCarrierConfigured",
                            "trace fired after configuring secondary carriers",
                            MakeTraceSourceAccessor(&NrUeRrc::m_sCarrierConfiguredTrace),
                            "ns3::NrUeRrc::SCarrierConfiguredTracedCallback")
            .AddTraceSource("Srb1Created",
                            "trace fired after SRB1 is created",
                            MakeTraceSourceAccessor(&NrUeRrc::m_srb1CreatedTrace),
                            "ns3::NrUeRrc::ImsiCidRntiTracedCallback")
            .AddTraceSource("DrbCreated",
                            "trace fired after DRB is created",
                            MakeTraceSourceAccessor(&NrUeRrc::m_drbCreatedTrace),
                            "ns3::NrUeRrc::ImsiCidRntiLcIdTracedCallback")
            .AddTraceSource("RadioLinkFailure",
                            "trace fired upon failure of radio link",
                            MakeTraceSourceAccessor(&NrUeRrc::m_radioLinkFailureTrace),
                            "ns3::NrUeRrc::ImsiCidRntiTracedCallback")
            .AddTraceSource("RadioLinkFailureCause",
                            "trace fired when the UE enters CONNECTED_PHY_PROBLEM, carrying the "
                            "failure cause and timing context",
                            MakeTraceSourceAccessor(&NrUeRrc::m_radioLinkFailureCauseTrace),
                            "ns3::NrUeRrc::RlfCauseTracedCallback")
            .AddTraceSource(
                "PhySyncDetection",
                "trace fired upon receiving in Sync or out of Sync indications from UE PHY",
                MakeTraceSourceAccessor(&NrUeRrc::m_phySyncDetectionTrace),
                "ns3::NrUeRrc::PhySyncDetectionTracedCallback");
    return tid;
}

void
NrUeRrc::SetNrUeCphySapProvider(NrUeCphySapProvider* s)
{
    NS_LOG_FUNCTION(this << s);
    m_cphySapProvider.at(GetPrimaryDlIndex()) = s;
}

void
NrUeRrc::SetNrUeCphySapProvider(NrUeCphySapProvider* s, uint8_t index)
{
    NS_LOG_FUNCTION(this << s);
    m_cphySapProvider.at(index) = s;
}

NrUeCphySapUser*
NrUeRrc::GetNrUeCphySapUser()
{
    NS_LOG_FUNCTION(this);
    return m_cphySapUser.at(GetPrimaryDlIndex());
}

NrUeCphySapUser*
NrUeRrc::GetNrUeCphySapUser(uint8_t index)
{
    NS_LOG_FUNCTION(this);
    return m_cphySapUser.at(index);
}

void
NrUeRrc::SetNrUeCmacSapProvider(NrUeCmacSapProvider* s)
{
    NS_LOG_FUNCTION(this << s);
    m_cmacSapProvider.at(GetPrimaryDlIndex()) = s;
}

void
NrUeRrc::SetNrUeCmacSapProvider(NrUeCmacSapProvider* s, uint8_t index)
{
    NS_LOG_FUNCTION(this << s);
    m_cmacSapProvider.at(index) = s;
}

NrUeCmacSapUser*
NrUeRrc::GetNrUeCmacSapUser()
{
    NS_LOG_FUNCTION(this);
    return m_cmacSapUser.at(GetPrimaryDlIndex());
}

NrUeCmacSapUser*
NrUeRrc::GetNrUeCmacSapUser(uint8_t index)
{
    NS_LOG_FUNCTION(this);
    return m_cmacSapUser.at(index);
}

void
NrUeRrc::SetNrUeRrcSapUser(NrUeRrcSapUser* s)
{
    NS_LOG_FUNCTION(this << s);
    m_rrcSapUser = s;
}

NrUeRrcSapProvider*
NrUeRrc::GetNrUeRrcSapProvider()
{
    NS_LOG_FUNCTION(this);
    return m_rrcSapProvider;
}

NrUeRrcSapUser*
NrUeRrc::GetNrUeRrcSapUser()
{
    NS_LOG_FUNCTION(this);
    return m_rrcSapUser;
}

void
NrUeRrc::SetNrMacSapProvider(NrMacSapProvider* s)
{
    NS_LOG_FUNCTION(this << s);
    m_macSapProvider = s;
}

void
NrUeRrc::SetNrCcmRrcSapProvider(NrUeCcmRrcSapProvider* s)
{
    NS_LOG_FUNCTION(this << s);
    m_ccmRrcSapProvider = s;
}

NrUeCcmRrcSapUser*
NrUeRrc::GetNrCcmRrcSapUser()
{
    NS_LOG_FUNCTION(this);
    return m_ccmRrcSapUser;
}

void
NrUeRrc::SetAsSapUser(NrAsSapUser* s)
{
    m_asSapUser = s;
}

NrAsSapProvider*
NrUeRrc::GetAsSapProvider()
{
    return m_asSapProvider;
}

void
NrUeRrc::SetImsi(uint64_t imsi)
{
    NS_LOG_FUNCTION(this << imsi);
    m_imsi = imsi;

    // Communicate the IMSI to MACs and PHYs for all the component carriers
    for (uint16_t i = 0; i < m_numberOfComponentCarriers; i++)
    {
        m_cmacSapProvider.at(i)->SetImsi(m_imsi);
        m_cphySapProvider.at(i)->SetImsi(m_imsi);
    }
}

void
NrUeRrc::StorePreviousCellId(uint16_t cellId)
{
    NS_LOG_FUNCTION(this << cellId);
    m_previousCellId = cellId;
}

uint64_t
NrUeRrc::GetImsi() const
{
    return m_imsi;
}

uint16_t
NrUeRrc::GetRnti() const
{
    NS_LOG_FUNCTION(this);
    return m_rnti;
}

uint16_t
NrUeRrc::GetCellId() const
{
    NS_LOG_FUNCTION(this);
    return m_cellId;
}

bool
NrUeRrc::IsServingCell(uint16_t cellId) const
{
    NS_LOG_FUNCTION(this);
    for (auto& cphySap : m_cphySapProvider)
    {
        if (cellId == cphySap->GetCellId())
        {
            return true;
        }
    }
    return false;
}

uint8_t
NrUeRrc::GetUlBandwidth() const
{
    NS_LOG_FUNCTION(this);
    return m_ulBandwidth;
}

uint8_t
NrUeRrc::GetDlBandwidth() const
{
    NS_LOG_FUNCTION(this);
    return m_dlBandwidth;
}

uint32_t
NrUeRrc::GetArfcn() const
{
    return m_initDlArfcn;
}

NrUeRrc::State
NrUeRrc::GetState() const
{
    NS_LOG_FUNCTION(this);
    return m_state;
}

uint16_t
NrUeRrc::GetPreviousCellId() const
{
    NS_LOG_FUNCTION(this);
    return m_previousCellId;
}

void
NrUeRrc::SetUseRlcSm(bool val)
{
    NS_LOG_FUNCTION(this);
    m_useRlcSm = val;
}

void
NrUeRrc::SetUseRrcReestablishment(bool val)
{
    NS_LOG_FUNCTION(this);
    m_useRrcReestablishment = val;
}

void
NrUeRrc::SetPrimaryUlIndex(uint16_t ulIndex)
{
    NS_LOG_FUNCTION(this << +ulIndex);
    m_primaryUlIndex = ulIndex;
    SyncBwpOutputLinks();
}

uint16_t
NrUeRrc::GetPrimaryUlIndex() const
{
    return m_primaryUlIndex;
}

void
NrUeRrc::SetUpdateBwpOutputLinkFn(std::function<void(uint32_t, uint32_t)> fn)
{
    m_updateBwpOutputLinkFn = std::move(fn);
    SyncBwpOutputLinks();
}

void
NrUeRrc::SyncBwpOutputLinks()
{
    if (!m_updateBwpOutputLinkFn)
    {
        return;
    }
    // Outgoing messages sourced on the primary DL BWP leave through the primary
    // UL BWP, which itself routes to itself; a DL-only primary BWP leaves through
    // the UL carrier the serving cell paired it with in dedicated RRC config.
    // Applying this on every primary index change overwrites links belonging to
    // a previous serving cell whose UL/DL carrier pairing differed (e.g. a
    // handover between FDD cells with inverted carrier roles).
    const auto pairIt = m_rrcBwpPairings.find(GetPrimaryDlIndex());
    const uint32_t dlOutput =
        pairIt != m_rrcBwpPairings.end() ? pairIt->second : GetPrimaryUlIndex();
    m_updateBwpOutputLinkFn(GetPrimaryDlIndex(), dlOutput);
    m_updateBwpOutputLinkFn(GetPrimaryUlIndex(), GetPrimaryUlIndex());
}

void
NrUeRrc::SetPrimaryDlIndex(uint16_t dlIndex)
{
    NS_LOG_FUNCTION(this << +dlIndex);
    m_primaryDlIndex = dlIndex;
    SyncBwpOutputLinks();
}

uint16_t
NrUeRrc::GetPrimaryDlIndex() const
{
    return m_primaryDlIndex;
}

void
NrUeRrc::SetCellIndividualOffset(uint16_t cellId, double offsetDb)
{
    NS_LOG_FUNCTION(this << cellId << offsetDb);
    if (offsetDb == 0.0)
    {
        m_cellIndividualOffsetDb.erase(cellId);
    }
    else
    {
        m_cellIndividualOffsetDb[cellId] = offsetDb;
    }
}

double
NrUeRrc::GetCellIndividualOffset(uint16_t cellId) const
{
    auto it = m_cellIndividualOffsetDb.find(cellId);
    return (it != m_cellIndividualOffsetDb.end()) ? it->second : 0.0;
}

void
NrUeRrc::InitializeSrb0()
{
    NS_LOG_FUNCTION(this);

    // setup the UE side of SRB0
    uint8_t lcid = 0;

    Ptr<NrRlc> rlc = CreateObject<NrRlcTm>()->GetObject<NrRlc>();
    rlc->SetNrMacSapProvider(m_macSapProvider);
    rlc->SetRnti(m_rnti);
    rlc->SetLcId(lcid);

    m_srb0 = CreateObject<NrSignalingRadioBearerInfo>();
    m_srb0->m_rlc = rlc;
    m_srb0->m_srbIdentity = 0;
    NrUeRrcSapUser::SetupParameters ueParams;
    ueParams.srb0SapProvider = m_srb0->m_rlc->GetNrRlcSapProvider();
    ueParams.srb1SapProvider = nullptr;
    m_rrcSapUser->Setup(ueParams);

    // CCCH (LCID 0) is pre-configured, here is the hardcoded configuration:
    NrUeCmacSapProvider::LogicalChannelConfig lcConfig{};
    lcConfig.priority = 0;                   // highest priority
    lcConfig.prioritizedBitRateKbps = 65535; // maximum
    lcConfig.bucketSizeDurationMs = 65535;   // maximum
    lcConfig.logicalChannelGroup = 0;        // all SRBs mapped to LCG 0
    // Arbitrary 5QI to route UE RRC UL messages through BWP manager
    lcConfig.fiveQi = NrQosFlow::GBR_CONV_VOICE;
    NrMacSapUser* msu =
        m_ccmRrcSapProvider->ConfigureSignalBearer(lcid, lcConfig, rlc->GetNrMacSapUser());
    for (auto& mac : m_cmacSapProvider)
    {
        mac->AddLc(lcid, lcConfig, msu);
    }
}

void
NrUeRrc::InitializeSap()
{
    NS_LOG_FUNCTION(this);
    if (m_numberOfComponentCarriers < nr::MIN_NO_CC || m_numberOfComponentCarriers > nr::MAX_NO_CC)
    {
        // this check is needed in order to maintain backward compatibility with scripts and tests
        // if case lte-helper is not used (like in several tests) the m_numberOfComponentCarriers
        // is not set and then an error is raised
        // In this case m_numberOfComponentCarriers is set to 1
        m_numberOfComponentCarriers = nr::MIN_NO_CC;
    }
    if (m_numberOfComponentCarriers > nr::MIN_NO_CC)
    {
        for (uint16_t i = 1; i < m_numberOfComponentCarriers; i++)
        {
            m_cphySapUser.push_back(new MemberNrUeCphySapUser<NrUeRrc>(this));
            m_cmacSapUser.push_back(new UeMemberNrUeCmacSapUser(this));
            m_cphySapProvider.push_back(nullptr);
            m_cmacSapProvider.push_back(nullptr);
        }
    }
}

void
NrUeRrc::DoSendData(Ptr<Packet> packet, uint8_t qfi)
{
    NS_LOG_FUNCTION(this << packet << qfi);

    uint8_t drbid = Qfi2Drbid(qfi);

    if (drbid != 0)
    {
        auto it = m_drbMap.find(drbid);
        NS_ASSERT_MSG(it != m_drbMap.end(),
                      "could not find bearer with drbid == " << +drbid << " for QFI " << +qfi);

        NrPdcpSapProvider::TransmitPdcpSduParameters params;
        params.pdcpSdu = packet;
        params.rnti = m_rnti;
        params.lcid = it->second->m_logicalChannelIdentity;

        NS_LOG_LOGIC(this << " RNTI=" << m_rnti << " sending packet " << packet << " on DRBID "
                          << (uint32_t)drbid << " (LCID " << (uint32_t)params.lcid << ")"
                          << " (" << packet->GetSize() << " bytes)");
        it->second->m_pdcp->GetNrPdcpSapProvider()->TransmitPdcpSdu(params);
    }
}

void
NrUeRrc::DoDisconnect()
{
    NS_LOG_FUNCTION(this);

    switch (m_state)
    {
    case IDLE_START:
    case IDLE_CELL_SEARCH:
    case IDLE_WAIT_MIB_SIB1:
    case IDLE_WAIT_MIB:
    case IDLE_WAIT_SIB1:
    case IDLE_CAMPED_NORMALLY:
        NS_LOG_INFO("already disconnected");
        break;

    case IDLE_WAIT_SIB2:
    case IDLE_CONNECTING:
        NS_FATAL_ERROR("cannot abort connection setup procedure");
        break;

    case CONNECTED_NORMALLY:
    case CONNECTED_HANDOVER:
    case CONNECTED_PHY_PROBLEM:
    case CONNECTED_REESTABLISHING:
        LeaveConnectedMode();
        break;

    default: // i.e. IDLE_RANDOM_ACCESS
        NS_FATAL_ERROR("method unexpected in state " << ToString(m_state));
        break;
    }
}

void
NrUeRrc::DoReceivePdcpSdu(NrPdcpSapUser::ReceivePdcpSduParameters params)
{
    NS_LOG_FUNCTION(this);
    m_asSapUser->RecvData(params.pdcpSdu, nr::Lcid2Qfi(params.lcid));
}

void
NrUeRrc::DoSetTemporaryCellRnti(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << "RNTI " << rnti << ", primary UL " << GetPrimaryUlIndex()
                         << ", primary DL " << GetPrimaryDlIndex());
    m_rnti = rnti;
    m_srb0->m_rlc->SetRnti(m_rnti);
    m_cphySapProvider.at(GetPrimaryUlIndex())->SetRnti(m_rnti);
    m_cphySapProvider.at(GetPrimaryDlIndex())->SetRnti(m_rnti);
    m_cmacSapProvider.at(GetPrimaryUlIndex())->SetRnti(m_rnti);
    m_cmacSapProvider.at(GetPrimaryDlIndex())->SetRnti(m_rnti);
}

void
NrUeRrc::DoNotifyRandomAccessSuccessful()
{
    NS_LOG_FUNCTION(this << m_imsi << ToString(m_state) << " cellId " << m_cellId << " rnti "
                         << m_rnti);
    m_randomAccessSuccessfulTrace(m_imsi, m_cellId, m_rnti);
    m_rachAttempts = 0;
    ClearRachLock();
    switch (m_state)
    {
    case IDLE_RANDOM_ACCESS: {
        // we just received a RAR with a T-C-RNTI and an UL grant
        // send RRC connection request as message 3 of the random access procedure
        SwitchToState(IDLE_CONNECTING);
        NrRrcSap::RrcConnectionRequest msg{};
        msg.ueIdentity = m_imsi;
        m_rrcSapUser->SendRrcConnectionRequest(msg);
        m_connectionTimeout = Simulator::Schedule(m_t300, &NrUeRrc::ConnectionTimeout, this);
    }
    break;

    case CONNECTED_HANDOVER: {
        NrRrcSap::RrcConnectionReconfigurationCompleted msg{};
        msg.rrcTransactionIdentifier = m_lastRrcTransactionIdentifier;
        m_rrcSapUser->SendRrcConnectionReconfigurationCompleted(msg);

        // 3GPP TS 36.331 section 5.5.6.1 Measurements related actions upon handover
        for (auto measIdIt = m_varMeasConfig.measIdList.begin();
             measIdIt != m_varMeasConfig.measIdList.end();
             ++measIdIt)
        {
            VarMeasReportListClear(measIdIt->second.measId);
        }

        SwitchToState(CONNECTED_NORMALLY);
        NotifyConnectionSuccessfulAllBwps(); // RA successful during handover
        m_handoverEndOkTrace(m_imsi, m_cellId, m_rnti);
        m_lastHoSuccessTime = Simulator::Now();
        if (m_mseEnable)
        {
            m_mseHandoverTimes.push_back(Simulator::Now());
            UpdateMobilityState();
        }
    }
    break;

    case CONNECTED_NORMALLY: {
        // Random access started while already connected, to recover uplink resources after the
        // scheduling request procedure gave up (3GPP TS 38.321, clause 5.4.4). It only restores
        // the uplink grant path, so the connection carries on with nothing to signal.
        NS_LOG_INFO("Random access successful while connected, uplink access recovered");
    }
    break;

    default:
        NS_FATAL_ERROR("unexpected event in state " << ToString(m_state));
        break;
    }
}

void
NrUeRrc::DoNotifyRandomAccessFailed()
{
    NS_LOG_FUNCTION(this << m_imsi << ToString(m_state) << " cellId " << m_cellId << " rnti "
                         << m_rnti);
    m_randomAccessErrorTrace(m_imsi, m_cellId, m_rnti);
    ClearRachLock();
    switch (m_state)
    {
    case IDLE_RANDOM_ACCESS: {
        m_rachAttempts++;
        if (m_rachAttempts < m_rachAttemptsLimit)
        {
            SwitchToState(IDLE_CAMPED_NORMALLY);
            m_asSapUser->NotifyConnectionFailed();
        }
        else
        {
            m_rachAttempts = 0;
            m_hasReceivedSib1 = false;
            m_hasReceivedSib2 = false;
            SwitchToState(IDLE_CELL_SEARCH);
            SynchronizeToStrongestCell();
        }
    }
    break;

    case CONNECTED_HANDOVER: {
        m_handoverEndErrorTrace(m_imsi, m_cellId, m_rnti);
        /**
         * @todo After a handover failure because of a random access failure,
         *       send an RRC Connection Re-establishment and switch to
         *       CONNECTED_REESTABLISHING state.
         */
        if (!m_leaveConnectedMode)
        {
            m_leaveConnectedMode = true;
            NS_LOG_DEBUG("Switch to CONNECTED_PHY_PROBLEM. Reason: Handover ongoing for IMSI: "
                         << m_imsi << " rnti: " << m_rnti << " cellId: " << m_cellId
                         << " in state: " << ToString(m_state) << ".");
            EnterPhyProblemState(RLF_DURING_HANDOVER);
            m_rrcSapUser->SendIdealUeContextRemoveRequest(m_rnti);
            // we should have called NotifyConnectionFailed
            // but that method would immediately ask you UE to
            // connect rather than doing cell selection again.
            m_asSapUser->NotifyConnectionReleased();
        }
    }
    break;

    case CONNECTED_NORMALLY: {
        // Random access started while already connected did not recover the uplink. The
        // connection is left alone: the MAC keeps requesting resources, and a persistent loss of
        // the uplink is detected by the radio link failure procedure.
        NS_LOG_WARN("Random access failed while connected, uplink access not recovered");
    }
    break;

    default:
        NS_FATAL_ERROR("unexpected event in state " << ToString(m_state));
        break;
    }
}

void
NrUeRrc::DoSetCsgWhiteList(uint32_t csgId)
{
    NS_LOG_FUNCTION(this << m_imsi << csgId);
    m_csgWhiteList = csgId;
}

void
NrUeRrc::DoStartCellSelection(uint32_t arfcn)
{
    NS_LOG_FUNCTION(this << m_imsi << arfcn);
    NS_ASSERT_MSG(m_state == IDLE_START,
                  "cannot start cell selection from state " << ToString(m_state));
    m_initDlArfcn = arfcn;
    m_cphySapProvider.at(GetPrimaryDlIndex())->SetNumerology(0);
    m_cphySapProvider.at(GetPrimaryDlIndex())->StartCellSearch(arfcn);
    SwitchToState(IDLE_CELL_SEARCH);
}

void
NrUeRrc::DoStartCellSelection()
{
    NS_LOG_FUNCTION(this << m_imsi);
    for (auto& phy : m_cphySapProvider)
    {
        phy->SetNumerology(0);
        phy->StartCellSearch(phy->GetArfcn());
    }
    SwitchToState(IDLE_CELL_SEARCH);
}

void
NrUeRrc::CampOnGnb(uint16_t cellId, uint32_t arfcn)
{
    m_cellId = cellId;
    m_initDlArfcn = arfcn;
    TrackCellArfcn(cellId, arfcn);
    auto bwpId = GetArfcnBwpId(arfcn);
    SetPrimaryDlIndex(bwpId);
    m_cphySapProvider.at(bwpId)->SetNumerology(0);
    m_cphySapProvider.at(bwpId)->SynchronizeWithGnb(m_cellId, m_initDlArfcn);
    m_cmacSapProvider.at(GetPrimaryUlIndex())->RegisterToGnb(m_cellId);
    if (GetPrimaryDlIndex() != GetPrimaryUlIndex())
    {
        m_cmacSapProvider.at(GetPrimaryDlIndex())->RegisterToGnb(m_cellId);
    }
}

void
NrUeRrc::DoForceCampedOnGnb(uint16_t cellId, uint32_t arfcn)
{
    NS_LOG_FUNCTION(this << m_imsi << " ,cellId " << cellId << ",arfcn " << arfcn);

    switch (m_state)
    {
    case IDLE_CELL_SEARCH:
    case IDLE_WAIT_MIB_SIB1:
    case IDLE_WAIT_SIB1:
        // An explicit force-camp request (Connect with a target cell/ARFCN) is a
        // deterministic override: it aborts an in-progress autonomous cell
        // selection and retunes directly to the requested cell. Fall through to
        // the camp logic below.
        NS_LOG_INFO("force-camp overrides in-progress cell selection " << ToString(m_state));
        [[fallthrough]];
    case IDLE_START: {
        CampOnGnb(cellId, arfcn);
        SwitchToState(IDLE_WAIT_MIB);
        // Start a fresh reselection sequence: if this cell's MIB never arrives,
        // MibWaitReselect() re-camps on the strongest measured neighbour.
        m_mibWaitAttempts = 0;
        m_mibCampTried.clear();
        ArmMibWaitReselect();
    }
    break;

    case IDLE_WAIT_MIB:
        NS_LOG_INFO("already forced to camp to cell " << m_cellId);
        break;

    case IDLE_CAMPED_NORMALLY:
    case IDLE_WAIT_SIB2:
    case IDLE_RANDOM_ACCESS:
    case IDLE_CONNECTING:
        NS_LOG_INFO("already camped to cell " << m_cellId);
        break;

    case CONNECTED_NORMALLY:
    case CONNECTED_HANDOVER:
    case CONNECTED_PHY_PROBLEM:
    case CONNECTED_REESTABLISHING:
        NS_LOG_INFO("already connected to cell " << m_cellId);
        break;

    default:
        NS_FATAL_ERROR("unexpected event in state " << ToString(m_state));
        break;
    }
}

void
NrUeRrc::DoConnect()
{
    NS_LOG_FUNCTION(this << "IMSI " << m_imsi);

    switch (m_state)
    {
    case IDLE_START:
    case IDLE_CELL_SEARCH:
    case IDLE_WAIT_MIB_SIB1:
    case IDLE_WAIT_SIB1:
    case IDLE_WAIT_MIB:
        m_connectionPending = true;
        break;

    case IDLE_CAMPED_NORMALLY:
        m_connectionPending = true;
        SwitchToState(IDLE_WAIT_SIB2);
        break;

    case IDLE_WAIT_SIB2:
    case IDLE_RANDOM_ACCESS:
    case IDLE_CONNECTING:
        NS_LOG_INFO("already connecting");
        break;

    case CONNECTED_NORMALLY:
    case CONNECTED_REESTABLISHING:
    case CONNECTED_HANDOVER:
        NS_LOG_INFO("already connected");
        break;

    default:
        NS_FATAL_ERROR("unexpected event in state " << ToString(m_state));
        break;
    }
}

// CPHY SAP methods

void
NrUeRrc::DoRecvMasterInformationBlock(uint16_t cellId,
                                      uint32_t arfcn,
                                      NrRrcSap::MasterInformationBlock msg)
{
    NS_LOG_FUNCTION(this << cellId << arfcn << m_cellId << +GetPrimaryDlIndex());

    // Per-carrier / per-BWP MIB routing.
    //
    // The UE keeps MULTIPLE BWPs tuned (one per carrier) so it can receive SSB
    // and measure RSRP on neighbour frequencies (and hand over between them).
    // A MIB is a per-CARRIER broadcast: it carries the numerology/bandwidth of
    // the carrier it was received on. Configure the BWP actually tuned to that
    // carrier, resolved by its ARFCN -- never "the primary serving" BWP.
    //
    // Routing by the primary index is wrong once a same-cell BWP switch has
    // moved the primary onto a carrier with a DIFFERENT numerology: applying
    // this MIB's (foreign) numerology to it desyncs that BWP's PHY slot timeline
    // and trips the "Cannot TX while RX" fatal in NrSpectrumPhy. The BWP that
    // received this MIB is, by construction, on the MIB's carrier, so
    // configuring it from the MIB is always self-consistent.
    //
    // The MIB subcarrier spacing is the one of SIB1, Msg2/4, paging and broadcast SI, used to
    // acquire a cell (TS 38.331, MIB). The numerology of the BWP a connected UE is served on
    // changes only with its configuration: RRC (re)configuration, handover or BWP switch.
    // Reconfiguring that BWP from a MIB would also abort the data reception in progress.
    const std::size_t mibBwp = GetArfcnBwpId(arfcn);
    const bool connected = m_state == CONNECTED_NORMALLY || m_state == CONNECTED_HANDOVER ||
                           m_state == CONNECTED_PHY_PROBLEM;
    const bool servingBwp = mibBwp == GetPrimaryDlIndex() || mibBwp == GetPrimaryUlIndex();
    if (!connected || !servingBwp)
    {
        m_cphySapProvider.at(mibBwp)->SetDlBandwidth(msg.dlBandwidth);
        m_cphySapProvider.at(mibBwp)->SetNumerology(msg.numerology);
    }

    // Update the SERVING DL bandwidth bookkeeping only when this MIB configures
    // the primary serving BWP (serving cell, primary carrier). Before commitment
    // (m_cellId == 0, initial cell selection / manual attach) the MIB is from the
    // cell we are synchronizing to; a connected UE's neighbour-measurement MIB
    // keeps its own BWP decodable for RSRP but must never disturb serving state.
    const bool isServingCell = (cellId == m_cellId) || (m_cellId == 0);
    if (isServingCell && mibBwp == GetPrimaryDlIndex())
    {
        m_dlBandwidth = msg.dlBandwidth;
    }

    m_hasReceivedMib = true;
    m_mibReceivedTrace(m_imsi, m_cellId, m_rnti, cellId);

    switch (m_state)
    {
    case IDLE_WAIT_MIB:
        // manual attachment
        m_mibWaitTimeoutEvent.Cancel(); // the camped cell answered; stop reselection
        SwitchToState(IDLE_CAMPED_NORMALLY);
        break;

    case IDLE_WAIT_MIB_SIB1:
        // automatic attachment from Idle mode cell selection
        SwitchToState(IDLE_WAIT_SIB1);
        break;

    default:
        // do nothing extra
        break;
    }
}

void
NrUeRrc::DoRecvSystemInformationBlockType1(uint16_t cellId,
                                           uint32_t arfcn,
                                           NrRrcSap::SystemInformationBlockType1 msg)
{
    NS_LOG_FUNCTION(this << cellId << arfcn << msg.servingCellConfigCommon.numerology);

    // Guard 1 – serving-cell filter.
    // While connected/connecting, reject SIB1 from any cell that is not the
    // current serving cell; it would otherwise re-trigger cell selection.
    if (m_state == CONNECTED_NORMALLY || m_state == IDLE_CONNECTING)
    {
        if (cellId != m_cellId)
        {
            NS_LOG_INFO("NrUeRrc: Discarding SIB1 from non-serving cell="
                        << cellId << " (serving=" << m_cellId << " state=" << ToString(m_state)
                        << " bwp=" << GetPrimaryDlIndex() << ")");
            return;
        }
    }

    // Guard 2 – multi-BWP RACH lock.
    // If a RACH is already in-flight on a different BWP, discard until the
    // deadline expires or the procedure completes (see ClearRachLock()).
    if (m_rachInProgress && Simulator::Now() < m_rachDeadline)
    {
        if (GetPrimaryDlIndex() != m_rachBwpId)
        {
            NS_LOG_INFO("Discarding SIB1 — RACH in-flight on bwp="
                        << +m_rachBwpId << " deadline=" << m_rachDeadline.As(Time::MS)
                        << " ignored bwp=" << GetPrimaryDlIndex() << " cellId=" << cellId);
            return;
        }
    }
    switch (m_state)
    {
    case IDLE_WAIT_SIB1:
        NS_ASSERT_MSG(cellId == msg.cellAccessRelatedInfo.cellIdentity,
                      "Cell identity in SIB1 does not match with the originating cell");
        m_hasReceivedSib1 = true;
        m_lastSib1 = msg;
        m_sib1ReceivedTrace(m_imsi, m_cellId, m_rnti, cellId);
        EvaluateCellForSelection();
        break;

    case IDLE_CAMPED_NORMALLY:
    case IDLE_RANDOM_ACCESS:
    case IDLE_CONNECTING:
    case CONNECTED_NORMALLY:
    case CONNECTED_HANDOVER:
    case CONNECTED_PHY_PROBLEM:
    case CONNECTED_REESTABLISHING:
        NS_ASSERT_MSG(cellId == msg.cellAccessRelatedInfo.cellIdentity,
                      "Cell identity in SIB1 does not match with the originating cell");
        m_hasReceivedSib1 = true;
        m_lastSib1 = msg;
        m_sib1ReceivedTrace(m_imsi, m_cellId, m_rnti, cellId);
        break;

    case IDLE_WAIT_MIB_SIB1:
        // MIB has not been received, so ignore this SIB1

    default: // e.g. IDLE_START, IDLE_CELL_SEARCH, IDLE_WAIT_MIB, IDLE_WAIT_SIB2
        // do nothing
        break;
    }
}

void
NrUeRrc::DoReportUeMeasurements(NrUeCphySapUser::UeMeasurementsParameters params)
{
    NS_LOG_FUNCTION(this);

    // layer 3 filtering does not apply in IDLE mode
    bool useLayer3Filtering = (m_state == CONNECTED_NORMALLY);
    bool triggering = true;
    for (auto newMeasIt = params.m_ueMeasurementsList.begin();
         newMeasIt != params.m_ueMeasurementsList.end();
         ++newMeasIt)
    {
        if (params.m_componentCarrierId != 0)
        {
            triggering = false; // report is triggered only when an event is on the primary carrier
            // in this case the measurement received is related to secondary carriers
        }
        SaveUeMeasurements(newMeasIt->m_cellId,
                           newMeasIt->m_rsrp,
                           newMeasIt->m_rsrq,
                           useLayer3Filtering,
                           params.m_componentCarrierId);
    }

    if (m_state == IDLE_CELL_SEARCH)
    {
        // start decoding BCH
        SynchronizeToStrongestCell();
    }
    else
    {
        if (triggering)
        {
            for (auto measIdIt = m_varMeasConfig.measIdList.begin();
                 measIdIt != m_varMeasConfig.measIdList.end();
                 ++measIdIt)
            {
                MeasurementReportTriggering(measIdIt->first);
            }
        }
    }

} // end of NrUeRrc::DoReportUeMeasurements

// RRC SAP methods

void
NrUeRrc::DoCompleteSetup(NrUeRrcSapProvider::CompleteSetupParameters params)
{
    NS_LOG_FUNCTION(this << " RNTI " << m_rnti);
    m_srb0->m_rlc->SetNrRlcSapUser(params.srb0SapUser);
    if (m_srb1)
    {
        m_srb1->m_pdcp->SetNrPdcpSapUser(params.srb1SapUser);
    }
}

void
NrUeRrc::DoRecvSystemInformation(NrRrcSap::SystemInformation msg)
{
    NS_LOG_FUNCTION(this << " RNTI " << m_rnti << ", primary UL " << GetPrimaryUlIndex()
                         << ", primary DL " << GetPrimaryDlIndex());

    if (msg.haveSib2)
    {
        switch (m_state)
        {
        case IDLE_CAMPED_NORMALLY:
        case IDLE_WAIT_SIB2:
        case IDLE_RANDOM_ACCESS:
        case IDLE_CONNECTING:
        case CONNECTED_NORMALLY:
        case CONNECTED_HANDOVER:
        case CONNECTED_PHY_PROBLEM:
        case CONNECTED_REESTABLISHING:
            m_hasReceivedSib2 = true;
            if (m_rachInProgress)
            {
                // A RACH is already in-flight; applying SIB2 RACH config now would
                // reset the MAC procedure mid-stream. Silently ignore — ClearRachLock()
                // will fire on success or failure and the config is still valid.
                NS_LOG_INFO(this << " IMSI " << m_imsi
                                 << " ignoring duplicate SIB2 while RACH in-flight"
                                    " on bwp "
                                 << (uint16_t)m_rachBwpId);
                break;
            }
            m_ulBandwidth = msg.sib2.freqInfo.ulBandwidth;
            m_initUlArfcn = msg.sib2.freqInfo.ulCarrierFreq;
            m_sib2ReceivedTrace(m_imsi, m_cellId, m_rnti);
            NrUeCmacSapProvider::RachConfig rc;
            rc.numberOfRaPreambles = msg.sib2.radioResourceConfigCommon.rachConfigCommon
                                         .preambleInfo.numberOfRaPreambles;
            rc.preambleTransMax = msg.sib2.radioResourceConfigCommon.rachConfigCommon
                                      .raSupervisionInfo.preambleTransMax;
            rc.raResponseWindowSize = msg.sib2.radioResourceConfigCommon.rachConfigCommon
                                          .raSupervisionInfo.raResponseWindowSize;
            rc.connEstFailCount =
                msg.sib2.radioResourceConfigCommon.rachConfigCommon.txFailParam.connEstFailCount;
            m_connEstFailCountLimit = rc.connEstFailCount;
            NS_ASSERT_MSG(m_connEstFailCountLimit > 0 && m_connEstFailCountLimit < 5,
                          "SIB2 msg contains wrong value " << m_connEstFailCountLimit
                                                           << "of connEstFailCount");
            m_cmacSapProvider.at(GetPrimaryUlIndex())->ConfigureRach(rc);
            m_cphySapProvider.at(GetPrimaryUlIndex())
                ->ConfigureUplink(m_initUlArfcn, m_ulBandwidth);
            m_cphySapProvider.at(GetPrimaryDlIndex())
                ->ConfigureUplink(m_initUlArfcn,
                                  m_ulBandwidth); // also needs to know that UL is configured, e.g.,
            // when FDD used it is necessary, otherwise CQI
            // will not be generated and routed to UL UE PHY
            m_cphySapProvider.at(GetPrimaryUlIndex())
                ->ConfigureReferenceSignalPower(
                    msg.sib2.radioResourceConfigCommon.pdschConfigCommon.referenceSignalPower);
            if (GetPrimaryUlIndex() != GetPrimaryDlIndex())
            {
                m_cphySapProvider.at(GetPrimaryUlIndex())->SetDlBandwidth(m_ulBandwidth);
            }
            if (m_state == IDLE_WAIT_SIB2)
            {
                NS_ASSERT(m_connectionPending);
                StartConnection();
            }
            break;

        default: // IDLE_START, IDLE_CELL_SEARCH, IDLE_WAIT_MIB, IDLE_WAIT_MIB_SIB1, IDLE_WAIT_SIB1
            // do nothing
            break;
        }
    }
}

void
NrUeRrc::DoRecvRrcConnectionSetup(NrRrcSap::RrcConnectionSetup msg)
{
    NS_LOG_FUNCTION(this << " RNTI " << m_rnti << ", primary UL " << GetPrimaryUlIndex()
                         << ", primary DL " << GetPrimaryDlIndex());
    switch (m_state)
    {
    case IDLE_CONNECTING: {
        ApplyRadioResourceConfigDedicated(msg.radioResourceConfigDedicated);
        m_connEstFailCount = 0;
        m_connectionTimeout.Cancel();
        SwitchToState(CONNECTED_NORMALLY);
        m_leaveConnectedMode = false;
        NrRrcSap::RrcConnectionSetupCompleted msg2;
        msg2.rrcTransactionIdentifier = msg.rrcTransactionIdentifier;
        m_rrcSapUser->SendRrcConnectionSetupCompleted(msg2);
        m_asSapUser->NotifyConnectionSuccessful();
        NotifyConnectionSuccessfulAllBwps();
        m_connectionEstablishedTrace(m_imsi, m_cellId, m_rnti);
        NS_ABORT_MSG_IF(m_noOfSyncIndications > 0,
                        "Sync indications should be zero "
                        "when a new RRC connection is established. Current value = "
                            << (uint16_t)m_noOfSyncIndications);
    }
    break;

    default:
        // Stale or duplicate RrcConnectionSetup: if the UE is already
        // CONNECTED_NORMALLY, drop the message and log a warning.
        // This can happen during handover when old SRB0 messages race
        // with the completed connection.
        if (m_state == CONNECTED_NORMALLY)
        {
            NS_LOG_WARN("RrcConnectionSetup received in CONNECTED_NORMALLY (stale/duplicate)");
            break;
        }
        NS_FATAL_ERROR("method unexpected in state " << ToString(m_state));
        break;
    }
}

void
NrUeRrc::DoRecvRrcConnectionReconfiguration(NrRrcSap::RrcConnectionReconfiguration msg)
{
    NS_LOG_FUNCTION(this << " RNTI " << m_rnti << ", cellId " << m_cellId);
    NS_LOG_INFO("DoRecvRrcConnectionReconfiguration haveNonCriticalExtension:"
                << msg.haveNonCriticalExtension);
    switch (m_state)
    {
    case CONNECTED_NORMALLY:
        if (msg.haveMobilityControlInfo)
        {
            NS_LOG_INFO("haveMobilityControlInfo == true");
            // TR 36.839 (5.3.2) handover-failure model. The handover command is
            // delivered over the *source* cell. If the source radio link is already
            // below Qout when the command arrives -- modelled here by the T310 timer
            // being active (N310 out-of-sync indications already accumulated) -- the
            // UE cannot reliably receive/act on the command, so the handover fails.
            // Otherwise the late command silently rescues the link (ResetRlfParams
            // below) and "too-late" handovers never fail, so the handover failure
            // rate does not grow with UE speed. Declaring RLF here, before the
            // HandoverStart trace, makes the event count as a too-late handover
            // failure and routes the UE through reestablishment. Off by default
            // (see the Tr36839HandoverFailure attribute).
            // Graded criterion: fail only if T310 has been running long enough that
            // the source is too degraded to receive the command. A command arriving
            // early in T310 (elapsed < Tr36839HoFailureMinT310Elapsed) still rescues
            // the link below. With the threshold at 0 this reduces to "fail on any
            // pending T310" (the original binary behaviour).
            const bool t310DegradedEnough =
                m_radioLinkFailureDetected.IsPending() &&
                (m_t310 - Simulator::GetDelayLeft(m_radioLinkFailureDetected)) >=
                    m_tr36839HoFailureMinT310Elapsed;
            if (m_tr36839HandoverFailure && t310DegradedEnough)
            {
                NS_LOG_INFO("HO command arrived while T310 active (source below Qout): "
                            "declaring TR 36.839 handover failure for IMSI "
                            << m_imsi);
                RadioLinkFailureDetected(RLF_HO_COMMAND_LATE);
                return;
            }
            SwitchToState(CONNECTED_HANDOVER);
            if (m_radioLinkFailureDetected.IsPending())
            {
                ResetRlfParams();
            }
            const NrRrcSap::MobilityControlInfo& mci = msg.mobilityControlInfo;
            m_handoverStartTrace(m_imsi, m_cellId, m_rnti, mci.targetPhysCellId);
            // We should reset the MACs and PHYs for all the component carriers
            for (auto cmacSapProvider : m_cmacSapProvider)
            {
                cmacSapProvider->Reset();
            }
            for (auto cphySapProvider : m_cphySapProvider)
            {
                cphySapProvider->Reset();
            }
            m_ccmRrcSapProvider->Reset();
            StorePreviousCellId(m_cellId);
            m_cellId = mci.targetPhysCellId;
            NS_ASSERT(mci.haveCarrierFreq);
            NS_ASSERT(mci.haveCarrierBandwidth);
            // We could reconfigure PHY and BWPs, or we can just switch the primary DL/UL
            // indexes to match the correct frequency
            if (m_previousCellId != mci.targetPhysCellId)
            {
                auto dlIt = std::find_if(m_cphySapProvider.begin(),
                                         m_cphySapProvider.end(),
                                         [arfcn = mci.carrierFreq.dlCarrierFreq](auto& phy) {
                                             return phy->GetArfcn() == arfcn;
                                         });
                auto ulIt = std::find_if(m_cphySapProvider.begin(),
                                         m_cphySapProvider.end(),
                                         [arfcn = mci.carrierFreq.ulCarrierFreq](auto& phy) {
                                             return phy->GetArfcn() == arfcn;
                                         });
                NS_ASSERT_MSG(
                    (dlIt != m_cphySapProvider.end()) && (ulIt != m_cphySapProvider.end()),
                    "ARFCN from gNB should have been configured as a BWP/CC on UE at setup time");
                auto dlBwp = std::distance(m_cphySapProvider.begin(), dlIt);
                auto ulBwp = std::distance(m_cphySapProvider.begin(), ulIt);
                // Prefer the target cell's broadcast PHY configuration carried in
                // the handover command. The UE last decoded SIB1 on the *source*
                // cell, so reusing m_lastSib1 here would re-tune the target BWP to
                // the source numerology/TDD pattern and break inter-numerology
                // handover. Fall back to m_lastSib1 only if the target config was
                // not supplied (keeps same-numerology behaviour unchanged).
                const NrRrcSap::ServingCellConfigCommon& targetScc =
                    mci.haveServingCellConfigCommon ? mci.servingCellConfigCommon
                                                    : m_lastSib1.servingCellConfigCommon;
                ReconfigureFromSib1(dlBwp,
                                    mci.targetPhysCellId,
                                    targetScc.dlCtrlSymsNum,
                                    targetScc.ulCtrlSymsNum,
                                    targetScc.symbolsPerSlot,
                                    targetScc.numerology,
                                    targetScc.tddPattern,
                                    targetScc.rbgSize);
                if (ulBwp != dlBwp)
                {
                    ReconfigureFromSib1(ulBwp,
                                        mci.targetPhysCellId,
                                        targetScc.dlCtrlSymsNum,
                                        targetScc.ulCtrlSymsNum,
                                        targetScc.symbolsPerSlot,
                                        targetScc.ulNumerology,
                                        "F",
                                        targetScc.rbgSize);
                }
                // The output links describe the previous cell's UL/DL carrier
                // pairing; drop them before pointing the primaries at the new
                // cell's carriers
                if (m_clearBwpOutputLinksFn)
                {
                    m_clearBwpOutputLinksFn();
                }
                SetPrimaryDlIndex(std::distance(m_cphySapProvider.begin(), dlIt));
                SetPrimaryUlIndex(std::distance(m_cphySapProvider.begin(), ulIt));
            }
            m_cphySapProvider.at(GetPrimaryDlIndex())
                ->SynchronizeWithGnb(m_cellId, mci.carrierFreq.dlCarrierFreq);
            if (GetPrimaryUlIndex() != GetPrimaryDlIndex())
            {
                // The UL PHY must also resync to the target cell, or it keeps
                // filtering on the source cell ID and never sees the target's
                // RAR during the handover random access
                m_cphySapProvider.at(GetPrimaryUlIndex())
                    ->SynchronizeWithGnb(m_cellId, mci.carrierFreq.ulCarrierFreq);
            }
            m_cphySapProvider.at(GetPrimaryDlIndex())
                ->SetDlBandwidth(mci.carrierBandwidth.dlBandwidth);
            if (GetPrimaryUlIndex() != GetPrimaryDlIndex())
            {
                m_cphySapProvider.at(GetPrimaryUlIndex())
                    ->SetDlBandwidth(mci.carrierBandwidth.ulBandwidth);
            }
            m_cphySapProvider.at(GetPrimaryUlIndex())
                ->ConfigureUplink(mci.carrierFreq.ulCarrierFreq, mci.carrierBandwidth.ulBandwidth);
            m_rnti = msg.mobilityControlInfo.newUeIdentity;
            m_srb0->m_rlc->SetRnti(m_rnti);
            NS_ASSERT_MSG(
                mci.haveRachConfigDedicated,
                "handover is only supported with non-contention-based random access procedure");

            // Apply the target cell's RACH parameters, which the handover command
            // carries for this purpose. The UL BWP being handed over to need never
            // have decoded the target's SIB2 (an inter-frequency handover moves the
            // primary onto a BWP that saw no system information at all), so without
            // this its MAC would size the RA response window, and bound the preamble
            // retransmissions, from whatever its RACH configuration happened to hold.
            NrUeCmacSapProvider::RachConfig rc;
            rc.numberOfRaPreambles =
                mci.radioResourceConfigCommon.rachConfigCommon.preambleInfo.numberOfRaPreambles;
            rc.preambleTransMax =
                mci.radioResourceConfigCommon.rachConfigCommon.raSupervisionInfo.preambleTransMax;
            rc.raResponseWindowSize = mci.radioResourceConfigCommon.rachConfigCommon
                                          .raSupervisionInfo.raResponseWindowSize;
            rc.connEstFailCount =
                mci.radioResourceConfigCommon.rachConfigCommon.txFailParam.connEstFailCount;
            m_cmacSapProvider.at(GetPrimaryUlIndex())->ConfigureRach(rc);

            m_cmacSapProvider.at(GetPrimaryUlIndex())->RegisterToGnb(mci.targetPhysCellId);
            m_cmacSapProvider.at(GetPrimaryUlIndex())
                ->StartNonContentionBasedRandomAccessProcedure(
                    m_rnti,
                    mci.rachConfigDedicated.raPreambleIndex,
                    mci.rachConfigDedicated.raPrachMaskIndex);
            m_cphySapProvider.at(GetPrimaryUlIndex())->SetRnti(m_rnti);
            m_cphySapProvider.at(GetPrimaryDlIndex())->SetRnti(m_rnti);
            m_cmacSapProvider.at(GetPrimaryUlIndex())->SetRnti(m_rnti);
            m_cmacSapProvider.at(GetPrimaryDlIndex())->SetRnti(m_rnti);
            m_lastRrcTransactionIdentifier = msg.rrcTransactionIdentifier;
            NS_ASSERT(msg.haveRadioResourceConfigDedicated);

            // we re-establish SRB1 by creating a new entity
            // note that we can't dispose the old entity now, because
            // it's in the current stack, so we would corrupt the stack
            // if we did so. Hence we schedule it for later disposal
            m_srb1Old = m_srb1;
            Simulator::ScheduleNow(&NrUeRrc::DisposeOldSrb1, this);
            m_srb1 =
                nullptr; // new instance will be be created within ApplyRadioResourceConfigDedicated

            m_drbMap.clear(); // dispose all DRBs
            ApplyRadioResourceConfigDedicated(msg.radioResourceConfigDedicated);
            if (msg.haveNonCriticalExtension)
            {
                NS_LOG_DEBUG(this << "RNTI " << m_rnti
                                  << " Handover. Configuring secondary carriers");
                ApplyRadioResourceConfigDedicatedSecondaryCarrier(msg.nonCriticalExtension);
            }

            if (msg.haveMeasConfig)
            {
                ApplyMeasConfig(msg.measConfig);
            }
            // RRC connection reconfiguration completed will be sent
            // after handover is complete
        }
        else
        {
            NS_LOG_INFO("haveMobilityControlInfo == false");
            if (msg.haveNonCriticalExtension)
            {
                ApplyRadioResourceConfigDedicatedSecondaryCarrier(msg.nonCriticalExtension);
                NS_LOG_DEBUG(this << "RNTI " << m_rnti << " Configured for CA");
            }
            if (msg.haveRadioResourceConfigDedicated)
            {
                ApplyRadioResourceConfigDedicated(msg.radioResourceConfigDedicated);
            }
            if (msg.haveMeasConfig)
            {
                ApplyMeasConfig(msg.measConfig);
            }
            NrRrcSap::RrcConnectionReconfigurationCompleted msg2{};
            msg2.rrcTransactionIdentifier = msg.rrcTransactionIdentifier;
            m_rrcSapUser->SendRrcConnectionReconfigurationCompleted(msg2);
            m_connectionReconfigurationTrace(m_imsi, m_cellId, m_rnti);
        }
        break;

    case IDLE_START:
    case IDLE_CELL_SEARCH:
    case IDLE_WAIT_MIB_SIB1:
    case IDLE_WAIT_MIB:
    case IDLE_WAIT_SIB1:
    case IDLE_CAMPED_NORMALLY:
    case IDLE_WAIT_SIB2:
    case IDLE_RANDOM_ACCESS:
    case IDLE_CONNECTING:
        // The reconfiguration belongs to a connection the UE has already left, e.g., a
        // handover command sent by the source cell after the UE declared a radio link failure
        NS_LOG_WARN("Ignoring RrcConnectionReconfiguration of a previous connection in state "
                    << ToString(m_state));
        break;

    default:
        NS_FATAL_ERROR("method unexpected in state " << ToString(m_state));
        break;
    }
}

void
NrUeRrc::DoRecvRrcConnectionReestablishment(NrRrcSap::RrcConnectionReestablishment msg)
{
    NS_LOG_FUNCTION(this << " RNTI " << m_rnti << ", cellId " << m_cellId);
    switch (m_state)
    {
    case CONNECTED_REESTABLISHING: {
        /**
         * @todo After receiving RRC Connection Re-establishment, stop timer
         *       T301, fire a new trace source, reply with RRC Connection
         *       Re-establishment Complete, and finally switch to
         *       CONNECTED_NORMALLY state. See Section 5.3.7.5 of 3GPP TS
         *       36.331.
         */
    }
    break;

    default:
        NS_FATAL_ERROR("method unexpected in state " << ToString(m_state));
        break;
    }
}

void
NrUeRrc::DoRecvRrcConnectionReestablishmentReject(NrRrcSap::RrcConnectionReestablishmentReject msg)
{
    NS_LOG_FUNCTION(this << " RNTI " << m_rnti << ", cellId " << m_cellId);
    switch (m_state)
    {
    case CONNECTED_REESTABLISHING: {
        /**
         * @todo After receiving RRC Connection Re-establishment Reject, stop
         *       timer T301. See Section 5.3.7.8 of 3GPP TS 36.331.
         */
        m_asSapUser->NotifyConnectionReleased(); // Inform upper layers
    }
    break;

    default:
        NS_FATAL_ERROR("method unexpected in state " << ToString(m_state));
        break;
    }
}

void
NrUeRrc::DoRecvRrcConnectionRelease(NrRrcSap::RrcConnectionRelease msg)
{
    NS_LOG_FUNCTION(this << " RNTI " << m_rnti << ", cellId " << m_cellId);
    /// @todo Currently not implemented, see Section 5.3.8 of 3GPP TS 36.331.

    m_lastRrcTransactionIdentifier = msg.rrcTransactionIdentifier;
    // release resources at UE
    if (!m_leaveConnectedMode)
    {
        m_leaveConnectedMode = true;
        NS_LOG_DEBUG("Switch to CONNECTED_PHY_PROBLEM. Reason: Received connection release message "
                     "for IMSI: "
                     << m_imsi << " rnti: " << m_rnti << " cellId: " << m_cellId
                     << " in state: " << ToString(m_state) << ".");
        EnterPhyProblemState(RLF_CONNECTION_RELEASE);
        m_rrcSapUser->SendIdealUeContextRemoveRequest(m_rnti);
        m_asSapUser->NotifyConnectionReleased();
    }
}

void
NrUeRrc::DoRecvRrcConnectionReject(NrRrcSap::RrcConnectionReject msg)
{
    NS_LOG_FUNCTION(this);
    m_connectionTimeout.Cancel();
    for (uint16_t i = 0; i < m_numberOfComponentCarriers; i++)
    {
        m_cmacSapProvider.at(i)->Reset(); // reset the MAC
    }
    m_hasReceivedSib2 = false; // invalidate the previously received SIB2
    SwitchToState(IDLE_CAMPED_NORMALLY);
    m_asSapUser->NotifyConnectionFailed(); // inform upper layer
}

void
NrUeRrc::DoSetNumberOfComponentCarriers(uint16_t noOfComponentCarriers)
{
    NS_LOG_FUNCTION(this);
    m_numberOfComponentCarriers = noOfComponentCarriers;
}

void
NrUeRrc::SynchronizeToStrongestCell()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_state == IDLE_CELL_SEARCH);

    uint16_t maxRsrpCellId = 0;
    double maxRsrp = -std::numeric_limits<double>::infinity();
    double minRsrp = -140.0; // Minimum RSRP in dBm a UE can report
    uint32_t maxRsrpArfcn = 0;

    for (auto it = m_storedMeasValues.begin(); it != m_storedMeasValues.end(); it++)
    {
        /*
         * This block attempts to find a cell with strongest RSRP and has not
         * yet been identified as "acceptable cell".
         */
        if (maxRsrp < it->second.rsrp && it->second.rsrp > minRsrp)
        {
            auto itCell = m_acceptableCell.find(it->first);
            if (itCell == m_acceptableCell.end())
            {
                maxRsrpCellId = it->first;
                maxRsrp = it->second.rsrp;
                maxRsrpArfcn = it->second.carrierFreq;
            }
        }
    }

    if (maxRsrpCellId == 0)
    {
        NS_LOG_WARN(this << " Cell search is unable to detect surrounding cell to attach to");
    }
    else
    {
        NS_LOG_LOGIC(this << " cell " << maxRsrpCellId << " via arfcn " << maxRsrpArfcn
                          << " is the strongest untried surrounding cell");
        // We may receive MIBs from different BWPs. When that happens, we switch active BWP.
        m_initDlArfcn = maxRsrpArfcn;
        TrackCellArfcn(maxRsrpCellId, maxRsrpArfcn);
        auto dlIt =
            std::find_if(m_cphySapProvider.begin(),
                         m_cphySapProvider.end(),
                         [maxRsrpArfcn](auto& phy) { return phy->GetArfcn() == maxRsrpArfcn; });
        NS_ASSERT_MSG((dlIt != m_cphySapProvider.end()),
                      "ARFCN from gNB should have been configured as a BWP/CC on UE at setup time");
        auto dlBwp = std::distance(m_cphySapProvider.begin(), dlIt);
        SetPrimaryDlIndex(dlBwp);
        m_cmacSapProvider.at(dlBwp)->Reset();
        m_cphySapProvider.at(dlBwp)->Reset();
        m_cphySapProvider.at(dlBwp)->SynchronizeWithGnb(maxRsrpCellId, m_initDlArfcn);
        SwitchToState(IDLE_WAIT_MIB_SIB1);
    }
} // end of void NrUeRrc::SynchronizeToStrongestCell ()

void
NrUeRrc::ArmMibWaitReselect()
{
    if (m_mibWaitReselectTimeout.IsZero())
    {
        return; // fallback disabled
    }
    m_mibWaitTimeoutEvent.Cancel();
    m_mibWaitTimeoutEvent =
        Simulator::Schedule(m_mibWaitReselectTimeout, &NrUeRrc::MibWaitReselect, this);
}

void
NrUeRrc::MibWaitReselect()
{
    if (m_state != IDLE_WAIT_MIB)
    {
        return; // the MIB arrived (or the UE moved on); nothing to rescue
    }

    if (++m_mibWaitAttempts > m_mibWaitReselectMaxAttempts)
    {
        NS_LOG_WARN("IMSI " << m_imsi << " gave up initial cell acquisition after "
                            << (m_mibWaitAttempts - 1) << " MIB-wait reselection attempts");
        return;
    }

    // Never retry the cell we are currently (and unsuccessfully) camped on.
    m_mibCampTried.insert(m_cellId);

    // Pick the strongest measured cell we have not tried yet.
    uint16_t bestCell = 0;
    double bestRsrp = -std::numeric_limits<double>::infinity();
    uint32_t bestArfcn = 0;
    for (const auto& [cellId, meas] : m_storedMeasValues)
    {
        if (m_mibCampTried.count(cellId) || meas.carrierFreq == 0)
        {
            continue;
        }
        if (meas.rsrp > bestRsrp)
        {
            bestRsrp = meas.rsrp;
            bestCell = cellId;
            bestArfcn = meas.carrierFreq;
        }
    }

    if (bestCell == 0)
    {
        // No untried measured cell yet (PSS measurements may still be accumulating);
        // look again after another interval.
        ArmMibWaitReselect();
        return;
    }

    NS_LOG_INFO("IMSI " << m_imsi << " MIB-wait reselect: cell " << m_cellId << " -> " << bestCell
                        << " (rsrp " << bestRsrp << " dBm, arfcn " << bestArfcn << ")");

    // Re-camp on the stronger cell and stay in IDLE_WAIT_MIB, waiting for its MIB.
    CampOnGnb(bestCell, bestArfcn);
    ArmMibWaitReselect();
}

std::size_t
NrUeRrc::GetArfcnBwpId(uint32_t arfcn) const
{
    for (std::size_t i = 0; i < m_cphySapProvider.size(); i++)
    {
        if (m_cphySapProvider.at(i)->GetArfcn() == arfcn)
        {
            return i;
        }
    }
    NS_FATAL_ERROR("No BWP found with arfcn " << arfcn);
}

void
NrUeRrc::TrackCellArfcn(uint16_t cellId, uint32_t arfcn)
{
    NS_LOG_FUNCTION(this << cellId << arfcn);
    if (cellId == 0 || arfcn == 0)
    {
        return;
    }
    // A cell can span several carriers (same cellId on several BWPs). Record
    // this carrier in the cell's carrier set rather than overwriting, so the
    // same-cell BWP-switch guard can recognize every BWP belonging to the cell.
    m_cellIdToArfcn[cellId].insert(arfcn);
}

bool
NrUeRrc::GetCellBwpId(uint16_t cellId, std::size_t& bwpId) const
{
    auto it = m_cellIdToArfcn.find(cellId);
    if (it == m_cellIdToArfcn.end() || it->second.empty())
    {
        return false;
    }
    // Resolve to any BWP tuned to one of the cell's carriers. When the cell
    // spans a single carrier this is unambiguous; when it spans several, this
    // returns the lowest-indexed tuned BWP, which callers that only need "a"
    // BWP of the cell (e.g. MIB routing) can use. The current primary BWP is
    // preferred when it already belongs to the cell.
    if (m_cellIdToArfcn.count(cellId))
    {
        const auto& carriers = it->second;
        const uint32_t primaryArfcn = m_cphySapProvider.at(m_primaryDlIndex)->GetArfcn();
        if (carriers.count(primaryArfcn))
        {
            bwpId = m_primaryDlIndex;
            return true;
        }
    }
    for (std::size_t i = 0; i < m_cphySapProvider.size(); i++)
    {
        if (it->second.count(m_cphySapProvider.at(i)->GetArfcn()))
        {
            bwpId = i;
            return true;
        }
    }
    return false;
}

bool
NrUeRrc::SwitchPrimaryBwpSameCell(std::size_t targetBwpId)
{
    NS_LOG_FUNCTION(this << targetBwpId << m_cellId << +GetPrimaryDlIndex());

    if (targetBwpId >= m_cphySapProvider.size())
    {
        NS_LOG_WARN("SwitchPrimaryBwpSameCell: target bwp=" << targetBwpId << " out of range");
        return false;
    }
    if (targetBwpId == GetPrimaryDlIndex())
    {
        // Already the active primary; nothing to do.
        return true;
    }

    // Same-cell guard, CELLID-based. Only ever move the serving BWP between BWPs
    // that belong to the CURRENT serving cell. Because all BWPs of a gNB share
    // one cellId and differ by ARFCN alone, the test is membership of the target
    // BWP's carrier in the serving cell's carrier SET (#m_cellIdToArfcn). Moving
    // to a BWP whose carrier is NOT in that set would be a handover (and, if both
    // were kept active, dual connectivity), which is explicitly out of scope.
    auto cellIt = m_cellIdToArfcn.find(m_cellId);
    if (cellIt == m_cellIdToArfcn.end() || cellIt->second.empty())
    {
        NS_LOG_WARN("SwitchPrimaryBwpSameCell: serving cell="
                    << m_cellId << " carrier set unknown; refusing to switch");
        return false;
    }
    const uint32_t targetArfcn = m_cphySapProvider.at(targetBwpId)->GetArfcn();
    if (cellIt->second.count(targetArfcn) == 0)
    {
        NS_LOG_WARN("SwitchPrimaryBwpSameCell: target bwp="
                    << targetBwpId << " (arfcn " << targetArfcn
                    << ") is not a carrier of serving cell=" << m_cellId
                    << "; refusing (would be handover/DC)");
        return false;
    }

    NS_LOG_INFO("Switching primary serving BWP " << +GetPrimaryDlIndex() << " -> " << targetBwpId
                                                 << " on serving cell=" << m_cellId);

    // Re-point the primary DL (and UL when they coincide) and re-bind the RNTI
    // on the new PHY/MAC so the data plane follows the new primary BWP. The UL
    // does not follow the DL onto a DL-only BWP (e.g., the DL carrier of an FDD
    // pair): the cell keeps receiving on its usual UL carrier.
    auto ulCapableIt = m_bwpUlCapable.find(targetBwpId);
    const bool targetUlCapable = ulCapableIt == m_bwpUlCapable.end() || ulCapableIt->second;
    const bool ulFollowsDl = (GetPrimaryUlIndex() == GetPrimaryDlIndex()) && targetUlCapable;
    SetPrimaryDlIndex(targetBwpId);
    if (ulFollowsDl)
    {
        SetPrimaryUlIndex(targetBwpId);
    }
    if (m_rnti != 0)
    {
        m_cphySapProvider.at(GetPrimaryDlIndex())->SetRnti(m_rnti);
        m_cmacSapProvider.at(GetPrimaryDlIndex())->SetRnti(m_rnti);
        if (ulFollowsDl)
        {
            m_cphySapProvider.at(GetPrimaryUlIndex())->SetRnti(m_rnti);
            m_cmacSapProvider.at(GetPrimaryUlIndex())->SetRnti(m_rnti);
        }
    }

    // TODO: this is the MECHANISM only. A same-cell BWP-switching POLICY (e.g.
    // RSRP/load-driven selection of which same-cell BWP to make primary, plus
    // re-application of the dedicated RadioResourceConfigDedicated / bearer
    // mapping to the new BWP and any required RRC signalling) is NOT implemented
    // and must be added before this is driven automatically.
    return true;
}

void
NrUeRrc::EvaluateSameCellBwpSwitch()
{
    // Only meaningful for a connected UE with a known serving cell that spans
    // more than one carrier (otherwise there is nothing to switch between).
    if (m_state != CONNECTED_NORMALLY || m_cellId == 0)
    {
        return;
    }
    auto cellIt = m_cellIdToArfcn.find(m_cellId);
    if (cellIt == m_cellIdToArfcn.end() || cellIt->second.size() < 2)
    {
        return;
    }

    const uint32_t servingArfcn = m_cphySapProvider.at(GetPrimaryDlIndex())->GetArfcn();
    // The serving BWP must itself be a carrier of the serving cell.
    if (cellIt->second.count(servingArfcn) == 0)
    {
        return;
    }
    auto servIt = m_rsrpPerArfcn.find(servingArfcn);
    if (servIt == m_rsrpPerArfcn.end())
    {
        return;
    }
    const double servingRsrp = servIt->second;

    // Find the strongest same-cell carrier (other than the serving one).
    uint32_t bestArfcn = 0;
    double bestRsrp = -std::numeric_limits<double>::infinity();
    for (const uint32_t arfcn : cellIt->second)
    {
        if (arfcn == servingArfcn)
        {
            continue;
        }
        auto rit = m_rsrpPerArfcn.find(arfcn);
        if (rit == m_rsrpPerArfcn.end())
        {
            continue;
        }
        if (rit->second > bestRsrp)
        {
            bestRsrp = rit->second;
            bestArfcn = arfcn;
        }
    }

    if (bestArfcn == 0 || bestRsrp < servingRsrp + m_bwpSwitchHysteresisDb)
    {
        return; // no candidate beats the serving BWP by the hysteresis margin
    }

    const std::size_t targetBwp = GetArfcnBwpId(bestArfcn);
    NS_LOG_INFO("Same-cell BWP switch trigger: serving arfcn "
                << servingArfcn << " rsrp " << servingRsrp << " -> arfcn " << bestArfcn << " rsrp "
                << bestRsrp << " (margin " << m_bwpSwitchHysteresisDb << " dB) on cell "
                << m_cellId);

    if (SwitchPrimaryBwpSameCell(targetBwp))
    {
        // Inform the gNB so its DL scheduling follows the UE's new primary BWP.
        // Without this the gNB would keep scheduling the old BWP and DL data
        // would stall (the UE/gNB would be desynchronized).
        if (m_reportedPrimaryArfcn != bestArfcn && m_rrcSapUser != nullptr)
        {
            m_rrcSapUser->SendIdealBwpSwitchIndication(m_rnti, static_cast<uint8_t>(targetBwp));
            m_reportedPrimaryArfcn = bestArfcn;
        }
    }
}

void
NrUeRrc::EvaluateCellForSelection()
{
    NS_LOG_FUNCTION(this << " primary UL " << GetPrimaryUlIndex() << ", primary DL "
                         << GetPrimaryDlIndex());
    NS_ASSERT(m_state == IDLE_WAIT_SIB1);
    NS_ASSERT(m_hasReceivedMib);
    NS_ASSERT(m_hasReceivedSib1);
    uint16_t cellId = m_lastSib1.cellAccessRelatedInfo.cellIdentity;

    // Cell selection criteria evaluation

    bool isSuitableCell = false;
    bool isAcceptableCell = false;
    auto storedMeasIt = m_storedMeasValues.find(cellId);
    double qRxLevMeas = storedMeasIt->second.rsrp;
    double qRxLevMin = nr::EutranMeasurementMapping::IeValue2ActualQRxLevMin(
        m_lastSib1.cellSelectionInfo.qRxLevMin);
    NS_LOG_LOGIC(this << " cell selection to cellId=" << cellId << " qrxlevmeas=" << qRxLevMeas
                      << " dBm"
                      << " qrxlevmin=" << qRxLevMin << " dBm");

    if (qRxLevMeas - qRxLevMin > 0)
    {
        isAcceptableCell = true;

        uint32_t cellCsgId = m_lastSib1.cellAccessRelatedInfo.csgIdentity;
        bool cellCsgIndication = m_lastSib1.cellAccessRelatedInfo.csgIndication;

        isSuitableCell = (!cellCsgIndication || cellCsgId == m_csgWhiteList);

        NS_LOG_LOGIC(this << " csg(ue/cell/indication)=" << m_csgWhiteList << "/" << cellCsgId
                          << "/" << cellCsgIndication);
    }

    // Cell selection decision

    if (isSuitableCell)
    {
        m_cellId = cellId;
        // todo: for maximum flexibility, we could create a new MAC/PHY for the ARFCN if there is no
        // currently setup
        auto bwpId = GetArfcnBwpId(m_initDlArfcn);
        SetPrimaryDlIndex(bwpId);
        // The cell advertises its UL carrier in SIB1; when it differs from the
        // camped (DL) carrier this is an FDD cell and the primary UL BWP must
        // move to the carrier the cell actually receives on
        const uint32_t sib1UlArfcn = m_lastSib1.servingCellConfigCommon.ulCarrierFreq;
        if (sib1UlArfcn != 0 && sib1UlArfcn != m_initDlArfcn)
        {
            SetPrimaryUlIndex(GetArfcnBwpId(sib1UlArfcn));
        }
        // Align the active BWP PHY numerology with the cell we are about to camp
        // on. The RNTI is unknown at this point (it is assigned by random access),
        // so it must not be touched here.
        m_cphySapProvider.at(bwpId)->SetNumerology(m_lastSib1.servingCellConfigCommon.numerology);
        m_cphySapProvider.at(bwpId)->SynchronizeWithGnb(cellId, m_initDlArfcn);
        m_cphySapProvider.at(bwpId)->SetDlBandwidth(m_dlBandwidth);
        m_initialCellSelectionEndOkTrace(m_imsi, cellId);
        auto dlBwpIndex = GetPrimaryDlIndex();
        auto ulBwpIndex = GetPrimaryUlIndex();
        ReconfigureFromSib1(dlBwpIndex,
                            m_cellId,
                            m_lastSib1.servingCellConfigCommon.dlCtrlSymsNum,
                            m_lastSib1.servingCellConfigCommon.ulCtrlSymsNum,
                            m_lastSib1.servingCellConfigCommon.symbolsPerSlot,
                            m_lastSib1.servingCellConfigCommon.numerology,
                            m_lastSib1.servingCellConfigCommon.tddPattern,
                            m_lastSib1.servingCellConfigCommon.rbgSize);

        if (ulBwpIndex != dlBwpIndex)
        {
            ReconfigureFromSib1(ulBwpIndex,
                                m_cellId,
                                m_lastSib1.servingCellConfigCommon.dlCtrlSymsNum,
                                m_lastSib1.servingCellConfigCommon.ulCtrlSymsNum,
                                m_lastSib1.servingCellConfigCommon.symbolsPerSlot,
                                m_lastSib1.servingCellConfigCommon.ulNumerology,
                                "F",
                                m_lastSib1.servingCellConfigCommon.rbgSize);
            m_cphySapProvider.at(ulBwpIndex)->SynchronizeWithGnb(m_cellId, sib1UlArfcn);
        }
        m_cmacSapProvider.at(ulBwpIndex)->RegisterToGnb(m_cellId);
        if (ulBwpIndex != dlBwpIndex)
        {
            m_cmacSapProvider.at(dlBwpIndex)->RegisterToGnb(m_cellId);
        }
        // Once the UE is connected, m_connectionPending is
        // set to false. So, when RLF occurs and UE performs
        // cell selection upon leaving RRC_CONNECTED state,
        // the following call to DoConnect will make the
        // m_connectionPending to be true again. Thus,
        // upon calling SwitchToState (IDLE_CAMPED_NORMALLY)
        // UE state is instantly change to IDLE_WAIT_SIB2.
        // This will make the UE to read the SIB2 message
        // and start random access.
        if (!m_connectionPending)
        {
            NS_LOG_DEBUG("Calling DoConnect in state = " << ToString(m_state));
            DoConnect();
        }
        SwitchToState(IDLE_CAMPED_NORMALLY);
    }
    else
    {
        // ignore the MIB and SIB1 received from this cell
        m_hasReceivedMib = false;
        m_hasReceivedSib1 = false;

        m_initialCellSelectionEndErrorTrace(m_imsi, cellId);

        if (isAcceptableCell)
        {
            /*
             * The cells inserted into this list will not be considered for
             * subsequent cell search attempt.
             */
            m_acceptableCell.insert(cellId);
        }

        SwitchToState(IDLE_CELL_SEARCH);
        SynchronizeToStrongestCell(); // retry to a different cell
    }

} // end of void NrUeRrc::EvaluateCellForSelection ()

void
NrUeRrc::ApplyRadioResourceConfigDedicatedSecondaryCarrier(
    NrRrcSap::NonCriticalExtensionConfiguration nonCec)
{
    NS_LOG_FUNCTION(this);

    m_sCellToAddModList = nonCec.sCellToAddModList;

    for (uint32_t sCellIndex : nonCec.sCellToReleaseList)
    {
        m_cphySapProvider.at(sCellIndex)->Reset();
        m_cmacSapProvider.at(sCellIndex)->Reset();
    }

    for (auto& scell : nonCec.sCellToAddModList)
    {
        uint8_t ccId = scell.sCellIndex;

        uint16_t physCellId = scell.cellIdentification.physCellId;
        uint16_t ulBand =
            scell.radioResourceConfigCommonSCell.ulConfiguration.ulFreqInfo.ulBandwidth;
        m_initUlArfcn =
            scell.radioResourceConfigCommonSCell.ulConfiguration.ulFreqInfo.ulCarrierFreq;
        uint16_t dlBand = scell.radioResourceConfigCommonSCell.nonUlConfiguration.dlBandwidth;
        m_initDlArfcn = scell.cellIdentification.dlCarrierFreq;
        uint8_t txMode = scell.radioResourceConfigDedicatedSCell.physicalConfigDedicatedSCell
                             .antennaInfo.transmissionMode;
        uint16_t srsIndex = scell.radioResourceConfigDedicatedSCell.physicalConfigDedicatedSCell
                                .soundingRsUlConfigDedicated.srsConfigIndex;

        m_cphySapProvider.at(ccId)->SynchronizeWithGnb(physCellId, m_initDlArfcn);
        m_cphySapProvider.at(ccId)->SetDlBandwidth(dlBand);
        m_cphySapProvider.at(ccId)->ConfigureUplink(m_initUlArfcn, ulBand);
        m_cphySapProvider.at(ccId)->ConfigureReferenceSignalPower(
            scell.radioResourceConfigCommonSCell.nonUlConfiguration.pdschConfigCommon
                .referenceSignalPower);
        m_cphySapProvider.at(ccId)->SetTransmissionMode(txMode);
        m_cphySapProvider.at(ccId)->SetRnti(m_rnti);
        m_cmacSapProvider.at(ccId)->SetRnti(m_rnti);
        // update PdschConfigDedicated (i.e. P_A value)
        NrRrcSap::PdschConfigDedicated pdschConfigDedicated =
            scell.radioResourceConfigDedicatedSCell.physicalConfigDedicatedSCell
                .pdschConfigDedicated;
        double paDouble = NrRrcSap::ConvertPdschConfigDedicated2Double(pdschConfigDedicated);
        m_cphySapProvider.at(ccId)->SetPa(paDouble);
        m_cphySapProvider.at(ccId)->SetSrsConfigurationIndex(srsIndex);
    }

    m_sCarrierConfiguredTrace(this, m_sCellToAddModList);
}

void
NrUeRrc::ApplyServingCellBwpConfig(const NrRrcSap::RadioResourceConfigDedicated& rrcd)
{
    NS_LOG_FUNCTION(this);
    m_rrcBwpPairings.clear();
    for (const auto& bwpConfig : rrcd.bwpConfigList)
    {
        auto phyIt =
            std::find_if(m_cphySapProvider.begin(),
                         m_cphySapProvider.end(),
                         [arfcn = bwpConfig.arfcn](auto& phy) { return phy->GetArfcn() == arfcn; });
        if (phyIt == m_cphySapProvider.end())
        {
            NS_LOG_WARN("Serving cell BWP with ARFCN " << bwpConfig.arfcn
                                                       << " has no matching local BWP; skipping");
            continue;
        }
        auto bwpId = std::distance(m_cphySapProvider.begin(), phyIt);
        ReconfigureFromSib1(bwpId,
                            m_cellId,
                            bwpConfig.config.dlCtrlSymsNum,
                            bwpConfig.config.ulCtrlSymsNum,
                            bwpConfig.config.symbolsPerSlot,
                            bwpConfig.config.numerology,
                            bwpConfig.config.tddPattern,
                            bwpConfig.config.rbgSize);
        // A DL-only (FDD) carrier sends its outgoing control messages through
        // the UL carrier it is paired with
        if (bwpConfig.config.ulCarrierFreq != 0 &&
            bwpConfig.config.ulCarrierFreq != bwpConfig.arfcn && m_updateBwpOutputLinkFn)
        {
            auto ulIt = std::find_if(m_cphySapProvider.begin(),
                                     m_cphySapProvider.end(),
                                     [arfcn = bwpConfig.config.ulCarrierFreq](auto& phy) {
                                         return phy->GetArfcn() == arfcn;
                                     });
            if (ulIt != m_cphySapProvider.end())
            {
                auto ulBwpId = std::distance(m_cphySapProvider.begin(), ulIt);
                m_rrcBwpPairings[bwpId] = ulBwpId;
                m_updateBwpOutputLinkFn(bwpId, ulBwpId);
            }
        }
    }
    if (m_updateQosFlowBwpFn)
    {
        for (const auto& mapping : rrcd.qosFlowToBwpList)
        {
            auto phyIt = std::find_if(
                m_cphySapProvider.begin(),
                m_cphySapProvider.end(),
                [arfcn = mapping.bwpArfcn](auto& phy) { return phy->GetArfcn() == arfcn; });
            if (phyIt == m_cphySapProvider.end())
            {
                NS_LOG_WARN("QoS flow BWP with ARFCN " << mapping.bwpArfcn
                                                       << " has no matching local BWP; skipping");
                continue;
            }
            auto bwpId = std::distance(m_cphySapProvider.begin(), phyIt);
            // A flow the network pins to a DL-only (FDD) BWP is served in the
            // uplink by the UL carrier that BWP is paired with: the advertised
            // mapping drives the gNB's downlink scheduling, while the UE needs
            // an UL-capable BWP for its own transmissions
            const auto pairIt = m_rrcBwpPairings.find(bwpId);
            const auto ulBwpId = pairIt != m_rrcBwpPairings.end() ? pairIt->second : bwpId;
            m_updateQosFlowBwpFn(mapping.fiveQi, ulBwpId);
        }
    }
}

void
NrUeRrc::ApplyRadioResourceConfigDedicated(NrRrcSap::RadioResourceConfigDedicated rrcd)
{
    NS_LOG_FUNCTION(this << " primary UL " << GetPrimaryUlIndex() << ", primary DL "
                         << GetPrimaryDlIndex());
    ApplyServingCellBwpConfig(rrcd);
    const NrRrcSap::PhysicalConfigDedicated& pcd = rrcd.physicalConfigDedicated;

    if (pcd.haveAntennaInfoDedicated)
    {
        m_cphySapProvider.at(GetPrimaryUlIndex())
            ->SetTransmissionMode(pcd.antennaInfo.transmissionMode);
    }
    if (pcd.haveSoundingRsUlConfigDedicated)
    {
        m_cphySapProvider.at(GetPrimaryUlIndex())
            ->SetSrsConfigurationIndex(pcd.soundingRsUlConfigDedicated.srsConfigIndex);
    }

    if (pcd.havePdschConfigDedicated)
    {
        // update PdschConfigDedicated (i.e. P_A value)
        m_pdschConfigDedicated = pcd.pdschConfigDedicated;
        double paDouble = NrRrcSap::ConvertPdschConfigDedicated2Double(m_pdschConfigDedicated);
        m_cphySapProvider.at(GetPrimaryDlIndex())->SetPa(paDouble);
    }

    auto stamIt = rrcd.srbToAddModList.begin();
    if (stamIt != rrcd.srbToAddModList.end())
    {
        if (!m_srb1)
        {
            // SRB1 not setup yet
            NS_ASSERT_MSG((m_state == IDLE_CONNECTING) || (m_state == CONNECTED_HANDOVER),
                          "unexpected state " << ToString(m_state));
            NS_ASSERT_MSG(stamIt->srbIdentity == 1, "only SRB1 supported");

            const uint8_t lcid = 1; // fixed LCID for SRB1

            Ptr<NrRlc> rlc = CreateObject<NrRlcAm>();
            rlc->SetNrMacSapProvider(m_macSapProvider);
            rlc->SetRnti(m_rnti);
            rlc->SetLcId(lcid);
            // SRB1 is RLC-AM: an undeliverable uplink (max-retx) is an RLF trigger.
            rlc->SetMaxRetxReachedCallback(MakeCallback(&NrUeRrc::DoNotifyRlcMaxRetx, this));

            Ptr<NrPdcp> pdcp = CreateObject<NrPdcp>();
            pdcp->SetRnti(m_rnti);
            pdcp->SetLcId(lcid);
            pdcp->SetNrPdcpSapUser(m_drbPdcpSapUser);
            pdcp->SetNrRlcSapProvider(rlc->GetNrRlcSapProvider());
            rlc->SetNrRlcSapUser(pdcp->GetNrRlcSapUser());

            m_srb1 = CreateObject<NrSignalingRadioBearerInfo>();
            m_srb1->m_rlc = rlc;
            m_srb1->m_pdcp = pdcp;
            m_srb1->m_srbIdentity = 1;
            m_srb1CreatedTrace(m_imsi, m_cellId, m_rnti);

            m_srb1->m_logicalChannelConfig.priority = stamIt->logicalChannelConfig.priority;
            m_srb1->m_logicalChannelConfig.fiveQi = stamIt->logicalChannelConfig.fiveQi;
            m_srb1->m_logicalChannelConfig.prioritizedBitRateKbps =
                stamIt->logicalChannelConfig.prioritizedBitRateKbps;
            m_srb1->m_logicalChannelConfig.bucketSizeDurationMs =
                stamIt->logicalChannelConfig.bucketSizeDurationMs;
            m_srb1->m_logicalChannelConfig.logicalChannelGroup =
                stamIt->logicalChannelConfig.logicalChannelGroup;

            NrUeCmacSapProvider::LogicalChannelConfig lcConfig;
            lcConfig.priority = stamIt->logicalChannelConfig.priority;
            lcConfig.fiveQi = stamIt->logicalChannelConfig.fiveQi;
            lcConfig.prioritizedBitRateKbps = stamIt->logicalChannelConfig.prioritizedBitRateKbps;
            lcConfig.bucketSizeDurationMs = stamIt->logicalChannelConfig.bucketSizeDurationMs;
            lcConfig.logicalChannelGroup = stamIt->logicalChannelConfig.logicalChannelGroup;
            NrMacSapUser* msu =
                m_ccmRrcSapProvider->ConfigureSignalBearer(lcid, lcConfig, rlc->GetNrMacSapUser());
            m_cmacSapProvider.at(GetPrimaryUlIndex())->AddLc(lcid, lcConfig, msu);
            if (GetPrimaryDlIndex() != GetPrimaryUlIndex())
            {
                m_cmacSapProvider.at(GetPrimaryDlIndex())->AddLc(lcid, lcConfig, msu);
            }
            ++stamIt;
            NS_ASSERT_MSG(stamIt == rrcd.srbToAddModList.end(), "at most one SrbToAdd supported");

            NrUeRrcSapUser::SetupParameters ueParams;
            ueParams.srb0SapProvider = m_srb0->m_rlc->GetNrRlcSapProvider();
            ueParams.srb1SapProvider = m_srb1->m_pdcp->GetNrPdcpSapProvider();
            m_rrcSapUser->Setup(ueParams);
        }
        else
        {
            NS_LOG_INFO("request to modify SRB1 (skipping as currently not implemented)");
            // would need to modify m_srb1, and then propagate changes to the MAC
        }
    }

    for (auto dtamIt = rrcd.drbToAddModList.begin(); dtamIt != rrcd.drbToAddModList.end(); ++dtamIt)
    {
        NS_LOG_INFO(this << " IMSI " << m_imsi << " adding/modifying DRBID "
                         << (uint32_t)dtamIt->drbIdentity << " LC "
                         << (uint32_t)dtamIt->logicalChannelIdentity << ", cellId " << m_cellId);
        NS_ASSERT_MSG(dtamIt->logicalChannelIdentity > 2,
                      "LCID value " << dtamIt->logicalChannelIdentity << " is reserved for SRBs");

        auto drbMapIt = m_drbMap.find(dtamIt->drbIdentity);
        if (drbMapIt == m_drbMap.end())
        {
            NS_LOG_INFO("New Data Radio Bearer");

            TypeId rlcTypeId;
            if (m_useRlcSm)
            {
                rlcTypeId = NrRlcSm::GetTypeId();
            }
            else
            {
                switch (dtamIt->rlcConfig.choice)
                {
                case NrRrcSap::RlcConfig::AM:
                    rlcTypeId = NrRlcAm::GetTypeId();
                    break;

                case NrRrcSap::RlcConfig::UM_BI_DIRECTIONAL:
                    rlcTypeId = NrRlcUm::GetTypeId();
                    break;

                default:
                    NS_FATAL_ERROR("unsupported RLC configuration");
                    break;
                }
            }

            ObjectFactory rlcObjectFactory;
            rlcObjectFactory.SetTypeId(rlcTypeId);
            Ptr<NrRlc> rlc = rlcObjectFactory.Create()->GetObject<NrRlc>();
            rlc->SetNrMacSapProvider(m_macSapProvider);
            rlc->SetRnti(m_rnti);
            rlc->SetLcId(dtamIt->logicalChannelIdentity);
            // For AM DRBs, max-retx is an RLF trigger (no-op for UM/SM RLC).
            rlc->SetMaxRetxReachedCallback(MakeCallback(&NrUeRrc::DoNotifyRlcMaxRetx, this));
            if (m_useRlcSm)
            {
                // Starts the chain of calls:
                // DoReportBufferStatus->DoNotifyTxOpp->DoReportBufferStatus...
                Simulator::ScheduleNow(&NrRlcSm::Initialize, rlc);
            }

            Ptr<NrDataRadioBearerInfo> drbInfo = CreateObject<NrDataRadioBearerInfo>();
            drbInfo->m_rlc = rlc;
            drbInfo->m_qosFlowIdentity = dtamIt->qosFlowIdentity;
            drbInfo->m_logicalChannelIdentity = dtamIt->logicalChannelIdentity;
            drbInfo->m_drbIdentity = dtamIt->drbIdentity;

            // we need PDCP only for real RLC, i.e., RLC/UM or RLC/AM
            // if we are using RLC/SM we don't care of anything above RLC
            if (rlcTypeId != NrRlcSm::GetTypeId())
            {
                Ptr<NrPdcp> pdcp = CreateObject<NrPdcp>();
                pdcp->SetRnti(m_rnti);
                pdcp->SetLcId(dtamIt->logicalChannelIdentity);
                pdcp->SetNrPdcpSapUser(m_drbPdcpSapUser);
                pdcp->SetNrRlcSapProvider(rlc->GetNrRlcSapProvider());
                rlc->SetNrRlcSapUser(pdcp->GetNrRlcSapUser());
                drbInfo->m_pdcp = pdcp;
            }

            m_qfi2DrbidMap[dtamIt->qosFlowIdentity] = dtamIt->drbIdentity;

            m_drbMap.insert(
                std::pair<uint8_t, Ptr<NrDataRadioBearerInfo>>(dtamIt->drbIdentity, drbInfo));
            NS_LOG_DEBUG("Inserting drbid " << +dtamIt->drbIdentity << " qfi "
                                            << +dtamIt->qosFlowIdentity << " to DRB map");

            m_drbCreatedTrace(m_imsi, m_cellId, m_rnti, dtamIt->drbIdentity);

            NrUeCmacSapProvider::LogicalChannelConfig lcConfig;
            lcConfig.priority = dtamIt->logicalChannelConfig.priority;
            lcConfig.fiveQi = dtamIt->logicalChannelConfig.fiveQi;
            lcConfig.prioritizedBitRateKbps = dtamIt->logicalChannelConfig.prioritizedBitRateKbps;
            lcConfig.bucketSizeDurationMs = dtamIt->logicalChannelConfig.bucketSizeDurationMs;
            lcConfig.logicalChannelGroup = dtamIt->logicalChannelConfig.logicalChannelGroup;

            NS_LOG_DEBUG(this << " UE RRC RNTI " << m_rnti << " Number Of Component Carriers "
                              << m_numberOfComponentCarriers << " lcID "
                              << (uint16_t)dtamIt->logicalChannelIdentity);
            // Call AddLc of UE component carrier manager
            std::vector<NrUeCcmRrcSapProvider::LcsConfig> lcOnCcMapping =
                m_ccmRrcSapProvider->AddLc(dtamIt->logicalChannelIdentity,
                                           lcConfig,
                                           rlc->GetNrMacSapUser());

            NS_LOG_DEBUG("Size of lcOnCcMapping vector " << lcOnCcMapping.size());
            auto itLcOnCcMapping = lcOnCcMapping.begin();
            NS_ASSERT_MSG(itLcOnCcMapping != lcOnCcMapping.end(),
                          "Component carrier manager failed to add LC for data radio bearer");

            for (itLcOnCcMapping = lcOnCcMapping.begin(); itLcOnCcMapping != lcOnCcMapping.end();
                 ++itLcOnCcMapping)
            {
                NS_LOG_DEBUG("RNTI " << m_rnti << " LCG id "
                                     << (uint16_t)itLcOnCcMapping->lcConfig.logicalChannelGroup
                                     << " ComponentCarrierId "
                                     << (uint16_t)itLcOnCcMapping->componentCarrierId);
                uint8_t index = itLcOnCcMapping->componentCarrierId;
                NrUeCmacSapProvider::LogicalChannelConfig lcConfigFromCcm =
                    itLcOnCcMapping->lcConfig;
                NrMacSapUser* msu = itLcOnCcMapping->msu;
                m_cmacSapProvider.at(index)->AddLc(dtamIt->logicalChannelIdentity,
                                                   lcConfigFromCcm,
                                                   msu);
            }
        }
        else
        {
            NS_LOG_INFO("request to modify existing DRBID");
            Ptr<NrDataRadioBearerInfo> drbInfo = drbMapIt->second;
            /// @todo currently not implemented. Would need to modify drbInfo, and then propagate
            /// changes to the MAC
        }
    }

    for (auto dtdmIt = rrcd.drbToReleaseList.begin(); dtdmIt != rrcd.drbToReleaseList.end();
         ++dtdmIt)
    {
        uint8_t drbid = *dtdmIt;
        NS_LOG_INFO(this << " IMSI " << m_imsi << " releasing DRB " << (uint32_t)drbid);
        auto it = m_drbMap.find(drbid);
        NS_ASSERT_MSG(it != m_drbMap.end(), "could not find bearer with given lcid");
        uint8_t qfi = it->second->m_qosFlowIdentity;
        m_drbMap.erase(it);
        m_qfi2DrbidMap.erase(qfi);
        NS_LOG_INFO("IMSI " << m_imsi << " releasing QFI " << +qfi);

        // Remove QoS rules from the classifier for this flow only
        m_asSapUser->DeactivateQosFlow(qfi);

        // Remove LCID using the new direct mapping: LCID = DRBID (not DRBID + 2)
        for (uint32_t i = 0; i < m_numberOfComponentCarriers; i++)
        {
            m_cmacSapProvider.at(i)->RemoveLc(nr::Drbid2Lcid(drbid));
        }
        // m_ccmRrcSapProvider->RemoveLc(nr::Drbid2Lcid(drbid));
    }
}

void
NrUeRrc::ApplyMeasConfig(NrRrcSap::MeasConfig mc)
{
    NS_LOG_FUNCTION(this);

    // perform the actions specified in 3GPP TS 36.331 section 5.5.2.1

    // 3GPP TS 36.331 section 5.5.2.4 Measurement object removal
    for (auto it = mc.measObjectToRemoveList.begin(); it != mc.measObjectToRemoveList.end(); ++it)
    {
        uint8_t measObjectId = *it;
        NS_LOG_LOGIC(this << " deleting measObjectId " << (uint32_t)measObjectId);
        m_varMeasConfig.measObjectList.erase(measObjectId);
        auto measIdIt = m_varMeasConfig.measIdList.begin();
        while (measIdIt != m_varMeasConfig.measIdList.end())
        {
            if (measIdIt->second.measObjectId == measObjectId)
            {
                uint8_t measId = measIdIt->second.measId;
                NS_ASSERT(measId == measIdIt->first);
                NS_LOG_LOGIC(this << " deleting measId " << (uint32_t)measId
                                  << " because referring to measObjectId "
                                  << (uint32_t)measObjectId);
                // note: postfix operator preserves iterator validity
                m_varMeasConfig.measIdList.erase(measIdIt++);
                VarMeasReportListClear(measId);
            }
            else
            {
                ++measIdIt;
            }
        }
    }

    // 3GPP TS 36.331 section 5.5.2.5  Measurement object addition/ modification
    for (auto it = mc.measObjectToAddModList.begin(); it != mc.measObjectToAddModList.end(); ++it)
    {
        // simplifying assumptions
        NS_ASSERT_MSG(it->measObjectEutra.cellsToRemoveList.empty(),
                      "cellsToRemoveList not supported");
        NS_ASSERT_MSG(it->measObjectEutra.cellsToAddModList.empty(),
                      "cellsToAddModList not supported");
        NS_ASSERT_MSG(it->measObjectEutra.cellsToRemoveList.empty(),
                      "blackCellsToRemoveList not supported");
        NS_ASSERT_MSG(it->measObjectEutra.blackCellsToAddModList.empty(),
                      "blackCellsToAddModList not supported");
        NS_ASSERT_MSG(it->measObjectEutra.haveCellForWhichToReportCGI == false,
                      "cellForWhichToReportCGI is not supported");

        uint8_t measObjectId = it->measObjectId;
        auto measObjectIt = m_varMeasConfig.measObjectList.find(measObjectId);
        if (measObjectIt != m_varMeasConfig.measObjectList.end())
        {
            NS_LOG_LOGIC("measObjectId " << (uint32_t)measObjectId << " exists, updating entry");
            measObjectIt->second = *it;
            for (auto measIdIt = m_varMeasConfig.measIdList.begin();
                 measIdIt != m_varMeasConfig.measIdList.end();
                 ++measIdIt)
            {
                if (measIdIt->second.measObjectId == measObjectId)
                {
                    uint8_t measId = measIdIt->second.measId;
                    NS_LOG_LOGIC(this << " found measId " << (uint32_t)measId
                                      << " referring to measObjectId " << (uint32_t)measObjectId);
                    VarMeasReportListClear(measId);
                }
            }
        }
        else
        {
            NS_LOG_LOGIC("measObjectId " << (uint32_t)measObjectId << " is new, adding entry");
            m_varMeasConfig.measObjectList[measObjectId] = *it;
        }
    }

    // 3GPP TS 36.331 section 5.5.2.6 Reporting configuration removal
    for (auto it = mc.reportConfigToRemoveList.begin(); it != mc.reportConfigToRemoveList.end();
         ++it)
    {
        uint8_t reportConfigId = *it;
        NS_LOG_LOGIC(this << " deleting reportConfigId " << (uint32_t)reportConfigId);
        m_varMeasConfig.reportConfigList.erase(reportConfigId);
        auto measIdIt = m_varMeasConfig.measIdList.begin();
        while (measIdIt != m_varMeasConfig.measIdList.end())
        {
            if (measIdIt->second.reportConfigId == reportConfigId)
            {
                uint8_t measId = measIdIt->second.measId;
                NS_ASSERT(measId == measIdIt->first);
                NS_LOG_LOGIC(this << " deleting measId " << (uint32_t)measId
                                  << " because referring to reportConfigId "
                                  << (uint32_t)reportConfigId);
                // note: postfix operator preserves iterator validity
                m_varMeasConfig.measIdList.erase(measIdIt++);
                VarMeasReportListClear(measId);
            }
            else
            {
                ++measIdIt;
            }
        }
    }

    // 3GPP TS 36.331 section 5.5.2.7 Reporting configuration addition/ modification
    for (auto it = mc.reportConfigToAddModList.begin(); it != mc.reportConfigToAddModList.end();
         ++it)
    {
        // simplifying assumptions
        // ntn-patch 22: patch 08 supports PERIODICAL reports (TS 38.331 5.5.4.1) but left this
        // assert, so a periodical report config aborted assert-enabled builds.
        NS_ASSERT_MSG(it->reportConfigEutra.triggerType == NrRrcSap::ReportConfigEutra::EVENT ||
                          it->reportConfigEutra.triggerType ==
                              NrRrcSap::ReportConfigEutra::PERIODICAL,
                      "trigger type must be EVENT or PERIODICAL");

        uint8_t reportConfigId = it->reportConfigId;
        auto reportConfigIt = m_varMeasConfig.reportConfigList.find(reportConfigId);
        if (reportConfigIt != m_varMeasConfig.reportConfigList.end())
        {
            NS_LOG_LOGIC("reportConfigId " << (uint32_t)reportConfigId
                                           << " exists, updating entry");
            m_varMeasConfig.reportConfigList[reportConfigId] = *it;
            for (auto measIdIt = m_varMeasConfig.measIdList.begin();
                 measIdIt != m_varMeasConfig.measIdList.end();
                 ++measIdIt)
            {
                if (measIdIt->second.reportConfigId == reportConfigId)
                {
                    uint8_t measId = measIdIt->second.measId;
                    NS_LOG_LOGIC(this << " found measId " << (uint32_t)measId
                                      << " referring to reportConfigId "
                                      << (uint32_t)reportConfigId);
                    VarMeasReportListClear(measId);
                }
            }
        }
        else
        {
            NS_LOG_LOGIC("reportConfigId " << (uint32_t)reportConfigId << " is new, adding entry");
            m_varMeasConfig.reportConfigList[reportConfigId] = *it;
        }
    }

    // 3GPP TS 36.331 section 5.5.2.8 Quantity configuration
    if (mc.haveQuantityConfig)
    {
        NS_LOG_LOGIC(this << " setting quantityConfig");
        m_varMeasConfig.quantityConfig = mc.quantityConfig;
        // Convey the filter coefficient to PHY layer so it can configure the power control
        // parameter
        for (uint16_t i = 0; i < m_numberOfComponentCarriers; i++)
        {
            m_cphySapProvider.at(i)->SetRsrpFilterCoefficient(
                mc.quantityConfig.filterCoefficientRSRP);
        }
        // we calculate here the coefficient a used for Layer 3 filtering, see 3GPP TS 36.331
        // section 5.5.3.2
        m_varMeasConfig.aRsrp = std::pow(0.5, mc.quantityConfig.filterCoefficientRSRP / 4.0);
        m_varMeasConfig.aRsrq = std::pow(0.5, mc.quantityConfig.filterCoefficientRSRQ / 4.0);
        NS_LOG_LOGIC(this << " new filter coefficients: aRsrp=" << m_varMeasConfig.aRsrp
                          << ", aRsrq=" << m_varMeasConfig.aRsrq);

        for (auto measIdIt = m_varMeasConfig.measIdList.begin();
             measIdIt != m_varMeasConfig.measIdList.end();
             ++measIdIt)
        {
            VarMeasReportListClear(measIdIt->second.measId);
        }
    }

    // 3GPP TS 36.331 section 5.5.2.2 Measurement identity removal
    for (auto it = mc.measIdToRemoveList.begin(); it != mc.measIdToRemoveList.end(); ++it)
    {
        uint8_t measId = *it;
        NS_LOG_LOGIC(this << " deleting measId " << (uint32_t)measId);
        m_varMeasConfig.measIdList.erase(measId);
        VarMeasReportListClear(measId);

        // removing time-to-trigger queues
        m_enteringTriggerQueue.erase(measId);
        m_leavingTriggerQueue.erase(measId);
    }

    // 3GPP TS 36.331 section 5.5.2.3 Measurement identity addition/ modification
    for (auto it = mc.measIdToAddModList.begin(); it != mc.measIdToAddModList.end(); ++it)
    {
        NS_LOG_LOGIC(this << " measId " << (uint32_t)it->measId
                          << " (measObjectId=" << (uint32_t)it->measObjectId
                          << ", reportConfigId=" << (uint32_t)it->reportConfigId << ")");
        NS_ASSERT(m_varMeasConfig.measObjectList.find(it->measObjectId) !=
                  m_varMeasConfig.measObjectList.end());
        NS_ASSERT(m_varMeasConfig.reportConfigList.find(it->reportConfigId) !=
                  m_varMeasConfig.reportConfigList.end());
        m_varMeasConfig.measIdList[it->measId] = *it; // side effect: create new entry if not exists
        auto measReportIt = m_varMeasReportList.find(it->measId);
        if (measReportIt != m_varMeasReportList.end())
        {
            measReportIt->second.periodicReportTimer.Cancel();
            m_varMeasReportList.erase(measReportIt);
        }
        // ntn-patch 08: PERIODICAL report configurations are supported
        // (TS 38.331 5.5.4.1 reportType periodical); see
        // MeasurementReportTriggering.

        // new empty queues for time-to-trigger
        std::list<PendingTrigger_t> s;
        m_enteringTriggerQueue[it->measId] = s;
        m_leavingTriggerQueue[it->measId] = s;
    }

    if (mc.haveMeasGapConfig)
    {
        NS_FATAL_ERROR("measurement gaps are currently not supported");
    }

    if (mc.haveSmeasure)
    {
        NS_FATAL_ERROR("s-measure is currently not supported");
    }

    if (mc.haveSpeedStatePars)
    {
        NS_FATAL_ERROR("SpeedStatePars are currently not supported");
    }
}

void
NrUeRrc::SaveUeMeasurements(uint16_t cellId,
                            double rsrp,
                            double rsrq,
                            bool useLayer3Filtering,
                            uint8_t componentCarrierId)
{
    NS_LOG_FUNCTION(this << cellId << +componentCarrierId << rsrp << rsrq << useLayer3Filtering);

    // A non-finite RSRP (e.g. -inf dBm from a zero-power sample) would poison
    // the Layer-3 filter below permanently: F = (1-a)F + aM stays -inf for
    // every subsequent finite M. Ignore such degenerate measurements.
    if (!std::isfinite(rsrp))
    {
        NS_LOG_WARN(this << " ignoring non-finite RSRP measurement of cell " << cellId);
        return;
    }

    // Cell<->BWP bookkeeping: remember which carrier this cell was measured on,
    // so a later broadcast (MIB) from this cell can be routed to the BWP that
    // is actually tuned to its carrier (see GetCellBwpId / DoRecvMIB).
    const uint32_t arfcn = m_cphySapProvider.at(componentCarrierId)->GetArfcn();
    TrackCellArfcn(cellId, arfcn);

    // Per-carrier RSRP, tracked ADDITIVELY (same L3 alpha as the cellId-keyed
    // store). This is the only RSRP that can distinguish two BWPs of the same
    // cell (which share a cellId), so it is what drives the same-cell switch.
    {
        auto rit = m_rsrpPerArfcn.find(arfcn);
        if (rit == m_rsrpPerArfcn.end() || !useLayer3Filtering)
        {
            m_rsrpPerArfcn[arfcn] = rsrp;
        }
        else
        {
            rit->second = (1 - m_varMeasConfig.aRsrp) * rit->second + m_varMeasConfig.aRsrp * rsrp;
        }
    }

    auto storedMeasIt = m_storedMeasValues.find(cellId);

    if (storedMeasIt != m_storedMeasValues.end())
    {
        if (useLayer3Filtering)
        {
            // F_n = (1-a) F_{n-1} + a M_n
            storedMeasIt->second.rsrp = (1 - m_varMeasConfig.aRsrp) * storedMeasIt->second.rsrp +
                                        m_varMeasConfig.aRsrp * rsrp;

            if (std::isnan(storedMeasIt->second.rsrq))
            {
                // the previous RSRQ measurements provided UE PHY are invalid
                storedMeasIt->second.rsrq = rsrq; // replace it with unfiltered value
            }
            else
            {
                storedMeasIt->second.rsrq =
                    (1 - m_varMeasConfig.aRsrq) * storedMeasIt->second.rsrq +
                    m_varMeasConfig.aRsrq * rsrq;
            }
        }
        else
        {
            storedMeasIt->second.rsrp = rsrp;
            storedMeasIt->second.rsrq = rsrq;
        }
    }
    else
    {
        // first value is always unfiltered
        MeasValues v;
        v.rsrp = rsrp;
        v.rsrq = rsrq;
        v.carrierFreq = m_cphySapProvider.at(componentCarrierId)->GetArfcn();

        std::pair<uint16_t, MeasValues> val(cellId, v);
        auto ret = m_storedMeasValues.insert(val);
        NS_ASSERT_MSG(ret.second == true, "element already existed");
        storedMeasIt = ret.first;
    }

    NS_LOG_DEBUG(this << " IMSI " << m_imsi << " state " << ToString(m_state) << ", measured cell "
                      << cellId << ", BWPid " << +componentCarrierId << ", arfcn "
                      << storedMeasIt->second.carrierFreq << ", new RSRP " << rsrp << " stored "
                      << storedMeasIt->second.rsrp << ", new RSRQ " << rsrq << " stored "
                      << storedMeasIt->second.rsrq);

    // Same-cell BWP-switch policy: after refreshing per-carrier RSRP, check
    // whether a better same-cell BWP exists and switch the primary BWP to it.
    EvaluateSameCellBwpSwitch();

} // end of void SaveUeMeasurements

void
NrUeRrc::MeasurementReportTriggering(uint8_t measId)
{
    NS_LOG_FUNCTION(this << (uint16_t)measId);

    auto measIdIt = m_varMeasConfig.measIdList.find(measId);
    NS_ASSERT(measIdIt != m_varMeasConfig.measIdList.end());
    NS_ASSERT(measIdIt->first == measIdIt->second.measId);

    auto reportConfigIt = m_varMeasConfig.reportConfigList.find(measIdIt->second.reportConfigId);
    NS_ASSERT(reportConfigIt != m_varMeasConfig.reportConfigList.end());
    NrRrcSap::ReportConfigEutra& reportConfigEutra = reportConfigIt->second.reportConfigEutra;

    auto measObjectIt = m_varMeasConfig.measObjectList.find(measIdIt->second.measObjectId);
    NS_ASSERT(measObjectIt != m_varMeasConfig.measObjectList.end());
    NrRrcSap::MeasObjectEutra& measObjectEutra = measObjectIt->second.measObjectEutra;

    auto measReportIt = m_varMeasReportList.find(measId);
    bool isMeasIdInReportList = (measReportIt != m_varMeasReportList.end());

    // ntn-patch 08: TS 38.331 5.5.4.1, reportType periodical. When a first
    // measurement result is available, include a reporting entry for this
    // measId with numberOfReportsSent = 0 and start the reporting procedure
    // (5.5.5); SendMeasurementReport then re-arms itself every reportInterval.
    // The applicable cells are all measured cells on the frequency other than
    // the serving cell, refreshed on every L1 report so newly detected cells
    // are included (5.5.5: purpose reportStrongestCells, maxReportCells).
    if (reportConfigEutra.triggerType == NrRrcSap::ReportConfigEutra::PERIODICAL)
    {
        if (m_storedMeasValues.find(m_cellId) == m_storedMeasValues.end())
        {
            return; // no serving-cell result yet
        }
        if (!isMeasIdInReportList)
        {
            VarMeasReport r;
            r.measId = measId;
            r.numberOfReportsSent = 0;
            measReportIt = m_varMeasReportList.emplace(measId, r).first;
        }
        measReportIt->second.cellsTriggeredList.clear();
        for (const auto& [cellId, values] : m_storedMeasValues)
        {
            if (cellId != m_cellId)
            {
                measReportIt->second.cellsTriggeredList.insert(cellId);
            }
        }
        if (!measReportIt->second.periodicReportTimer.IsPending())
        {
            measReportIt->second.periodicReportTimer =
                Simulator::Schedule(NR_UE_MEASUREMENT_REPORT_DELAY,
                                    &NrUeRrc::SendMeasurementReport,
                                    this,
                                    measId);
        }
        return;
    }
    NS_ASSERT_MSG(reportConfigEutra.triggerType == NrRrcSap::ReportConfigEutra::EVENT,
                  "only triggerType == event is supported");
    // only EUTRA is supported, no need to check for it

    NS_LOG_LOGIC(this << " considering measId " << (uint32_t)measId);
    bool eventEntryCondApplicable = false;
    bool eventLeavingCondApplicable = false;
    ConcernedCells_t concernedCellsEntry;
    ConcernedCells_t concernedCellsLeaving;

    /*
     * Find which serving cell corresponds to measObjectEutra.carrierFreq
     * It is used, for example, by A1 event:
     * See TS 36.331 5.5.4.2: "for this measurement, consider the primary or
     * secondary cell that is configured on the frequency indicated in the
     * associated measObjectEUTRA to be the serving cell"
     */
    uint16_t servingCellId = 0;
    for (auto cphySapProvider : m_cphySapProvider)
    {
        if (cphySapProvider->GetArfcn() == measObjectEutra.carrierFreq)
        {
            servingCellId = cphySapProvider->GetCellId();
        }
    }

    /*
     * Events A1 and A2 evaluate the serving cell configured on the measObject's
     * frequency; if there is none (e.g. an inter-frequency neighbour object),
     * there is nothing to evaluate for those events.
     * Events A3, A4 and A5 instead compare neighbour cells against the PCell
     * (m_cellId) and therefore must be evaluated even when the measObject
     * describes an inter-frequency neighbour, for which no serving cell exists
     * on that frequency. Otherwise inter-frequency handover could never be
     * triggered.
     */
    bool isServingCellEvent =
        (reportConfigEutra.eventId == NrRrcSap::ReportConfigEutra::EVENT_A1) ||
        (reportConfigEutra.eventId == NrRrcSap::ReportConfigEutra::EVENT_A2);
    if (servingCellId == 0 && isServingCellEvent)
    {
        return;
    }

    switch (reportConfigEutra.eventId)
    {
    case NrRrcSap::ReportConfigEutra::EVENT_A1: {
        /*
         * Event A1 (Serving becomes better than threshold)
         * Please refer to 3GPP TS 36.331 Section 5.5.4.2
         */

        double ms;     // Ms, the measurement result of the serving cell
        double thresh; // Thresh, the threshold parameter for this event
        // Hys, the hysteresis parameter for this event.
        double hys =
            nr::EutranMeasurementMapping::IeValue2ActualHysteresis(reportConfigEutra.hysteresis);

        switch (reportConfigEutra.triggerQuantity)
        {
        case NrRrcSap::ReportConfigEutra::RSRP:
            ms = m_storedMeasValues[servingCellId].rsrp;

            NS_ASSERT(reportConfigEutra.threshold1.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRP);
            thresh =
                nr::EutranMeasurementMapping::RsrpRange2Dbm(reportConfigEutra.threshold1.range);
            break;
        case NrRrcSap::ReportConfigEutra::RSRQ:
            ms = m_storedMeasValues[servingCellId].rsrq;
            NS_ASSERT(reportConfigEutra.threshold1.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRQ);
            thresh = nr::EutranMeasurementMapping::RsrqRange2Db(reportConfigEutra.threshold1.range);
            break;
        default:
            NS_FATAL_ERROR("unsupported triggerQuantity");
            break;
        }

        // Inequality A1-1 (Entering condition): Ms - Hys > Thresh
        bool entryCond = ms - hys > thresh;

        if (entryCond)
        {
            if (!isMeasIdInReportList)
            {
                concernedCellsEntry.push_back(servingCellId);
                eventEntryCondApplicable = true;
            }
            else
            {
                /*
                 * This is to check that the triggered cell recorded in the
                 * VarMeasReportList is the serving cell.
                 */
                NS_ASSERT(measReportIt->second.cellsTriggeredList.find(servingCellId) !=
                          measReportIt->second.cellsTriggeredList.end());
            }
        }
        else if (reportConfigEutra.timeToTrigger > 0)
        {
            CancelEnteringTrigger(measId);
        }

        // Inequality A1-2 (Leaving condition): Ms + Hys < Thresh
        bool leavingCond = ms + hys < thresh;

        if (leavingCond)
        {
            if (isMeasIdInReportList)
            {
                /*
                 * This is to check that the triggered cell recorded in the
                 * VarMeasReportList is the serving cell.
                 */
                NS_ASSERT(measReportIt->second.cellsTriggeredList.find(m_cellId) !=
                          measReportIt->second.cellsTriggeredList.end());
                concernedCellsLeaving.push_back(m_cellId);
                eventLeavingCondApplicable = true;
            }
        }
        else if (reportConfigEutra.timeToTrigger > 0)
        {
            CancelLeavingTrigger(measId);
        }

        NS_LOG_LOGIC(this << " event A1: serving cell " << servingCellId << " ms=" << ms
                          << " thresh=" << thresh << " entryCond=" << entryCond
                          << " leavingCond=" << leavingCond);

    } // end of case NrRrcSap::ReportConfigEutra::EVENT_A1

    break;

    case NrRrcSap::ReportConfigEutra::EVENT_A2: {
        /*
         * Event A2 (Serving becomes worse than threshold)
         * Please refer to 3GPP TS 36.331 Section 5.5.4.3
         */

        double ms;     // Ms, the measurement result of the serving cell
        double thresh; // Thresh, the threshold parameter for this event
        // Hys, the hysteresis parameter for this event.
        double hys =
            nr::EutranMeasurementMapping::IeValue2ActualHysteresis(reportConfigEutra.hysteresis);

        switch (reportConfigEutra.triggerQuantity)
        {
        case NrRrcSap::ReportConfigEutra::RSRP:
            ms = m_storedMeasValues[servingCellId].rsrp;
            NS_ASSERT(reportConfigEutra.threshold1.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRP);
            thresh =
                nr::EutranMeasurementMapping::RsrpRange2Dbm(reportConfigEutra.threshold1.range);
            break;
        case NrRrcSap::ReportConfigEutra::RSRQ:
            ms = m_storedMeasValues[servingCellId].rsrq;
            NS_ASSERT(reportConfigEutra.threshold1.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRQ);
            thresh = nr::EutranMeasurementMapping::RsrqRange2Db(reportConfigEutra.threshold1.range);
            break;
        default:
            NS_FATAL_ERROR("unsupported triggerQuantity");
            break;
        }

        // Inequality A2-1 (Entering condition): Ms + Hys < Thresh
        bool entryCond = ms + hys < thresh;

        if (entryCond)
        {
            if (!isMeasIdInReportList)
            {
                concernedCellsEntry.push_back(servingCellId);
                eventEntryCondApplicable = true;
            }
            else
            {
                /*
                 * This is to check that the triggered cell recorded in the
                 * VarMeasReportList is the serving cell.
                 */
                NS_ASSERT(measReportIt->second.cellsTriggeredList.find(servingCellId) !=
                          measReportIt->second.cellsTriggeredList.end());
            }
        }
        else if (reportConfigEutra.timeToTrigger > 0)
        {
            CancelEnteringTrigger(measId);
        }

        // Inequality A2-2 (Leaving condition): Ms - Hys > Thresh
        bool leavingCond = ms - hys > thresh;

        if (leavingCond)
        {
            if (isMeasIdInReportList)
            {
                /*
                 * This is to check that the triggered cell recorded in the
                 * VarMeasReportList is the serving cell.
                 */
                NS_ASSERT(measReportIt->second.cellsTriggeredList.find(servingCellId) !=
                          measReportIt->second.cellsTriggeredList.end());
                concernedCellsLeaving.push_back(servingCellId);
                eventLeavingCondApplicable = true;
            }
        }
        else if (reportConfigEutra.timeToTrigger > 0)
        {
            CancelLeavingTrigger(measId);
        }

        NS_LOG_LOGIC(this << " event A2: serving cell " << servingCellId << " ms=" << ms
                          << " thresh=" << thresh << " entryCond=" << entryCond
                          << " leavingCond=" << leavingCond);

    } // end of case NrRrcSap::ReportConfigEutra::EVENT_A2

    break;

    case NrRrcSap::ReportConfigEutra::EVENT_A3: {
        /*
         * Event A3 (Neighbour becomes offset better than PCell)
         * Please refer to 3GPP TS 36.331 Section 5.5.4.4
         */

        double mn; // Mn, the measurement result of the neighbouring cell
        double ofn = measObjectEutra
                         .offsetFreq; // Ofn, the frequency specific offset of the frequency of the
        double ocn = 0.0;             // Ocn, the cell specific offset of the neighbour cell
        double mp;                    // Mp, the measurement result of the PCell
        double ofp = measObjectEutra
                         .offsetFreq; // Ofp, the frequency specific offset of the primary frequency
        // Ocp, the cell specific offset of the PCell (e.g. pico cell-range expansion bias)
        double ocp = GetCellIndividualOffset(m_cellId);
        // Off, the offset parameter for this event.
        double off =
            nr::EutranMeasurementMapping::IeValue2ActualA3Offset(reportConfigEutra.a3Offset);
        // Hys, the hysteresis parameter for this event.
        double hys =
            nr::EutranMeasurementMapping::IeValue2ActualHysteresis(reportConfigEutra.hysteresis);

        switch (reportConfigEutra.triggerQuantity)
        {
        case NrRrcSap::ReportConfigEutra::RSRP:
            mp = m_storedMeasValues[m_cellId].rsrp;
            NS_ASSERT(reportConfigEutra.threshold1.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRP);
            break;
        case NrRrcSap::ReportConfigEutra::RSRQ:
            mp = m_storedMeasValues[m_cellId].rsrq;
            NS_ASSERT(reportConfigEutra.threshold1.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRQ);
            break;
        default:
            NS_FATAL_ERROR("unsupported triggerQuantity");
            break;
        }

        for (auto storedMeasIt = m_storedMeasValues.begin();
             storedMeasIt != m_storedMeasValues.end();
             ++storedMeasIt)
        {
            uint16_t cellId = storedMeasIt->first;
            if (cellId == m_cellId)
            {
                continue;
            }

            // Only cell(s) on the frequency indicated in the associated measObject can trigger
            // event.
            if (m_storedMeasValues.at(cellId).carrierFreq != measObjectEutra.carrierFreq)
            {
                continue;
            }

            // Ocn, the cell specific offset of this neighbour cell (e.g. pico
            // cell-range expansion bias). Re-evaluated per neighbour.
            ocn = GetCellIndividualOffset(cellId);

            switch (reportConfigEutra.triggerQuantity)
            {
            case NrRrcSap::ReportConfigEutra::RSRP:
                mn = storedMeasIt->second.rsrp;
                break;
            case NrRrcSap::ReportConfigEutra::RSRQ:
                mn = storedMeasIt->second.rsrq;
                break;
            default:
                NS_FATAL_ERROR("unsupported triggerQuantity");
                break;
            }

            bool hasTriggered =
                isMeasIdInReportList && (measReportIt->second.cellsTriggeredList.find(cellId) !=
                                         measReportIt->second.cellsTriggeredList.end());

            // Inequality A3-1 (Entering condition): Mn + Ofn + Ocn - Hys > Mp + Ofp + Ocp + Off
            bool entryCond = mn + ofn + ocn - hys > mp + ofp + ocp + off;

            if (entryCond)
            {
                if (!hasTriggered)
                {
                    concernedCellsEntry.push_back(cellId);
                    eventEntryCondApplicable = true;
                }
            }
            else if (reportConfigEutra.timeToTrigger > 0)
            {
                CancelEnteringTrigger(measId, cellId);
            }

            // Inequality A3-2 (Leaving condition): Mn + Ofn + Ocn + Hys < Mp + Ofp + Ocp + Off
            bool leavingCond = mn + ofn + ocn + hys < mp + ofp + ocp + off;

            if (leavingCond)
            {
                if (hasTriggered)
                {
                    concernedCellsLeaving.push_back(cellId);
                    eventLeavingCondApplicable = true;
                }
            }
            else if (reportConfigEutra.timeToTrigger > 0)
            {
                CancelLeavingTrigger(measId, cellId);
            }

            NS_LOG_LOGIC(this << " event A3: neighbor cell " << cellId << " mn=" << mn
                              << " mp=" << mp << " offset=" << off << " entryCond=" << entryCond
                              << " leavingCond=" << leavingCond);

        } // end of for (storedMeasIt)

    } // end of case NrRrcSap::ReportConfigEutra::EVENT_A3

    break;

    case NrRrcSap::ReportConfigEutra::EVENT_A4: {
        /*
         * Event A4 (Neighbour becomes better than threshold)
         * Please refer to 3GPP TS 36.331 Section 5.5.4.5
         */

        double mn; // Mn, the measurement result of the neighbouring cell
        double ofn = measObjectEutra
                         .offsetFreq; // Ofn, the frequency specific offset of the frequency of the
        double ocn = 0.0;             // Ocn, the cell specific offset of the neighbour cell
        double thresh;                // Thresh, the threshold parameter for this event
        // Hys, the hysteresis parameter for this event.
        double hys =
            nr::EutranMeasurementMapping::IeValue2ActualHysteresis(reportConfigEutra.hysteresis);

        switch (reportConfigEutra.triggerQuantity)
        {
        case NrRrcSap::ReportConfigEutra::RSRP:
            NS_ASSERT(reportConfigEutra.threshold1.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRP);
            thresh =
                nr::EutranMeasurementMapping::RsrpRange2Dbm(reportConfigEutra.threshold1.range);
            break;
        case NrRrcSap::ReportConfigEutra::RSRQ:
            NS_ASSERT(reportConfigEutra.threshold1.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRQ);
            thresh = nr::EutranMeasurementMapping::RsrqRange2Db(reportConfigEutra.threshold1.range);
            break;
        default:
            NS_FATAL_ERROR("unsupported triggerQuantity");
            break;
        }

        for (auto storedMeasIt = m_storedMeasValues.begin();
             storedMeasIt != m_storedMeasValues.end();
             ++storedMeasIt)
        {
            uint16_t cellId = storedMeasIt->first;
            if (cellId == m_cellId)
            {
                continue;
            }

            switch (reportConfigEutra.triggerQuantity)
            {
            case NrRrcSap::ReportConfigEutra::RSRP:
                mn = storedMeasIt->second.rsrp;
                break;
            case NrRrcSap::ReportConfigEutra::RSRQ:
                mn = storedMeasIt->second.rsrq;
                break;
            default:
                NS_FATAL_ERROR("unsupported triggerQuantity");
                break;
            }

            bool hasTriggered =
                isMeasIdInReportList && (measReportIt->second.cellsTriggeredList.find(cellId) !=
                                         measReportIt->second.cellsTriggeredList.end());

            // Inequality A4-1 (Entering condition): Mn + Ofn + Ocn - Hys > Thresh
            bool entryCond = mn + ofn + ocn - hys > thresh;

            if (entryCond)
            {
                if (!hasTriggered)
                {
                    concernedCellsEntry.push_back(cellId);
                    eventEntryCondApplicable = true;
                }
            }
            else if (reportConfigEutra.timeToTrigger > 0)
            {
                CancelEnteringTrigger(measId, cellId);
            }

            // Inequality A4-2 (Leaving condition): Mn + Ofn + Ocn + Hys < Thresh
            bool leavingCond = mn + ofn + ocn + hys < thresh;

            if (leavingCond)
            {
                if (hasTriggered)
                {
                    concernedCellsLeaving.push_back(cellId);
                    eventLeavingCondApplicable = true;
                }
            }
            else if (reportConfigEutra.timeToTrigger > 0)
            {
                CancelLeavingTrigger(measId, cellId);
            }

            NS_LOG_LOGIC(this << " event A4: neighbor cell " << cellId << " mn=" << mn
                              << " thresh=" << thresh << " entryCond=" << entryCond
                              << " leavingCond=" << leavingCond);

        } // end of for (storedMeasIt)

    } // end of case NrRrcSap::ReportConfigEutra::EVENT_A4

    break;

    case NrRrcSap::ReportConfigEutra::EVENT_A5: {
        /*
         * Event A5 (PCell becomes worse than threshold1 and neighbour
         * becomes better than threshold2)
         * Please refer to 3GPP TS 36.331 Section 5.5.4.6
         */

        double mp; // Mp, the measurement result of the PCell
        double mn; // Mn, the measurement result of the neighbouring cell
        double ofn = measObjectEutra
                         .offsetFreq; // Ofn, the frequency specific offset of the frequency of the
        double ocn = 0.0;             // Ocn, the cell specific offset of the neighbour cell
        double thresh1;               // Thresh1, the threshold parameter for this event
        double thresh2;               // Thresh2, the threshold parameter for this event
        // Hys, the hysteresis parameter for this event.
        double hys =
            nr::EutranMeasurementMapping::IeValue2ActualHysteresis(reportConfigEutra.hysteresis);

        switch (reportConfigEutra.triggerQuantity)
        {
        case NrRrcSap::ReportConfigEutra::RSRP:
            mp = m_storedMeasValues[m_cellId].rsrp;
            NS_ASSERT(reportConfigEutra.threshold1.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRP);
            NS_ASSERT(reportConfigEutra.threshold2.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRP);
            thresh1 =
                nr::EutranMeasurementMapping::RsrpRange2Dbm(reportConfigEutra.threshold1.range);
            thresh2 =
                nr::EutranMeasurementMapping::RsrpRange2Dbm(reportConfigEutra.threshold2.range);
            break;
        case NrRrcSap::ReportConfigEutra::RSRQ:
            mp = m_storedMeasValues[m_cellId].rsrq;
            NS_ASSERT(reportConfigEutra.threshold1.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRQ);
            NS_ASSERT(reportConfigEutra.threshold2.choice ==
                      NrRrcSap::ThresholdEutra::THRESHOLD_RSRQ);
            thresh1 =
                nr::EutranMeasurementMapping::RsrqRange2Db(reportConfigEutra.threshold1.range);
            thresh2 =
                nr::EutranMeasurementMapping::RsrqRange2Db(reportConfigEutra.threshold2.range);
            break;
        default:
            NS_FATAL_ERROR("unsupported triggerQuantity");
            break;
        }

        // Inequality A5-1 (Entering condition 1): Mp + Hys < Thresh1
        bool entryCond = mp + hys < thresh1;

        if (entryCond)
        {
            for (auto storedMeasIt = m_storedMeasValues.begin();
                 storedMeasIt != m_storedMeasValues.end();
                 ++storedMeasIt)
            {
                uint16_t cellId = storedMeasIt->first;
                if (cellId == m_cellId)
                {
                    continue;
                }

                switch (reportConfigEutra.triggerQuantity)
                {
                case NrRrcSap::ReportConfigEutra::RSRP:
                    mn = storedMeasIt->second.rsrp;
                    break;
                case NrRrcSap::ReportConfigEutra::RSRQ:
                    mn = storedMeasIt->second.rsrq;
                    break;
                default:
                    NS_FATAL_ERROR("unsupported triggerQuantity");
                    break;
                }

                bool hasTriggered =
                    isMeasIdInReportList && (measReportIt->second.cellsTriggeredList.find(cellId) !=
                                             measReportIt->second.cellsTriggeredList.end());

                // Inequality A5-2 (Entering condition 2): Mn + Ofn + Ocn - Hys > Thresh2

                entryCond = mn + ofn + ocn - hys > thresh2;

                if (entryCond)
                {
                    if (!hasTriggered)
                    {
                        concernedCellsEntry.push_back(cellId);
                        eventEntryCondApplicable = true;
                    }
                }
                else if (reportConfigEutra.timeToTrigger > 0)
                {
                    CancelEnteringTrigger(measId, cellId);
                }

                NS_LOG_LOGIC(this << " event A5: neighbor cell " << cellId << " mn=" << mn
                                  << " mp=" << mp << " thresh2=" << thresh2
                                  << " thresh1=" << thresh1 << " entryCond=" << entryCond);

            } // end of for (storedMeasIt)

        } // end of if (entryCond)
        else
        {
            NS_LOG_LOGIC(this << " event A5: serving cell " << m_cellId << " mp=" << mp
                              << " thresh1=" << thresh1 << " entryCond=" << entryCond);

            if (reportConfigEutra.timeToTrigger > 0)
            {
                CancelEnteringTrigger(measId);
            }
        }

        if (isMeasIdInReportList)
        {
            // Inequality A5-3 (Leaving condition 1): Mp - Hys > Thresh1
            bool leavingCond = mp - hys > thresh1;

            if (leavingCond)
            {
                if (reportConfigEutra.timeToTrigger == 0)
                {
                    // leaving condition #2 does not have to be checked

                    for (auto storedMeasIt = m_storedMeasValues.begin();
                         storedMeasIt != m_storedMeasValues.end();
                         ++storedMeasIt)
                    {
                        uint16_t cellId = storedMeasIt->first;
                        if (cellId == m_cellId)
                        {
                            continue;
                        }

                        if (measReportIt->second.cellsTriggeredList.find(cellId) !=
                            measReportIt->second.cellsTriggeredList.end())
                        {
                            concernedCellsLeaving.push_back(cellId);
                            eventLeavingCondApplicable = true;
                        }
                    }
                } // end of if (reportConfigEutra.timeToTrigger == 0)
                else
                {
                    // leaving condition #2 has to be checked to cancel time-to-trigger

                    for (auto storedMeasIt = m_storedMeasValues.begin();
                         storedMeasIt != m_storedMeasValues.end();
                         ++storedMeasIt)
                    {
                        uint16_t cellId = storedMeasIt->first;
                        if (cellId == m_cellId)
                        {
                            continue;
                        }

                        if (measReportIt->second.cellsTriggeredList.find(cellId) !=
                            measReportIt->second.cellsTriggeredList.end())
                        {
                            switch (reportConfigEutra.triggerQuantity)
                            {
                            case NrRrcSap::ReportConfigEutra::RSRP:
                                mn = storedMeasIt->second.rsrp;
                                break;
                            case NrRrcSap::ReportConfigEutra::RSRQ:
                                mn = storedMeasIt->second.rsrq;
                                break;
                            default:
                                NS_FATAL_ERROR("unsupported triggerQuantity");
                                break;
                            }

                            // Inequality A5-4 (Leaving condition 2): Mn + Ofn + Ocn + Hys < Thresh2

                            leavingCond = mn + ofn + ocn + hys < thresh2;

                            if (!leavingCond)
                            {
                                CancelLeavingTrigger(measId, cellId);
                            }

                            /*
                             * Whatever the result of leaving condition #2, this
                             * cell is still "in", because leaving condition #1
                             * is already true.
                             */
                            concernedCellsLeaving.push_back(cellId);
                            eventLeavingCondApplicable = true;

                            NS_LOG_LOGIC(this << " event A5: neighbor cell " << cellId
                                              << " mn=" << mn << " mp=" << mp
                                              << " thresh2=" << thresh2 << " thresh1=" << thresh1
                                              << " leavingCond=" << leavingCond);

                        } // end of if (measReportIt->second.cellsTriggeredList.find (cellId)
                          //            != measReportIt->second.cellsTriggeredList.end ())

                    } // end of for (storedMeasIt)

                } // end of else of if (reportConfigEutra.timeToTrigger == 0)

                NS_LOG_LOGIC(this << " event A5: serving cell " << m_cellId << " mp=" << mp
                                  << " thresh1=" << thresh1 << " leavingCond=" << leavingCond);

            } // end of if (leavingCond)
            else
            {
                if (reportConfigEutra.timeToTrigger > 0)
                {
                    CancelLeavingTrigger(measId);
                }

                // check leaving condition #2

                for (auto storedMeasIt = m_storedMeasValues.begin();
                     storedMeasIt != m_storedMeasValues.end();
                     ++storedMeasIt)
                {
                    uint16_t cellId = storedMeasIt->first;
                    if (cellId == m_cellId)
                    {
                        continue;
                    }

                    if (measReportIt->second.cellsTriggeredList.find(cellId) !=
                        measReportIt->second.cellsTriggeredList.end())
                    {
                        switch (reportConfigEutra.triggerQuantity)
                        {
                        case NrRrcSap::ReportConfigEutra::RSRP:
                            mn = storedMeasIt->second.rsrp;
                            break;
                        case NrRrcSap::ReportConfigEutra::RSRQ:
                            mn = storedMeasIt->second.rsrq;
                            break;
                        default:
                            NS_FATAL_ERROR("unsupported triggerQuantity");
                            break;
                        }

                        // Inequality A5-4 (Leaving condition 2): Mn + Ofn + Ocn + Hys < Thresh2
                        leavingCond = mn + ofn + ocn + hys < thresh2;

                        if (leavingCond)
                        {
                            concernedCellsLeaving.push_back(cellId);
                            eventLeavingCondApplicable = true;
                        }

                        NS_LOG_LOGIC(this << " event A5: neighbor cell " << cellId << " mn=" << mn
                                          << " mp=" << mp << " thresh2=" << thresh2 << " thresh1="
                                          << thresh1 << " leavingCond=" << leavingCond);

                    } // end of if (measReportIt->second.cellsTriggeredList.find (cellId)
                      //            != measReportIt->second.cellsTriggeredList.end ())

                } // end of for (storedMeasIt)

            } // end of else of if (leavingCond)

        } // end of if (isMeasIdInReportList)

    } // end of case NrRrcSap::ReportConfigEutra::EVENT_A5

    break;

    default:
        NS_FATAL_ERROR("unsupported eventId " << reportConfigEutra.eventId);
        break;

    } // switch (event type)

    NS_LOG_LOGIC(this << " eventEntryCondApplicable=" << eventEntryCondApplicable
                      << " eventLeavingCondApplicable=" << eventLeavingCondApplicable);

    if (eventEntryCondApplicable)
    {
        if (reportConfigEutra.timeToTrigger == 0)
        {
            VarMeasReportListAdd(measId, concernedCellsEntry);
        }
        else
        {
            PendingTrigger_t t;
            t.measId = measId;
            t.concernedCells = concernedCellsEntry;
            const double tttScale = GetTttScale();
            t.timer = Simulator::Schedule(
                MilliSeconds(static_cast<int64_t>(reportConfigEutra.timeToTrigger * tttScale)),
                &NrUeRrc::VarMeasReportListAdd,
                this,
                measId,
                concernedCellsEntry);
            auto enteringTriggerIt = m_enteringTriggerQueue.find(measId);
            NS_ASSERT(enteringTriggerIt != m_enteringTriggerQueue.end());
            enteringTriggerIt->second.push_back(t);
        }
    }

    if (eventLeavingCondApplicable)
    {
        // reportOnLeave will only be set when eventId = eventA3
        bool reportOnLeave = (reportConfigEutra.eventId == NrRrcSap::ReportConfigEutra::EVENT_A3) &&
                             reportConfigEutra.reportOnLeave;

        if (reportConfigEutra.timeToTrigger == 0)
        {
            VarMeasReportListErase(measId, concernedCellsLeaving, reportOnLeave);
        }
        else
        {
            PendingTrigger_t t;
            t.measId = measId;
            t.concernedCells = concernedCellsLeaving;
            const double tttScale = GetTttScale();
            t.timer = Simulator::Schedule(
                MilliSeconds(static_cast<int64_t>(reportConfigEutra.timeToTrigger * tttScale)),
                &NrUeRrc::VarMeasReportListErase,
                this,
                measId,
                concernedCellsLeaving,
                reportOnLeave);
            auto leavingTriggerIt = m_leavingTriggerQueue.find(measId);
            NS_ASSERT(leavingTriggerIt != m_leavingTriggerQueue.end());
            leavingTriggerIt->second.push_back(t);
        }
    }

} // end of void NrUeRrc::MeasurementReportTriggering (uint8_t measId)

void
NrUeRrc::CancelEnteringTrigger(uint8_t measId)
{
    NS_LOG_FUNCTION(this << (uint16_t)measId);

    auto it1 = m_enteringTriggerQueue.find(measId);
    NS_ASSERT(it1 != m_enteringTriggerQueue.end());

    if (!it1->second.empty())
    {
        for (auto it2 = it1->second.begin(); it2 != it1->second.end(); ++it2)
        {
            NS_ASSERT(it2->measId == measId);
            NS_LOG_LOGIC(this << " canceling entering time-to-trigger event at "
                              << Simulator::GetDelayLeft(it2->timer).GetSeconds());
            Simulator::Cancel(it2->timer);
        }

        it1->second.clear();
    }
}

void
NrUeRrc::CancelEnteringTrigger(uint8_t measId, uint16_t cellId)
{
    NS_LOG_FUNCTION(this << (uint16_t)measId << cellId);

    auto it1 = m_enteringTriggerQueue.find(measId);
    NS_ASSERT(it1 != m_enteringTriggerQueue.end());

    auto it2 = it1->second.begin();
    while (it2 != it1->second.end())
    {
        NS_ASSERT(it2->measId == measId);

        for (auto it3 = it2->concernedCells.begin(); it3 != it2->concernedCells.end(); ++it3)
        {
            if (*it3 == cellId)
            {
                it3 = it2->concernedCells.erase(it3);
            }
        }

        if (it2->concernedCells.empty())
        {
            NS_LOG_LOGIC(this << " canceling entering time-to-trigger event at "
                              << Simulator::GetDelayLeft(it2->timer).GetSeconds());
            Simulator::Cancel(it2->timer);
            it2 = it1->second.erase(it2);
        }
        else
        {
            it2++;
        }
    }
}

void
NrUeRrc::CancelLeavingTrigger(uint8_t measId)
{
    NS_LOG_FUNCTION(this << (uint16_t)measId);

    auto it1 = m_leavingTriggerQueue.find(measId);
    NS_ASSERT(it1 != m_leavingTriggerQueue.end());

    if (!it1->second.empty())
    {
        for (auto it2 = it1->second.begin(); it2 != it1->second.end(); ++it2)
        {
            NS_ASSERT(it2->measId == measId);
            NS_LOG_LOGIC(this << " canceling leaving time-to-trigger event at "
                              << Simulator::GetDelayLeft(it2->timer).GetSeconds());
            Simulator::Cancel(it2->timer);
        }

        it1->second.clear();
    }
}

void
NrUeRrc::CancelLeavingTrigger(uint8_t measId, uint16_t cellId)
{
    NS_LOG_FUNCTION(this << (uint16_t)measId << cellId);

    auto it1 = m_leavingTriggerQueue.find(measId);
    NS_ASSERT(it1 != m_leavingTriggerQueue.end());

    auto it2 = it1->second.begin();
    while (it2 != it1->second.end())
    {
        NS_ASSERT(it2->measId == measId);

        for (auto it3 = it2->concernedCells.begin(); it3 != it2->concernedCells.end(); ++it3)
        {
            if (*it3 == cellId)
            {
                it3 = it2->concernedCells.erase(it3);
            }
        }

        if (it2->concernedCells.empty())
        {
            NS_LOG_LOGIC(this << " canceling leaving time-to-trigger event at "
                              << Simulator::GetDelayLeft(it2->timer).GetSeconds());
            Simulator::Cancel(it2->timer);
            it2 = it1->second.erase(it2);
        }
        else
        {
            it2++;
        }
    }
}

void
NrUeRrc::VarMeasReportListAdd(uint8_t measId, ConcernedCells_t enteringCells)
{
    NS_LOG_FUNCTION(this << (uint16_t)measId);
    NS_ASSERT(!enteringCells.empty());

    auto measReportIt = m_varMeasReportList.find(measId);

    if (measReportIt == m_varMeasReportList.end())
    {
        VarMeasReport r;
        r.measId = measId;
        std::pair<uint8_t, VarMeasReport> val(measId, r);
        auto ret = m_varMeasReportList.insert(val);
        NS_ASSERT_MSG(ret.second == true, "element already existed");
        measReportIt = ret.first;
    }

    NS_ASSERT(measReportIt != m_varMeasReportList.end());

    for (auto it = enteringCells.begin(); it != enteringCells.end(); ++it)
    {
        measReportIt->second.cellsTriggeredList.insert(*it);
    }

    NS_ASSERT(!measReportIt->second.cellsTriggeredList.empty());

    // #issue 224, schedule only when there is no periodic event scheduled already
    if (!measReportIt->second.periodicReportTimer.IsPending())
    {
        measReportIt->second.numberOfReportsSent = 0;
        measReportIt->second.periodicReportTimer =
            Simulator::Schedule(NR_UE_MEASUREMENT_REPORT_DELAY,
                                &NrUeRrc::SendMeasurementReport,
                                this,
                                measId);
    }

    auto enteringTriggerIt = m_enteringTriggerQueue.find(measId);
    NS_ASSERT(enteringTriggerIt != m_enteringTriggerQueue.end());
    if (!enteringTriggerIt->second.empty())
    {
        /*
         * Assumptions at this point:
         *  - the call to this function was delayed by time-to-trigger;
         *  - the time-to-trigger delay is fixed (not adaptive/dynamic); and
         *  - the first element in the list is associated with this function call.
         */
        enteringTriggerIt->second.pop_front();

        if (!enteringTriggerIt->second.empty())
        {
            /*
             * To prevent the same set of cells triggering again in the future,
             * we clean up the time-to-trigger queue. This case might occur when
             * time-to-trigger > 200 ms.
             */
            for (auto it = enteringCells.begin(); it != enteringCells.end(); ++it)
            {
                CancelEnteringTrigger(measId, *it);
            }
        }

    } // end of if (!enteringTriggerIt->second.empty ())

} // end of NrUeRrc::VarMeasReportListAdd

void
NrUeRrc::VarMeasReportListErase(uint8_t measId, ConcernedCells_t leavingCells, bool reportOnLeave)
{
    NS_LOG_FUNCTION(this << (uint16_t)measId);
    NS_ASSERT(!leavingCells.empty());

    auto measReportIt = m_varMeasReportList.find(measId);
    NS_ASSERT(measReportIt != m_varMeasReportList.end());

    for (auto it = leavingCells.begin(); it != leavingCells.end(); ++it)
    {
        measReportIt->second.cellsTriggeredList.erase(*it);
    }

    if (reportOnLeave)
    {
        // runs immediately without NR_UE_MEASUREMENT_REPORT_DELAY
        SendMeasurementReport(measId);
    }

    if (measReportIt->second.cellsTriggeredList.empty())
    {
        measReportIt->second.periodicReportTimer.Cancel();
        m_varMeasReportList.erase(measReportIt);
    }

    auto leavingTriggerIt = m_leavingTriggerQueue.find(measId);
    NS_ASSERT(leavingTriggerIt != m_leavingTriggerQueue.end());
    if (!leavingTriggerIt->second.empty())
    {
        /*
         * Assumptions at this point:
         *  - the call to this function was delayed by time-to-trigger; and
         *  - the time-to-trigger delay is fixed (not adaptive/dynamic); and
         *  - the first element in the list is associated with this function call.
         */
        leavingTriggerIt->second.pop_front();

        if (!leavingTriggerIt->second.empty())
        {
            /*
             * To prevent the same set of cells triggering again in the future,
             * we clean up the time-to-trigger queue. This case might occur when
             * time-to-trigger > 200 ms.
             */
            for (auto it = leavingCells.begin(); it != leavingCells.end(); ++it)
            {
                CancelLeavingTrigger(measId, *it);
            }
        }

    } // end of if (!leavingTriggerIt->second.empty ())

} // end of NrUeRrc::VarMeasReportListErase

void
NrUeRrc::VarMeasReportListClear(uint8_t measId)
{
    NS_LOG_FUNCTION(this << (uint16_t)measId);

    // remove the measurement reporting entry for this measId from the VarMeasReportList
    auto measReportIt = m_varMeasReportList.find(measId);
    if (measReportIt != m_varMeasReportList.end())
    {
        NS_LOG_LOGIC(this << " deleting existing report for measId " << (uint16_t)measId);
        measReportIt->second.periodicReportTimer.Cancel();
        m_varMeasReportList.erase(measReportIt);
    }

    CancelEnteringTrigger(measId);
    CancelLeavingTrigger(measId);
}

void
NrUeRrc::SendMeasurementReport(uint8_t measId)
{
    NS_LOG_FUNCTION(this << (uint16_t)measId);
    //  3GPP TS 36.331 section 5.5.5 Measurement reporting

    auto measIdIt = m_varMeasConfig.measIdList.find(measId);
    NS_ASSERT(measIdIt != m_varMeasConfig.measIdList.end());

    auto reportConfigIt = m_varMeasConfig.reportConfigList.find(measIdIt->second.reportConfigId);
    NS_ASSERT(reportConfigIt != m_varMeasConfig.reportConfigList.end());
    NrRrcSap::ReportConfigEutra& reportConfigEutra = reportConfigIt->second.reportConfigEutra;

    NrRrcSap::MeasurementReport measurementReport;
    NrRrcSap::MeasResults& measResults = measurementReport.measResults;
    measResults.measId = measId;

    auto measReportIt = m_varMeasReportList.find(measId);
    if (measReportIt == m_varMeasReportList.end())
    {
        NS_LOG_ERROR("no entry found in m_varMeasReportList for measId " << (uint32_t)measId);
    }
    else
    {
        auto servingMeasIt = m_storedMeasValues.find(m_cellId);
        if (servingMeasIt == m_storedMeasValues.end())
        {
            // ntn-patch 08: no serving-cell result (e.g. right after a
            // handover); a periodical entry is re-armed by the next L1 report.
            NS_LOG_WARN(this << " no serving-cell measurement for measId " << +measId);
            return;
        }
        measResults.measResultPCell.rsrpResult =
            nr::EutranMeasurementMapping::Dbm2RsrpRange(servingMeasIt->second.rsrp);
        measResults.measResultPCell.rsrqResult =
            nr::EutranMeasurementMapping::Db2RsrqRange(servingMeasIt->second.rsrq);
        NS_LOG_INFO(this << " reporting serving cell "
                            "RSRP "
                         << +measResults.measResultPCell.rsrpResult << " ("
                         << servingMeasIt->second.rsrp
                         << " dBm) "
                            "RSRQ "
                         << +measResults.measResultPCell.rsrqResult << " ("
                         << servingMeasIt->second.rsrq << " dB)");

        measResults.haveMeasResultServFreqList = false;
        for (uint16_t componentCarrierId = 1; componentCarrierId < m_numberOfComponentCarriers;
             componentCarrierId++)
        {
            const uint16_t cellId = m_cphySapProvider.at(componentCarrierId)->GetCellId();
            auto measValuesIt = m_storedMeasValues.find(cellId);
            if (measValuesIt != m_storedMeasValues.end())
            {
                measResults.haveMeasResultServFreqList = true;
                NrRrcSap::MeasResultServFreq measResultServFreq;
                measResultServFreq.servFreqId = componentCarrierId;
                measResultServFreq.haveMeasResultSCell = true;
                measResultServFreq.measResultSCell.rsrpResult =
                    nr::EutranMeasurementMapping::Dbm2RsrpRange(measValuesIt->second.rsrp);
                measResultServFreq.measResultSCell.rsrqResult =
                    nr::EutranMeasurementMapping::Db2RsrqRange(measValuesIt->second.rsrq);
                measResultServFreq.haveMeasResultBestNeighCell = false;
                measResults.measResultServFreqList.push_back(measResultServFreq);
            }
        }

        measResults.haveMeasResultNeighCells = false;

        if (!(measReportIt->second.cellsTriggeredList.empty()))
        {
            std::multimap<double, uint16_t> sortedNeighCells;
            for (auto cellsTriggeredIt = measReportIt->second.cellsTriggeredList.begin();
                 cellsTriggeredIt != measReportIt->second.cellsTriggeredList.end();
                 ++cellsTriggeredIt)
            {
                uint16_t cellId = *cellsTriggeredIt;
                if (cellId != m_cellId)
                {
                    auto neighborMeasIt = m_storedMeasValues.find(cellId);
                    double triggerValue;
                    switch (reportConfigEutra.triggerQuantity)
                    {
                    case NrRrcSap::ReportConfigEutra::RSRP:
                        triggerValue = neighborMeasIt->second.rsrp;
                        break;
                    case NrRrcSap::ReportConfigEutra::RSRQ:
                        triggerValue = neighborMeasIt->second.rsrq;
                        break;
                    default:
                        NS_FATAL_ERROR("unsupported triggerQuantity");
                        break;
                    }
                    sortedNeighCells.insert(std::pair<double, uint16_t>(triggerValue, cellId));
                }
            }

            std::multimap<double, uint16_t>::reverse_iterator sortedNeighCellsIt;
            uint32_t count;
            for (sortedNeighCellsIt = sortedNeighCells.rbegin(), count = 0;
                 sortedNeighCellsIt != sortedNeighCells.rend() &&
                 count < reportConfigEutra.maxReportCells;
                 ++sortedNeighCellsIt, ++count)
            {
                uint16_t cellId = sortedNeighCellsIt->second;
                auto neighborMeasIt = m_storedMeasValues.find(cellId);
                NS_ASSERT(neighborMeasIt != m_storedMeasValues.end());
                NrRrcSap::MeasResultEutra measResultEutra;
                measResultEutra.physCellId = cellId;
                measResultEutra.haveCgiInfo = false;
                measResultEutra.haveRsrpResult = true;
                measResultEutra.rsrpResult =
                    nr::EutranMeasurementMapping::Dbm2RsrpRange(neighborMeasIt->second.rsrp);
                measResultEutra.haveRsrqResult = true;
                measResultEutra.rsrqResult =
                    nr::EutranMeasurementMapping::Db2RsrqRange(neighborMeasIt->second.rsrq);
                NS_LOG_INFO(this << " reporting neighbor cell "
                                 << (uint32_t)measResultEutra.physCellId << " RSRP "
                                 << (uint32_t)measResultEutra.rsrpResult << " ("
                                 << neighborMeasIt->second.rsrp << " dBm)"
                                 << " RSRQ " << (uint32_t)measResultEutra.rsrqResult << " ("
                                 << neighborMeasIt->second.rsrq << " dB)");
                measResults.measResultListEutra.push_back(measResultEutra);
                measResults.haveMeasResultNeighCells = true;
            }
        }
        else
        {
            NS_LOG_WARN(this << " cellsTriggeredList is empty");
        }

        /*
         * The current NrRrcSap implementation is broken in that it does not
         * allow for infinite values of reportAmount, which is probably the most
         * reasonable setting. So we just always assume infinite reportAmount.
         */
        measReportIt->second.numberOfReportsSent++;
        measReportIt->second.periodicReportTimer.Cancel();

        Time reportInterval;
        switch (reportConfigEutra.reportInterval)
        {
        case NrRrcSap::ReportConfigEutra::MS120:
            reportInterval = MilliSeconds(120);
            break;
        case NrRrcSap::ReportConfigEutra::MS240:
            reportInterval = MilliSeconds(240);
            break;
        case NrRrcSap::ReportConfigEutra::MS480:
            reportInterval = MilliSeconds(480);
            break;
        case NrRrcSap::ReportConfigEutra::MS640:
            reportInterval = MilliSeconds(640);
            break;
        case NrRrcSap::ReportConfigEutra::MS1024:
            reportInterval = MilliSeconds(1024);
            break;
        case NrRrcSap::ReportConfigEutra::MS2048:
            reportInterval = MilliSeconds(2048);
            break;
        case NrRrcSap::ReportConfigEutra::MS5120:
            reportInterval = MilliSeconds(5120);
            break;
        case NrRrcSap::ReportConfigEutra::MS10240:
            reportInterval = MilliSeconds(10240);
            break;
        case NrRrcSap::ReportConfigEutra::MIN1:
            reportInterval = Seconds(60);
            break;
        case NrRrcSap::ReportConfigEutra::MIN6:
            reportInterval = Seconds(360);
            break;
        case NrRrcSap::ReportConfigEutra::MIN12:
            reportInterval = Seconds(720);
            break;
        case NrRrcSap::ReportConfigEutra::MIN30:
            reportInterval = Seconds(1800);
            break;
        case NrRrcSap::ReportConfigEutra::MIN60:
            reportInterval = Seconds(3600);
            break;
        default:
            NS_FATAL_ERROR("Unsupported reportInterval "
                           << (uint16_t)reportConfigEutra.reportInterval);
            break;
        }

        // schedule the next measurement reporting
        measReportIt->second.periodicReportTimer =
            Simulator::Schedule(reportInterval, &NrUeRrc::SendMeasurementReport, this, measId);

        // send the measurement report to gNB
        m_rrcSapUser->SendMeasurementReport(measurementReport);
    }
}

void
NrUeRrc::ClearRachLock()
{
    NS_LOG_FUNCTION(this << "IMSI" << m_imsi);
    m_rachInProgress = false;
    m_rachBwpId = UINT8_MAX;
    m_rachDeadline = Seconds(0);
    m_rachTimeoutEvent.Cancel();
}

void
NrUeRrc::StartConnection()
{
    NS_LOG_FUNCTION(this << "IMSI" << m_imsi << ", cellId " << m_cellId);
    NS_ASSERT(m_hasReceivedMib);
    NS_ASSERT(m_hasReceivedSib2);

    // Covers raResponseWindow (up to 40 slots) + contentionResolutionTimer
    // (up to 64 ms) + processing margin.
    // 120 ms is safe for FR1. 20 ms is safe for FR2. A force-camped UE never
    // acquires SIB1 (IDLE_WAIT_MIB camps straight on the cell, as an explicit
    // camp request needs no cell evaluation), so without a decoded numerology
    // fall back to the FR1-safe duration: a longer lock than needed merely
    // defers another BWP, a shorter one reopens the race the lock closes.
    if (m_hasReceivedSib1 && m_lastSib1.servingCellConfigCommon.numerology >= 3)
    {
        m_rachLockDuration = MilliSeconds(20);
    }
    else
    {
        m_rachLockDuration = MilliSeconds(120);
    }

    // Set RACH lock, blocking other BWPs from calling StartConnection()
    // concurrently via their own SIB1/MIB pipeline until we finish or timeout.
    m_rachInProgress = true;
    m_rachBwpId = static_cast<uint8_t>(GetPrimaryDlIndex());
    m_rachDeadline = Simulator::Now() + m_rachLockDuration;
    if (m_rachTimeoutEvent.IsPending())
    {
        m_rachTimeoutEvent.Cancel();
    }
    m_rachTimeoutEvent =
        Simulator::Schedule(m_rachLockDuration + MilliSeconds(10), &NrUeRrc::ClearRachLock, this);

    m_connectionPending = false; // reset the flag
    SwitchToState(IDLE_RANDOM_ACCESS);
    // Bind the PHY to the selected cell before contention-based random access.
    // RegisterToGnb() sets the PHY cellId so the RACH preamble (Msg1) is sent to
    // this gNB and its response (RAR/Msg2) is accepted rather than filtered out;
    // it also (re)initializes the L1/L2 control-message queue that carries the
    // preamble. Without it, Msg1 would go out on a stale cellId into an
    // uninitialized queue, so random access would never complete.
    m_cmacSapProvider.at(GetPrimaryUlIndex())->RegisterToGnb(m_cellId);
    m_cmacSapProvider.at(GetPrimaryUlIndex())->StartContentionBasedRandomAccessProcedure();
}

void
NrUeRrc::LeaveConnectedMode()
{
    NS_LOG_FUNCTION(this << "IMSI" << m_imsi << ", cellId " << m_cellId << ", primary UL "
                         << GetPrimaryUlIndex() << ", primary DL " << GetPrimaryDlIndex());
    m_leaveConnectedMode = true;
    m_storedMeasValues.clear();
    ResetRlfParams();

    for (auto measIdIt = m_varMeasConfig.measIdList.begin();
         measIdIt != m_varMeasConfig.measIdList.end();
         ++measIdIt)
    {
        VarMeasReportListClear(measIdIt->second.measId);
    }
    m_varMeasConfig.measIdList.clear();

    m_ccmRrcSapProvider->Reset();

    for (uint32_t i = 0; i < m_numberOfComponentCarriers; i++)
    {
        m_cmacSapProvider.at(i)->Reset(); // reset the MAC
    }

    m_drbMap.clear();
    m_qfi2DrbidMap.clear();
    m_srb1 = nullptr;
    m_hasReceivedMib = false;
    m_hasReceivedSib1 = false;
    m_hasReceivedSib2 = false;

    for (uint32_t i = 0; i < m_numberOfComponentCarriers; i++)
    {
        m_cphySapProvider.at(i)->ResetPhyAfterRlf(); // reset the PHY
    }
    SwitchToState(IDLE_START);
    DoStartCellSelection(m_initDlArfcn);
    // Save the cell id UE was attached to
    StorePreviousCellId(m_cellId);
    m_cellId = 0;
    m_rnti = 0;
    m_srb0->m_rlc->SetRnti(m_rnti);
    m_cphySapProvider.at(GetPrimaryUlIndex())->SetRnti(m_rnti);
    m_cphySapProvider.at(GetPrimaryDlIndex())->SetRnti(m_rnti);
    m_cmacSapProvider.at(GetPrimaryUlIndex())->SetRnti(m_rnti);
    m_cmacSapProvider.at(GetPrimaryDlIndex())->SetRnti(m_rnti);
}

void
NrUeRrc::ConnectionTimeout()
{
    NS_LOG_FUNCTION(this << "IMSI " << m_imsi << ", cellId " << m_cellId);
    ++m_connEstFailCount;
    if (m_connEstFailCount >= m_connEstFailCountLimit)
    {
        m_connectionTimeoutTrace(m_imsi, m_cellId, m_rnti, m_connEstFailCount);
        NS_LOG_DEBUG("Switch to CONNECTED_PHY_PROBLEM. Reason: Connection timeout for IMSI: "
                     << m_imsi << " rnti: " << m_rnti << " cellId: " << m_cellId
                     << " in state: " << ToString(m_state) << ".");
        EnterPhyProblemState(RLF_CONNECTION_TIMEOUT);
        // Assumption: The gNB connection request timer would expire
        // before the expiration of T300 at UE. Upon which, the gNB deletes
        // the UE context. Therefore, here we don't need to send the UE context
        // deletion request to the gNB.
        m_asSapUser->NotifyConnectionReleased();
        m_connEstFailCount = 0;
    }
    else
    {
        for (uint16_t i = 0; i < m_numberOfComponentCarriers; i++)
        {
            m_cmacSapProvider.at(i)->Reset(); // reset the MAC
        }
        m_hasReceivedSib2 = false; // invalidate the previously received SIB2
        SwitchToState(IDLE_CAMPED_NORMALLY);
        m_connectionTimeoutTrace(m_imsi, m_cellId, m_rnti, m_connEstFailCount);
        // Following call to UE NAS will force the UE to immediately
        // perform the random access to the same cell again.
        m_asSapUser->NotifyConnectionFailed(); // inform upper layer
    }
}

void
NrUeRrc::DisposeOldSrb1()
{
    NS_LOG_FUNCTION(this);
    m_srb1Old = nullptr;
}

uint8_t
NrUeRrc::Qfi2Drbid(uint8_t qfi)
{
    auto it = m_qfi2DrbidMap.find(qfi);
    // NS_ASSERT_MSG (it != m_qfi2DrbidMap.end (), "could not find QFI " << +qfi);
    if (it == m_qfi2DrbidMap.end())
    {
        return 0;
    }
    else
    {
        return it->second;
    }
}

void
NrUeRrc::SwitchToState(State newState)
{
    NS_LOG_FUNCTION(this << ToString(newState));
    State oldState = m_state;
    m_state = newState;
    NS_LOG_INFO(this << " IMSI " << m_imsi << " RNTI " << m_rnti << " UeRrc " << ToString(oldState)
                     << " --> " << ToString(newState) << ", cellId " << m_cellId);
    m_stateTransitionTrace(m_imsi, m_cellId, m_rnti, oldState, newState);

    switch (newState)
    {
    case IDLE_START:
        if (m_leaveConnectedMode)
        {
            NS_LOG_INFO("Starting initial cell selection after RLF");
        }
        else
        {
            NS_FATAL_ERROR("cannot switch to an initial state");
        }
        break;

    case IDLE_CELL_SEARCH:
    case IDLE_WAIT_MIB_SIB1:
    case IDLE_WAIT_MIB:
    case IDLE_WAIT_SIB1:
        break;

    case IDLE_CAMPED_NORMALLY:
        if (m_connectionPending)
        {
            SwitchToState(IDLE_WAIT_SIB2);
        }
        break;

    case IDLE_WAIT_SIB2:
        if (m_hasReceivedSib2)
        {
            NS_ASSERT(m_connectionPending);
            StartConnection();
        }
        break;

    case IDLE_RANDOM_ACCESS:
    case CONNECTED_NORMALLY:
        // A fresh/restored connection clears the RLC-max-retx RLF guard so a later
        // failure on this new connection can be declared again.
        m_rlcMaxRetxRlfDeclared = false;
        break;

    case IDLE_CONNECTING:
    case CONNECTED_HANDOVER:
    case CONNECTED_PHY_PROBLEM:
    case CONNECTED_REESTABLISHING:
    default:
        break;
    }
}

void
NrUeRrc::DoNotifyRlcMaxRetx()
{
    NS_LOG_FUNCTION(this << "IMSI " << m_imsi << " state " << ToString(m_state));
    if (!m_rlcMaxRetxTriggersRlf)
    {
        // Legacy behaviour: RLC retransmission is effectively unbounded at the RRC
        // level (the RLC requeues and keeps retransmitting); RLF is driven only by
        // T310 / handover-command timing. Ignore the max-retx indication.
        return;
    }
    // RLC-AM max-retx is a radio-link-failure trigger independent of T310 state
    // (TS 38.331 5.3.10.3). It may fire from a healthy connection or while a T310
    // (DL out-of-sync) is already running, in which case it brings the failure
    // forward. A UE has several AM bearers, so guard against declaring the RLF more
    // than once per connection.
    if (m_rlcMaxRetxRlfDeclared)
    {
        return;
    }
    if (m_state == CONNECTED_NORMALLY || m_state == CONNECTED_HANDOVER ||
        m_state == CONNECTED_PHY_PROBLEM)
    {
        m_rlcMaxRetxRlfDeclared = true;
        // Supersede any pending T310 so the original timer cannot fire a second,
        // duplicate RLF after this RLC-driven one.
        if (m_radioLinkFailureDetected.IsPending())
        {
            m_radioLinkFailureDetected.Cancel();
        }
        NS_LOG_INFO("RLC-AM reached maxRetxThreshold for IMSI "
                    << m_imsi << "; declaring radio link failure (TS 38.331 5.3.10.3)");
        RadioLinkFailureDetected(RLF_RLC_MAX_RETX);
    }
}

void
NrUeRrc::RadioLinkFailureDetected(RadioLinkFailureCause cause)
{
    NS_LOG_FUNCTION(this << "IMSI " << m_imsi << m_rnti << ", cellId " << m_cellId);
    m_radioLinkFailureTrace(m_imsi, m_cellId, m_rnti);
    if (m_useRrcReestablishment)
    {
        NS_LOG_DEBUG(
            "Switch to CONNECTED_PHY_PROBLEM. Reason: Radio link failure detected for IMSI: "
            << m_imsi << " rnti: " << m_rnti << " cellId: " << m_cellId
            << " in state: " << ToString(m_state) << ".");
        EnterPhyProblemState(cause);
        m_rrcSapUser->SendIdealUeContextRemoveRequest(m_rnti);
        m_asSapUser->NotifyConnectionReleased();
    }
    else
    {
        NS_LOG_INFO(
            "RRC reestablishment disabled. UE directly clears context and transitions to IDLE "
            "mode for IMSI: "
            << m_imsi << " rnti: " << m_rnti << " cellId: " << m_cellId);
        m_rrcSapUser->SendIdealUeContextRemoveRequest(m_rnti);
        m_asSapUser->NotifyConnectionReleased();
        ResetRlfParams();
        SwitchToState(IDLE_CELL_SEARCH);
    }
}

std::string
ToString(NrUeRrc::RadioLinkFailureCause cause)
{
    switch (cause)
    {
    case NrUeRrc::RLF_T310_EXPIRY:
        return "T310_EXPIRY";
    case NrUeRrc::RLF_HO_COMMAND_LATE:
        return "HO_COMMAND_LATE";
    case NrUeRrc::RLF_DURING_HANDOVER:
        return "DURING_HANDOVER";
    case NrUeRrc::RLF_CONNECTION_RELEASE:
        return "CONNECTION_RELEASE";
    case NrUeRrc::RLF_CONNECTION_TIMEOUT:
        return "CONNECTION_TIMEOUT";
    case NrUeRrc::RLF_RLC_MAX_RETX:
        return "RLC_MAX_RETX";
    case NrUeRrc::RLF_NONE:
    default:
        return "NONE";
    }
}

double
NrUeRrc::GetTttScale()
{
    if (m_mseFixedScale > 0.0)
    {
        return m_mseFixedScale;
    }
    if (m_mseEnable)
    {
        UpdateMobilityState();
        return m_mseTttScaleFactor;
    }
    return 1.0;
}

void
NrUeRrc::UpdateMobilityState()
{
    NS_LOG_FUNCTION(this);
    const Time now = Simulator::Now();
    // Drop handover timestamps that fell out of the counting window.
    while (!m_mseHandoverTimes.empty() && (now - m_mseHandoverTimes.front()) > m_mseCountWindow)
    {
        m_mseHandoverTimes.pop_front();
    }
    const auto nHo = static_cast<uint32_t>(m_mseHandoverTimes.size());

    // Classify from the recent handover count. An elevated (Medium/High) state is held for at
    // least MseHystNormal (via m_mseElevatedUntil) so a momentary lull does not snap back to
    // Normal and cause the scale factor to chatter.
    double target;
    if (nHo >= m_mseThreshHigh)
    {
        target = m_mseSfHigh;
        m_mseElevatedUntil = now + m_mseHystNormal;
    }
    else if (nHo >= m_mseThreshMedium)
    {
        target = m_mseSfMedium;
        m_mseElevatedUntil = now + m_mseHystNormal;
    }
    else if (now < m_mseElevatedUntil)
    {
        target = m_mseTttScaleFactor; // hysteresis: hold the current elevated factor
    }
    else
    {
        target = 1.0; // Normal mobility
    }

    if (target != m_mseTttScaleFactor)
    {
        NS_LOG_INFO("MSE IMSI " << m_imsi << ": " << nHo << " HOs in "
                                << m_mseCountWindow.As(Time::S) << " -> TTT scale factor "
                                << m_mseTttScaleFactor << " -> " << target);
        m_mseTttScaleFactor = target;
    }
}

void
NrUeRrc::EnterPhyProblemState(RadioLinkFailureCause cause)
{
    NS_LOG_FUNCTION(this << "IMSI " << m_imsi << " cause " << ToString(cause));
    // Capture the timing context at the instant of failure, before any reset of
    // the RLF parameters cancels the T310 event. If T310 is still pending we know
    // exactly how long the DL has been below Qout; if this very call IS the T310
    // expiry, the elapsed time is the full T310 duration; otherwise T310 was not
    // running (e.g. a network connection release) and there is no Qout interval.
    int64_t t310ElapsedMs = -1;
    if (m_radioLinkFailureDetected.IsPending())
    {
        t310ElapsedMs =
            (m_t310 - Simulator::GetDelayLeft(m_radioLinkFailureDetected)).GetMilliSeconds();
    }
    else if (cause == RLF_T310_EXPIRY)
    {
        t310ElapsedMs = m_t310.GetMilliSeconds();
    }
    const int64_t msSinceLastHoSuccess =
        (m_lastHoSuccessTime > Seconds(0))
            ? (Simulator::Now() - m_lastHoSuccessTime).GetMilliSeconds()
            : -1;
    m_rlfCause = cause;
    m_radioLinkFailureCauseTrace(m_imsi,
                                 m_cellId,
                                 m_rnti,
                                 static_cast<uint16_t>(m_state),
                                 ToString(cause),
                                 t310ElapsedMs,
                                 msSinceLastHoSuccess);
    SwitchToState(CONNECTED_PHY_PROBLEM);
}

void
NrUeRrc::DoNotifyInSync()
{
    NS_LOG_FUNCTION(this << m_imsi << ", cellId " << m_cellId);
    m_noOfSyncIndications++;
    NS_LOG_INFO("noOfSyncIndications " << (uint16_t)m_noOfSyncIndications);
    m_phySyncDetectionTrace(m_imsi, m_rnti, m_cellId, "Notify in sync", m_noOfSyncIndications);
    if (m_noOfSyncIndications == m_n311)
    {
        ResetRlfParams();
    }
}

void
NrUeRrc::DoNotifyOutOfSync()
{
    NS_LOG_FUNCTION(this << "IMSI " << m_imsi << ", cellId " << m_cellId);
    m_noOfSyncIndications++;
    NS_LOG_INFO(this << " Total Number of Sync indications from PHY "
                     << (uint16_t)m_noOfSyncIndications << "N310 value : " << (uint16_t)m_n310);
    m_phySyncDetectionTrace(m_imsi, m_rnti, m_cellId, "Notify out of sync", m_noOfSyncIndications);
    if (m_noOfSyncIndications == m_n310)
    {
        m_radioLinkFailureDetected =
            Simulator::Schedule(m_t310, &NrUeRrc::RadioLinkFailureDetected, this, RLF_T310_EXPIRY);
        if (m_radioLinkFailureDetected.IsPending())
        {
            NS_LOG_INFO("t310 started");
        }
        m_cphySapProvider.at(GetPrimaryDlIndex())->StartInSyncDetection();
        m_noOfSyncIndications = 0;
    }
}

void
NrUeRrc::DoResetSyncIndicationCounter()
{
    NS_LOG_FUNCTION(this << "IMSI " << m_imsi << ", cellId " << m_cellId);

    NS_LOG_DEBUG("The number of sync indication received by RRC from PHY: "
                 << (uint16_t)m_noOfSyncIndications);
    m_noOfSyncIndications = 0;
}

void
NrUeRrc::ResetRlfParams()
{
    NS_LOG_FUNCTION(this << "IMSI " << m_imsi << ", cellId " << m_cellId << ", primary UL "
                         << GetPrimaryUlIndex() << ", primary DL " << GetPrimaryDlIndex());
    m_radioLinkFailureDetected.Cancel();
    m_noOfSyncIndications = 0;
    m_cphySapProvider.at(GetPrimaryDlIndex())->ResetRlfParams();
    m_cphySapProvider.at(GetPrimaryUlIndex())->ResetRlfParams();
}

void
NrUeRrc::ReconfigureFromSib1(const uint8_t bwpId,
                             const uint16_t cellId,
                             const uint8_t dlCtrlSym,
                             const uint8_t ulCtrlSym,
                             const uint32_t symPerSlot,
                             const uint16_t numerology,
                             const std::string& tddPattern,
                             const uint8_t numRbsPerRbg)
{
    NS_LOG_FUNCTION(this << "IMSI " << m_imsi << ", cellId " << m_cellId << ", primary UL "
                         << GetPrimaryUlIndex() << ", primary DL " << GetPrimaryDlIndex());
    NrDeviceRegistry::SetUeTargetCell(cellId, m_imsi);
    // A BWP can host the primary UL only if its pattern has UL-capable slots
    m_bwpUlCapable[bwpId] = tddPattern.find('U') != std::string::npos ||
                            tddPattern.find('F') != std::string::npos ||
                            tddPattern.find('S') != std::string::npos;
    m_cphySapProvider.at(bwpId)->SetDlCtrlSyms(dlCtrlSym);
    m_cphySapProvider.at(bwpId)->SetUlCtrlSyms(ulCtrlSym);
    m_cphySapProvider.at(bwpId)->SetSymbolsPerSlot(symPerSlot);
    m_cphySapProvider.at(bwpId)->SetNumerology(numerology);
    m_cphySapProvider.at(bwpId)->SetPattern(tddPattern);
    m_cphySapProvider.at(bwpId)->SetNumRbPerRbg(numRbsPerRbg);
}
} // namespace ns3

/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// oran-ntn-feeder-mgmt: O-RAN control management of an LEO NTN cell under
// intermittent feeder connectivity, measured at packet level.
//
// What is real here
//   * Orbits: a Walker shell from ntn-constellation, propagated per satellite
//     by Sgp4MobilityModel (Kepler + J2 path for Walker elements). Each seed
//     draws a different epoch, so the constellation sits differently over the
//     Earth in every run.
//   * Serving schedule: the satellite with the highest elevation over the cell
//     serves until it drops below the serving mask (earth-fixed cell switch).
//   * Feeder visibility: each satellite sees a gateway when its elevation from
//     that gateway is above the gateway mask; windows come from the contact
//     plan built on the TRUE orbit. Rain-fade outages and a 1.5 s
//     re-acquisition gap after every gateway change are added on top.
//   * Transport: every E2, A1, heartbeat, lease and reconciliation message is
//     a UDP/IPv4 packet crossing ns-3 point-to-point links: a feeder link
//     per (satellite, gateway) pair whose propagation delay follows the live
//     slant range and whose rate is the control-plane bearer, terrestrial
//     backhaul to the Near-RT RIC, and a cloud link to the SMO. A link that
//     is down drops what is in flight. KPM indication bodies are produced and
//     decoded by the module's E2SM-KPM codec; A1 policies are held by the
//     module's A1 policy manager and adapters.
//
// What is NOT real here (stated in the paper as well)
//   * No radio stack: cell demand is a stochastic session process, and the
//     slice share is a control parameter, not scheduled PRBs.
//   * E2AP-over-SCTP is not simulated; UDP carries the messages, padded by
//     the SCTP common header plus DATA chunk size.
//
// Strategies (--strategy)
//   ground    all three function classes on the ground Near-RT RIC
//   onboard   all three classes on the on-board Space RIC; A1 over the feeder
//   reactive  RT class on board, near-RT classes on the ground, heartbeat
//             failover to on-board autonomy, ground-wins on reconnection
//   reactive-ttl  reactive, but the satellite's A1 copy carries the same
//             lease (validUntil = last contact + tau*) as in pal: the lease
//             without prediction, merge or pre-provisioning
//   reactive-ttl-pre  reactive-ttl plus the pre-provisioning of the next
//             serving satellite that pal uses
//   pal       predictive authority leasing (this work): contact-plan driven
//             placement and handoff before predicted feeder loss, A1 policy
//             snapshots carried as leases (validUntil), pre-provisioning of
//             the next serving satellite, versioned merge on reconnection;
//             heartbeat failover for unpredicted outages as in `reactive`
//   pal-nopre pal without pre-provisioning (ablation)
//   pal-gw    pal with ground-wins reconciliation (ablation)
//   onboard-ttl  onboard, but the satellite's A1 copy carries the same lease
//             as pal (renewed by an SMO refresh that names the held version,
//             then the safe intent): the strongest on-board baseline (review D1)
//   sat-pred  reactive-ttl-pre, plus satellite-side prediction (review
//             R3-M4): the satellite itself enters autonomy T_lead before the
//             loss its own on-board contact plan predicts, with no LEASE from
//             the RIC, and resumes through the ground-wins path
//   pred-ho   recent design (Baranda et al., IEEE OJ-COMS 2025): the RIC moves
//             near-RT control on board before the ephemeris-predicted loss,
//             as pal does, but the intent is held without expiry (the A1
//             default), the next satellite is not provisioned and control
//             returns through the ground-wins path
//   ground-pre recent design (KubeSpace, Zhao et al. 2026): all classes stay
//             on the ground, and the next serving satellite is provisioned
//             (E2 setup and subscriptions) ahead of the switch from the
//             predicted schedule
//
// Variants (all strategies): --softSwitch (Rel-17 make-before-break feeder
// switch-over, no gap on a geometric gateway change), --linkDetectS (the
// satellite enters autonomy on link-layer loss of carrier instead of the
// heartbeat timeout), --arqRtoS (SCTP-like retransmission of KPM and control
// datagrams), --a1Process (poisson, periodic or bursty intent updates).
//
// Near-RT fast (NF) outcome: the NF xApp is a beam-tracking controller. The
// best beam of the cell follows a jump process; the ground runs the full
// xApp (decision accuracy nfAccGround), the on-board Space RIC a reduced
// (INT8) xApp whose accuracy is lower by delta. Missed or late NF epochs keep
// a stale beam. The misaligned share of served time is reported for several
// dwell times and accuracy gaps in one run (the decision does not feed back
// into anything else, so all variants see the same dynamics).

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/oran-ntn-a1-interface.h"
#include "ns3/oran-ntn-authority-lease.h"
#include "ns3/oran-ntn-contact-plan.h"
#include "ns3/oran-ntn-flexric-types.h"
#include "ns3/oran-ntn-service-model-kpm.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <vector>

using namespace ns3;
using L = OranNtnAuthorityLease;

NS_LOG_COMPONENT_DEFINE("OranNtnFeederMgmt");

namespace
{

// ---------------------------------------------------------------------------
// Message header
// ---------------------------------------------------------------------------
enum MsgType : uint8_t
{
    KPM_IND = 1,
    RC_CTRL = 2,
    E2_SETUP_REQ = 3,
    E2_SETUP_RSP = 4,
    SUB_REQ = 5,
    SUB_RSP = 6,
    A1_PUT = 7,
    A1_ACK = 8,
    HB = 9,
    HB_ACK = 10,
    LEASE = 11,
    RECON = 12,
    RECON_ACK = 13,
    PREP = 14,
    ARQ_ACK = 15,
};

class MgmtHeader : public Header
{
  public:
    uint8_t type{0};
    uint8_t cls{0};
    uint16_t sat{0};
    uint32_t seq{0};
    int64_t tMeasNs{0};
    int64_t tSendNs{0};
    uint32_t ver{0};
    double v1{0}; // decision value / policy g
    double v2{0}; // demand / policy h
    int64_t untilNs{0};
    uint16_t flags{0};

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::OranNtnFeederMgmtHeader")
                                .SetParent<Header>()
                                .AddConstructor<MgmtHeader>();
        return tid;
    }
    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 1 + 1 + 2 + 4 + 8 + 8 + 4 + 8 + 8 + 8 + 2; }
    void Serialize(Buffer::Iterator i) const override
    {
        i.WriteU8(type);
        i.WriteU8(cls);
        i.WriteHtonU16(sat);
        i.WriteHtonU32(seq);
        i.WriteHtonU64(static_cast<uint64_t>(tMeasNs));
        i.WriteHtonU64(static_cast<uint64_t>(tSendNs));
        i.WriteHtonU32(ver);
        uint64_t b;
        std::memcpy(&b, &v1, 8);
        i.WriteHtonU64(b);
        std::memcpy(&b, &v2, 8);
        i.WriteHtonU64(b);
        i.WriteHtonU64(static_cast<uint64_t>(untilNs));
        i.WriteHtonU16(flags);
    }
    uint32_t Deserialize(Buffer::Iterator i) override
    {
        type = i.ReadU8();
        cls = i.ReadU8();
        sat = i.ReadNtohU16();
        seq = i.ReadNtohU32();
        tMeasNs = static_cast<int64_t>(i.ReadNtohU64());
        tSendNs = static_cast<int64_t>(i.ReadNtohU64());
        ver = i.ReadNtohU32();
        uint64_t b = i.ReadNtohU64();
        std::memcpy(&v1, &b, 8);
        b = i.ReadNtohU64();
        std::memcpy(&v2, &b, 8);
        untilNs = static_cast<int64_t>(i.ReadNtohU64());
        flags = i.ReadNtohU16();
        return GetSerializedSize();
    }
    void Print(std::ostream& os) const override { os << "type=" << int(type); }
};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
enum class Strategy
{
    Ground,
    Onboard,
    Reactive,
    ReactiveTtl,
    ReactiveTtlPre,
    Pal,
    PalNoPre,
    PalGw,
    OnboardTtl,
    SatPred,
    PredHo,
    GroundPre
};

constexpr int kRT = 0;
constexpr int kNF = 1;
constexpr int kNS = 2;
constexpr int kClasses = 3;
const char* kClassName[kClasses] = {"rt", "nf", "ns"};

struct Cfg
{
    Strategy strat{Strategy::Pal};
    std::string stratName{"pal"};
    double simS{1800};
    uint32_t shell{550};
    uint32_t numGw{2};
    double gwMaskDeg{20};
    double serveMaskDeg{25};
    double cellLat{20.0};
    double cellLon{64.0};
    double feederMbps{5.0};
    double feederPer{1e-3};
    double acqS{1.5};
    double rainPerHour{2.0};
    double rainMeanS{40.0};
    double ephSigmaS{0.5};
    double hbPeriodS{0.1};
    uint32_t hbMiss{3};
    double leadS{2.0};
    double maxLeaseS{300.0};
    double leaseEps{0.2};      // supersession bound for the lease horizon
    double priorUpdateS{60.0}; // prior for the RIC's update-interval estimate
    double preS{60.0};
    double slack{0.2};
    double minContactS{3.0};
    double a1MeanS{60.0};
    double a1RetryS{1.0};
    double arrivalMeanS{10.0};
    double holdMeanS{60.0};
    double sessionShare{0.05};
    double safeG{0.6};
    double safeH{0.3};
    double defaultShare{0.3};
    std::array<double, kClasses> periodS{0.01, 0.1, 1.0};
    std::array<double, kClasses> deadlineS{0.01, 0.1, 1.0};
    std::array<double, kClasses> procGroundS{0.0002, 0.0005, 0.002};
    std::array<double, kClasses> procOnboardS{0.001, 0.003, 0.010};
    uint32_t sctpPad{20};
    double procScaleOnboard{1.0};
    bool softSwitch{false};    // Rel-17 soft feeder-link switch-over (D11)
    double linkDetectS{-1.0};  // link-layer loss-of-carrier detection delay; <0 off (D12)
    double arqRtoS{0.0};       // SCTP-like retransmission timeout; 0 off (D27)
    uint32_t arqMax{2};        // retransmissions per datagram
    std::string a1Process{"poisson"}; // poisson | periodic | bursty (D8)
    double a1BurstCv2{4.0};    // squared coefficient of variation, bursty process
    double nfAccGround{0.90};  // decision accuracy of the full (ground) NF xApp
    uint32_t numBeams{16};
    std::string outDir{"oran-ntn-feeder-mgmt-output"};
    uint32_t run{1};
};

Cfg g;

// Gateways around the Arabian Sea (teleport-class coastal sites). The order
// is the order in which --numGw adds them.
struct GwSite
{
    const char* name;
    double lat;
    double lon;
};

const GwSite kGw[4] = {
    {"Karachi", 24.86, 67.01},
    {"Muscat", 23.59, 58.41},
    {"Mumbai", 19.08, 72.88},
    {"Salalah", 17.02, 54.09},
};

// Near-RT RIC site: regional edge cloud in Karachi.
const double kRicLat = 24.86;
const double kRicLon = 67.01;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
struct Policy
{
    uint32_t ver{0};
    double g{0};
    double h{0};
    Time until{Seconds(0)};
};

struct BufKpm
{
    Ptr<Packet> p;
    Time tMeas;
    int cls;
};

struct Sat
{
    uint32_t id{0};
    Ptr<Node> node;
    Ptr<ntncon::Sgp4MobilityModel> mob;
    double ephDtS{0};
    std::vector<Ptr<PointToPointNetDevice>> devSat, devGw;
    std::vector<Ptr<PointToPointChannel>> ch;
    std::vector<Ptr<RateErrorModel>> emSat, emGw;
    std::vector<Ipv4Address> addrSat, addrGw;
    std::vector<uint32_t> ifSat;
    int activeGw{-1};
    int lastGw{-1};
    Time acqUntil{Seconds(0)};
    bool linkUp{false};
    Ptr<Socket> sock;
    // E2 node
    bool e2Setup{false};
    bool setupPending{false};
    std::array<bool, kClasses> subscribed{false, false, false};
    std::deque<BufKpm> buf;
    // Space RIC
    std::array<bool, kClasses> leased{false, false, false};
    bool autonomous{false};
    bool hbArmed{false};
    Time leaseGrantedAt{Seconds(0)};
    Time lastHbRx{Seconds(0)};
    Time autonomyStart{Seconds(0)};
    Ptr<OranNtnA1Adapter> a1;
    bool provisioned{false};
    Policy snap; // last policy snapshot received (re-anchored by heartbeats)
    double leaseTauS{0}; // lease length carried by the last A1 put (onboard-ttl)
    // applied NS parameter
    double share{-1};
    uint32_t shareVer{0};
    bool shareDefault{true}; // still the configured share set at the switch
    std::vector<L::Write> log;
    double lastDemand{0};
    Time lastDemandAt{Seconds(0)};
    uint32_t seq{0};
    // sat-pred: autonomy entered from the on-board contact plan (R3-M4)
    bool predAuto{false};
    Time predAt{Seconds(0)};
};

struct Stats
{
    std::array<uint64_t, kClasses> opp{}, hit{}, applied{}, late{}, lostSwitch{};
    std::array<std::vector<float>, kClasses> loopMs;
    std::array<uint64_t, kClasses> hostOnboardEpochs{};
    // NS / policy
    uint64_t slaSamples{0}, slaViol{0};
    double overSum{0};
    // Shortfall depth (review R2-M5): sum of (r - x)^+ over the samples, and
    // the same sum over the samples counted as short (x < r - 0.01).
    double shortSum{0}, shortSumViol{0};
    uint64_t satPredEntries{0}; // sat-pred: predictive autonomy entries
    uint64_t nsDecisions{0}, nsSuperseded{0};
    std::vector<float> staleS; // time since the used version was superseded
    std::vector<float> policyAgeS;
    // outages
    struct Outage
    {
        double start, end;
        bool predicted;
        bool endedBySwitch;
        // 0 start-up acquisition, 1 geometric (window edge), 2 fade or other
        // loss of an established feeder, 3 takeover by a satellite with no
        // feeder yet, 4 still open at the end of the run (censored).
        int kind{2};
        uint32_t direct{0}, policyStale{0}, fallback{0};
        double ttc{-1};
        uint32_t nfMiss{0};
    };
    std::vector<Outage> outages;
    // switches
    struct Switch
    {
        double t;
        uint32_t from, to;
        double blackoutNf{-1};
        uint64_t sigBytes{0};
        uint32_t sigMsgs{0};
        bool preProvisioned{false};
        double firstEpochS{-1}; // switch -> first NF measurement epoch
    };
    std::vector<Switch> switches;
    uint64_t feederBytes{0}, feederPkts{0};
    std::map<int, uint64_t> bytesByType;
    uint64_t autonomyEntries{0}, falseAutonomy{0};
    uint64_t leasesGranted{0}, leasesLost{0};
    uint64_t kpmDiscarded{0}, kpmDelivered{0}, ctrlLost{0};
    double servedS{0}, linkUpServedS{0};
    uint64_t kpmCodecFail{0};
    uint64_t mergeGroundApplied{0}; // RECON_ACKs that carried a ground-authoritative value (D5)
    uint64_t nfOnbDecisions{0};     // NF decisions taken by the reduced on-board xApp
    uint64_t nsOnbDecisions{0};
    uint64_t arqRetx{0}, arqDup{0};
    uint64_t mergeFallbackWrites{0}; // kept on-board writes under the safe intent (D10)
};

Stats st;

std::vector<Sat> sats;               // only satellites that ever serve
std::map<uint32_t, int> satIdx;      // element index -> sats[] index
struct ServeSeg
{
    double start, end;
    int sat; // sats[] index, -1 = no service
};
std::vector<ServeSeg> serveTl;
int servingIdx = -1; // sats[] index currently serving
size_t serveSeg = 0;

Ptr<Node> smoNode, ricNode;
std::vector<Ptr<Node>> gwNodes;
Ipv4Address ricAddrToSmo, smoAddr;
std::vector<Ipv4Address> ricAddrToGw; // RIC side of RIC-GW links
Ptr<Socket> ricSock, smoSock;
const uint16_t kRicPort = 36421;
const uint16_t kSatPort = 36422;
const uint16_t kSmoPort = 9000;

Ptr<OranNtnContactPlan> planTrue, planPred;
std::vector<std::vector<std::pair<double, double>>> rain; // per gateway

// SMO / A1
Ptr<OranNtnA1PolicyManager> smoMgr;
uint32_t smoPolicyId = 0;
Policy smoCur;
std::map<uint32_t, double> verIssued; // version -> issue time
Ptr<OranNtnA1Adapter> ricA1;
Policy ricPolicy;
std::map<int, std::vector<L::Write>> groundLog; // per satellite (D5)

// Per-sat ground view
struct GroundView
{
    bool e2{false};                                        // RIC answered E2 setup
    bool prepSent{false};                                  // PREP sent (PAL)
    Time prepAt{Seconds(-10)};
    std::array<bool, kClasses> subAcked{false, false, false}; // SUB_RSP seen
    std::array<bool, kClasses> leased{false, false, false};
    bool hbArmed{false};
    Time lastRenew{Seconds(0)};
    Time lastHbAck{Seconds(0)};
    bool autonomous{false};
    bool reconPending{false};
    Time leaseStart{Seconds(0)};
    uint32_t onboardVerAtLease{0};
};
std::vector<GroundView> gview;

// The RIC's own view of reachability: some message from the satellite
// arrived within the heartbeat timeout.
bool
RicSeesUp(int s)
{
    return Simulator::Now() - gview[s].lastHbAck <= Seconds(g.hbPeriodS * g.hbMiss);
}
// On-board write log snapshot carried by a RECON or a resume HB_ACK. The
// payload of those messages is sized from it (24 B per entry); the receiver
// reads the snapshot taken at send time, never the live on-board log.
std::map<int, std::vector<L::Write>> reconSnap;
uint64_t rtGroundFeasibleTicks = 0, rtTicks = 0;
// SLA violation breakdown: [linkUp][appliedVersionCurrent]
uint64_t catSamp[4] = {0, 0, 0, 0}, catViol[4] = {0, 0, 0, 0};
double catOver[4] = {0, 0, 0, 0};
double catShort[4] = {0, 0, 0, 0}; // shortfall depth by state of the applied share
uint64_t violBy[2][2] = {{0, 0}, {0, 0}}, sampBy[2][2] = {{0, 0}, {0, 0}};

// Demand process
uint32_t sessions = 0;
Ptr<ExponentialRandomVariable> rvArr, rvHold, rvA1;
Ptr<UniformRandomVariable> rvU, rvIntent;

// ---------------------------------------------------------------------------
// NF outcome: beam tracking (review D1, D4)
// ---------------------------------------------------------------------------
// The best beam of the cell jumps to a neighbouring beam as a Poisson process
// with mean dwell kDwell[i]; the processes run on their own stream on a fixed
// 10 ms grid, so they are identical for every strategy of a seed. An NF
// decision picks the measured best beam with probability nfAccGround on the
// ground (full FP32 xApp) and nfAccGround - kGap[j] on board (reduced INT8
// xApp), and a neighbour otherwise. The applied beam is held until the next
// actuated NF epoch; a missed or late epoch therefore keeps a stale beam.
constexpr int kNT = 4;
constexpr int kNG = 8;
const double kDwell[kNT] = {0.5, 1.0, 2.0, 5.0};
const double kGap[kNG] = {0.0, 0.005, 0.01, 0.015, 0.02, 0.03, 0.05, 0.10};
int bestBeam[kNT] = {0, 0, 0, 0};
int appliedBeam[kNT][kNG];
uint64_t beamMis[kNT][kNG];
uint64_t beamSamp = 0;
int64_t lastNfAppliedNs = -1;
Ptr<UniformRandomVariable> rvBeam, rvDec;
std::map<int64_t, std::array<int, kNT>> nfMeasured; // NF epoch -> measured best beams
std::map<int64_t, std::array<int, kNT>> rcBeam;     // NF epoch -> ground decision

int
WrongBeam(int m, double u)
{
    const int B = static_cast<int>(g.numBeams);
    return (m + (u < 0.5 ? 1 : B - 1)) % B;
}

void
ResetAppliedBeams()
{
    for (int i = 0; i < kNT; ++i)
    {
        for (int j = 0; j < kNG; ++j)
        {
            appliedBeam[i][j] = -1; // a new satellite has no beam applied yet
        }
    }
    lastNfAppliedNs = -1;
}

void
BeamTick()
{
    Simulator::Schedule(MilliSeconds(10), &BeamTick);
    const int B = static_cast<int>(g.numBeams);
    for (int i = 0; i < kNT; ++i)
    {
        const double u = rvBeam->GetValue();
        const double d = rvBeam->GetValue();
        if (u < 1.0 - std::exp(-0.01 / kDwell[i]))
        {
            bestBeam[i] = (bestBeam[i] + (d < 0.5 ? 1 : B - 1)) % B;
        }
    }
    if (servingIdx < 0)
    {
        return;
    }
    ++beamSamp;
    for (int i = 0; i < kNT; ++i)
    {
        for (int j = 0; j < kNG; ++j)
        {
            if (appliedBeam[i][j] != bestBeam[i])
            {
                ++beamMis[i][j];
            }
        }
    }
    // forget decisions of epochs that can no longer be applied in time
    if (beamSamp % 100 == 0)
    {
        const int64_t cut = (Simulator::Now() - Seconds(5)).GetNanoSeconds();
        while (!nfMeasured.empty() && nfMeasured.begin()->first < cut)
        {
            nfMeasured.erase(nfMeasured.begin());
        }
        while (!rcBeam.empty() && rcBeam.begin()->first < cut)
        {
            rcBeam.erase(rcBeam.begin());
        }
    }
}

// Ground (full xApp) decision for an NF epoch from the measured beams.
std::array<int, kNT>
GroundBeamDecision(const std::array<int, kNT>& m)
{
    const double x = rvDec->GetValue();
    const double u = rvDec->GetValue();
    std::array<int, kNT> d{};
    for (int i = 0; i < kNT; ++i)
    {
        d[i] = x < g.nfAccGround ? m[i] : WrongBeam(m[i], u);
    }
    return d;
}

// Apply an NF action to the cell's beam state.
void
ApplyBeam(int64_t tMeasNs, L::Host writer)
{
    if (tMeasNs <= lastNfAppliedNs)
    {
        return; // an older action arriving after a newer one changes nothing
    }
    if (writer == L::Host::Ground)
    {
        auto it = rcBeam.find(tMeasNs);
        if (it == rcBeam.end())
        {
            return;
        }
        for (int i = 0; i < kNT; ++i)
        {
            for (int j = 0; j < kNG; ++j)
            {
                appliedBeam[i][j] = it->second[i];
            }
        }
    }
    else
    {
        auto it = nfMeasured.find(tMeasNs);
        if (it == nfMeasured.end())
        {
            return;
        }
        ++st.nfOnbDecisions;
        const double x = rvDec->GetValue();
        const double u = rvDec->GetValue();
        for (int i = 0; i < kNT; ++i)
        {
            for (int j = 0; j < kNG; ++j)
            {
                appliedBeam[i][j] =
                    x < g.nfAccGround - kGap[j] ? it->second[i] : WrongBeam(it->second[i], u);
            }
        }
    }
    lastNfAppliedNs = tMeasNs;
}

// Pending consistency tracking for the outage currently being restored.
struct PendingTtc
{
    int sat{-1};
    double restoreAt{-1};
    size_t outageIdx{0};
};
PendingTtc pendTtc;
// Current outage of the serving satellite.
bool inOutage = false;
size_t curOutage = 0;

// Switch tracking
bool switchPending = false;
size_t curSwitch = 0;

double
Demand()
{
    return std::min(0.9, g.sessionShare * sessions);
}

double
Decide(double demand, const Policy& p)
{
    return std::min(0.9, std::max(p.g, demand * (1.0 + p.h)));
}

Policy
SafePolicy()
{
    return Policy{0, g.safeG, g.safeH, Seconds(0)};
}

Policy
FromA1(const OranNtnA1Adapter& a)
{
    auto v = a.GetPoliciesByType(A1PolicyType::SLICE_SLA);
    if (v.empty())
    {
        return SafePolicy();
    }
    const A1NtnPolicy* best = &v[0];
    for (const auto& x : v)
    {
        if (x.param3 > best->param3)
        {
            best = &x;
        }
    }
    return Policy{static_cast<uint32_t>(best->param3), best->param1, best->param2, best->validUntil};
}

A1NtnPolicy
ToA1(const Policy& p)
{
    A1NtnPolicy a;
    a.timestamp = Simulator::Now().GetSeconds();
    a.policyId = 1;
    a.type = A1PolicyType::SLICE_SLA;
    a.scope = "slice:1";
    a.priority = 1;
    a.param1 = p.g;
    a.param2 = p.h;
    a.param3 = p.ver;
    a.active = true;
    a.orbitalPlaneId = 0;
    a.satelliteId = 0;
    a.beamGroupId = 0;
    a.validFrom = Seconds(0);
    a.validUntil = p.until;
    a.orbitAware = false;
    a.federatedAggregationWeight = 0;
    a.minParticipants = 0;
    a.flRoundInterval = Seconds(0);
    a.maxHandoverRate = 0;
    a.minCoverageOverlap = 0;
    return a;
}

bool
IsOnboardLike()
{
    return g.strat == Strategy::Onboard || g.strat == Strategy::OnboardTtl;
}

bool
IsPal()
{
    return g.strat == Strategy::Pal || g.strat == Strategy::PalNoPre || g.strat == Strategy::PalGw ||
           g.strat == Strategy::PredHo;
}

// Predictive strategies whose control returns through the ground-wins path.
bool
GroundWinsReturn()
{
    return g.strat == Strategy::PalGw || g.strat == Strategy::PredHo;
}

bool
IsReactiveLike()
{
    return g.strat == Strategy::Reactive || g.strat == Strategy::ReactiveTtl ||
           g.strat == Strategy::ReactiveTtlPre || g.strat == Strategy::SatPred;
}

// Strategies whose satellite-side A1 copy carries a validUntil lease.
bool
IsLeasing()
{
    return (IsPal() && g.strat != Strategy::PredHo) || g.strat == Strategy::ReactiveTtl ||
           g.strat == Strategy::ReactiveTtlPre || g.strat == Strategy::OnboardTtl ||
           g.strat == Strategy::SatPred;
}

// Strategies that provision the next serving satellite in advance.
bool
UsesPreprov()
{
    return g.strat == Strategy::Pal || g.strat == Strategy::PalGw ||
           g.strat == Strategy::ReactiveTtlPre || g.strat == Strategy::SatPred ||
           g.strat == Strategy::GroundPre;
}

bool
UsesHeartbeat()
{
    return IsReactiveLike() || IsPal();
}

// Payload sizes (bytes, after the fixed header) of the non-KPM messages.
uint32_t
PayloadSize(uint8_t type, uint32_t n)
{
    switch (type)
    {
    case RC_CTRL:
        return 32;
    case E2_SETUP_REQ:
        return 380;
    case E2_SETUP_RSP:
        return 120;
    case SUB_REQ:
        return 140;
    case SUB_RSP:
        return 60;
    case A1_PUT:
        return 260;
    case A1_ACK:
        return 40;
    case LEASE:
        return 200;
    case RECON:
        return 24 * n;
    case RECON_ACK:
        return 64;
    case PREP:
        return 40;
    case ARQ_ACK:
        return 16;
    default:
        return 0;
    }
}

Ptr<Packet>
MakePkt(const MgmtHeader& h, uint32_t payload)
{
    Ptr<Packet> p = Create<Packet>(payload + g.sctpPad);
    p->AddHeader(h);
    return p;
}

Ptr<Packet>
MakeKpm(MgmtHeader h, const std::vector<double>& values)
{
    // E2SM-KPM Indication Format 1 body, produced by the module codec.
    using namespace oranntn::flexric::kpm_v3;
    kpm_ind_msg_format_1_t body;
    body.gran_period_ms = static_cast<uint32_t>(g.periodS[h.cls] * 1000.0);
    meas_info_format_1_lst_t mi;
    mi.meas_type.form = meas_type_form_t::name;
    mi.meas_type.meas_name = h.cls == kNS ? "RRU.PrbUsedDl" : "DRB.UEThpDl";
    for (double value : values)
    {
        meas_record_item_t rec;
        rec.form = meas_value_form_t::real;
        rec.int_val = 0;
        rec.real_val = value;
        mi.meas_record_lst.push_back(rec);
    }
    label_info_t lab;
    lab.s_nssai = "01-000001";
    mi.label_info_lst.push_back(lab);
    body.meas_info_lst.push_back(mi);
    static Ptr<OranNtnServiceModelKpm> sm = CreateObject<OranNtnServiceModelKpm>();
    std::vector<uint8_t> bytes = sm->EncodeIndication(&body);
    Ptr<Packet> p = Create<Packet>(bytes.data(), static_cast<uint32_t>(bytes.size()));
    p->AddPaddingAtEnd(g.sctpPad);
    p->AddHeader(h);
    return p;
}

bool
DecodeKpm(Ptr<Packet> p, std::vector<double>& values)
{
    const uint32_t n = p->GetSize() - g.sctpPad;
    std::vector<uint8_t> bytes(n);
    p->CopyData(bytes.data(), n);
    static Ptr<OranNtnServiceModelKpm> sm = CreateObject<OranNtnServiceModelKpm>();
    oranntn::flexric::kpm_v3::kpm_ind_msg_format_1_t out;
    if (!sm->DecodeIndication(bytes, out) || out.meas_info_lst.empty() ||
        out.meas_info_lst[0].meas_record_lst.empty())
    {
        return false;
    }
    values.clear();
    for (const auto& r : out.meas_info_lst[0].meas_record_lst)
    {
        values.push_back(r.real_val);
    }
    return true;
}

// Switch-time signaling accounting for the satellite that just took over.
void
CountSig(int s, uint8_t type, uint32_t bytes)
{
    if (switchPending && curSwitch < st.switches.size() &&
        static_cast<int>(st.switches[curSwitch].to) == static_cast<int>(sats[s].id))
    {
        if (type == E2_SETUP_REQ || type == E2_SETUP_RSP || type == SUB_REQ ||
            type == SUB_RSP || type == A1_PUT || type == A1_ACK || type == LEASE ||
            type == PREP)
        {
            st.switches[curSwitch].sigBytes += bytes;
            st.switches[curSwitch].sigMsgs += 1;
        }
    }
}

// Pre-switch provisioning traffic is charged to the switch it prepares.
std::map<int, std::pair<uint64_t, uint32_t>> preSig;

void
AccountFeeder(int s, uint8_t type, Ptr<Packet> p, bool preSwitch)
{
    const uint32_t bytes = p->GetSize() + 30; // + IPv4 + UDP + PPP (D30)
    st.feederBytes += bytes;
    st.feederPkts += 1;
    st.bytesByType[type] += bytes;
    if (preSwitch)
    {
        preSig[s].first += bytes;
        preSig[s].second += 1;
    }
    else
    {
        CountSig(s, type, bytes);
    }
}

// ---------------------------------------------------------------------------
// Sending
// ---------------------------------------------------------------------------
bool
IsServing(int s)
{
    return s >= 0 && s == servingIdx;
}

struct GroundView;
bool RicSeesUp(int s);

// SCTP-like reliability for the near-RT datagrams (D27): with --arqRtoS > 0,
// every KPM indication and RIC control is acknowledged by the receiver
// (16-byte ack) and retransmitted after the timeout, up to arqMax times;
// duplicates are recognized by sequence number and not acted on twice.
struct ArqPend
{
    MgmtHeader h;
    uint32_t payload{0};
    Ptr<Packet> p;
    uint32_t tries{0};
};
std::map<std::pair<int, uint32_t>, ArqPend> arqSat, arqRic;
std::set<std::pair<int, uint32_t>> seenAtRic, seenAtSat;
uint32_t ricSeq = 0;
bool
ArqOn()
{
    return g.arqRtoS > 0;
}
void ArqRicTimer(int s, uint32_t seq);
void ArqSatTimer(int s, uint32_t seq);

void
SendToSat(int s, MgmtHeader h, uint32_t payload, bool fromSmo = false)
{
    Sat& sat = sats[s];
    const int gw = sat.activeGw >= 0 ? sat.activeGw : sat.lastGw;
    if (gw < 0)
    {
        if (h.type == RC_CTRL)
        {
            ++st.ctrlLost;
        }
        return;
    }
    h.sat = static_cast<uint16_t>(s);
    h.tSendNs = Simulator::Now().GetNanoSeconds();
    Ptr<Packet> p = MakePkt(h, payload);
    AccountFeeder(s, h.type, p, !IsServing(s));
    if (ArqOn() && h.type == RC_CTRL && !arqRic.count({s, h.seq}))
    {
        arqRic[{s, h.seq}] = ArqPend{h, payload, nullptr, 0};
        Simulator::Schedule(Seconds(g.arqRtoS), &ArqRicTimer, s, h.seq);
    }
    Ptr<Socket> sock = fromSmo ? smoSock : ricSock;
    sock->SendTo(p, 0, InetSocketAddress(sat.addrSat[gw], kSatPort));
}

void
SendFromSat(int s, Ptr<Packet> p, uint8_t type, bool toSmo = false)
{
    AccountFeeder(s, type, p, !IsServing(s));
    if (ArqOn() && type == KPM_IND)
    {
        MgmtHeader hh;
        p->PeekHeader(hh);
        if (!arqSat.count({s, hh.seq}))
        {
            arqSat[{s, hh.seq}] = ArqPend{hh, 0, p->Copy(), 0};
            Simulator::Schedule(Seconds(g.arqRtoS), &ArqSatTimer, s, hh.seq);
        }
    }
    sats[s].sock->SendTo(p,
                         0,
                         InetSocketAddress(toSmo ? smoAddr : ricAddrToSmo,
                                           toSmo ? kSmoPort : kRicPort));
}

void
ArqRicTimer(int s, uint32_t seq)
{
    auto it = arqRic.find({s, seq});
    if (it == arqRic.end())
    {
        return;
    }
    if (it->second.tries >= g.arqMax || !IsServing(s))
    {
        arqRic.erase(it);
        return;
    }
    ++it->second.tries;
    ++st.arqRetx;
    SendToSat(s, it->second.h, it->second.payload);
    Simulator::Schedule(Seconds(g.arqRtoS), &ArqRicTimer, s, seq);
}

void
ArqSatTimer(int s, uint32_t seq)
{
    auto it = arqSat.find({s, seq});
    if (it == arqSat.end())
    {
        return;
    }
    if (it->second.tries >= g.arqMax || !IsServing(s))
    {
        arqSat.erase(it);
        return;
    }
    ++it->second.tries;
    ++st.arqRetx;
    Ptr<Packet> c = it->second.p->Copy();
    SendFromSat(s, c, KPM_IND);
    Simulator::Schedule(Seconds(g.arqRtoS), &ArqSatTimer, s, seq);
}

void
SendSatMsg(int s, MgmtHeader h, uint32_t payload, bool toSmo = false)
{
    h.sat = static_cast<uint16_t>(s);
    h.seq = ++sats[s].seq;
    h.tSendNs = Simulator::Now().GetNanoSeconds();
    SendFromSat(s, MakePkt(h, payload), h.type, toSmo);
}

// ---------------------------------------------------------------------------
// Host selection
// ---------------------------------------------------------------------------
bool
HostedOnboard(int cls, const Sat& s)
{
    switch (g.strat)
    {
    case Strategy::Ground:
    case Strategy::GroundPre:
        return false;
    case Strategy::Onboard:
    case Strategy::OnboardTtl:
        return true;
    case Strategy::Reactive:
    case Strategy::ReactiveTtl:
    case Strategy::ReactiveTtlPre:
    case Strategy::SatPred:
        return cls == kRT || s.autonomous;
    default:
        // PAL pins RT on board: the placement rule is evaluated for RT every
        // tick and logged (checks.csv); it never selects the ground for RT in
        // the evaluated geometries.
        return cls == kRT || s.leased[cls] || s.autonomous;
    }
}

// ---------------------------------------------------------------------------
// SLA / staleness bookkeeping
// ---------------------------------------------------------------------------
uint64_t nsFallback = 0;
uint64_t softSwitches = 0;
void EnterAutonomy(int s);
// NF epoch bookkeeping for miss attribution: epoch time -> state code at the
// epoch (bit0 link up, bit1 hosted on board, bit2 E2 set up, bit3 subscribed,
// bit4 autonomous, bit5 leased, bit6 within 5 s after a switch).
std::map<int64_t, uint16_t> nfPending;
std::map<int64_t, uint16_t> nfMissed;
double lastSwitchT = -1e9;

void
RecordNsDecision(uint32_t verUsed)
{
    ++st.nsDecisions;
    if (verUsed == 0)
    {
        // Deliberate safe fallback after a lease lapsed: not a stale intent.
        ++nsFallback;
        return;
    }
    const double now = Simulator::Now().GetSeconds();
    auto it = verIssued.find(verUsed);
    st.policyAgeS.push_back(it == verIssued.end() ? 0.0f : float(now - it->second));
    // Superseded: a newer version existed at the SMO at decision time. The
    // safe default (version 0) counts as superseded only if a real policy
    // existed when it was used.
    double supAt = -1;
    for (const auto& [v, t] : verIssued)
    {
        if (v > verUsed && t <= now)
        {
            supAt = (supAt < 0) ? t : std::min(supAt, t);
        }
    }
    if (supAt >= 0)
    {
        ++st.nsSuperseded;
        st.staleS.push_back(float(now - supAt));
    }
    else
    {
        st.staleS.push_back(0.0f);
    }
}

void
CheckTtc(int s, uint32_t verApplied, bool confirming)
{
    if (pendTtc.sat == s && confirming && verApplied >= smoCur.ver)
    {
        auto& o = st.outages[pendTtc.outageIdx];
        o.ttc = Simulator::Now().GetSeconds() - pendTtc.restoreAt;
        pendTtc.sat = -1;
    }
}

void
ApplyAction(int s, int cls, Time tMeas, double value, uint32_t ver, L::Host writer, bool epochAction = true)
{
    Sat& sat = sats[s];
    if (!IsServing(s))
    {
        if (epochAction)
        {
            ++st.lostSwitch[cls];
        }
        return;
    }
    if (!epochAction)
    {
        // Reconciliation write: not tied to a measurement epoch, so it is not
        // a deadline sample; it only changes the parameter.
        sat.share = value;
        sat.shareVer = ver;
        sat.shareDefault = false;
        CheckTtc(s, ver, true);
        return;
    }
    const double loop = (Simulator::Now() - tMeas).GetSeconds();
    st.applied[cls]++;
    st.loopMs[cls].push_back(float(loop * 1e3));
    if (loop <= g.deadlineS[cls] + 1e-12)
    {
        st.hit[cls]++;
    }
    else
    {
        st.late[cls]++;
    }
    if (cls == kNF)
    {
        nfPending.erase(tMeas.GetNanoSeconds());
        nfMissed.erase(tMeas.GetNanoSeconds());
        ApplyBeam(tMeas.GetNanoSeconds(), writer);
    }
    if (cls == kNF && switchPending && curSwitch < st.switches.size() &&
        st.switches[curSwitch].blackoutNf < 0 && st.switches[curSwitch].to == sat.id)
    {
        st.switches[curSwitch].blackoutNf =
            Simulator::Now().GetSeconds() - st.switches[curSwitch].t;
        switchPending = false;
    }
    if (cls == kNS)
    {
        sat.share = value;
        sat.shareVer = ver;
        sat.shareDefault = false;
        bool confirming;
        if (writer == L::Host::Onboard)
        {
            ++st.nsOnbDecisions;
            sat.log.push_back({1, value, ver, tMeas, L::Host::Onboard});
            confirming = IsOnboardLike();
        }
        else
        {
            confirming = tMeas.GetSeconds() >= pendTtc.restoreAt;
        }
        CheckTtc(s, ver, confirming);
    }
}

// ---------------------------------------------------------------------------
// Measurement epochs (one chain per class, at the serving satellite)
// ---------------------------------------------------------------------------
void FlushBuffer(int s);

void
Epoch(int cls)
{
    Simulator::Schedule(Seconds(g.periodS[cls]), &Epoch, cls);
    const int s = servingIdx;
    if (s < 0)
    {
        return;
    }
    Sat& sat = sats[s];
    st.opp[cls]++;
    const Time tMeas = Simulator::Now();
    const double demand = Demand();
    if (cls == kNS)
    {
        sat.lastDemand = demand;
        sat.lastDemandAt = tMeas;
    }
    std::array<int, kNT> meas{};
    if (cls == kNF)
    {
        for (int i = 0; i < kNT; ++i)
        {
            meas[i] = bestBeam[i];
        }
        nfMeasured[tMeas.GetNanoSeconds()] = meas;
    }
    if (cls == kNF && switchPending && curSwitch < st.switches.size() &&
        st.switches[curSwitch].firstEpochS < 0)
    {
        st.switches[curSwitch].firstEpochS = tMeas.GetSeconds() - st.switches[curSwitch].t;
    }
    if (cls == kNF)
    {
        // Settle epochs older than 2 deadlines: whatever is still pending missed.
        const int64_t cut = (tMeas - Seconds(2 * g.deadlineS[kNF])).GetNanoSeconds();
        while (!nfPending.empty() && nfPending.begin()->first < cut)
        {
            nfMissed.insert(*nfPending.begin());
            nfPending.erase(nfPending.begin());
        }
        uint16_t code = (sat.linkUp ? 1 : 0) | (HostedOnboard(cls, sat) ? 2 : 0) |
                        (sat.e2Setup ? 4 : 0) | (sat.subscribed[kNF] ? 8 : 0) |
                        (sat.autonomous ? 16 : 0) | (sat.leased[kNF] ? 32 : 0) |
                        (tMeas.GetSeconds() - lastSwitchT < 5.0 ? 64 : 0);
        nfPending[tMeas.GetNanoSeconds()] = code;
    }
    if (HostedOnboard(cls, sat))
    {
        st.hostOnboardEpochs[cls]++;
        const double proc =
            g.procScaleOnboard * g.procOnboardS[cls] * (0.8 + 0.4 * rvU->GetValue());
        double value = 0;
        uint32_t ver = 0;
        if (cls == kNS)
        {
            const Policy p = FromA1(*sat.a1);
            value = Decide(demand, p);
            ver = p.ver;
        }
        Simulator::Schedule(Seconds(proc), [=]() {
            if (cls == kNS)
            {
                RecordNsDecision(ver);
            }
            ApplyAction(s, cls, tMeas, value, ver, L::Host::Onboard);
        });
        return;
    }
    // Ground hosted: needs E2 setup and a subscription for this class.
    if (!sat.e2Setup || !sat.subscribed[cls])
    {
        return;
    }
    MgmtHeader h;
    h.type = KPM_IND;
    h.cls = static_cast<uint8_t>(cls);
    h.sat = static_cast<uint16_t>(s);
    h.seq = ++sat.seq;
    h.tMeasNs = tMeas.GetNanoSeconds();
    h.v2 = demand;
    std::vector<double> kv;
    if (cls == kNS)
    {
        kv.push_back(demand);
    }
    else if (cls == kNF)
    {
        for (int i = 0; i < kNT; ++i)
        {
            kv.push_back(meas[i]); // measured best-beam index, one record per dwell setting
        }
    }
    else
    {
        kv.push_back(rvU->GetValue());
    }
    Ptr<Packet> p = MakeKpm(h, kv);
    if (sat.linkUp)
    {
        // Re-stamp send time right before sending.
        MgmtHeader hh;
        p->RemoveHeader(hh);
        hh.tSendNs = Simulator::Now().GetNanoSeconds();
        p->AddHeader(hh);
        SendFromSat(s, p, KPM_IND);
    }
    else
    {
        // Store-and-forward (module default): hold on board until the feeder
        // returns; entries older than the class deadline are discarded at
        // flush, which is the tuned setting (older reports cannot yield a
        // timely action).
        sat.buf.push_back({p, tMeas, cls});
        if (sat.buf.size() > 2000)
        {
            sat.buf.pop_front();
            ++st.kpmDiscarded;
        }
    }
}

void
FlushBuffer(int s)
{
    Sat& sat = sats[s];
    while (!sat.buf.empty())
    {
        BufKpm b = sat.buf.front();
        sat.buf.pop_front();
        if ((Simulator::Now() - b.tMeas).GetSeconds() > g.deadlineS[b.cls])
        {
            ++st.kpmDiscarded;
            continue;
        }
        MgmtHeader hh;
        b.p->RemoveHeader(hh);
        hh.tSendNs = Simulator::Now().GetNanoSeconds();
        b.p->AddHeader(hh);
        SendFromSat(s, b.p, KPM_IND);
    }
}

// ---------------------------------------------------------------------------
// E2 setup and subscriptions
// ---------------------------------------------------------------------------
void
StartE2Setup(int s)
{
    Sat& sat = sats[s];
    if (sat.e2Setup || sat.setupPending)
    {
        return;
    }
    sat.setupPending = true;
    MgmtHeader h;
    h.type = E2_SETUP_REQ;
    SendSatMsg(s, h, PayloadSize(E2_SETUP_REQ, 0));
    // Retry every second until the response arrives (E2AP setup timer).
    Simulator::Schedule(Seconds(1.0), [s]() {
        if (!sats[s].e2Setup)
        {
            sats[s].setupPending = false;
            if (IsServing(s) || sats[s].provisioned)
            {
                StartE2Setup(s);
            }
        }
    });
}

bool
GroundNeedsSub(int cls)
{
    switch (g.strat)
    {
    case Strategy::Ground:
    case Strategy::GroundPre:
        return true;
    case Strategy::Onboard:
    case Strategy::OnboardTtl:
        return false;
    case Strategy::Reactive:
    case Strategy::ReactiveTtl:
    case Strategy::ReactiveTtlPre:
    case Strategy::SatPred:
        return cls != kRT;
    default:
        return cls != kRT; // pal: RT is never placed on the ground (predicted loop > 10 ms)
    }
}

void PushPolicyToSat(int s, bool lease);
int64_t OnbLeaseUntilNs();

void
RicOnSetup(int s)
{
    gview[s].e2 = true;
    MgmtHeader r;
    r.type = E2_SETUP_RSP;
    SendToSat(s, r, PayloadSize(E2_SETUP_RSP, 0));
    for (int c = 0; c < kClasses; ++c)
    {
        if (GroundNeedsSub(c))
        {
            MgmtHeader q;
            q.type = SUB_REQ;
            q.cls = static_cast<uint8_t>(c);
            SendToSat(s, q, PayloadSize(SUB_REQ, 0));
        }
    }
    if (IsOnboardLike())
    {
        // The RIC notifies the SMO, which PUTs the current policy on the
        // satellite's A1 termination (with retries until acknowledged).
        MgmtHeader a;
        a.type = A1_PUT;
        a.ver = smoCur.ver;
        a.v1 = smoCur.g;
        a.v2 = smoCur.h;
        a.untilNs = OnbLeaseUntilNs();
        SendToSat(s, a, PayloadSize(A1_PUT, 0), true);
    }
    else if (IsReactiveLike())
    {
        PushPolicyToSat(s, IsLeasing()); // shadow copy for autonomy
    }
    else if (IsPal())
    {
        PushPolicyToSat(s, IsLeasing());
    }
}

// Predicted end of the CURRENT feeder contact: the end of the active
// gateway's pair window in the predicted plan (a gateway change costs a
// re-acquisition gap even when another gateway is visible).
Time
PredLoss(int s)
{
    const Time now = Simulator::Now();
    const Sat& sat = sats[s];
    if (sat.activeGw < 0)
    {
        return now;
    }
    Time end = now;
    for (const auto& w : planPred->GetPairWindows(sat.id, sat.activeGw))
    {
        if (now >= w.start && now < w.end)
        {
            end = w.end;
            break;
        }
        if (w.start > now)
        {
            break;
        }
    }
    if (!g.softSwitch || end == now)
    {
        return end; // the plan already expects no contact on this gateway if end == now
    }
    // With soft switch-over the contact continues on any gateway whose
    // predicted window already covers the edge (make-before-break).
    int cur = sat.activeGw;
    for (int hop = 0; hop < 8; ++hop)
    {
        bool extended = false;
        for (int j = 0; j < static_cast<int>(g.numGw) && !extended; ++j)
        {
            if (j == cur)
            {
                continue;
            }
            for (const auto& w : planPred->GetPairWindows(sat.id, j))
            {
                if (w.start <= end - MilliSeconds(20) && w.end > end + MilliSeconds(20))
                {
                    end = w.end;
                    cur = j;
                    extended = true;
                    break;
                }
            }
        }
        if (!extended)
        {
            break;
        }
    }
    return end;
}

// The RIC's estimate of the mean A1 update spacing: prior plus observations.
double a1IntervalSum = 0;
uint32_t a1IntervalN = 0;
double a1LastAt = -1;

Time
LeaseTau()
{
    const double meanS = (g.priorUpdateS + a1IntervalSum) / (1.0 + a1IntervalN);
    return std::min(L::LeaseHorizon(Seconds(meanS), g.leaseEps), Seconds(g.maxLeaseS));
}

// Lease expiry for a snapshot handed over now. The horizon tau* minimizes the
// expected cost of holding a possibly superseded intent versus falling back,
// and the minimizer does not depend on how long the outage will last (paper,
// Sec. IV-C), so the predicted restoration time is not used here.
Time
PredictedLeaseUntil(int /*s*/)
{
    return Simulator::Now() + LeaseTau();
}

// onboard-ttl: the SMO puts the intent with the same lease horizon (0 = no
// expiry, the A1 default, for onboard).
int64_t
OnbLeaseUntilNs()
{
    return g.strat == Strategy::OnboardTtl ? PredictedLeaseUntil(0).GetNanoSeconds() : 0;
}

void
PushPolicyToSat(int s, bool lease)
{
    MgmtHeader a;
    a.type = lease ? LEASE : A1_PUT;
    a.ver = ricPolicy.ver;
    a.v1 = ricPolicy.g;
    a.v2 = ricPolicy.h;
    a.untilNs = lease ? PredictedLeaseUntil(s).GetNanoSeconds() : 0;
    a.flags = 0; // policy refresh only, no authority change
    SendToSat(s, a, PayloadSize(a.type, 0));
}

// ---------------------------------------------------------------------------
// Satellite receive
// ---------------------------------------------------------------------------
void
SatInstallPolicy(int s, uint32_t ver, double gg, double hh, Time until)
{
    // Versions never regress on board (D20): an older version is ignored,
    // the same version only renews its lease, a newer one replaces it.
    Sat& sat = sats[s];
    if (ver < sat.snap.ver)
    {
        return;
    }
    Policy p{ver, gg, hh, until};
    if (ver == sat.snap.ver && sat.snap.ver > 0)
    {
        const bool forever = until <= Seconds(0) || sat.snap.until <= Seconds(0);
        p.until = forever ? Seconds(0) : std::max(until, sat.snap.until);
    }
    sat.snap = p;
    sat.a1->HandleIncomingPolicy(ToA1(p));
}

// Ground-wins resume (reactive hybrid and the PAL ablation): leave autonomy
// and report the on-board write log and a fresh demand measurement so the
// ground can decide at once.
void
SatResume(int s)
{
    Sat& sat = sats[s];
    sat.autonomous = false;
    sat.predAuto = false;
    // Link-up is contact: restart the heartbeat timeout here, or the next
    // check would put the satellite straight back into autonomy before any
    // ground message has had time to arrive.
    sat.lastHbRx = Simulator::Now();
    reconSnap[s] = sat.log;
    sat.log.clear();
    MgmtHeader r;
    r.type = HB_ACK;
    r.flags = 1;
    r.untilNs = sat.autonomyStart.GetNanoSeconds(); // t0 of the on-board log (M4b)
    r.v2 = Demand();
    r.tMeasNs = Simulator::Now().GetNanoSeconds();
    r.ver = sat.shareVer;
    r.v1 = sat.share;
    SendSatMsg(s, r, PayloadSize(RECON, static_cast<uint32_t>(reconSnap[s].size())));
}

// PAL return path: report the write log of the lease interval and a fresh
// demand measurement; the Space RIC keeps control until RECON_ACK.
void
SatSendRecon(int s)
{
    Sat& sat = sats[s];
    bool any = sat.autonomous;
    for (int c = 0; c < kClasses; ++c)
    {
        any = any || sat.leased[c];
    }
    if (!any || !sat.e2Setup)
    {
        return;
    }
    MgmtHeader r;
    r.type = RECON;
    r.flags = 0;
    for (int c = 0; c < kClasses; ++c)
    {
        if (sat.leased[c] || sat.autonomous)
        {
            r.flags |= (2u << c);
        }
    }
    r.ver = sat.shareVer;
    r.v1 = sat.share;
    r.v2 = Demand();
    r.untilNs = sat.autonomyStart.GetNanoSeconds(); // t0 of the on-board log (M4b)
    r.tMeasNs = Simulator::Now().GetNanoSeconds();
    reconSnap[s] = sat.log;
    SendSatMsg(s, r, PayloadSize(RECON, static_cast<uint32_t>(sat.log.size())));
}

void
SatRecv(int s, Ptr<Socket> sock)
{
    Ptr<Packet> p;
    Address from;
    while ((p = sock->RecvFrom(from)))
    {
        MgmtHeader h;
        p->RemoveHeader(h);
        Sat& sat = sats[s];
        // Any message from the ground proves the path is up; the heartbeat
        // timeout counts from the latest one. (Without this, a satellite whose
        // reconciliation reply arrived before the first heartbeat fell back
        // into autonomy with its feeder up.)
        const bool wasAutoAny = sat.autonomous;
        sat.lastHbRx = Simulator::Now();
        switch (h.type)
        {
        case RC_CTRL:
            if (ArqOn())
            {
                MgmtHeader a;
                a.type = ARQ_ACK;
                a.sat = static_cast<uint16_t>(s);
                a.seq = h.seq;
                a.tSendNs = Simulator::Now().GetNanoSeconds();
                SendFromSat(s, MakePkt(a, PayloadSize(ARQ_ACK, 0)), ARQ_ACK);
                if (!seenAtSat.insert({s, h.seq}).second)
                {
                    ++st.arqDup;
                    break;
                }
            }
            ApplyAction(s, h.cls, NanoSeconds(h.tMeasNs), h.v1, h.ver, L::Host::Ground,
                        (h.flags & 16) == 0);
            break;
        case ARQ_ACK:
            arqSat.erase({s, h.seq});
            break;
        case E2_SETUP_RSP:
            sat.e2Setup = true;
            sat.setupPending = false;
            break;
        case SUB_REQ: {
            sat.subscribed[h.cls] = true;
            MgmtHeader r;
            r.type = SUB_RSP;
            r.cls = h.cls;
            SendSatMsg(s, r, PayloadSize(SUB_RSP, 0));
            break;
        }
        case A1_PUT: {
            if (h.untilNs > 0)
            {
                sat.leaseTauS = (h.untilNs - h.tSendNs) * 1e-9;
            }
            if ((h.flags & 2) && h.ver <= sat.snap.ver)
            {
                // onboard-ttl lease refresh from the SMO: it names the current
                // version, so only a held snapshot of that version is renewed
                // (the same rule as the hybrids' heartbeat re-anchor, D20).
                if (h.ver == sat.snap.ver)
                {
                    SatInstallPolicy(s, h.ver, h.v1, h.v2, NanoSeconds(h.untilNs));
                }
                break;
            }
            SatInstallPolicy(s, h.ver, h.v1, h.v2, NanoSeconds(h.untilNs));
            MgmtHeader r;
            r.type = A1_ACK;
            r.ver = h.ver;
            SendSatMsg(s, r, PayloadSize(A1_ACK, 0), IsOnboardLike());
            break;
        }
        case LEASE: {
            SatInstallPolicy(s, h.ver, h.v1, h.v2, NanoSeconds(h.untilNs));
            if (h.flags & 1)
            {
                // Authority grant for the classes in the high bits.
                for (int c = 0; c < kClasses; ++c)
                {
                    if (h.flags & (2u << c))
                    {
                        if (!sat.leased[c])
                        {
                            sat.leased[c] = true;
                        }
                    }
                }
                sat.autonomyStart = Simulator::Now();
                sat.leaseGrantedAt = Simulator::Now();
                sat.log.clear();
            }
            break;
        }
        case HB: {
            const bool wasAuto = wasAutoAny && sat.autonomous;
            if (h.untilNs > 0 && sat.snap.ver > 0 && sat.snap.ver == h.ver)
            {
                // Every contact re-anchors the lease of the held snapshot, but
                // only if the heartbeat confirms that snapshot is still the
                // RIC's current version (D20); a superseded one is replaced
                // by the next policy push or RECON_ACK instead.
                SatInstallPolicy(s, sat.snap.ver, sat.snap.g, sat.snap.h, NanoSeconds(h.untilNs));
            }
            if (!IsServing(s))
            {
                // Heartbeat to a provisioned, not yet serving satellite (D16):
                // it only keeps the snapshot's lease anchored.
                break;
            }
            if (wasAuto && g.strat == Strategy::SatPred && sat.predAuto)
            {
                // sat-pred: autonomy taken from the on-board plan ahead of a
                // predicted loss is kept while the feeder is still up; the
                // satellite returns it itself (SatTick) or at link-up.
                MgmtHeader r;
                r.type = HB_ACK;
                SendSatMsg(s, r, 0);
            }
            else if (wasAuto && (!IsPal() || GroundWinsReturn()))
            {
                SatResume(s);
            }
            else
            {
                if (wasAuto)
                {
                    SatSendRecon(s); // PAL: the RECON sent at link-up was lost
                }
                MgmtHeader r;
                r.type = HB_ACK;
                SendSatMsg(s, r, 0);
            }
            break;
        }
        case RECON_ACK: {
            // Merged state from the ground: authority returns to the ground
            // for the classes it names, the current policy rides along and a
            // recomputed decision is applied if the merge asked for one.
            SatInstallPolicy(s, h.ver, h.v1, h.v2, NanoSeconds(h.untilNs));
            for (int c = 0; c < kClasses; ++c)
            {
                if (h.flags & (2u << c))
                {
                    sat.leased[c] = false;
                }
            }
            sat.autonomous = false;
            sat.log.clear();
            if (h.flags & 1)
            {
                // Recomputed NS decision (share x 1e6 rides in the seq field).
                ApplyAction(s, kNS, NanoSeconds(h.tMeasNs), h.seq / 1e6, h.ver,
                            L::Host::Ground, false);
            }
            else
            {
                CheckTtc(s, sat.shareVer, true);
            }
            break;
        }
        case PREP:
            sat.provisioned = true;
            StartE2Setup(s);
            break;
        default:
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// RIC receive
// ---------------------------------------------------------------------------
void
RicDecideAndSend(int s, int cls, Time tMeas, double demand, bool recon = false)
{
    const double proc = g.procGroundS[cls] * (0.8 + 0.4 * rvU->GetValue());
    Simulator::Schedule(Seconds(proc), [=]() {
        MgmtHeader c;
        c.type = RC_CTRL;
        c.cls = static_cast<uint8_t>(cls);
        c.tMeasNs = tMeas.GetNanoSeconds();
        c.flags = recon ? 16 : 0;
        c.seq = ++ricSeq;
        if (cls == kNS)
        {
            const Policy p = FromA1(*ricA1);
            c.v1 = Decide(demand, p);
            c.ver = p.ver;
            RecordNsDecision(p.ver);
            groundLog[s].push_back({1, c.v1, c.ver, tMeas, L::Host::Ground});
        }
        SendToSat(s, c, PayloadSize(RC_CTRL, 0));
    });
}

void
ReconcileGroundWins(int s, const MgmtHeader& h)
{
    // Reactive: the ground resumes and decides from the fresh measurement in
    // the resume message; the on-board writes made in autonomy are compared
    // with that decision and overwritten.
    GroundView& gv = gview[s];
    const Policy p = FromA1(*ricA1);
    const double gDecision = Decide(h.v2, p);
    std::vector<L::Write> gl = {{1, gDecision, p.ver, Simulator::Now(), L::Host::Ground}};
    const Time t0 = h.untilNs > 0 ? NanoSeconds(h.untilNs) : gv.leaseStart; // from the message (M4b)
    auto m = L::Merge(L::MergeRule::GroundWins,
                      gl,
                      reconSnap[s],
                      t0,
                      Simulator::Now(),
                      p.ver, // the RIC's own version, not the SMO's (D19)
                      0.01);
    if (pendTtc.sat == s)
    {
        auto& o = st.outages[pendTtc.outageIdx];
        o.direct += m.directConflicts;
        o.policyStale += m.policyStaleWrites;
        o.fallback += m.fallbackWrites;
    }
    reconSnap[s].clear();
    PushPolicyToSat(s, IsLeasing());
    RicDecideAndSend(s, kNS, NanoSeconds(h.tMeasNs), h.v2, true);
}

void
RicRecv(Ptr<Socket> sock)
{
    Ptr<Packet> p;
    Address from;
    while ((p = sock->RecvFrom(from)))
    {
        MgmtHeader h;
        p->RemoveHeader(h);
        const int s = h.sat;
        if (s < 0 || s >= static_cast<int>(sats.size()))
        {
            continue;
        }
        // Any message from the satellite proves the path is up.
        gview[s].lastHbAck = Simulator::Now();
        switch (h.type)
        {
        case ARQ_ACK:
            arqRic.erase({s, h.seq});
            break;
        case KPM_IND: {
            if (ArqOn())
            {
                MgmtHeader a;
                a.type = ARQ_ACK;
                a.seq = h.seq;
                SendToSat(s, a, PayloadSize(ARQ_ACK, 0));
                if (!seenAtRic.insert({s, h.seq}).second)
                {
                    ++st.arqDup;
                    break;
                }
            }
            std::vector<double> vv;
            if (!DecodeKpm(p, vv))
            {
                ++st.kpmCodecFail;
                break;
            }
            ++st.kpmDelivered;
            const Time tMeas = NanoSeconds(h.tMeasNs);
            if ((Simulator::Now() - tMeas).GetSeconds() > g.deadlineS[h.cls])
            {
                ++st.kpmDiscarded; // too old to act on in time
                break;
            }
            if (IsPal() && gview[s].leased[h.cls])
            {
                gview[s].leased[h.cls] = false; // the grant did not arrive; re-grant
            }
            if (h.cls == kNF && vv.size() == static_cast<size_t>(kNT))
            {
                std::array<int, kNT> m{};
                for (int i = 0; i < kNT; ++i)
                {
                    m[i] = static_cast<int>(std::lround(vv[i]));
                }
                rcBeam[h.tMeasNs] = GroundBeamDecision(m); // rides in the 32-byte control payload
            }
            RicDecideAndSend(s, h.cls, tMeas, h.cls == kNS ? vv[0] : 0.0);
            break;
        }
        case E2_SETUP_REQ:
            RicOnSetup(s);
            break;
        case SUB_RSP:
            gview[s].subAcked[h.cls] = true;
            break;
        case A1_ACK:
            break;
        case HB_ACK: {
            GroundView& gv = gview[s];
            gv.lastHbAck = Simulator::Now();
            if (h.flags & 1)
            {
                gv.autonomous = false;
                if (IsReactiveLike() || GroundWinsReturn())
                {
                    ReconcileGroundWins(s, h);
                }
            }
            break;
        }
        case RECON: {
            // Versioned merge of the on-board log (lease holder) with the
            // ground log, then return authority. The latest on-board demand
            // sample travels in the message, so a write made under a
            // superseded policy is recomputed at once.
            GroundView& gv = gview[s];
            const Policy cur = FromA1(*ricA1);
            const std::vector<L::Write> ol = reconSnap[s];
            const Time t0 = h.untilNs > 0 ? NanoSeconds(h.untilNs) : gv.leaseStart; // (M4b)
            auto m = L::Merge(L::MergeRule::LeaseVersioned,
                              groundLog[s],
                              ol,
                              t0,
                              Simulator::Now(),
                              cur.ver,
                              0.01);
            // Direct conflicts are counted with the same definition as on the
            // ground-wins path: the latest on-board write of the lease against
            // the ground's fresh decision from the measurement in this message
            // (D3), so the column compares like with like.
            uint32_t direct = 0;
            if (h.v2 >= 0)
            {
                std::vector<L::Write> gl = {
                    {1, Decide(h.v2, cur), cur.ver, Simulator::Now(), L::Host::Ground}};
                direct = L::Merge(L::MergeRule::GroundWins,
                                  gl,
                                  ol,
                                  t0,
                                  Simulator::Now(),
                                  cur.ver,
                                  0.01)
                             .directConflicts;
            }
            if (pendTtc.sat == s)
            {
                auto& o = st.outages[pendTtc.outageIdx];
                o.direct += direct;
                o.policyStale += m.policyStaleWrites;
                o.fallback += m.fallbackWrites;
            }
            reconSnap[s].clear();
            gv.autonomous = false;
            MgmtHeader r;
            r.type = RECON_ACK;
            r.ver = cur.ver;
            r.v1 = cur.g;
            r.v2 = cur.h;
            r.untilNs = IsLeasing() ? PredictedLeaseUntil(s).GetNanoSeconds() : 0;
            r.flags = 0;
            for (int c = 0; c < kClasses; ++c)
            {
                if (c != kRT && (h.flags & (2u << c)))
                {
                    r.flags |= (2u << c);
                    gv.leased[c] = false;
                }
            }
            const bool needRecompute = !m.recompute.empty() || h.ver < cur.ver;
            if (needRecompute && h.v2 >= 0)
            {
                const double share = Decide(h.v2, cur);
                r.flags |= 1;
                r.seq = static_cast<uint32_t>(std::lround(share * 1e6));
                r.tMeasNs = h.tMeasNs;
                RecordNsDecision(cur.ver);
                groundLog[s].push_back({1, share, cur.ver, Simulator::Now(), L::Host::Ground});
            }
            else
            {
                // Apply the merge (D5): when the authoritative write is the
                // ground's and it differs from what the satellite holds, the
                // reconciled value is sent back with the authority.
                // Compare with the share the satellite reports holding (M4c).
                const auto w = m.winner.find(1);
                if (w != m.winner.end() && w->second == L::Host::Ground && m.state.count(1) &&
                    std::fabs(h.v1 - m.state.at(1)) > 0.01)
                {
                    r.flags |= 1;
                    r.seq = static_cast<uint32_t>(std::lround(m.state.at(1) * 1e6));
                    r.tMeasNs = h.tMeasNs;
                    ++st.mergeGroundApplied;
                }
            }
            SendToSat(s, r, PayloadSize(RECON_ACK, 0));
            break;
        }
        default:
            break;
        }
    }
}

// RIC subscription timer: a subscription request whose response has not
// arrived is re-sent every second (lost requests and responses happen on the
// feeder), as an E2AP RIC subscription procedure would on timer expiry.
void
SubRetryLoop()
{
    Simulator::Schedule(Seconds(1.0), &SubRetryLoop);
    for (int s = 0; s < static_cast<int>(sats.size()); ++s)
    {
        if (!gview[s].e2 || !(IsServing(s) || gview[s].prepSent))
        {
            continue;
        }
        for (int c = 0; c < kClasses; ++c)
        {
            if (GroundNeedsSub(c) && !gview[s].subAcked[c])
            {
                MgmtHeader q;
                q.type = SUB_REQ;
                q.cls = static_cast<uint8_t>(c);
                SendToSat(s, q, PayloadSize(SUB_REQ, 0));
            }
        }
    }
}

// SMO receive (A1 acks from satellites in the onboard strategy).
std::map<int, uint32_t> satAckedVer;

void
SmoRecv(Ptr<Socket> sock)
{
    Ptr<Packet> p;
    Address from;
    while ((p = sock->RecvFrom(from)))
    {
        MgmtHeader h;
        p->RemoveHeader(h);
        if (h.type == A1_ACK)
        {
            satAckedVer[h.sat] = std::max(satAckedVer[h.sat], h.ver);
        }
    }
}

// ---------------------------------------------------------------------------
// SMO: policy updates, A1 to the RIC, and (onboard strategy) to satellites
// ---------------------------------------------------------------------------
// Spacing of A1 intent versions (D8): Poisson (the lemma's assumption),
// periodic with a random phase, or bursty (two-phase hyperexponential with
// balanced means and squared coefficient of variation a1BurstCv2). All have
// mean a1MeanS.
Ptr<UniformRandomVariable> rvA1u;
double
NextA1Gap()
{
    if (g.a1Process == "periodic")
    {
        return g.a1MeanS;
    }
    if (g.a1Process == "bursty")
    {
        const double c2 = g.a1BurstCv2;
        const double p1 = 0.5 * (1.0 + std::sqrt((c2 - 1.0) / (c2 + 1.0)));
        const bool first = rvA1u->GetValue() < p1;
        const double m = first ? g.a1MeanS / (2.0 * p1) : g.a1MeanS / (2.0 * (1.0 - p1));
        return rvA1->GetValue() * m / g.a1MeanS;
    }
    return rvA1->GetValue();
}

void
SmoIssue()
{
    Simulator::Schedule(Seconds(NextA1Gap()), &SmoIssue);
    static const double gs[4] = {0.2, 0.3, 0.4, 0.5};
    static const double hs[3] = {0.1, 0.2, 0.3};
    Policy p;
    p.ver = smoCur.ver + 1;
    // Intent values come from their own stream so that every strategy sees
    // the same sequence of intents for the same run (paired comparison).
    p.g = gs[rvIntent->GetInteger(0, 3)];
    p.h = hs[rvIntent->GetInteger(0, 2)];
    smoCur = p;
    verIssued[p.ver] = Simulator::Now().GetSeconds();
    smoMgr->UpdatePolicy(smoPolicyId, ToA1(p));
    if (IsOnboardLike() && servingIdx >= 0 && gview[servingIdx].e2)
    {
        // On-board only: the SMO pushes a new version at once (the retry loop
        // covers losses), as the RIC mirrors it in the hybrid strategies.
        MgmtHeader a;
        a.type = A1_PUT;
        a.ver = p.ver;
        a.v1 = p.g;
        a.v2 = p.h;
        a.untilNs = OnbLeaseUntilNs();
        SendToSat(servingIdx, a, PayloadSize(A1_PUT, 0), true);
    }
}

// SMO -> ground RIC over the cloud link (10 ms), modeled through the A1
// manager's delivery delay; the RIC then mirrors to satellites per strategy.
void
RicOnA1(A1NtnPolicy a)
{
    const double now = Simulator::Now().GetSeconds();
    if (a1LastAt >= 0)
    {
        a1IntervalSum += now - a1LastAt;
        ++a1IntervalN;
    }
    a1LastAt = now;
    ricA1->HandleIncomingPolicy(a);
    ricPolicy = FromA1(*ricA1);
    for (int s = 0; s < static_cast<int>(sats.size()); ++s)
    {
        if (!gview[s].e2 || !(IsServing(s) || (UsesPreprov() && gview[s].prepSent)))
        {
            continue;
        }
        if (IsReactiveLike())
        {
            PushPolicyToSat(s, IsLeasing());
        }
        else if (IsPal())
        {
            PushPolicyToSat(s, IsLeasing());
        }
    }
}

void
SmoRetryLoop()
{
    Simulator::Schedule(Seconds(g.a1RetryS), &SmoRetryLoop);
    if (!IsOnboardLike() || servingIdx < 0)
    {
        return;
    }
    const int s = servingIdx;
    if (!gview[s].e2)
    {
        return;
    }
    if (satAckedVer[s] < smoCur.ver)
    {
        MgmtHeader a;
        a.type = A1_PUT;
        a.ver = smoCur.ver;
        a.v1 = smoCur.g;
        a.v2 = smoCur.h;
        a.untilNs = OnbLeaseUntilNs();
        SendToSat(s, a, PayloadSize(A1_PUT, 0), true);
    }
}

void
EnterAutonomy(int s)
{
    Sat& sat = sats[s];
    sat.autonomous = true;
    // A satellite that already holds a lease has had authority since the
    // grant: keep its log and start (M4a). Otherwise authority starts now.
    bool leasedAny = false;
    for (int c = 0; c < kClasses; ++c)
    {
        leasedAny = leasedAny || sat.leased[c];
    }
    if (!leasedAny)
    {
        sat.autonomyStart = Simulator::Now();
        sat.log.clear();
    }
    ++st.autonomyEntries;
    if (sat.linkUp)
    {
        ++st.falseAutonomy;
    }
}

// onboard-ttl: the SMO refreshes the lease of the serving satellite's intent
// at the heartbeat period with a small A1 message that names the current
// version; the satellite renews only a snapshot of that version.
void
SmoLeaseRefreshLoop()
{
    Simulator::Schedule(Seconds(g.hbPeriodS), &SmoLeaseRefreshLoop);
    const int s = servingIdx;
    if (s < 0 || !gview[s].e2)
    {
        return;
    }
    MgmtHeader a;
    a.type = A1_PUT;
    a.flags = 2;
    a.ver = smoCur.ver;
    a.v1 = smoCur.g;
    a.v2 = smoCur.h;
    a.untilNs = OnbLeaseUntilNs();
    SendToSat(s, a, PayloadSize(A1_ACK, 0), true);
}

void
HbLoop()
{
    Simulator::Schedule(Seconds(g.hbPeriodS), &HbLoop);
    const int s = servingIdx;
    if (s < 0)
    {
        return;
    }
    Sat& sat = sats[s];
    GroundView& gv = gview[s];
    if (gv.e2)
    {
        MgmtHeader h;
        h.type = HB;
        h.ver = ricPolicy.ver;
        h.untilNs = IsLeasing() ? PredictedLeaseUntil(s).GetNanoSeconds() : 0;
        SendToSat(s, h, 0);
    }
    if (UsesPreprov() && IsLeasing())
    {
        // Provisioned successors are heartbeated too, so the lease of their
        // snapshot is measured from the last contact (D16).
        for (int n = 0; n < static_cast<int>(sats.size()); ++n)
        {
            if (n != s && gview[n].prepSent && gview[n].e2)
            {
                MgmtHeader h;
                h.type = HB;
                h.ver = ricPolicy.ver;
                h.untilNs = PredictedLeaseUntil(n).GetNanoSeconds();
                SendToSat(n, h, 0);
            }
        }
    }
    const Time to = Seconds(g.hbPeriodS * g.hbMiss);
    // Satellite side: no heartbeat for hbMiss periods -> autonomy. This holds
    // also for a satellite that has not completed E2 setup yet.
    if (sat.hbArmed && !sat.autonomous && Simulator::Now() - sat.lastHbRx > to)
    {
        EnterAutonomy(s);
    }
    // Ground side mirror.
    if (gv.hbArmed && !gv.autonomous && Simulator::Now() - gv.lastHbAck > to)
    {
        gv.autonomous = true;
    }
}

// ---------------------------------------------------------------------------
// PAL: predictive placement, lease grant before predicted loss, return on
// restoration, pre-provisioning of the next serving satellite.
// ---------------------------------------------------------------------------
Time
PredictedGroundLoop(int s)
{
    // Both feeder legs at the PREDICTED slant range to the gateway the plan
    // expects, plus backhaul both ways and ground processing.
    Sat& sat = sats[s];
    const int gw = sat.activeGw >= 0 ? sat.activeGw : sat.lastGw;
    if (gw < 0)
    {
        return Seconds(10);
    }
    Vector pos, vel;
    sat.mob->GetEcefStateAt(Seconds(sat.ephDtS), pos, vel);
    const Vector gs = planPred->GetGatewayEcef(gw);
    const double slant = CalculateDistance(pos, gs);
    Ptr<PointToPointChannel> bh =
        DynamicCast<PointToPointChannel>(gwNodes[gw]->GetDevice(1)->GetChannel());
    TimeValue tv;
    bh->GetAttribute("Delay", tv);
    return Seconds(2.0 * slant / 299792458.0) + 2 * tv.Get();
}

void
PalTick()
{
    Simulator::Schedule(MilliSeconds(100), &PalTick);
    const Time now = Simulator::Now();
    const int s = servingIdx;
    // Uses only what the RIC knows: its own E2 state and recent traffic.
    if (s >= 0 && gview[s].e2 && RicSeesUp(s))
    {
        GroundView& gv = gview[s];
        const Time remain = PredLoss(s) - now;
        // RT placement check (logged): does the rule ever put RT on the ground?
        ++rtTicks;
        if (L::Place(PredictedGroundLoop(s) + Seconds(g.procGroundS[kRT]), Seconds(g.deadlineS[kRT]),
                     g.slack, remain, Seconds(g.leadS)) == L::Host::Ground)
        {
            ++rtGroundFeasibleTicks;
        }
        uint16_t grant = 0;
        for (int c = kNF; c < kClasses; ++c)
        {
            const Time loop = PredictedGroundLoop(s) + Seconds(g.procGroundS[c]);
            const auto host = L::Place(loop, Seconds(g.deadlineS[c]), g.slack, remain,
                                       Seconds(g.leadS));
            if (host == L::Host::Onboard && !gv.leased[c])
            {
                grant |= (2u << c);
            }
        }
        if (grant)
        {
            MgmtHeader a;
            a.type = LEASE;
            a.ver = ricPolicy.ver;
            a.v1 = ricPolicy.g;
            a.v2 = ricPolicy.h;
            a.untilNs = IsLeasing() ? PredictedLeaseUntil(s).GetNanoSeconds() : 0;
            a.flags = 1 | grant;
            gv.leaseStart = now;
            for (int c = kNF; c < kClasses; ++c)
            {
                if (grant & (2u << c))
                {
                    gv.leased[c] = true;
                }
            }
            ++st.leasesGranted;
            gv.lastRenew = now;
            SendToSat(s, a, PayloadSize(LEASE, 0));
        }
        else if (now - gv.lastRenew > Seconds(30))
        {
            gv.lastRenew = now;
            PushPolicyToSat(s, IsLeasing()); // lease renewal while connected
        }
    }
}

// Satellite-side timer, every 100 ms (D18): the return of a lease granted
// for a loss that did not come uses only the satellite's own state.
void
SatTick()
{
    Simulator::Schedule(MilliSeconds(100), &SatTick);
    const Time now = Simulator::Now();
    const int s = servingIdx;
    if (s >= 0 && g.strat == Strategy::SatPred)
    {
        // Satellite-side prediction (R3-M4): the satellite evaluates its own
        // on-board contact plan (same ephemeris error as the RIC's plan) and
        // takes the near-RT classes T_lead before the predicted loss, with
        // no message from the ground. Same lead and the same return-after
        // T_lead + 1 s rule as PAL, decided from on-board state only.
        Sat& sat = sats[s];
        if (sat.predAuto && sat.autonomous && sat.linkUp &&
            now - sat.predAt > Seconds(g.leadS + 1.0))
        {
            SatResume(s); // the predicted loss did not come
        }
        else if (!sat.autonomous && sat.linkUp && sat.e2Setup && sat.hbArmed &&
                 PredLoss(s) - now < Seconds(g.leadS))
        {
            sat.autonomous = true;
            sat.predAuto = true;
            sat.predAt = now;
            sat.autonomyStart = now;
            sat.log.clear();
            ++st.satPredEntries;
        }
        return;
    }
    if (s < 0 || !IsPal() || GroundWinsReturn())
    {
        return;
    }
    Sat& sat = sats[s];
    bool held = false;
    for (int c = 0; c < kClasses; ++c)
    {
        held = held || sat.leased[c];
    }
    if (held && !sat.autonomous && now - sat.leaseGrantedAt > Seconds(g.leadS + 1.0))
    {
        sat.leaseGrantedAt = now + Seconds(3600); // one return attempt per grant
        MgmtHeader r;
        r.type = RECON;
        for (int c = 0; c < kClasses; ++c)
        {
            if (sat.leased[c])
            {
                r.flags |= (2u << c);
            }
        }
        r.ver = sat.shareVer;
        r.v1 = sat.share;
        r.v2 = Demand();
        r.untilNs = sat.autonomyStart.GetNanoSeconds(); // t0 (M4b)
        r.tMeasNs = now.GetNanoSeconds();
        reconSnap[s] = sat.log;
        SendSatMsg(s, r, PayloadSize(RECON, static_cast<uint32_t>(sat.log.size())));
    }
}

// Pre-provisioning of the next serving satellite (PAL, and the reactive
// hybrid with lease and pre-provisioning). Uses the serving schedule and the
// predicted contact plan only.
void
PreprovTick()
{
    Simulator::Schedule(MilliSeconds(100), &PreprovTick);
    const Time now = Simulator::Now();
    for (size_t k = serveSeg + 1; k < serveTl.size() && k <= serveSeg + 2; ++k)
    {
        const int n = serveTl[k].sat;
        if (n < 0 || gview[n].e2 || now - gview[n].prepAt < Seconds(1.0))
        {
            continue;
        }
        if (serveTl[k].start - now.GetSeconds() > g.preS)
        {
            break;
        }
        if (planPred->IsVisible(sats[n].id, now))
        {
            gview[n].prepSent = true;
            gview[n].prepAt = now;
            MgmtHeader h;
            h.type = PREP;
            SendToSat(n, h, PayloadSize(PREP, 0));
        }
    }
}

void
OnFeederUp(int s)
{
    Sat& sat = sats[s];
    FlushBuffer(s);
    if (IsPal() && !GroundWinsReturn())
    {
        SatSendRecon(s);
    }
    if (GroundWinsReturn())
    {
        // Ablation: ground-wins return. Leases are dropped at once and the
        // resume path of the reactive hybrid reconciles.
        bool any = false;
        for (int c = 0; c < kClasses; ++c)
        {
            any = any || sat.leased[c];
            sat.leased[c] = false;
            gview[s].leased[c] = false;
        }
        if (any)
        {
            sat.autonomous = true;
        }
    }
    // Every hybrid reports at link-up, not at the next heartbeat, so the
    // return paths of all strategies start from the same trigger.
    if ((IsReactiveLike() || GroundWinsReturn()) && sat.autonomous && sat.e2Setup)
    {
        SatResume(s);
    }
    if ((IsServing(s) || sat.provisioned) && !sat.e2Setup)
    {
        StartE2Setup(s);
    }
}

// ---------------------------------------------------------------------------
// Topology: feeder selection, link state, delays, serving schedule
// ---------------------------------------------------------------------------
bool
RainAt(int gw, double t)
{
    for (const auto& [a, b] : rain[gw])
    {
        if (t >= a && t < b)
        {
            return true;
        }
        if (a > t)
        {
            break;
        }
    }
    return false;
}

bool
PairVisible(const Sat& sat, int gw, Time t)
{
    for (const auto& w : planTrue->GetPairWindows(sat.id, gw))
    {
        if (t >= w.start && t < w.end)
        {
            return true;
        }
        if (w.start > t)
        {
            break;
        }
    }
    return false;
}

void
SetPairUp(Sat& sat, int gw, bool up)
{
    const double r = up ? g.feederPer : 1.0;
    sat.emSat[gw]->SetRate(r);
    sat.emGw[gw]->SetRate(r);
}

void
SetSatDefaultRoute(Sat& sat, int gw)
{
    Ipv4StaticRoutingHelper h;
    Ptr<Ipv4StaticRouting> sr = h.GetStaticRouting(sat.node->GetObject<Ipv4>());
    for (uint32_t i = sr->GetNRoutes(); i-- > 0;)
    {
        if (sr->GetRoute(i).IsDefault())
        {
            sr->RemoveRoute(i);
        }
    }
    sr->SetDefaultRoute(sat.addrGw[gw], sat.ifSat[gw]);
}

void
StartOutage(int s, int gwLost)
{
    if (inOutage)
    {
        return;
    }
    inOutage = true;
    Stats::Outage o;
    const Time now = Simulator::Now();
    o.start = now.GetSeconds();
    o.end = -1;
    // Geometric (predictable) if the gateway that was lost has a window edge
    // within the lead time of now in the TRUE plan; otherwise it was a rain
    // fade or a switch onto a satellite without contact.
    o.predicted = false;
    if (gwLost >= 0)
    {
        for (const auto& w : planTrue->GetPairWindows(sats[s].id, gwLost))
        {
            if (std::fabs((w.end - now).GetSeconds()) <= g.leadS)
            {
                o.predicted = true;
            }
        }
    }
    o.endedBySwitch = false;
    if (o.start < 2.0)
    {
        o.kind = 0;
    }
    else if (gwLost < 0)
    {
        o.kind = 3;
    }
    else
    {
        o.kind = o.predicted ? 1 : 2;
    }
    st.outages.push_back(o);
    curOutage = st.outages.size() - 1;
}

void
EndOutage(int s, bool bySwitch)
{
    if (!inOutage)
    {
        return;
    }
    inOutage = false;
    auto& o = st.outages[curOutage];
    o.end = Simulator::Now().GetSeconds();
    o.endedBySwitch = bySwitch;
    if (!bySwitch)
    {
        pendTtc.sat = s;
        pendTtc.restoreAt = o.end;
        pendTtc.outageIdx = curOutage;
        if (IsOnboardLike())
        {
            CheckTtc(s, sats[s].shareVer, true); // already consistent if current
        }
    }
}

void
TopoTick()
{
    Simulator::Schedule(MilliSeconds(20), &TopoTick);
    const Time now = Simulator::Now();
    const double t = now.GetSeconds();

    // Serving schedule.
    while (serveSeg + 1 < serveTl.size() && t >= serveTl[serveSeg + 1].start)
    {
        const int prev = servingIdx;
        ++serveSeg;
        servingIdx = serveTl[serveSeg].sat;
        if (inOutage)
        {
            EndOutage(prev, true);
        }
        if (pendTtc.sat == prev)
        {
            pendTtc.sat = -1;
        }
        if (servingIdx >= 0)
        {
            lastSwitchT = t;
            Stats::Switch sw;
            sw.t = t;
            sw.from = prev >= 0 ? sats[prev].id : 0;
            sw.to = sats[servingIdx].id;
            Sat& ns = sats[servingIdx];
            sw.preProvisioned = ns.e2Setup;
            sw.sigBytes = preSig[servingIdx].first;
            sw.sigMsgs = preSig[servingIdx].second;
            st.switches.push_back(sw);
            curSwitch = st.switches.size() - 1;
            switchPending = true;
            ns.share = g.defaultShare;
            ns.shareVer = 0;
            ns.shareDefault = true;
            ResetAppliedBeams(); // the new satellite has no beam applied yet
            ns.lastHbRx = now; // heartbeats start with the role
            ns.hbArmed = true;
            gview[servingIdx].lastHbAck = now;
            gview[servingIdx].hbArmed = true;
            gview[servingIdx].autonomous = false;
            if (ns.linkUp && !ns.e2Setup)
            {
                StartE2Setup(servingIdx);
            }
        }
        if (prev >= 0)
        {
            Sat& ps = sats[prev];
            ps.e2Setup = false;
            ps.setupPending = false;
            ps.subscribed = {false, false, false};
            ps.leased = {false, false, false};
            ps.autonomous = false;
            ps.predAuto = false;
            ps.hbArmed = false;
            ps.provisioned = false;
            ps.buf.clear();
            ps.log.clear();
            gview[prev] = GroundView();
        }
    }

    // Feeder selection and link state for every satellite in the set.
    for (size_t k = 0; k < sats.size(); ++k)
    {
        Sat& sat = sats[k];
        auto usable = [&](int gw) { return PairVisible(sat, gw, now) && !RainAt(gw, t); };
        int want = sat.activeGw;
        if (want >= 0 && !usable(want))
        {
            want = -1;
        }
        if (want < 0)
        {
            double best = -90;
            Vector pos = sat.mob->GetPosition();
            for (int gw = 0; gw < static_cast<int>(g.numGw); ++gw)
            {
                if (!usable(gw))
                {
                    continue;
                }
                const double el =
                    OranNtnContactPlan::ElevationDeg(pos, planTrue->GetGatewayEcef(gw));
                if (el > best)
                {
                    best = el;
                    want = gw;
                }
            }
        }
        const int gwBefore = sat.activeGw;
        // Rel-17 soft (make-before-break) feeder-link switch-over (D11): a
        // gateway change forced by geometry, with the next gateway already
        // usable, costs no re-acquisition gap. A change forced by a fade is
        // unplanned and stays a hard switch.
        const bool soft = g.softSwitch && sat.linkUp && gwBefore >= 0 && want >= 0 &&
                          want != gwBefore && !PairVisible(sat, gwBefore, now) &&
                          !RainAt(gwBefore, t);
        if (want != sat.activeGw)
        {
            if (sat.activeGw >= 0)
            {
                if (soft)
                {
                    const int old = sat.activeGw;
                    const size_t kk = k;
                    Simulator::Schedule(MilliSeconds(50), [kk, old]() {
                        if (sats[kk].activeGw != old)
                        {
                            SetPairUp(sats[kk], old, false);
                        }
                    });
                }
                else
                {
                    SetPairUp(sat, sat.activeGw, false);
                }
            }
            sat.activeGw = want;
            if (want >= 0)
            {
                sat.lastGw = want;
                sat.acqUntil = soft ? now : now + Seconds(g.acqS);
                SetSatDefaultRoute(sat, want);
                if (soft)
                {
                    SetPairUp(sat, want, true);
                    ++softSwitches;
                }
            }
        }
        const bool up = sat.activeGw >= 0 && now >= sat.acqUntil;
        if (up != sat.linkUp)
        {
            sat.linkUp = up;
            SetPairUp(sat, sat.activeGw >= 0 ? sat.activeGw : sat.lastGw, up);
            if (static_cast<int>(k) == servingIdx)
            {
                if (!up)
                {
                    StartOutage(static_cast<int>(k), gwBefore);
                    if (g.linkDetectS >= 0 && (IsReactiveLike() || IsPal()))
                    {
                        // Link-layer loss of carrier (D12): the satellite
                        // knows a gateway change it commands at once, and any
                        // other loss of lock after linkDetectS.
                        const bool commanded = gwBefore >= 0 && want >= 0 && want != gwBefore;
                        const int kk = static_cast<int>(k);
                        Simulator::Schedule(Seconds(commanded ? 0.0 : g.linkDetectS), [kk]() {
                            Sat& x = sats[kk];
                            if (IsServing(kk) && !x.linkUp && !x.autonomous && x.hbArmed)
                            {
                                EnterAutonomy(kk);
                            }
                        });
                    }
                }
                else
                {
                    EndOutage(static_cast<int>(k), false);
                }
            }
            if (up)
            {
                OnFeederUp(static_cast<int>(k));
            }
        }
        if (sat.activeGw >= 0)
        {
            const Vector gs = planTrue->GetGatewayEcef(sat.activeGw);
            const double d = CalculateDistance(sat.mob->GetPosition(), gs);
            sat.ch[sat.activeGw]->SetAttribute("Delay", TimeValue(Seconds(d / 299792458.0)));
        }
    }
    if (servingIdx >= 0 && !sats[servingIdx].linkUp && !inOutage)
    {
        StartOutage(servingIdx, -1);
    }
}

void
SlaTick()
{
    Simulator::Schedule(MilliSeconds(100), &SlaTick);
    if (servingIdx < 0)
    {
        return;
    }
    const Sat& sat = sats[servingIdx];
    st.servedS += 0.1;
    if (sat.linkUp)
    {
        st.linkUpServedS += 0.1;
    }
    const double r = Decide(Demand(), smoCur);
    const double a = sat.share < 0 ? g.defaultShare : sat.share;
    ++st.slaSamples;
    const int lu = sat.linkUp ? 1 : 0;
    const int vc = sat.shareVer >= smoCur.ver ? 1 : 0;
    ++sampBy[lu][vc];
    // State of the applied share: 0 configured default after a switch,
    // 1 safe fallback, 2 decided under a superseded intent, 3 current intent.
    const int cat = sat.shareDefault ? 0
                    : sat.shareVer == 0 ? 1
                    : sat.shareVer < smoCur.ver ? 2 : 3;
    ++catSamp[cat];
    catOver[cat] += std::max(0.0, a - r);
    catShort[cat] += std::max(0.0, r - a);
    st.shortSum += std::max(0.0, r - a);
    if (a < r - 0.01)
    {
        ++st.slaViol;
        ++violBy[lu][vc];
        ++catViol[cat];
        st.shortSumViol += r - a;
    }
    st.overSum += std::max(0.0, a - r);
}

// Demand: M/M/inf sessions on the critical slice.
void
Arrival()
{
    ++sessions;
    Simulator::Schedule(Seconds(rvHold->GetValue()), []() {
        if (sessions > 0)
        {
            --sessions;
        }
    });
    Simulator::Schedule(Seconds(rvArr->GetValue()), &Arrival);
}

// ---------------------------------------------------------------------------
// Output helpers
// ---------------------------------------------------------------------------
double
Quantile(std::vector<float> v, double q)
{
    if (v.empty())
    {
        return std::nan("");
    }
    std::sort(v.begin(), v.end());
    const double pos = q * (v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = std::min(v.size() - 1, lo + 1);
    return v[lo] + (pos - lo) * (v[hi] - v[lo]);
}

double
Mean(const std::vector<float>& v)
{
    if (v.empty())
    {
        return std::nan("");
    }
    double s = 0;
    for (float x : v)
    {
        s += x;
    }
    return s / v.size();
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string strat = "pal";
    CommandLine cmd(__FILE__);
    cmd.AddValue("strategy",
                 "ground|onboard|onboard-ttl|reactive|reactive-ttl|reactive-ttl-pre|pal|pal-nopre|pal-gw|sat-pred|"
                 "pred-ho|ground-pre",
                 strat);
    cmd.AddValue("simSeconds", "simulated time (s)", g.simS);
    cmd.AddValue("shell", "550 (Walker-Delta 1584/72/1, 53 deg) or 1200 (Walker-Star 648/18, 87.9 deg)", g.shell);
    cmd.AddValue("numGw", "number of gateways (1-4)", g.numGw);
    cmd.AddValue("gwMaskDeg", "gateway elevation mask (deg)", g.gwMaskDeg);
    cmd.AddValue("serveMaskDeg", "serving elevation mask (deg)", g.serveMaskDeg);
    cmd.AddValue("feederMbps", "feeder control-plane bearer (Mb/s)", g.feederMbps);
    cmd.AddValue("feederPer", "residual feeder packet error rate", g.feederPer);
    cmd.AddValue("acqS", "feeder re-acquisition time after a gateway change (s)", g.acqS);
    cmd.AddValue("rainPerHour", "unpredicted outage rate per gateway (1/h)", g.rainPerHour);
    cmd.AddValue("rainMeanS", "mean unpredicted outage duration (s)", g.rainMeanS);
    cmd.AddValue("ephSigmaS", "along-track ephemeris timing error sigma (s)", g.ephSigmaS);
    cmd.AddValue("hbPeriodS", "heartbeat period (s)", g.hbPeriodS);
    cmd.AddValue("hbMiss", "missed heartbeats before failover", g.hbMiss);
    cmd.AddValue("leadS", "PAL handoff lead before a predicted loss (s)", g.leadS);
    cmd.AddValue("maxLeaseS", "PAL lease cap (s)", g.maxLeaseS);
    cmd.AddValue("preS", "PAL pre-provisioning horizon (s)", g.preS);
    cmd.AddValue("leaseEps", "PAL supersession bound eps of the lease horizon", g.leaseEps);
    cmd.AddValue("priorUpdateS", "PAL prior for the mean A1 update spacing (s)", g.priorUpdateS);
    cmd.AddValue("a1MeanS", "mean time between A1 policy updates (s)", g.a1MeanS);
    cmd.AddValue("a1RetryS", "SMO A1 retry period, onboard strategy (s)", g.a1RetryS);
    cmd.AddValue("procScaleOnboard", "scale factor on on-board processing times", g.procScaleOnboard);
    cmd.AddValue("softSwitch", "Rel-17 soft feeder-link switch-over on geometric gateway changes", g.softSwitch);
    cmd.AddValue("linkDetectS", "link-layer loss-of-carrier detection delay (s); <0 off", g.linkDetectS);
    cmd.AddValue("arqRtoS", "SCTP-like retransmission timeout for KPM/control (s); 0 off", g.arqRtoS);
    cmd.AddValue("arqMax", "maximum retransmissions per datagram", g.arqMax);
    cmd.AddValue("a1Process", "A1 update process: poisson|periodic|bursty", g.a1Process);
    cmd.AddValue("a1BurstCv2", "squared CV of the bursty update process", g.a1BurstCv2);
    cmd.AddValue("nfAccGround", "decision accuracy of the ground (full) NF xApp", g.nfAccGround);
    cmd.AddValue("safeG", "guaranteed share of the safe intent", g.safeG);
    cmd.AddValue("safeH", "headroom of the safe intent", g.safeH);
    cmd.AddValue("outputDir", "output directory", g.outDir);
    cmd.AddValue("run", "RNG run (seed index)", g.run);
    cmd.Parse(argc, argv);
    g.stratName = strat;
    if (strat == "ground")
        g.strat = Strategy::Ground;
    else if (strat == "onboard")
        g.strat = Strategy::Onboard;
    else if (strat == "reactive")
        g.strat = Strategy::Reactive;
    else if (strat == "reactive-ttl")
        g.strat = Strategy::ReactiveTtl;
    else if (strat == "reactive-ttl-pre")
        g.strat = Strategy::ReactiveTtlPre;
    else if (strat == "pal")
        g.strat = Strategy::Pal;
    else if (strat == "pal-nopre")
        g.strat = Strategy::PalNoPre;
    else if (strat == "pal-gw")
        g.strat = Strategy::PalGw;
    else if (strat == "onboard-ttl")
        g.strat = Strategy::OnboardTtl;
    else if (strat == "sat-pred")
        g.strat = Strategy::SatPred;
    else if (strat == "pred-ho")
        g.strat = Strategy::PredHo;
    else if (strat == "ground-pre")
        g.strat = Strategy::GroundPre;
    else
        NS_ABORT_MSG("unknown strategy " << strat);
    NS_ABORT_MSG_IF(g.numGw < 1 || g.numGw > 4, "numGw must be 1..4");

    RngSeedManager::SetSeed(20261016);
    RngSeedManager::SetRun(g.run);
    rvU = CreateObject<UniformRandomVariable>();
    rvIntent = CreateObject<UniformRandomVariable>();
    rvIntent->SetStream(15);
    rvArr = CreateObject<ExponentialRandomVariable>();
    rvArr->SetAttribute("Mean", DoubleValue(g.arrivalMeanS));
    rvHold = CreateObject<ExponentialRandomVariable>();
    rvHold->SetAttribute("Mean", DoubleValue(g.holdMeanS));
    rvA1 = CreateObject<ExponentialRandomVariable>();
    rvA1->SetAttribute("Mean", DoubleValue(g.a1MeanS));
    // Scenario randomness that must be identical across strategies for the
    // same run (epoch, rain, ephemeris error) comes from a separate stream.
    Ptr<UniformRandomVariable> rvScen = CreateObject<UniformRandomVariable>();
    rvScen->SetStream(1000);
    Ptr<NormalRandomVariable> rvEph = CreateObject<NormalRandomVariable>();
    rvEph->SetStream(1001);
    Ptr<ExponentialRandomVariable> rvRainGap = CreateObject<ExponentialRandomVariable>();
    rvRainGap->SetStream(1002);
    Ptr<ExponentialRandomVariable> rvRainDur = CreateObject<ExponentialRandomVariable>();
    rvRainDur->SetStream(1003);
    rvU->SetStream(10);
    rvArr->SetStream(11);
    rvHold->SetStream(12);
    rvA1->SetStream(13);
    rvA1u = CreateObject<UniformRandomVariable>();
    rvA1u->SetStream(16);
    rvBeam = CreateObject<UniformRandomVariable>();
    rvBeam->SetStream(17); // beam processes: identical for every strategy of a seed
    rvDec = CreateObject<UniformRandomVariable>();
    rvDec->SetStream(18);

    // ---- Constellation ----------------------------------------------------
    const double epoch = 1767225600.0 + rvScen->GetValue(0.0, 86400.0); // 2026-01-01 + U(0,1 d)
    ntncon::WalkerConfig wc;
    wc.epoch_unix_s = epoch;
    std::vector<ntncon::KeplerianElements> el;
    if (g.shell == 550)
    {
        wc.inclination_deg = 53.0;
        wc.total_sats = 1584;
        wc.num_planes = 72;
        wc.phasing_f = 1;
        wc.altitude_km = 550.0;
        el = ntncon::WalkerConstellation::BuildDelta(wc);
    }
    else if (g.shell == 1200)
    {
        wc.inclination_deg = 87.9;
        wc.total_sats = 648;
        wc.num_planes = 18;
        wc.phasing_f = 1;
        wc.altitude_km = 1200.0;
        el = ntncon::WalkerConstellation::BuildStar(wc);
    }
    else
    {
        NS_ABORT_MSG("shell must be 550 or 1200");
    }
    std::vector<Ptr<ntncon::Sgp4MobilityModel>> allMob(el.size());
    for (size_t i = 0; i < el.size(); ++i)
    {
        allMob[i] = CreateObject<ntncon::Sgp4MobilityModel>();
        allMob[i]->SetElements(el[i]);
    }
    const Vector cell = OranNtnContactPlan::GeodeticToEcef(g.cellLat, g.cellLon, 0.0);

    // ---- Serving schedule (1 s resolution, true orbits) --------------------
    {
        int cur = -1;
        double segStart = 0;
        std::vector<std::pair<double, int>> raw; // (start, elementIdx)
        const int steps = static_cast<int>(g.simS) + 1;
        for (int k = 0; k <= steps; ++k)
        {
            const double t = k;
            auto elev = [&](size_t i) {
                Vector p, v;
                allMob[i]->GetEcefStateAt(Seconds(t), p, v);
                return OranNtnContactPlan::ElevationDeg(p, cell);
            };
            bool keep = cur >= 0 && elev(cur) >= g.serveMaskDeg;
            if (!keep)
            {
                // Refine the instant the serving satellite crossed the mask
                // (bisection to 1 ms), so switch times are continuous and not
                // locked to whole seconds; the successor is chosen there.
                double tc = t;
                if (cur >= 0 && k > 0)
                {
                    auto elevAt = [&](size_t i, double tt) {
                        Vector p, v;
                        allMob[i]->GetEcefStateAt(Seconds(tt), p, v);
                        return OranNtnContactPlan::ElevationDeg(p, cell);
                    };
                    double lo = t - 1.0, hi = t;
                    while (hi - lo > 1e-3)
                    {
                        const double mid = 0.5 * (lo + hi);
                        if (elevAt(cur, mid) >= g.serveMaskDeg)
                        {
                            lo = mid;
                        }
                        else
                        {
                            hi = mid;
                        }
                    }
                    tc = hi;
                }
                int best = -1;
                double be = g.serveMaskDeg - 1e-6;
                for (size_t i = 0; i < allMob.size(); ++i)
                {
                    // cheap prefilter: a satellite far from the cell cannot serve
                    Vector p, v;
                    allMob[i]->GetEcefStateAt(Seconds(tc), p, v);
                    if (CalculateDistance(p, cell) > 3.5e6)
                    {
                        continue;
                    }
                    const double e = OranNtnContactPlan::ElevationDeg(p, cell);
                    if (e >= be)
                    {
                        be = e;
                        best = static_cast<int>(i);
                    }
                }
                if (best != cur || k == 0)
                {
                    raw.push_back({tc, best});
                    cur = best;
                }
            }
            (void)segStart;
        }
        for (size_t i = 0; i < raw.size(); ++i)
        {
            const int e = raw[i].second;
            int idx = -1;
            if (e >= 0)
            {
                if (!satIdx.count(e))
                {
                    satIdx[e] = static_cast<int>(sats.size());
                    Sat s;
                    s.id = static_cast<uint32_t>(e);
                    s.mob = allMob[e];
                    sats.push_back(s);
                }
                idx = satIdx[e];
            }
            const double end = (i + 1 < raw.size()) ? raw[i + 1].first : g.simS + 1;
            serveTl.push_back({raw[i].first, end, idx});
        }
    }
    allMob.clear();
    gview.resize(sats.size());

    // ---- Contact plans (true and predicted) --------------------------------
    planTrue = CreateObject<OranNtnContactPlan>();
    planPred = CreateObject<OranNtnContactPlan>();
    for (uint32_t j = 0; j < g.numGw; ++j)
    {
        planTrue->AddGateway(j, kGw[j].lat, kGw[j].lon, 0.0, g.gwMaskDeg);
        planPred->AddGateway(j, kGw[j].lat, kGw[j].lon, 0.0, g.gwMaskDeg);
    }
    for (auto& s : sats)
    {
        s.ephDtS = rvEph->GetValue(0.0, g.ephSigmaS * g.ephSigmaS);
        auto mob = s.mob;
        const double dt = s.ephDtS;
        planTrue->AddSatellite(s.id, [mob](Time t) {
            Vector p, v;
            mob->GetEcefStateAt(t - Simulator::Now(), p, v);
            return p;
        });
        planPred->AddSatellite(s.id, [mob, dt](Time t) {
            Vector p, v;
            mob->GetEcefStateAt(t + Seconds(dt) - Simulator::Now(), p, v);
            return p;
        });
    }
    planTrue->Build(Seconds(0), Seconds(g.simS + 700), Seconds(2), MilliSeconds(10));
    planPred->Build(Seconds(0), Seconds(g.simS + 700), Seconds(2), MilliSeconds(10));

    // Rain-fade (unpredicted) outages per gateway.
    rain.resize(g.numGw);
    for (uint32_t j = 0; j < g.numGw; ++j)
    {
        if (g.rainPerHour <= 0)
        {
            continue;
        }
        rvRainGap->SetAttribute("Mean", DoubleValue(3600.0 / g.rainPerHour));
        rvRainDur->SetAttribute("Mean", DoubleValue(g.rainMeanS));
        double t = rvRainGap->GetValue();
        while (t < g.simS)
        {
            const double d = rvRainDur->GetValue();
            rain[j].push_back({t, t + d});
            t += d + rvRainGap->GetValue();
        }
    }

    // ---- Network -----------------------------------------------------------
    smoNode = CreateObject<Node>();
    ricNode = CreateObject<Node>();
    for (uint32_t j = 0; j < g.numGw; ++j)
    {
        gwNodes.push_back(CreateObject<Node>());
    }
    for (auto& s : sats)
    {
        s.node = CreateObject<Node>();
    }
    InternetStackHelper inet;
    inet.Install(smoNode);
    inet.Install(ricNode);
    for (auto& n : gwNodes)
    {
        inet.Install(n);
    }
    for (auto& s : sats)
    {
        inet.Install(s.node);
    }
    Ipv4StaticRoutingHelper srh;
    PointToPointHelper cloud;
    cloud.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    cloud.SetChannelAttribute("Delay", StringValue("10ms"));
    NetDeviceContainer dSmo = cloud.Install(smoNode, ricNode);
    Ipv4AddressHelper ah;
    ah.SetBase("10.0.0.0", "255.255.255.252");
    Ipv4InterfaceContainer iSmo = ah.Assign(dSmo);
    smoAddr = iSmo.GetAddress(0);
    ricAddrToSmo = iSmo.GetAddress(1);
    srh.GetStaticRouting(smoNode->GetObject<Ipv4>())->SetDefaultRoute(ricAddrToSmo, 1);

    const Vector ricPos = OranNtnContactPlan::GeodeticToEcef(kRicLat, kRicLon, 0.0);
    for (uint32_t j = 0; j < g.numGw; ++j)
    {
        const Vector gp = OranNtnContactPlan::GeodeticToEcef(kGw[j].lat, kGw[j].lon, 0.0);
        // Fibre: 1.5 x straight-line distance at 2e8 m/s, plus 2 ms switching.
        const double bhS = 0.002 + 1.5 * CalculateDistance(gp, ricPos) / 2e8;
        PointToPointHelper bh;
        bh.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
        bh.SetChannelAttribute("Delay", TimeValue(Seconds(bhS)));
        NetDeviceContainer d = bh.Install(ricNode, gwNodes[j]); // gw device index 1
        char base[32];
        std::snprintf(base, sizeof(base), "10.1.%u.0", j);
        ah.SetBase(base, "255.255.255.252");
        Ipv4InterfaceContainer ii = ah.Assign(d);
        ricAddrToGw.push_back(ii.GetAddress(0));
        srh.GetStaticRouting(gwNodes[j]->GetObject<Ipv4>())->SetDefaultRoute(ii.GetAddress(0), 1);
        char net[32];
        std::snprintf(net, sizeof(net), "10.%u.0.0", 10 + j);
        srh.GetStaticRouting(ricNode->GetObject<Ipv4>())
            ->AddNetworkRouteTo(Ipv4Address(net), Ipv4Mask("255.255.0.0"), ii.GetAddress(1),
                                ricNode->GetObject<Ipv4>()->GetInterfaceForDevice(d.Get(0)));
    }
    for (size_t k = 0; k < sats.size(); ++k)
    {
        Sat& s = sats[k];
        for (uint32_t j = 0; j < g.numGw; ++j)
        {
            PointToPointHelper fl;
            fl.SetDeviceAttribute("DataRate",
                                  DataRateValue(DataRate(static_cast<uint64_t>(g.feederMbps * 1e6))));
            fl.SetChannelAttribute("Delay", StringValue("5ms"));
            fl.SetQueue("ns3::DropTailQueue<Packet>", "MaxSize", StringValue("200p"));
            NetDeviceContainer d = fl.Install(gwNodes[j], s.node);
            char base[32];
            std::snprintf(base, sizeof(base), "10.%u.%zu.%zu", 10 + j, k / 64, (k % 64) * 4);
            ah.SetBase(base, "255.255.255.252");
            Ipv4InterfaceContainer ii = ah.Assign(d);
            s.devGw.push_back(DynamicCast<PointToPointNetDevice>(d.Get(0)));
            s.devSat.push_back(DynamicCast<PointToPointNetDevice>(d.Get(1)));
            s.ch.push_back(DynamicCast<PointToPointChannel>(d.Get(0)->GetChannel()));
            s.addrGw.push_back(ii.GetAddress(0));
            s.addrSat.push_back(ii.GetAddress(1));
            s.ifSat.push_back(s.node->GetObject<Ipv4>()->GetInterfaceForDevice(d.Get(1)));
            Ptr<RateErrorModel> e1 = CreateObject<RateErrorModel>();
            e1->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
            e1->SetRate(1.0);
            e1->SetRandomVariable(rvU);
            Ptr<RateErrorModel> e2 = CreateObject<RateErrorModel>();
            e2->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
            e2->SetRate(1.0);
            e2->SetRandomVariable(rvU);
            s.devSat.back()->SetReceiveErrorModel(e1);
            s.devGw.back()->SetReceiveErrorModel(e2);
            s.emSat.push_back(e1);
            s.emGw.push_back(e2);
        }
        // Satellite-side Space RIC A1 termination (module adapter).
        s.a1 = CreateObject<OranNtnA1Adapter>();
        // Every satellite carries the operator's initial policy (version 1)
        // from launch-time configuration; leases apply to later versions.
        s.sock = Socket::CreateSocket(s.node, UdpSocketFactory::GetTypeId());
        s.sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), kSatPort));
        const int idx = static_cast<int>(k);
        s.sock->SetRecvCallback(MakeBoundCallback(&SatRecv, idx));
    }
    ricSock = Socket::CreateSocket(ricNode, UdpSocketFactory::GetTypeId());
    ricSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), kRicPort));
    ricSock->SetRecvCallback(MakeCallback(&RicRecv));
    smoSock = Socket::CreateSocket(smoNode, UdpSocketFactory::GetTypeId());
    smoSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), kSmoPort));
    smoSock->SetRecvCallback(MakeCallback(&SmoRecv));

    // ---- A1: SMO policy manager -> ground RIC adapter ----------------------
    smoMgr = CreateObject<OranNtnA1PolicyManager>();
    smoMgr->SetDeliveryDelay(MilliSeconds(10)); // SMO-RIC cloud link
    ricA1 = CreateObject<OranNtnA1Adapter>();
    smoMgr->SetDistributionCallback(MakeCallback(&RicOnA1));
    smoCur = Policy{1, 0.3, 0.2, Seconds(0)};
    verIssued[1] = 0.0;
    smoPolicyId = smoMgr->CreatePolicy(ToA1(smoCur));
    ricA1->HandleIncomingPolicy(ToA1(smoCur)); // initial provisioning
    ricPolicy = smoCur;
    // Launch-time configuration: every satellite holds version 1. Under the
    // leasing strategies that copy carries the same lease as every other
    // snapshot, counted from configuration at t = 0, so a satellite that is
    // never reached falls back to the safe intent after tau* (D2); without a
    // lease it is held forever, the A1 default.
    for (size_t k = 0; k < sats.size(); ++k)
    {
        Policy p0 = smoCur;
        p0.until = IsLeasing() ? LeaseTau() : Seconds(0);
        sats[k].snap = p0;
        sats[k].leaseTauS = IsLeasing() ? LeaseTau().GetSeconds() : 0.0;
        sats[k].a1->HandleIncomingPolicy(ToA1(p0));
    }
    ResetAppliedBeams();

    // ---- Health check: contact-plan geometry agrees with the propagator ----
    double maxElErr = 0;
    for (auto& s : sats)
    {
        Vector p = s.mob->GetPosition();
        for (uint32_t j = 0; j < g.numGw; ++j)
        {
            const double e1 = OranNtnContactPlan::ElevationDeg(p, planTrue->GetGatewayEcef(j));
            const double e2 = s.mob->GetElevationDeg(kGw[j].lat, kGw[j].lon);
            maxElErr = std::max(maxElErr, std::fabs(e1 - e2));
        }
    }

    // ---- Schedule ----------------------------------------------------------
    servingIdx = -1;
    serveSeg = 0;
    // First segment is applied by TopoTick at t=0 via a sentinel.
    serveTl.insert(serveTl.begin(), ServeSeg{-1.0, 0.0, -1});
    Simulator::Schedule(Seconds(0), &TopoTick);
    for (int c = 0; c < kClasses; ++c)
    {
        Simulator::Schedule(Seconds(1.0 + rvU->GetValue(0, g.periodS[c])), &Epoch, c);
    }
    Simulator::Schedule(MilliSeconds(50), &SlaTick);
    Simulator::Schedule(Seconds(1.0), &SubRetryLoop);
    Simulator::Schedule(Seconds(rvArr->GetValue()), &Arrival);
    // Warm the session process to stationarity: start with Poisson(lambda*T)
    // sessions already active.
    {
        // Poisson draw by inversion (ns-3.43 has no Poisson variable).
        Ptr<UniformRandomVariable> pu = CreateObject<UniformRandomVariable>();
        pu->SetStream(14);
        const double lam = g.holdMeanS / g.arrivalMeanS;
        double u = pu->GetValue(), pk = std::exp(-lam), cdf = pk;
        uint32_t k = 0;
        while (u > cdf && k < 1000)
        {
            ++k;
            pk *= lam / k;
            cdf += pk;
        }
        sessions = k;
        for (uint32_t i = 0; i < sessions; ++i)
        {
            Simulator::Schedule(Seconds(rvHold->GetValue()), []() {
                if (sessions > 0)
                {
                    --sessions;
                }
            });
        }
    }
    NS_ABORT_MSG_IF(g.a1Process != "poisson" && g.a1Process != "periodic" && g.a1Process != "bursty",
                    "a1Process must be poisson, periodic or bursty");
    Simulator::Schedule(Seconds(g.a1Process == "periodic" ? rvA1u->GetValue(0.0, g.a1MeanS) : NextA1Gap()),
                        &SmoIssue);
    if (IsOnboardLike())
    {
        Simulator::Schedule(Seconds(g.a1RetryS), &SmoRetryLoop);
    }
    Simulator::Schedule(Seconds(0.1), &SatTick);
    if (g.strat == Strategy::OnboardTtl)
    {
        Simulator::Schedule(Seconds(0.45), &SmoLeaseRefreshLoop);
    }
    Simulator::Schedule(MilliSeconds(5), &BeamTick);
    if (UsesHeartbeat())
    {
        Simulator::Schedule(Seconds(0.5), &HbLoop);
    }
    if (IsPal())
    {
        Simulator::Schedule(Seconds(0.55), &PalTick);
    }
    if (UsesPreprov())
    {
        Simulator::Schedule(Seconds(0.6), &PreprovTick);
    }

    Simulator::Stop(Seconds(g.simS));
    Simulator::Run();

    // Close an outage still open at the end.
    if (inOutage)
    {
        st.outages[curOutage].end = g.simS;
        st.outages[curOutage].endedBySwitch = true;
        st.outages[curOutage].kind = 4;
    }

    // ---- Output ------------------------------------------------------------
    SystemPath::MakeDirectories(g.outDir);
    {
        std::ofstream f(g.outDir + "/summary.csv");
        f << "strategy,shell,numGw,run,served_s,feeder_avail,max_el_err_deg,sats_in_set,"
             "switches,outages,outages_closed";
        for (int c = 0; c < kClasses; ++c)
        {
            const std::string n = kClassName[c];
            f << "," << n << "_opp," << n << "_hit," << n << "_applied," << n << "_late," << n
              << "_lost_switch," << n << "_hit_ratio," << n << "_p50_ms," << n << "_p95_ms," << n
              << "_p99_ms," << n << "_onboard_frac";
        }
        f << ",sla_viol_frac,overprov_mean,ns_decisions,superseded_frac,stale_mean_s,stale_p95_s,"
             "policy_age_mean_s,conflicts_per_outage,policy_conflicts_per_outage,ttc_mean_s,"
             "ttc_p95_s,ttc_n,blackout_mean_s,blackout_p95_s,blackout_n,sig_bytes_per_switch,"
             "sig_msgs_per_switch,preprov_frac,feeder_bytes,feeder_kbps,autonomy_entries,"
             "false_autonomy,leases,kpm_delivered,kpm_discarded,ctrl_lost,kpm_codec_fail,fallback_frac,"
             "hb_kbps,lease_kbps,e2setup_kbps,kpm_kbps,ctrl_kbps,a1_kbps,recon_kbps,lease_tau_s,merge_ground_applied,"
             "nf_onb_decisions,ns_onb_decisions,arq_retx,arq_dup,arq_kbps,soft_switches,beam_samples";
        for (int i = 0; i < kNT; ++i)
        {
            for (int j = 0; j < kNG; ++j)
            {
                char nm[64];
                std::snprintf(nm, sizeof(nm), ",beam_mis_T%g_d%g", kDwell[i], kGap[j]);
                f << nm;
            }
        }
        f << ",short_depth_mean,short_depth_cond,sat_pred_entries";
        f << "\n";
        uint32_t closed = 0;
        std::vector<float> ttc, conf, pconf, bo;
        for (const auto& o : st.outages)
        {
            if (!o.endedBySwitch)
            {
                ++closed;
                conf.push_back(o.direct);
                pconf.push_back(o.policyStale);
                if (o.ttc >= 0)
                {
                    ttc.push_back(o.ttc);
                }
            }
        }
        double sigB = 0, sigM = 0, pre = 0;
        size_t realSw = 0;
        for (const auto& s : st.switches)
        {
            if (s.t < 1.0)
            {
                continue; // the initial attach is not a switch (D1, D29)
            }
            if (s.blackoutNf >= 0)
            {
                bo.push_back(s.blackoutNf);
            }
            ++realSw;
            sigB += s.sigBytes;
            sigM += s.sigMsgs;
            pre += s.preProvisioned ? 1 : 0;
        }
        const double nsw = std::max<size_t>(1, realSw);
        f << g.stratName << "," << g.shell << "," << g.numGw << "," << g.run << "," << st.servedS
          << "," << (st.servedS > 0 ? st.linkUpServedS / st.servedS : 0) << "," << maxElErr << ","
          << sats.size() << "," << realSw << "," << st.outages.size() << "," << closed;
        for (int c = 0; c < kClasses; ++c)
        {
            f << "," << st.opp[c] << "," << st.hit[c] << "," << st.applied[c] << "," << st.late[c]
              << "," << st.lostSwitch[c] << ","
              << (st.opp[c] ? double(st.hit[c]) / st.opp[c] : 0) << ","
              << Quantile(st.loopMs[c], 0.5) << "," << Quantile(st.loopMs[c], 0.95) << ","
              << Quantile(st.loopMs[c], 0.99) << ","
              << (st.opp[c] ? double(st.hostOnboardEpochs[c]) / st.opp[c] : 0);
        }
        f << "," << (st.slaSamples ? double(st.slaViol) / st.slaSamples : 0) << ","
          << (st.slaSamples ? st.overSum / st.slaSamples : 0) << "," << st.nsDecisions << ","
          << (st.nsDecisions > nsFallback ? double(st.nsSuperseded) / (st.nsDecisions - nsFallback) : 0) << ","
          << Mean(st.staleS) << "," << Quantile(st.staleS, 0.95) << "," << Mean(st.policyAgeS)
          << "," << (conf.empty() ? 0 : Mean(conf)) << "," << (pconf.empty() ? 0 : Mean(pconf))
          << "," << Mean(ttc) << "," << Quantile(ttc, 0.95) << "," << ttc.size() << ","
          << Mean(bo) << "," << Quantile(bo, 0.95) << "," << bo.size() << "," << sigB / nsw << ","
          << sigM / nsw << "," << pre / nsw << "," << st.feederBytes << ","
          << st.feederBytes * 8.0 / 1e3 / g.simS << "," << st.autonomyEntries << ","
          << st.falseAutonomy << "," << st.leasesGranted << "," << st.kpmDelivered << ","
          << st.kpmDiscarded << "," << st.ctrlLost << "," << st.kpmCodecFail << ","
          << (st.nsDecisions ? double(nsFallback) / st.nsDecisions : 0);
        auto kbps = [&](std::initializer_list<int> types) {
            uint64_t b = 0;
            for (int t : types)
            {
                b += st.bytesByType[t];
            }
            return b * 8.0 / 1e3 / g.simS;
        };
        f << "," << kbps({HB, HB_ACK}) << "," << kbps({LEASE, PREP}) << ","
          << kbps({E2_SETUP_REQ, E2_SETUP_RSP, SUB_REQ, SUB_RSP}) << "," << kbps({KPM_IND}) << ","
          << kbps({RC_CTRL}) << "," << kbps({A1_PUT, A1_ACK}) << "," << kbps({RECON, RECON_ACK})
          << "," << LeaseTau().GetSeconds() << "," << st.mergeGroundApplied << ","
          << st.nfOnbDecisions << "," << st.nsOnbDecisions << "," << st.arqRetx << "," << st.arqDup
          << "," << kbps({ARQ_ACK}) << "," << softSwitches << "," << beamSamp;
        for (int i = 0; i < kNT; ++i)
        {
            for (int j = 0; j < kNG; ++j)
            {
                f << "," << (beamSamp ? double(beamMis[i][j]) / beamSamp : 0.0);
            }
        }
        f << "," << (st.slaSamples ? st.shortSum / st.slaSamples : 0) << ","
          << (st.slaViol ? st.shortSumViol / st.slaViol : 0) << "," << st.satPredEntries;
        f << "\n";
    }
    {
        std::ofstream f(g.outDir + "/outages.csv");
        f << "start,end,dur,predicted,ended_by_switch,direct_conflicts,policy_stale,ttc,kind,fallback\n";
        for (const auto& o : st.outages)
        {
            f << o.start << "," << o.end << "," << (o.end - o.start) << "," << o.predicted << ","
              << o.endedBySwitch << "," << o.direct << "," << o.policyStale << "," << o.ttc
              << "," << o.kind << "," << o.fallback << "\n";
        }
    }
    {
        std::ofstream f(g.outDir + "/switches.csv");
        f << "t,from,to,pre_provisioned,blackout_nf_s,sig_bytes,sig_msgs,first_epoch_s\n";
        for (const auto& s : st.switches)
        {
            f << s.t << "," << s.from << "," << s.to << "," << s.preProvisioned << ","
              << s.blackoutNf << "," << s.sigBytes << "," << s.sigMsgs << "," << s.firstEpochS
              << "\n";
        }
    }
    {
        // Loop-time histogram per class, 0.25 ms bins (sparse).
        std::ofstream f(g.outDir + "/loop_hist.csv");
        f << "cls,bin_ms,count\n";
        for (int c = 0; c < kClasses; ++c)
        {
            std::map<int, uint64_t> h;
            for (float x : st.loopMs[c])
            {
                h[static_cast<int>(x / 0.25f)]++;
            }
            for (const auto& [b, n] : h)
            {
                f << kClassName[c] << "," << b * 0.25 << "," << n << "\n";
            }
        }
    }
    {
        // Late-but-applied epochs are removed from nfPending when applied, so
        // nfMissed holds epochs never actuated (late actuations are counted
        // separately in summary.csv).
        for (const auto& kv : nfPending)
        {
            nfMissed.insert(kv);
        }
        std::ofstream f(g.outDir + "/nf_miss.csv");
        f << "t,link_up,onboard,e2_setup,subscribed,autonomous,leased,after_switch\n";
        for (const auto& [t, c] : nfMissed)
        {
            f << t * 1e-9 << "," << (c & 1) << "," << ((c >> 1) & 1) << "," << ((c >> 2) & 1)
              << "," << ((c >> 3) & 1) << "," << ((c >> 4) & 1) << "," << ((c >> 5) & 1) << ","
              << ((c >> 6) & 1) << "\n";
        }
    }
    {
        std::ofstream f(g.outDir + "/checks.csv");
        f << "rt_ticks,rt_ground_feasible_ticks,samp_down_stale,samp_down_cur,samp_up_stale,samp_up_cur,"
             "viol_down_stale,viol_down_cur,viol_up_stale,viol_up_cur,"
             "samp_default,viol_default,over_default,samp_fallback,viol_fallback,over_fallback,"
             "samp_stale,viol_stale,over_stale,samp_current,viol_current,over_current,"
             "short_default,short_fallback,short_stale,short_current\n"
          << rtTicks << "," << rtGroundFeasibleTicks << "," << sampBy[0][0] << "," << sampBy[0][1] << ","
          << sampBy[1][0] << "," << sampBy[1][1] << "," << violBy[0][0] << "," << violBy[0][1] << ","
          << violBy[1][0] << "," << violBy[1][1];
        for (int k = 0; k < 4; ++k)
        {
            f << "," << catSamp[k] << "," << catViol[k] << "," << catOver[k];
        }
        for (int k = 0; k < 4; ++k)
        {
            f << "," << catShort[k];
        }
        f << "\n";
    }
    std::printf("# feeder-mgmt %s shell=%u gw=%u run=%u served=%.0fs avail=%.3f sats=%zu "
                "switches=%zu outages=%zu nf_hit=%.4f ns_hit=%.4f rt_hit=%.4f sla_viol=%.4f "
                "maxElErr=%.3f kpm=%lu codecFail=%lu\n",
                g.stratName.c_str(), g.shell, g.numGw, g.run, st.servedS,
                st.servedS > 0 ? st.linkUpServedS / st.servedS : 0.0, sats.size(),
                st.switches.size(), st.outages.size(),
                st.opp[kNF] ? double(st.hit[kNF]) / st.opp[kNF] : 0.0,
                st.opp[kNS] ? double(st.hit[kNS]) / st.opp[kNS] : 0.0,
                st.opp[kRT] ? double(st.hit[kRT]) / st.opp[kRT] : 0.0,
                st.slaSamples ? double(st.slaViol) / st.slaSamples : 0.0, maxElErr,
                (unsigned long)st.kpmDelivered, (unsigned long)st.kpmCodecFail);
    Simulator::Destroy();
    return 0;
}

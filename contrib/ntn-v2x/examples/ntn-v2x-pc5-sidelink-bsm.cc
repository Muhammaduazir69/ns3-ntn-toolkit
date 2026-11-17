/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// NR PC5 sidelink (V1) — vehicles broadcast SAE J2735 BSMs directly over PC5
// with autonomous Mode-2 resource selection (TS 38.321 §5.22), NO gNB in the
// loop. Reports the TS 38.885 KPI: Packet Reception Ratio (PRR) vs distance.
//
// Contrast with ntn-v2x-real-stack (Uu relay via a satellite gNB): here the V2V
// safety layer is genuine sidelink — sensing-based selection, half-duplex, and
// co-channel collisions all shape the delivered PRR.
//
// Wave 7A: the highway is placed on the Earth (WGS-84 ECEF) under an SGP4 LEO
// satellite (ntn-constellation), each vehicle is a TR 38.811 vehicular UE
// (ntn-cho, Gauss-Markov heading about the road direction) instead of a
// constant-velocity point, and the BSM carries the vehicle's real geodetic
// position. Every vehicle is also a UE of a 5G-LENA NR-NTN cell on that
// satellite (ntn-traffic NtnRealStackHelper, TR 38.821 Set-1 S-band link
// budget): every --uuPeriodMs it sends its current J2735 BSM to a V2X
// application server over the NTN uplink (V2N, TS 23.287), and the server sends
// each vehicle a 300-byte awareness digest every 100 ms on the downlink
// (CbrApplication). Both directions carry an in-band SeqTsSizeHeader, so
// delivery, delay and IPDV are measured per direction; ue_kpi.csv,
// cell_kpi.csv and sim_health.csv are written to --outputDir.
// --uuPeriodMs=0 runs the sidelink alone.

#include "ns3/applications-module.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/ntn-realistic-mobility.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/ntn-traffic-module.h"
#include "ns3/seq-ts-size-header.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"
#include "ns3/ntn-nr-sidelink.h"
#include "ns3/ntn-v2x-bsm-header.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/nr-module.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <cstdio>
#include <map>
#include <vector>

using namespace ns3;
using ns3::ntnv2x::NtnV2xBsmHeader;

namespace
{
/// PUSCH transport blocks of the V2N uplink decoded at the gNB (nr RxPacketTraceGnb).
struct PuschStats
{
    uint64_t tbs{0};
    uint64_t corrupt{0};
    double sumSinrLin{0.0};
    uint64_t sumRb{0};
    uint64_t sumMcs{0};

    void Rx(RxPacketTraceParams p)
    {
        ++tbs;
        corrupt += p.m_corrupt ? 1 : 0;
        sumSinrLin += p.m_sinr;
        sumRb += p.m_rbAssignedNum;
        sumMcs += p.m_mcs;
    }

    double MeanSinrDb() const
    {
        return tbs > 0 ? 10.0 * std::log10(sumSinrLin / tbs) : std::nan("");
    }
};

/// In-band delay, IPDV and loss of one direction of the V2N leg.
struct UuStats
{
    uint64_t rx{0};
    double sumDelayMs{0.0};
    double maxDelayMs{0.0};
    uint64_t ipdvN{0};
    double ipdvSumMs{0.0};
    std::map<uint32_t, std::pair<uint32_t, double>> last; ///< per source: (seq, delay ms)
    uint64_t bsmDecoded{0};

    void Sample(uint32_t src, const SeqTsSizeHeader& h)
    {
        const double d = (Simulator::Now() - h.GetTs()).GetSeconds() * 1e3;
        ++rx;
        sumDelayMs += d;
        maxDelayMs = std::max(maxDelayMs, d);
        auto it = last.find(src);
        if (it != last.end() && h.GetSeq() == it->second.first + 1)
        {
            ipdvSumMs += std::fabs(d - it->second.second);
            ++ipdvN;
        }
        last[src] = {h.GetSeq(), d};
    }
};
} // namespace

int
main(int argc, char* argv[])
{
    uint32_t numVehicles = 20;
    uint32_t numSubchannels = 5;
    double spacingM = 25.0;     // inter-vehicle spacing along the highway
    double speedMps = 27.0;     // ~100 km/h
    double bsmPeriodMs = 100.0; // 10 Hz BSM cadence (SAE J2735 typical)
    double durationS = 5.0;
    double txPowerDbm = 23.0;
    std::string outputDir = "ntn-v2x-pc5-sidelink-out";
    double uuPeriodMs = 1000.0; // V2N BSM report period over the NTN uplink (0 = off)
    double altitudeKm = 600.0;
    double highwayNorthKm = 300.0;
    double gwDistanceKm = 500.0;

    CommandLine cmd;
    cmd.AddValue("numVehicles", "Vehicles on the highway", numVehicles);
    cmd.AddValue("numSubchannels", "SL-ResourcePool subchannels", numSubchannels);
    cmd.AddValue("spacingM", "Inter-vehicle spacing (m)", spacingM);
    cmd.AddValue("bsmPeriodMs", "BSM period (ms)", bsmPeriodMs);
    cmd.AddValue("duration", "Sim duration (s)", durationS);
    cmd.AddValue("txPowerDbm", "PC5 Tx power (dBm)", txPowerDbm);
    cmd.AddValue("outputDir",
                 "WF-15: directory for the persisted result. This example printed its PRR to "
                 "stdout and wrote nothing, the only one of the toolkit's examples with no "
                 "persisted result at all, so a sweep over it had nothing to collect.",
                 outputDir);
    cmd.AddValue("uuPeriodMs",
                 "Wave 7A: period of the BSM each vehicle reports to the V2X server over the "
                 "NR-NTN uplink (ms); 0 = sidelink only",
                 uuPeriodMs);
    cmd.AddValue("altitudeKm", "Satellite altitude (km)", altitudeKm);
    cmd.AddValue("highwayNorthKm",
                 "Highway start, ground distance north of the t=0 sub-satellite point (km)",
                 highwayNorthKm);
    cmd.AddValue("gwDistanceKm", "Feeder gateway ground distance north of the highway (km)",
                 gwDistanceKm);
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_IF(numVehicles < 1, "--numVehicles must be >= 1");

    // ---- SGP4 satellite (ntn-constellation) and the highway under it ----
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = 1;
    wcfg.total_sats = 80;
    wcfg.altitude_km = altitudeKm;
    wcfg.inclination_deg = 53.0;
    wcfg.epoch_unix_s = 1735689600.0; // 2025-01-01T00:00:00Z
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    Ptr<ns3::ntncon::Sgp4MobilityModel> satMob = CreateObject<ns3::ntncon::Sgp4MobilityModel>();
    satMob->SetElementsSgp4(elements[0]);
    double subLat;
    double subLon;
    double subAlt;
    satMob->GetGeodetic(subLat, subLon, subAlt);
    const double hwLat = subLat + highwayNorthKm / 111.195;
    const double hwLon0 = subLon;
    // metres east -> degrees of longitude at the highway latitude (spheroid normal radius)
    const double mPerDegLon = 111320.0 * std::cos(hwLat * M_PI / 180.0);

    // TR 38.811 vehicular UEs (ntn-cho): one shared motion engine, eastbound road.
    auto engine = std::make_shared<NtnRealisticMobilityHelper>(1);
    NodeContainer vehNodes;
    vehNodes.Create(numVehicles);

    auto ch = CreateObject<NtnSlChannel>();
    NtnSlResourcePool pool;
    pool.numSubchannels = numSubchannels;
    pool.slotDuration = MilliSeconds(1); // mu=0
    ch->SetResourcePool(pool);
    ch->SetTxPowerDbm(txPowerDbm);
    ch->SetDecodeThresholdDbm(-115.0);

    const uint32_t prsvpSlots = static_cast<uint32_t>(bsmPeriodMs); // @ 1 ms slots

    // V2X-3. Every transmission now carries a real SAE J2735 basic safety
    // message as packet bytes, and the receiver deserialises it. Before this
    // the sidelink delivered an (id, byte count) pair: NtnV2xBsmHeader existed
    // but only ever crossed the Uu path, so the README's claim that vehicles
    // broadcast BSMs over PC5 described something the code did not do.
    std::vector<uint32_t> rxPerUe(numVehicles, 0);
    std::vector<Ptr<MobilityModel>> mobs(numVehicles);
    std::vector<uint8_t> msgCnt(numVehicles, 0);
    // Per (receiver, sender) continuity bookkeeping, which is only checkable
    // because a real header now arrives.
    std::vector<std::map<uint32_t, int>> lastCnt(numVehicles);
    uint64_t bsmDecoded = 0;
    uint64_t bsmSeqBreaks = 0;
    double maxPosErrM = 0.0;

    for (uint32_t i = 0; i < numVehicles; ++i)
    {
        auto ue = CreateObject<NtnSlUeMac>();
        ue->SetUeId(i);
        RealisticUe rue;
        rue.id = i;
        rue.ueClass = VEHICULAR;
        rue.className = "vehicular";
        rue.lat = hwLat;
        rue.lon = hwLon0 + spacingM * i / mPerDegLon;
        rue.altitude_m = 0.0;
        rue.heading_rad = M_PI / 2.0; // east
        rue.meanHeading_rad = M_PI / 2.0;
        rue.speed_mps = speedMps;
        rue.vEast = speedMps;
        auto mob = CreateObject<NtnTr38811MobilityModel>();
        mob->SetUeState(rue, engine);
        vehNodes.Get(i)->AggregateObject(mob);
        ue->SetMobility(mob);
        mobs[i] = mob;
        // Selection window within the packet delay budget (one BSM period).
        // TS 38.214 Sec. 8.1.4: T2 <= remaining PDB. The transport block is
        // received at the END of its slot, so T2 = Prsvp - 1 keeps the delivery
        // inside the 100 ms budget of TS 22.185 (T2 = Prsvp landed at 101 ms).
        ue->SetSelectionWindow(1, prsvpSlots - 1);
        ue->SetReservationPeriod(prsvpSlots);
        // SAE J2735 BSMs are generated on a fixed 10 Hz clock. Without aligned
        // reselection each reselection drew a new slot relative to the CURRENT
        // transmission, so a vehicle could send two BSMs less than 100 ms apart
        // (1043 TX in a 20 x 5 s run where 1000 BSMs exist). Aligned mode keeps
        // one arrival per Prsvp and selects inside [arrival+T1, arrival+T2].
        ue->SetAlignReselection(true);
        ue->SetPacketBytes(190);
        ue->AssignStreams(1 + i);
        uint32_t self = i;

        // Build one BSM per transmission opportunity, stamped with this
        // vehicle's CURRENT kinematics and an incrementing J2735 msgCnt. The
        // highway runs along +x, so a metre of x maps to a degree of longitude
        // through a fixed scale; the point is that the receiver reads back the
        // sender's real position, not that the projection is a survey.
        NtnSlUeMac::SlTxPacketCallback txCb =
            [&mobs, &msgCnt](uint32_t id) -> Ptr<Packet> {
            // The BSM carries the vehicle's real WGS-84 position and heading.
            double lat;
            double lon;
            double alt;
            ntngeo::EcefToGeodetic(mobs[id]->GetPosition(), lat, lon, alt);
            const Vector v = mobs[id]->GetVelocity();
            NtnV2xBsmHeader h;
            h.SetFromState(msgCnt[id],
                           id,
                           static_cast<uint16_t>(Simulator::Now().GetMilliSeconds() % 60000),
                           lat,
                           lon,
                           alt,
                           v.GetLength(),
                           90.0); // eastbound road (Gauss-Markov heading about it)
            msgCnt[id] = static_cast<uint8_t>((msgCnt[id] + 1) % 128); // J2735 wraps at 127
            Ptr<Packet> pkt = Create<Packet>(190 - h.GetSerializedSize());
            pkt->AddHeader(h);
            return pkt;
        };
        ue->SetTxPacketCallback(txCb);

        NtnSlUeMac::SlRxCallback cb =
            [&rxPerUe, &lastCnt, &mobs, &bsmDecoded, &bsmSeqBreaks, &maxPosErrM, self](
                uint32_t from, Ptr<Packet> pkt) {
                rxPerUe[self]++;
                NtnV2xBsmHeader h;
                if (pkt->GetSize() < h.GetSerializedSize())
                {
                    return;
                }
                pkt->RemoveHeader(h);
                ++bsmDecoded;
                // The sender's identity must survive the air interface.
                if (h.GetId() != from)
                {
                    ++bsmSeqBreaks;
                    return;
                }
                // msgCnt continuity: consecutive BSMs from one sender differ by
                // one modulo 128. A gap is a real loss, which is exactly the
                // signal a safety application would act on.
                auto it = lastCnt[self].find(from);
                if (it != lastCnt[self].end())
                {
                    const int expected = (it->second + 1) % 128;
                    if (static_cast<int>(h.GetMsgCnt()) != expected)
                    {
                        ++bsmSeqBreaks;
                    }
                }
                lastCnt[self][from] = static_cast<int>(h.GetMsgCnt());
                // The decoded position must be the sender's actual position
                // (it moved for at most one slot since the BSM was built).
                const Vector decoded =
                    ntngeo::GeodeticToEcef(h.GetLatDeg(), h.GetLonDeg(), 0.0);
                maxPosErrM = std::max(maxPosErrM,
                                      CalculateDistance(decoded, mobs[from]->GetPosition()));
            };
        ue->SetRxCallback(cb);
        ch->AddUe(ue);
    }

    ch->Start(MilliSeconds(1), Seconds(durationS));

    // ---- Wave 7A: V2N leg over a 5G-LENA NR-NTN cell on the SGP4 satellite ----
    std::error_code mkEc;
    std::filesystem::create_directories(outputDir, mkEc);
    NtnRealStackHelper rs;
    PuschStats pusch;
    UuStats ulStats;
    UuStats dlStats;
    uint64_t ulSent = 0;
    std::vector<uint32_t> ulSeq(numVehicles, 0);
    std::vector<Ptr<Socket>> ulSocks;
    ApplicationContainer dlApps;
    const bool uu = uuPeriodMs > 0.0;
    std::function<void(uint32_t)> report; // outlives the scheduled reports
    Ipv4Address srvAddr;
    if (uu)
    {
        NodeContainer gnbNodes;
        gnbNodes.Create(1);
        gnbNodes.Get(0)->AggregateObject(satMob);
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(1);
        rs.SetSimTime(Seconds(durationS) + MilliSeconds(1));
        rs.SetOutputDir(outputDir);
        rs.SetRunTag("ntn-v2x-pc5-sidelink-bsm");
        rs.Build(gnbNodes, vehNodes);
        for (uint32_t d = 0; d < gnbNodes.Get(0)->GetNDevices(); ++d)
        {
            Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(gnbNodes.Get(0)->GetDevice(d));
            if (gnb)
            {
                gnb->GetPhy(0)->GetSpectrumPhy()->TraceConnectWithoutContext(
                    "RxPacketTraceGnb",
                    MakeCallback(&PuschStats::Rx, &pusch));
            }
        }
        if (gwDistanceKm > 0.0)
        {
            rs.SetFeederGatewayGeodetic(hwLat + gwDistanceKm / 111.195, hwLon0, 0.0);
        }
        // V2X application server on the remote host: parses SeqTs + J2735 BSM.
        const uint16_t ulPort = 5000;
        const uint16_t dlPort = 5001;
        Ptr<Socket> srv = Socket::CreateSocket(rs.GetRemoteHost(), UdpSocketFactory::GetTypeId());
        srv->Bind(InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        srv->SetRecvCallback(Callback<void, Ptr<Socket>>(
            [&ulStats](Ptr<Socket> sock) {
                Ptr<Packet> pkt;
                Address from;
                while ((pkt = sock->RecvFrom(from)))
                {
                    SeqTsSizeHeader st;
                    NtnV2xBsmHeader bsm;
                    if (pkt->GetSize() < st.GetSerializedSize() + bsm.GetSerializedSize())
                    {
                        continue;
                    }
                    pkt->RemoveHeader(st);
                    pkt->RemoveHeader(bsm);
                    ++ulStats.bsmDecoded;
                    ulStats.Sample(bsm.GetId(), st);
                }
            }));
        // Each vehicle reports its current BSM every uuPeriodMs (staggered start).
        srvAddr = rs.GetRemoteHostIpv4Address();
        for (uint32_t i = 0; i < numVehicles; ++i)
        {
            Ptr<Socket> s = Socket::CreateSocket(vehNodes.Get(i), UdpSocketFactory::GetTypeId());
            s->Bind();
            ulSocks.push_back(s);
        }
        report = [&, ulPort](uint32_t i) {
            double lat;
            double lon;
            double alt;
            ntngeo::EcefToGeodetic(mobs[i]->GetPosition(), lat, lon, alt);
            NtnV2xBsmHeader bsm;
            bsm.SetFromState(msgCnt[i],
                             i,
                             static_cast<uint16_t>(Simulator::Now().GetMilliSeconds() % 60000),
                             lat,
                             lon,
                             alt,
                             mobs[i]->GetVelocity().GetLength(),
                             90.0);
            SeqTsSizeHeader st;
            st.SetSeq(ulSeq[i]++);
            Ptr<Packet> pkt = Create<Packet>(190 - bsm.GetSerializedSize());
            pkt->AddHeader(bsm);
            st.SetSize(pkt->GetSize() + st.GetSerializedSize());
            pkt->AddHeader(st);
            ulSocks[i]->SendTo(pkt, 0, InetSocketAddress(srvAddr, ulPort));
            ++ulSent;
            if (Simulator::Now() + MilliSeconds(uuPeriodMs) < Seconds(durationS))
            {
                Simulator::Schedule(MilliSeconds(uuPeriodMs), [&report, i]() { report(i); });
            }
        };
        for (uint32_t i = 0; i < numVehicles; ++i)
        {
            Simulator::Schedule(Seconds(1.0) + MilliSeconds(uuPeriodMs * i / numVehicles),
                                [&report, i]() { report(i); });
        }
        // Downlink awareness digest: 300 B every 100 ms to each vehicle.
        for (uint32_t i = 0; i < numVehicles; ++i)
        {
            PacketSinkHelper sinkH("ns3::UdpSocketFactory",
                                   InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            sinkH.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
            ApplicationContainer sink = sinkH.Install(vehNodes.Get(i));
            sink.Get(0)->TraceConnectWithoutContext(
                "RxWithSeqTsSize",
                Callback<void,
                         Ptr<const Packet>,
                         const Address&,
                         const Address&,
                         const SeqTsSizeHeader&>(
                    [&dlStats, i](Ptr<const Packet>, const Address&, const Address&,
                                  const SeqTsSizeHeader& h) { dlStats.Sample(i, h); }));
            CbrHelper cbr("ns3::UdpSocketFactory",
                          InetSocketAddress(rs.GetUeIpv4Address(i), dlPort));
            cbr.SetConstantTraffic(MilliSeconds(100), 300);
            cbr.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
            ApplicationContainer app = cbr.Install(rs.GetRemoteHost());
            app.Start(Seconds(1.0));
            app.Stop(Seconds(durationS));
            rs.AttachExternalDlFlow(i, app.Get(0), sink.Get(0), Seconds(1.0), Seconds(durationS));
            dlApps.Add(app);
        }
    }

    Simulator::Stop(Seconds(durationS) + MilliSeconds(1));
    Simulator::Run();
    uint64_t dlSent = 0;
    for (uint32_t i = 0; i < dlApps.GetN(); ++i)
    {
        dlSent += DynamicCast<CbrApplication>(dlApps.Get(i))->GetSent() / 300;
    }
    if (uu)
    {
        rs.Collect();
        rs.WriteHealthReport();
    }

    std::printf("# NR PC5 sidelink Mode 2 — %u vehicles, %u subchannels, "
                "BSM %.0f ms, %.0f s\n",
                numVehicles, numSubchannels, bsmPeriodMs, durationS);
    std::printf("#   total TX BSMs         : %llu\n",
                static_cast<unsigned long long>(ch->GetTxTotal()));
    std::printf("#   RX attempts / success : %llu / %llu\n",
                static_cast<unsigned long long>(ch->GetRxAttemptTotal()),
                static_cast<unsigned long long>(ch->GetRxSuccessTotal()));
    std::printf("#   BSMs decoded          : %llu (J2735 header read back from PC5 bytes)\n",
                static_cast<unsigned long long>(bsmDecoded));
    std::printf("#   msgCnt discontinuities: %llu (gaps = genuine losses)\n",
                static_cast<unsigned long long>(bsmSeqBreaks));
    std::printf("#   max position error    : %.3f m (decode vs sender truth)\n", maxPosErrM);
    std::printf("#   PC5 latency (arrival->rx): mean %.3f ms, max %.3f ms over %llu receptions\n",
                ch->GetMeanDelayMs(), ch->GetMaxDelayMs(),
                static_cast<unsigned long long>(ch->GetDelaySampleCount()));
    std::printf("#   PRR (TS 38.885) by distance:\n");
    for (double r : {50.0, 100.0, 200.0, 300.0, 500.0})
    {
        std::printf("#     within %4.0f m : PRR = %.3f\n", r, ch->GetPrrWithinRange(r));
    }

    if (uu)
    {
        const double ulLoss = ulSent ? 1.0 - static_cast<double>(ulStats.rx) / ulSent : 0.0;
        const double dlLoss = dlSent ? 1.0 - static_cast<double>(dlStats.rx) / dlSent : 0.0;
        std::printf("#   V2N uplink BSM reports: sent %llu, decoded at server %llu (loss %.4f), "
                    "OWD mean %.3f ms max %.3f ms, IPDV %.3f ms\n",
                    static_cast<unsigned long long>(ulSent),
                    static_cast<unsigned long long>(ulStats.bsmDecoded), ulLoss,
                    ulStats.rx ? ulStats.sumDelayMs / ulStats.rx : 0.0, ulStats.maxDelayMs,
                    ulStats.ipdvN ? ulStats.ipdvSumMs / ulStats.ipdvN : 0.0);
        std::printf("#   V2N uplink PUSCH      : %llu TBs, %.3f corrupt, SINR %.2f dB, mean %.2f "
                    "PRB, mean MCS %.2f; UL power control + PHR PRB limit %s (P0 %.0f dBm), "
                    "PHR sent %llu received %llu\n",
                    static_cast<unsigned long long>(pusch.tbs),
                    pusch.tbs ? static_cast<double>(pusch.corrupt) / pusch.tbs : 0.0,
                    pusch.MeanSinrDb(),
                    pusch.tbs ? static_cast<double>(pusch.sumRb) / pusch.tbs : 0.0,
                    pusch.tbs ? static_cast<double>(pusch.sumMcs) / pusch.tbs : 0.0,
                    rs.IsUlPowerHeadroomPrbLimitActive() ? "on" : "off",
                    rs.GetUlP0NominalPuschDbm(),
                    static_cast<unsigned long long>(rs.GetPhrSentCount()),
                    static_cast<unsigned long long>(rs.GetPhrReceivedCount()));
        std::printf("#   V2N downlink digest   : sent %llu, received %llu (loss %.4f), OWD mean "
                    "%.3f ms, IPDV %.3f ms, DL SINR %.2f dB, TBLER %.4f\n",
                    static_cast<unsigned long long>(dlSent),
                    static_cast<unsigned long long>(dlStats.rx), dlLoss,
                    dlStats.rx ? dlStats.sumDelayMs / dlStats.rx : 0.0,
                    dlStats.ipdvN ? dlStats.ipdvSumMs / dlStats.ipdvN : 0.0, rs.GetMeanDlSinrDb(),
                    rs.GetMeanDlTbler());
        std::ofstream uuCsv(outputDir + "/v2n_uu.csv");
        uuCsv << "metric,value,unit,provenance\n"
              << "ul_bsm_sent," << ulSent << ",count,socket\n"
              << "ul_bsm_decoded," << ulStats.bsmDecoded << ",count,j2735-header\n"
              << "ul_loss_ratio," << ulLoss << ",ratio,in-band\n"
              << "ul_owd_mean_ms," << (ulStats.rx ? ulStats.sumDelayMs / ulStats.rx : 0.0)
              << ",ms,in-band SeqTsSizeHeader\n"
              << "ul_ipdv_mean_ms," << (ulStats.ipdvN ? ulStats.ipdvSumMs / ulStats.ipdvN : 0.0)
              << ",ms,in-band RFC 3393\n"
              << "dl_sent," << dlSent << ",count,CbrApplication\n"
              << "dl_rx," << dlStats.rx << ",count,PacketSink\n"
              << "dl_loss_ratio," << dlLoss << ",ratio,in-band\n"
              << "dl_owd_mean_ms," << (dlStats.rx ? dlStats.sumDelayMs / dlStats.rx : 0.0)
              << ",ms,in-band SeqTsSizeHeader\n"
              << "dl_ipdv_mean_ms," << (dlStats.ipdvN ? dlStats.ipdvSumMs / dlStats.ipdvN : 0.0)
              << ",ms,in-band RFC 3393\n"
              << "dl_sinr_db," << rs.GetMeanDlSinrDb() << ",dB,RxPacketTraceUe\n"
              << "dl_tbler," << rs.GetMeanDlTbler() << ",ratio,RxPacketTraceUe\n";
    }

    // WF-15: persist the result.
    //
    // This example printed its PRR to stdout and wrote no CSV or JSON, the only
    // one of the toolkit's examples with nothing persisted. A sweep, a
    // regression check or a figure script had nothing to read, and the numbers
    // existed only in whatever terminal happened to run it.
    {
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        const std::string path = outputDir + "/pc5_sidelink_prr.csv";
        std::ofstream csv(path);
        if (!csv)
        {
            std::printf("#   WARNING: could not write %s\n", path.c_str());
        }
        else
        {
            csv << "metric,value,unit,provenance\n";
            csv << "num_vehicles," << numVehicles << ",count,config\n";
            csv << "num_subchannels," << numSubchannels << ",count,config\n";
            csv << "spacing_m," << spacingM << ",m,config\n";
            csv << "bsm_period_ms," << bsmPeriodMs << ",ms,config\n";
            csv << "tx_power_dbm," << txPowerDbm << ",dBm,config\n";
            csv << "duration_s," << durationS << ",s,config\n";
            csv << "tx_bsms," << ch->GetTxTotal() << ",count,sidelink-mac\n";
            csv << "rx_attempts," << ch->GetRxAttemptTotal() << ",count,sidelink-mac\n";
            csv << "rx_success," << ch->GetRxSuccessTotal() << ",count,sidelink-mac\n";
            csv << "bsms_decoded," << bsmDecoded << ",count,j2735-header\n";
            csv << "msgcnt_discontinuities," << bsmSeqBreaks << ",count,j2735-header\n";
            csv << "max_position_error_m," << maxPosErrM << ",m,decode-vs-truth\n";
            csv << "pc5_owd_mean_ms," << ch->GetMeanDelayMs() << ",ms,sidelink-mac\n";
            csv << "pc5_owd_max_ms," << ch->GetMaxDelayMs() << ",ms,sidelink-mac\n";
            csv << "pc5_delay_samples," << ch->GetDelaySampleCount() << ",count,sidelink-mac\n";
            for (double r : {50.0, 100.0, 200.0, 300.0, 500.0})
            {
                csv << "prr_within_" << static_cast<int>(r) << "m,"
                    << ch->GetPrrWithinRange(r) << ",ratio,ts38885\n";
            }
            std::printf("#   wrote %s\n", path.c_str());
            // TR 37.885 6.1.6 PRR type 1 in 20 m distance bins.
            std::ofstream curve(outputDir + "/prr_type1.csv");
            curve << "bin_lo_m,bin_hi_m,rx_attempts,rx_success,half_duplex_miss,prr\n";
            for (const auto& b : ch->GetPrrType1Curve(20.0))
            {
                curve << b.loM << "," << b.hiM << "," << b.attempts << "," << b.success << ","
                      << b.halfDuplex << "," << b.prr << "\n";
            }
        }
    }

    Simulator::Destroy();
    return 0;
}

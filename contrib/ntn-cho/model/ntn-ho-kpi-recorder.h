/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * NtnHoKpiRecorder: handover KPIs MEASURED from the UE RRC traces and the UE's
 * own packet arrivals, not priced by a formula.
 *
 * What it measures, per UE and per handover (TS 38.300 9.2.3 procedure):
 *   t_start   UE RRC HandoverStart: RRCReconfiguration with reconfigurationWithSync
 *             applied at the UE (TS 38.331 5.3.5.5.2, T304 starts).
 *   t_end     UE RRC HandoverEndOk: random access on the target finished and
 *             RRCReconfigurationComplete submitted (TS 38.331 5.3.5.3, T304 stops).
 *   cplane    t_end - t_start (control-plane interruption, TS 38.133 handover
 *             interruption definition: from reconfiguration to the target being usable).
 *   uplane    first application packet received after t_end minus the last one
 *             received before t_start (user-plane gap seen by the UE).
 *   outcome   success (HandoverEndOk) or failure (HandoverEndError).
 *   ping-pong return to the source cell within a window after the previous handover
 *             (TS 38.300 15.5.2.2 MRO wording; window configurable, 1 s and 5 s reported).
 *   time of stay  time on the source cell, from the previous handover completion
 *             (or the first connection) to t_start.
 *   RLF       UE RRC RadioLinkFailure (T310 expiry, TS 38.331 5.3.10.3).
 *
 * The recorder never decides anything and never computes a delay from geometry.
 * A caller may attach a MODELED interruption (e.g. NtnChoAlgorithm's priced value)
 * and the name of the decision maker to the next handover of a UE; both are written
 * next to the measured columns, labeled as modeled, so the two cannot be confused.
 *
 * Works on the nr UE RRC (NrUeNetDevice/NrUeRrc) and on the mmwave/LTE UE RRC
 * (LteUeRrc), whose trace names are the same.
 */

#ifndef NTN_HO_KPI_RECORDER_H
#define NTN_HO_KPI_RECORDER_H

#include <ns3/address.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/ptr.h>

#include <map>
#include <string>
#include <vector>

namespace ns3
{

class Node;
class Application;

class NtnHoKpiRecorder : public Object
{
  public:
    static TypeId GetTypeId();
    NtnHoKpiRecorder();
    ~NtnHoKpiRecorder() override;

    /// One handover of one UE.
    struct HoRecord
    {
        uint32_t ueIndex{0};
        uint64_t imsi{0};
        uint16_t sourceCell{0};
        uint16_t targetCell{0};
        Time tStart{Seconds(-1)};         //!< UE HandoverStart
        Time tEnd{Seconds(-1)};           //!< UE HandoverEndOk / EndError (-1 = still open)
        Time lastRxBefore{Seconds(-1)};   //!< last app packet before tStart (-1 = none)
        Time firstRxAfter{Seconds(-1)};   //!< first app packet after tEnd (-1 = none yet)
        bool success{false};
        bool closed{false};               //!< HandoverEndOk or EndError seen
        double timeOfStayS{-1.0};         //!< on the source cell before this HO (-1 = unknown)
        bool pingPong{false};             //!< returned to the previous source within the window
        bool pingPong1s{false};
        bool pingPong5s{false};
        double modeledInterruptionMs{-1.0}; //!< attached by the caller (-1 = none)
        std::string decidedBy;              //!< attached by the caller ("" = not stated)

        double CplaneMs() const;  //!< -1 when not closed
        double UplaneGapMs() const; //!< -1 when either side is missing
    };

    /// Ping-pong window (default 1 s, TS 38.300 15.5.2.2 MRO wording).
    void SetPingPongWindow(Time w) { m_ppWindow = w; }
    Time GetPingPongWindow() const { return m_ppWindow; }

    /// Connect the UE RRC traces of this node (nr or LTE/mmwave UE RRC) and the
    /// "Rx" trace of every application already installed on it. Call after the
    /// radio is built and the traffic installed.
    void AttachUe(uint32_t ueIndex, Ptr<Node> ueNode);
    /// Connect one more receive application of a UE (anything with an "Rx" trace
    /// of signature (Ptr<const Packet>, const Address&)).
    bool AttachRxApplication(uint32_t ueIndex, Ptr<Application> app);

    /// Annotate the next handover of a UE (the one whose HandoverStart comes next)
    /// with a modeled interruption and the name of the decision maker. If the UE
    /// already has an open handover without annotation, it is annotated instead.
    void AnnotateNext(uint32_t ueIndex, double modeledInterruptionMs, const std::string& decidedBy);

    // ---- direct notifications (public so tests and other RRC stacks can drive it)
    void NotifyHandoverStart(uint32_t ueIndex, uint64_t imsi, uint16_t cellId, uint16_t targetCellId);
    void NotifyHandoverEnd(uint32_t ueIndex, uint64_t imsi, uint16_t cellId, bool ok);
    void NotifyConnected(uint32_t ueIndex, uint16_t cellId);
    void NotifyRadioLinkFailure(uint32_t ueIndex, uint16_t cellId);
    void NotifyRx(uint32_t ueIndex);

    // ---- results
    const std::vector<HoRecord>& GetRecords() const { return m_records; }
    std::vector<HoRecord> GetUeRecords(uint32_t ueIndex) const;
    uint32_t GetHandoverCount(bool successfulOnly = true) const;
    uint32_t GetHandoverFailureCount() const;
    uint32_t GetRlfCount(uint32_t ueIndex) const;
    uint32_t GetTotalRlfCount() const;
    /// Ping-pongs for an arbitrary window (recomputed from the records).
    uint32_t GetPingPongCount(Time window) const;
    double GetMeanCplaneMs() const;
    double GetMeanUplaneGapMs() const;
    /// Median application inter-arrival time of a UE over the run (ms), so the
    /// U-plane gap can be read against the traffic cadence. -1 when unknown.
    double GetMedianInterArrivalMs(uint32_t ueIndex) const;
    double GetMeanTimeOfStayS() const;

    /// handover_events.csv: measured columns plus the attached modeled ones.
    void WriteCsv(const std::string& path) const;
    /// TS 28.552 5.1.1.6.1 inter-gNB handover execution counters
    /// (MM.HoExeInterReq, MM.HoExeInterSucc, MM.HoExeInterFail) plus measured
    /// interruption statistics, one row per metric.
    void WriteKpm(const std::string& path) const;

  protected:
    void DoDispose() override;

  private:
    struct UeState
    {
        uint16_t cellId{0};          //!< serving cell according to the RRC traces
        Time servingSince{Seconds(-1)};
        int64_t openRecord{-1};      //!< index into m_records of the open HO, or -1
        int64_t pendingUplane{-1};   //!< closed record waiting for its first Rx
        Time lastRx{Seconds(-1)};
        uint32_t rlf{0};
        bool havePendingAnnotation{false};
        double pendingModeledMs{-1.0};
        std::string pendingDecidedBy;
        std::vector<double> iatMs;   //!< inter-arrival samples (bounded)
        uint16_t prevSource{0};      //!< source of the last completed handover
        Time prevEnd{Seconds(-1)};   //!< completion of the last handover
    };

    UeState& Ue(uint32_t ueIndex);
    static bool IsPingPong(const HoRecord& r, const std::vector<HoRecord>& all, Time window);

    std::map<uint32_t, UeState> m_ue;
    std::map<uint64_t, uint32_t> m_imsiToUe;
    std::vector<HoRecord> m_records;
    Time m_ppWindow{Seconds(1)};
};

} // namespace ns3

#endif // NTN_HO_KPI_RECORDER_H

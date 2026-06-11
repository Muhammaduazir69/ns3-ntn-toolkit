// SPDX-License-Identifier: GPL-2.0-only
//
// NtnCommandAndControlApp — the paper's (Deng 2026, Sec. III-C) NTN platform
// command-and-control interface as a REAL application
// (AI-Native ORAN-NTN adoption WS1). Each period it samples the node's
// REAL ns-3 MobilityModel (SGP4 satellite / TR 38.811 HAPS / UAV), derives
// attitude from the velocity frame, updates a linear battery model, and sends
// the NtnCncTelemetry record as real bytes (after an NtnOranPayloadHeader with
// payloadType = CNC_TELEMETRY, 5QI 69 mission-critical signalling) to the
// SMO endpoint, where NtnOranSink parses it (GetLatestTelemetry). The SMO
// therefore acts on telemetry that crossed the simulated network — late or
// lost C&C is a real failure mode, exactly as on an operational platform.

#ifndef NTN_COMMAND_AND_CONTROL_APP_H
#define NTN_COMMAND_AND_CONTROL_APP_H

#include "ntn-oran-sink.h"

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/traced-callback.h"

namespace ns3
{

class Socket;
class Packet;

class NtnCommandAndControlApp : public Application
{
  public:
    static TypeId GetTypeId();
    NtnCommandAndControlApp();
    ~NtnCommandAndControlApp() override;

    void SetRemote(const Address& addr) { m_remote = addr; }
    /// Battery fraction remaining right now (1.0 if no energy model configured).
    double GetBatteryFraction() const;
    uint32_t GetTxPackets() const { return m_seq; }

  protected:
    void DoDispose() override;

  private:
    void StartApplication() override;
    void StopApplication() override;
    void SendTelemetry();

    Address m_remote;
    Time m_period{Seconds(1.0)};
    uint16_t m_srcId{0};
    uint16_t m_dstId{0};
    uint8_t m_sst{1};
    uint32_t m_sd{0x000001};
    double m_batteryCapacityWh{0.0}; ///< 0 = unconstrained (e.g. GEO bus)
    double m_powerDrawW{0.0};

    Ptr<Socket> m_socket;
    EventId m_sendEvent;
    uint32_t m_seq{0};
    Time m_startTime{Seconds(0)};

    TracedCallback<Ptr<const Packet>> m_txTrace;
};

} // namespace ns3

#endif // NTN_COMMAND_AND_CONTROL_APP_H

/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_UE_LOCATION_REPORT_H
#define NTN_UE_LOCATION_REPORT_H

#include "ntn-rrc-types.h"

#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/random-variable-stream.h>
#include <ns3/traced-callback.h>
#include <ns3/vector.h>

namespace ns3
{

class MobilityModel;

namespace ntnrrc
{

/**
 * GNSS location report (TS 38.331 §5.7.4 LocationInformation IE, NTN-augmented).
 *
 * The fix is delivered in WGS-84 geodetic coordinates plus a velocity in
 * Earth-fixed metres/second. `accuracyMetres` is the 1-σ horizontal accuracy
 * the UE claims; `timestamp` is the simulator wall-clock at which the fix
 * was sampled.
 */
struct UeLocationReport
{
    Time timestamp{Seconds(0)};
    double latDeg{0.0};
    double lonDeg{0.0};
    double altMetres{0.0};
    Vector velocityEcefMps{0.0, 0.0, 0.0};
    double accuracyMetres{5.0};
    uint32_t reportSequence{0};
};

/// Trigger that drives the next report.
enum class LocationReportMode : uint8_t
{
    Periodic = 0,         //!< fire every `period`
    EventTriggered = 1,   //!< fire when UE moves > `triggerDistanceM`
    OnDemand = 2,         //!< fire only when ReportNow() is called
};

/// Convert ECEF (metres) to WGS-84 geodetic (lat°, lon°, alt m).
/// Uses Heikkinen's closed-form, accurate to <1 mm at LEO altitudes.
void EcefToGeodeticWgs84(const Vector& ecefMetres,
                         double& latDeg,
                         double& lonDeg,
                         double& altMetres);

/// Convert WGS-84 geodetic to ECEF (metres). Round-trip with the function above.
Vector GeodeticWgs84ToEcef(double latDeg, double lonDeg, double altMetres);

/**
 * \ingroup ntn-rrc
 *
 * Periodic / event-triggered / on-demand UE location reporter.
 *
 * The UE samples its mobility model, converts to WGS-84 geodetic, and adds
 * Gaussian horizontal noise of 1-σ = `accuracyMetres`. The latest report is
 * cached for retrieval; a TraceSource fires on every fresh report.
 *
 * In `EventTriggered` mode the reporter checks at fixed intervals (the
 * `period` field is reused as the polling cadence) whether the UE has
 * moved more than `triggerDistanceM` since the last report; if so, a fresh
 * report is emitted.
 */
class NtnUeLocationReporter : public Object
{
  public:
    static TypeId GetTypeId();

    NtnUeLocationReporter();
    ~NtnUeLocationReporter() override;

    void SetUeMobility(Ptr<MobilityModel> ue);
    void SetReportingMode(LocationReportMode mode);
    void SetReportingPeriod(Time period);
    void SetAccuracyMetres(double sigma);
    void SetEventTriggerDistanceMetres(double metres);
    /// Plug an explicit RNG (testing). Defaults to the ns-3 default stream.
    void SetRandomVariable(Ptr<NormalRandomVariable> rv);

    void Start();
    void Stop();
    /// Force an immediate report regardless of mode (TS 38.331 on-demand).
    UeLocationReport ReportNow();

    bool HasReport() const;
    const UeLocationReport& GetLatestReport() const;

  protected:
    void DoDispose() override;

  private:
    void Tick();
    UeLocationReport SampleReport();

    Ptr<MobilityModel> m_ue;
    LocationReportMode m_mode{LocationReportMode::Periodic};
    Time m_period{Seconds(1.0)};
    double m_accuracyMetres{5.0};
    double m_triggerDistanceM{50.0};

    Ptr<NormalRandomVariable> m_rv;
    EventId m_event;
    bool m_running{false};
    bool m_hasReport{false};
    uint32_t m_seq{0};
    UeLocationReport m_latest;
    Vector m_lastReportedEcef{0.0, 0.0, 0.0};

    TracedCallback<const UeLocationReport&> m_reportTrace;
};

} // namespace ntnrrc
} // namespace ns3

#endif // NTN_UE_LOCATION_REPORT_H

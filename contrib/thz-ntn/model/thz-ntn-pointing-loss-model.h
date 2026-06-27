// SPDX-License-Identifier: GPL-2.0-only
//
// ThzNtnPointingLossModel — re-homes the THz beam pointing impairment
// (architectural-boundary gap A2) onto the MEASURED packet plane.
//
// The composite pointing error (satellite vibration + J2 perturbation +
// P.834 refraction + tracking latency) and the resulting off-boresight
// array-gain reduction were previously computed only by the offline
// ThzNtnPointingError calculator and written to CSV; packets never saw them,
// so the measured SINR of the *-traffic examples reflected FSPL + atmosphere
// only. By wrapping ThzNtnPointingError in a real ns-3 PropagationLossModel
// (chained via NtnRealStackHelper::AddExtraPropagationLoss), the pointing loss
// now attenuates packets and appears in the MEASURED SINR — and, because a
// fresh jitter sample is drawn per call, pointing jitter shows up as honest
// SINR variance rather than a static offset.
//
// SCOPE / OPEN ITEM: the off-boresight loss uses a single-frequency array
// factor (the parabolic / Gaussian beam model in ThzNtnPointingError). Beam
// squint across the wide THz band (the beam pointing in a frequency-dependent
// direction over the occupied bandwidth) is still NOT modelled here; that
// remains an open gap.

#ifndef THZ_NTN_POINTING_LOSS_MODEL_H
#define THZ_NTN_POINTING_LOSS_MODEL_H

#include "thz-ntn-pointing-error.h"

#include "ns3/propagation-loss-model.h"

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief THz beam pointing loss as a real PropagationLossModel.
 *
 * Computes the link geometry (elevation of the higher node, satellite
 * altitude) from the Tx/Rx mobility, then asks an internal
 * ThzNtnPointingError for the off-boresight pointing loss (a positive dB
 * value) and subtracts it from the Tx power. A per-call Rayleigh jitter
 * sample is RSS-combined with the deterministic RMS misalignment so the
 * loss carries realistic variance.
 */
class ThzNtnPointingLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();
    ThzNtnPointingLossModel();
    ~ThzNtnPointingLossModel() override;

    /// Set the half-power (3 dB) beamwidth (deg) used for the loss mapping.
    void SetBeamwidth3dBDeg(double bwDeg) { m_beamwidth3dBDeg = bwDeg; }
    /// Set the RMS satellite platform vibration jitter (deg).
    void SetVibrationRmsDeg(double rmsDeg);
    /// Set the beam tracking update rate (Hz).
    void SetTrackingUpdateRateHz(double rateHz);
    /// Set the beam tracking processing latency (ms).
    void SetTrackingLatencyMs(double latencyMs);
    /// Inject an externally configured pointing-error model.
    void SetPointingError(Ptr<ThzNtnPointingError> pe);
    /// Access the internal pointing-error model (for fine configuration).
    Ptr<ThzNtnPointingError> GetPointingError() const { return m_pointingError; }
    /// Last total pointing loss applied (dB) — for logging/inspection.
    double GetLastLossDb() const { return m_lastLossDb; }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    /// Elevation (deg) of the higher node seen from the lower; robust to
    /// ECEF vs local-ENU coordinates (mirrors Ntn38811ExcessLossModel).
    double ElevationDeg(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;

    Ptr<ThzNtnPointingError> m_pointingError; //!< Underlying impairment model.
    double m_beamwidth3dBDeg{1.0};            //!< Half-power beamwidth (deg).
    mutable double m_lastLossDb{0.0};         //!< Last loss applied (dB).
};

} // namespace ns3

#endif // THZ_NTN_POINTING_LOSS_MODEL_H

// SPDX-License-Identifier: GPL-2.0-only
//
// NtnSatBeamGainModel — TR 38.811 §6.4.1 satellite/HAPS antenna beam pattern as
// a chainable ns-3 PropagationLossModel (gap A5(ii)).
//
// TR 38.811 §6.4.1 normalized circular-aperture (Airy) pattern:
//   G(theta)/Gmax = 1                                  for theta = 0
//   G(theta)/Gmax = 4 * | J1(k a sin theta) / (k a sin theta) |^2   else
// where J1 is the first-order Bessel function, k a is the dimensionless beam
// shape parameter, and theta is the off-boresight angle. The half-power crossing
// is at k a sin(theta_3dB/2) ~= 1.6163, so this model parameterises k a directly
// from the configured 3 dB beamwidth (default 4.4127 deg = TR 38.821 Set-1
// LEO-600 S-band).
//
// IMPORTANT (no double-count): on the mmwave spine the radio's phased-array
// already supplies the peak boresight gain, so this model applies ONLY the
// off-boresight ROLL-OFF (<= 0 dB, exactly 0 dB at boresight). A UE at beam edge
// then sees the correct gain drop, without re-adding the ~30 dBi peak. The
// absolute peak gain is exposed (informational) for a future nr spine that can
// disable the radio's own antenna gain.
//
// Boresight: if a beam-centre mobility is set, the beam points at it (a fixed
// cell beam, so off-centre UEs roll off); otherwise the beam tracks the rx UE
// (theta = 0 -> roll-off 0, a safe no-op default).

#ifndef NTN_SAT_BEAM_GAIN_MODEL_H
#define NTN_SAT_BEAM_GAIN_MODEL_H

#include "ns3/propagation-loss-model.h"

namespace ns3
{

class MobilityModel;

class NtnSatBeamGainModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();
    NtnSatBeamGainModel();
    ~NtnSatBeamGainModel() override;

    void SetBeamwidth3dBDeg(double d) { m_beamwidth3dBDeg = d; }
    double GetBeamwidth3dBDeg() const { return m_beamwidth3dBDeg; }
    void SetPeakGainDbi(double g) { m_peakGainDbi = g; }   ///< informational only
    double GetPeakGainDbi() const { return m_peakGainDbi; }
    /// Fixed beam boresight target (cell beam centre). If null, the beam tracks
    /// the rx node (roll-off 0). (Out-of-line: MobilityModel is forward-declared.)
    void SetBeamCenter(Ptr<MobilityModel> c);
    /// Most recent off-boresight angle (deg) and applied roll-off (dB).
    double GetLastThetaDeg() const { return m_lastThetaDeg; }
    double GetLastRolloffDb() const { return m_lastRolloffDb; }

    /// Numerical-Recipes first-order Bessel J1 (portable; no libstdc++ special
    /// math dependency).
    static double BesselJ1(double x);

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    double m_beamwidth3dBDeg{4.4127}; ///< TR 38.821 Set-1 LEO-600 S-band 3 dB BW
    double m_peakGainDbi{30.0};       ///< informational (not added; see header)
    double m_rolloffFloorDb{-40.0};   ///< deepest applied sidelobe roll-off
    Ptr<MobilityModel> m_beamCenter;
    mutable double m_lastThetaDeg{0.0};
    mutable double m_lastRolloffDb{0.0};
};

} // namespace ns3

#endif // NTN_SAT_BEAM_GAIN_MODEL_H

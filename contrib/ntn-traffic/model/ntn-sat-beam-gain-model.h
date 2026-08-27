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
// off-boresight ROLL-OFF (<= 0 dB, exactly 0 dB at boresight) BY DEFAULT. A UE
// at beam edge then sees the correct gain drop, without re-adding the ~30 dBi
// peak. Set ApplyPeakGain (default false) to make the model produce the ABSOLUTE
// pattern G(theta) = PeakGainDbi + rolloff(theta) instead — for an nr spine that
// disables the radio's own antenna gain, or for an offline pattern cut.
//
// Boresight (BoresightMode):
//   TrackUe        (DEFAULT) the beam is steered at the rx UE, so theta == 0 and
//                  the roll-off is identically 0 dB. This is a steered/tracking
//                  spot beam — and it is exactly why an end-to-end link-budget
//                  calibration run against a tracking beam sees a CONSTANT
//                  antenna-gain offset: the angle dependence is modelled, it is
//                  simply not EXERCISED when the boresight follows the terminal.
//   FixedPoint     the beam points at a fixed position (a cell beam centre),
//                  either a mobility model (SetBeamCenter) or a bare coordinate
//                  (SetBoresightFixedPoint), so a moving UE traverses the lobe.
//   FixedDirection the boresight is a fixed direction in simulation coordinates
//                  (SetBoresightFixed), i.e. an attitude-locked beam.
//   Nadir          the boresight is the satellite->geocentre direction
//                  (SetBoresightNadir), i.e. a beam fixed in the satellite body
//                  frame: the canonical TR 38.821 fixed-beam-layout case, where
//                  theta is the UE's off-nadir angle and sweeps the whole
//                  mainlobe/sidelobe structure over one pass.
// Only TrackUe/FixedPoint existed before; the other two were added for the
// angle-dependence experiment (examples/ntn-tr38821-array-gain-calibration.cc).

#ifndef NTN_SAT_BEAM_GAIN_MODEL_H
#define NTN_SAT_BEAM_GAIN_MODEL_H

#include "ns3/propagation-loss-model.h"
#include "ns3/vector.h"

#include <cstdint>

namespace ns3
{

class MobilityModel;

class NtnSatBeamGainModel : public PropagationLossModel
{
  public:
    /// Where the beam boresight points; see the file header.
    enum class BoresightMode : uint8_t
    {
        TrackUe,        ///< steered at the rx UE (theta == 0) — default
        FixedPoint,     ///< pinned at a fixed position (cell beam centre)
        FixedDirection, ///< pinned along a fixed direction in sim coordinates
        Nadir,          ///< satellite -> geocentre (fixed in the body frame)
    };

    static TypeId GetTypeId();
    NtnSatBeamGainModel();
    ~NtnSatBeamGainModel() override;

    void SetBeamwidth3dBDeg(double d) { m_beamwidth3dBDeg = d; }
    double GetBeamwidth3dBDeg() const { return m_beamwidth3dBDeg; }
    void SetPeakGainDbi(double g) { m_peakGainDbi = g; }
    double GetPeakGainDbi() const { return m_peakGainDbi; }
    void SetRolloffFloorDb(double f) { m_rolloffFloorDb = f; }
    double GetRolloffFloorDb() const { return m_rolloffFloorDb; }
    /// When true the model applies PeakGainDbi + rolloff(theta); when false
    /// (DEFAULT, and what every pre-existing scenario gets) only the <= 0 dB
    /// roll-off, because the radio's array already supplies the peak.
    void SetApplyPeakGain(bool a) { m_applyPeakGain = a; }
    bool GetApplyPeakGain() const { return m_applyPeakGain; }

    /// Fixed beam boresight target (cell beam centre) -> BoresightMode::FixedPoint.
    /// Passing null restores BoresightMode::TrackUe (the historical behaviour).
    /// (Out-of-line: MobilityModel is forward-declared.)
    void SetBeamCenter(Ptr<MobilityModel> c);
    /// Pin the boresight at a bare coordinate (no node/mobility model needed)
    /// -> BoresightMode::FixedPoint.
    void SetBoresightFixedPoint(Vector point);
    /// Pin the boresight along a FIXED direction in simulation coordinates
    /// (need not be normalised) -> BoresightMode::FixedDirection.
    void SetBoresightFixed(Vector direction);
    /// Pin the boresight along satellite -> \p geocentre, i.e. nadir, which is
    /// fixed in the satellite body frame -> BoresightMode::Nadir. In a local ENU
    /// scenario frame referenced at (lat0, lon0, 0) the geocentre sits at
    /// (0, 0, -R_earth); in an ECEF/ECI frame it is the origin (the default).
    void SetBoresightNadir(Vector geocentre = Vector(0.0, 0.0, 0.0));
    BoresightMode GetBoresightMode() const { return m_boresightMode; }

    /// Most recent off-boresight angle (deg), applied roll-off (dB) and total
    /// applied gain (dB, = roll-off plus peak when ApplyPeakGain).
    double GetLastThetaDeg() const { return m_lastThetaDeg; }
    double GetLastRolloffDb() const { return m_lastRolloffDb; }
    double GetLastGainDb() const { return m_lastRolloffDb + (m_applyPeakGain ? m_peakGainDbi : 0.0); }

    /**
     * \brief Analytic TR 38.811 §6.4.1 pattern gain at an ARBITRARY off-boresight
     *        angle, independent of any simulation state or geometry.
     *
     * Returns rolloff(theta) (<= 0 dB, floored at RolloffFloorDb), plus
     * PeakGainDbi when ApplyPeakGain is set. This is the closed form the measured
     * plane can be differenced against, so the angle dependence is verifiable by
     * a reviewer without running a scenario.
     */
    double GainDbAtThetaDeg(double thetaDeg) const;

    /// Numerical-Recipes first-order Bessel J1 (portable; no libstdc++ special
    /// math dependency).
    static double BesselJ1(double x);

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    /// Normalized Airy roll-off (dB, <= 0, floored) at off-boresight angle
    /// \p thetaRad. Single definition shared by DoCalcRxPower() and
    /// GainDbAtThetaDeg(), so the measured plane and the closed form can never
    /// drift apart.
    double RolloffDbAtThetaRad(double thetaRad) const;

    double m_beamwidth3dBDeg{4.4127}; ///< TR 38.821 Set-1 LEO-600 S-band 3 dB BW
    double m_peakGainDbi{30.0};       ///< applied only when m_applyPeakGain
    double m_rolloffFloorDb{-40.0};   ///< deepest applied sidelobe roll-off
    bool m_applyPeakGain{false};      ///< default off: no double-count (see header)
    BoresightMode m_boresightMode{BoresightMode::TrackUe};
    Ptr<MobilityModel> m_beamCenter;  ///< FixedPoint via a mobility model
    Vector m_boresightPoint{0.0, 0.0, 0.0};     ///< FixedPoint via a coordinate
    Vector m_boresightDir{0.0, 0.0, -1.0};      ///< FixedDirection
    Vector m_geocentre{0.0, 0.0, 0.0};          ///< Nadir reference
    mutable double m_lastThetaDeg{0.0};
    mutable double m_lastRolloffDb{0.0};
};

} // namespace ns3

#endif // NTN_SAT_BEAM_GAIN_MODEL_H

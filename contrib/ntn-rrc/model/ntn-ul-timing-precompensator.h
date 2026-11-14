/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef NTN_UL_TIMING_PRECOMPENSATOR_H
#define NTN_UL_TIMING_PRECOMPENSATOR_H

#include "ntn-sib19.h"

#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/random-variable-stream.h>
#include <ns3/vector.h>

namespace ns3
{

class MobilityModel;

namespace ntnrrc
{

/**
 * \ingroup ntn-rrc
 * UE-side uplink timing pre-compensation from SIB19 (TS 38.213 cl. 4.2, TS 38.211 cl. 4.3.1).
 *
 * The UE keeps the last SIB19 it decoded and, at any later instant t, forms
 *   TA(t) = 2 |s_hat(t) - p_hat| / c + T_C + T_D tau + T_V tau^2,   tau = t - t_epoch,
 * where s_hat is the satellite position propagated from the broadcast ECEF state vector with
 * the chosen model and p_hat is the UE's last GNSS fix. The UE is uplink synchronized while
 * tau does not exceed ul-SyncValidityDuration.
 *
 * Before this class no UE in the toolkit decoded SIB19, so none of its timing fields were ever
 * exercised by a receiver.
 */
class NtnUlTimingPrecompensator : public Object
{
  public:
    static TypeId GetTypeId();
    NtnUlTimingPrecompensator();
    ~NtnUlTimingPrecompensator() override;

    /// Decoded SIB19 (as received; already quantized by the broadcaster).
    void ReceiveSib19(const Sib19Content& sib);
    /// True UE mobility; the UE samples it as GNSS fixes.
    void SetUeMobility(Ptr<MobilityModel> ue);
    /// GNSS fix period and 1-sigma horizontal error (vertical error is half of it).
    void SetGnss(Time fixPeriod, double sigmaHorizontalM);
    void SetEphemerisModel(UeEphemerisModel m) { m_model = m; }
    /// Project the last GNSS fix with the velocity measured at the fix (TS 38.133 cl. 7.1C.2,
    /// NOTE 2 assumes such a projection); off, the UE uses the fix as it is.
    void SetPositionProjection(bool on) { m_project = on; }
    /// ul-SyncValidityDuration of the SIB19 the UE currently holds.
    Time GetHeldValidity() const { return m_sib.ulSyncValidity; }
    /// Integration step of the numerical propagators.
    void SetStep(Time step) { m_step = step; }

    /// Pre-compensated two-way delay at the current simulation time.
    Time ComputeTa();
    /// Satellite position the UE predicts at the current simulation time.
    Vector PredictSatellitePosition();
    /// Age of the SIB19 information in use (now minus epoch).
    Time GetInformationAge() const;
    /// Whether the information is still within ul-SyncValidityDuration.
    bool IsUplinkSynchronized() const;
    bool HasSib19() const { return m_have; }

    /// Assign a fixed stream to the GNSS noise.
    int64_t AssignStreams(int64_t stream);

  private:
    void UpdateGnssFix();
    void Propagate(double tau);

    Sib19Content m_sib;
    bool m_have{false};
    Ptr<MobilityModel> m_ue;
    UeEphemerisModel m_model{UeEphemerisModel::KeplerJ2};
    Time m_step{Seconds(1)};
    Time m_fixPeriod{Seconds(1)};
    double m_sigmaH{0.0};
    Time m_lastFix{Seconds(-1e9)};
    Vector m_fix{0, 0, 0};
    Vector m_fixVel{0, 0, 0};
    bool m_project{false};
    Ptr<NormalRandomVariable> m_noise;

    // integration cache: state (inertial or ECEF, depending on model) at age m_cacheTau
    double m_cacheTau{0.0};
    double m_y[6]{0, 0, 0, 0, 0, 0};
};

} // namespace ntnrrc
} // namespace ns3

#endif // NTN_UL_TIMING_PRECOMPENSATOR_H

// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Muhammad Uzair
//
// NtnGeocentricMobilityAdapter (C3, Wave 1): gives any ECEF-frame mobility
// model the GeocentricConstantPositionMobilityModel interface that the ns-3
// mainline TR 38.811 NTN channel requires of BOTH link ends
// (ThreeGppChannelModel "NTN-*" scenarios, ThreeGppNTN*ChannelConditionModel,
// ThreeGppNTN*PropagationLossModel). The wrapped model keeps its own motion
// (constant ECEF position, ENU-projected platforms, HAPS/UAV paths); the
// adapter adds the geographic view from WGS-84 (NIMA TR8350.2) and forwards
// position, velocity and course changes. Sgp4MobilityModel and
// NtnTr38811MobilityModel already derive from the geocentric base and need no
// adapter.

#ifndef NTN_GEOCENTRIC_MOBILITY_ADAPTER_H
#define NTN_GEOCENTRIC_MOBILITY_ADAPTER_H

#include "ns3/geocentric-constant-position-mobility-model.h"

namespace ns3
{
namespace ntncon
{

/**
 * \ingroup ntn-constellation
 * \brief Geocentric (TR 38.811-channel-ready) view of an ECEF mobility model.
 */
class NtnGeocentricMobilityAdapter : public GeocentricConstantPositionMobilityModel
{
  public:
    static TypeId GetTypeId();
    /// ns-3.48: copy for MobilityModel::Copy() (spectrum wraparound); keeps the dynamics.
    Ptr<MobilityModel> Copy() const override
    {
        return CreateObject<NtnGeocentricMobilityAdapter>(*this);
    }
    NtnGeocentricMobilityAdapter();
    ~NtnGeocentricMobilityAdapter() override;

    /// The wrapped model; its GetPosition() must be ECEF metres.
    void SetSource(Ptr<MobilityModel> ecefSource);
    Ptr<MobilityModel> GetSource() const { return m_source; }

    /// WGS-84 ECEF (m) to geodetic (deg, deg, m), Bowring's method.
    static Vector EcefToGeodetic(const Vector& ecef);
    /// WGS-84 geodetic (deg, deg, m) to ECEF (m).
    static Vector GeodeticToEcef(const Vector& latLonAlt);

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;
    Vector DoGetGeographicPosition() const override;
    void DoSetGeographicPosition(const Vector& latLonAlt) override;
    Vector DoGetGeocentricPosition() const override;
    void DoSetGeocentricPosition(const Vector& position) override;
    void SourceCourseChange(Ptr<const MobilityModel> m);

    Ptr<MobilityModel> m_source;
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_GEOCENTRIC_MOBILITY_ADAPTER_H

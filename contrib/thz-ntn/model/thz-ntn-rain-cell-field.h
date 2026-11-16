/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef THZ_NTN_RAIN_CELL_FIELD_H
#define THZ_NTN_RAIN_CELL_FIELD_H

#include "thz-ntn-itu-recommendations.h"
#include "thz-ntn-link-geometry.h"

#include "ns3/nstime.h"
#include "ns3/object.h"

#include <vector>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief A space-time rain-rate field made of moving rain cells, integrated
 *        along the actual slant path of each link.
 *
 * Replaces scripted weather phases (a rain rate switched on at a fraction of
 * the run) with weather caused by geometry and time: each cell is an EXCELL
 * exponential cell (Capsoni et al., Radio Science 22(3), 1987, the cell shape
 * the ITU-R P.1853 spatial synthesis builds on):
 *
 *     R(r) = R_peak exp(-r / r0),   r = horizontal distance to the cell centre,
 *
 * advected with a constant velocity, optionally with a finite lifetime. The
 * field rate at a point is the largest of the cell rates there (EXCELL cells
 * do not overlap) and is zero below \p MinRate.
 *
 * The attenuation of a link is the path integral of the ITU-R P.838-3
 * specific attenuation through the field below the rain height h_R:
 *
 *     A(f) = sum_i k(f) R(x_i, y_i, t)^alpha(f) * ds_i     [dB]
 *
 * along the straight path from the lower end towards the upper end, up to the
 * slant length L_s of ITU-R P.618-13 eq. (2)/(3) (Earth curvature included
 * below 5 deg). With an instantaneous field there is no exceedance
 * probability and therefore no P.618 path-reduction factor: the field itself
 * is the spatial inhomogeneity that factor approximates.
 *
 * h_R is the P.839 rain height. Without the P.839-4 digital map (not bundled)
 * it is taken from the Itu618LossModel climate-region table, or set
 * explicitly with SetRainHeightKm().
 *
 * Coordinates: the field lives in a local ENU plane (x east, y north, metres).
 * Flat-frame links use their own x/y. ECEF links are mapped onto the plane
 * with an equirectangular projection about SetReference(lat, lon).
 */
class ThzNtnRainCellField : public Object
{
  public:
    static TypeId GetTypeId();
    ThzNtnRainCellField();
    ~ThzNtnRainCellField() override;

    /// One EXCELL rain cell.
    struct Cell
    {
        double x0{0.0};       //!< centre east at t = start (m)
        double y0{0.0};       //!< centre north at t = start (m)
        double vx{0.0};       //!< advection velocity east (m/s)
        double vy{0.0};       //!< advection velocity north (m/s)
        double peakRate{0.0}; //!< R_peak (mm/h)
        double r0{2000.0};    //!< e-folding radius (m)
        Time start{Seconds(0)};
        Time stop{Time::Max()};
    };

    /// Add a cell; returns its index.
    uint32_t AddCell(const Cell& c);
    const std::vector<Cell>& GetCells() const { return m_cells; }
    void Clear() { m_cells.clear(); }

    /// Field reference point for ECEF links (WGS-84 deg).
    void SetReference(double latDeg, double lonDeg);
    /// Rain height h_R above mean sea level (km).
    void SetRainHeightKm(double h) { m_rainHeightKm = h; }
    double GetRainHeightKm() const { return m_rainHeightKm; }
    /// Rain height from the P.839-style climate table of Itu618LossModel.
    void SetRainHeightFromClimate(itu::Itu618LossModel::ClimateRegion region);
    void SetPolarization(itu::Polarization p) { m_pol = p; }

    /// Rain rate (mm/h) at field point (x, y) and time t.
    double RainRateAt(double x, double y, Time t) const;

    /// Field coordinates (m) of the lower end of a link.
    void LowerEndPlane(const ThzNtnLinkGeometry& g, double& x, double& y) const;

    /// Path samples of the rain rate below h_R along the link, at time t.
    /// \param[out] ds sample length (km)
    /// \return rain rate per sample (mm/h); empty if the path has no rain part
    std::vector<double> SamplePath(const ThzNtnLinkGeometry& g, Time t, double& ds) const;

    /// Path-integrated attenuation (dB) at \p freqHz from samples of SamplePath().
    double IntegrateDb(const std::vector<double>& rates, double dsKm, double freqHz) const;

    /// Convenience: SamplePath + IntegrateDb at the current simulation time.
    double PathAttenuationDb(const ThzNtnLinkGeometry& g, double freqHz) const;

    /// P.618-13 eq. (2)/(3) slant length below the rain height (km).
    static double SlantLengthKm(double elevationDeg, double hsKm, double hrKm);

  private:
    std::vector<Cell> m_cells;
    double m_refLatDeg{0.0};
    double m_refLonDeg{0.0};
    double m_rainHeightKm{3.5};
    double m_minRate{0.1};
    double m_maxStepM{50.0};
    itu::Polarization m_pol{itu::Polarization::circular};
};

} // namespace ns3

#endif // THZ_NTN_RAIN_CELL_FIELD_H

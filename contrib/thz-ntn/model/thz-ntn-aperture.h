/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef THZ_NTN_APERTURE_H
#define THZ_NTN_APERTURE_H

#include <cmath>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief Declared-payload helpers: a link closes because of a stated
 *        transmit power and stated apertures, not a power knob.
 *
 *  - Aperture gain of a reflector of diameter D with efficiency eta:
 *      G = 10 log10( eta (pi D / lambda)^2 )  [dBi]
 *    (ITU-R S.580 / S.465 use the same D/lambda law for earth-station
 *    antennas; eta 0.6 is the customary parabolic value).
 *  - Half-power beamwidth of a parabolic aperture: theta_3dB ~ 70 lambda / D
 *    [deg] (ITU-R S.580 reference pattern scaling).
 *  - Free-space loss 20 log10(4 pi d / lambda) [dB].
 */
struct ThzNtnAperture
{
    static constexpr double kC = 299792458.0;

    static double GainDbi(double diameterM, double freqHz, double eta = 0.6)
    {
        const double lambda = kC / freqHz;
        const double x = M_PI * diameterM / lambda;
        return 10.0 * std::log10(eta * x * x);
    }

    static double Beamwidth3dBDeg(double diameterM, double freqHz)
    {
        return 70.0 * (kC / freqHz) / diameterM;
    }

    static double FsplDb(double distanceM, double freqHz)
    {
        return 20.0 * std::log10(4.0 * M_PI * distanceM * freqHz / kC);
    }

    /// Thermal noise (dBm) over \p bwHz with noise figure \p nfDb (T0 = 290 K).
    static double NoiseDbm(double bwHz, double nfDb)
    {
        return -174.0 + 10.0 * std::log10(bwHz) + nfDb;
    }
};

} // namespace ns3

#endif // THZ_NTN_APERTURE_H

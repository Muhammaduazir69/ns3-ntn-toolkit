/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * Slant-path gaseous absorption sweep of the thz-ntn absorption engine.
 *
 * Evaluates ThzNtnMolecularAbsorption::ComputeAbsorptionLoss_dB, the call the
 * channel, link-budget and propagation-loss classes make, over a grid of
 * carrier frequency, elevation and integrated water-vapour column (PWV) for a
 * ground terminal at sea level and a LEO satellite, and writes one CSV row per
 * point. Three engine paths can be compared:
 *
 *   inproc  ITU-R P.676-13 Annex 1 line-by-line at every quadrature point
 *   lut     the bundled tabulated specific attenuation (default atmosphere)
 *   lutfile any other table given with --lutFile (e.g. a legacy release)
 *
 * The PWV column is set through the WaterVapourScale attribute; the scale for
 * each target column is found by bisection on GetWaterVapourColumn(), so the
 * reported PWV is the column the engine actually integrates.
 *
 * Wave 7C: the sweep is a validation tool for the absorption kernel and is
 * labeled as one in its output. It is followed by a measured packet
 * companion (thz-ntn-pass-kit.h) whose ThzNtnPacketLink uses the same engine
 * path (--mode, --lutFile, --profile): an SGP4 satellite of an
 * ntn-constellation shell over a TR 38.811 static terminal, DVB-S2 ACM, and an
 * ntn-traffic 5QI 2 flow with KPM output. --companion=0 skips it.
 *
 * Usage:
 *   ./ns3 run "thz-ntn-absorption-sweep --mode=inproc --out=sweep.csv"
 */

#include "thz-ntn-pass-kit.h"

#include <ns3/command-line.h>
#include <ns3/core-module.h>
#include <ns3/thz-ntn-molecular-absorption.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThzNtnAbsorptionSweep");

namespace
{

std::vector<double>
ParseList(const std::string& s)
{
    std::vector<double> v;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ','))
    {
        if (!tok.empty())
        {
            v.push_back(std::stod(tok));
        }
    }
    return v;
}

/// Slant range (m) from a ground point at sea level to altitude hSat (km).
double
SlantRangeM(double elevDeg, double hSatKm)
{
    const double re = 6371.0;
    const double e = elevDeg * M_PI / 180.0;
    const double rs = re + hSatKm;
    return 1e3 * (std::sqrt(rs * rs - re * re * std::cos(e) * std::cos(e)) - re * std::sin(e));
}

double
ScaleForColumn(Ptr<ThzNtnMolecularAbsorption> m, double pwvMm)
{
    double lo = 0.0;
    double hi = 20.0;
    for (int i = 0; i < 50; ++i)
    {
        const double mid = 0.5 * (lo + hi);
        m->SetAttribute("WaterVapourScale", DoubleValue(mid));
        if (m->GetWaterVapourColumn() < pwvMm)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string mode = "inproc";
    std::string lutFile = "contrib/thz-ntn/data/hitran2024-lut-subthz.csv";
    std::string profile = "global-annual";
    std::string out = "thz-ntn-absorption-sweep.csv";
    std::string elevs = "10,15,20,30,45,60,90";
    std::string pwvs = "1,2,5,10,15,20,30,45,60";
    double fStart = 300.0;
    double fStop = 1000.0;
    double fStep = 0.5;
    double hSat = 550.0;
    uint32_t steps = 10;
    bool companion = true;
    double companionFreqGhz = 300.0;
    double simSeconds = 20.0;
    double appRateMbps = 1000.0;
    std::string outputDir = "thz-ntn-absorption-sweep-out";

    CommandLine cmd(__FILE__);
    cmd.AddValue("mode", "inproc | lut | lutfile", mode);
    cmd.AddValue("lutFile", "table for --mode=lut / lutfile", lutFile);
    cmd.AddValue("profile", "reference atmosphere (HumidityProfile attribute)", profile);
    cmd.AddValue("out", "output CSV", out);
    cmd.AddValue("elevs", "comma-separated elevations, deg", elevs);
    cmd.AddValue("pwvs", "comma-separated PWV columns, mm (none: profile as published)", pwvs);
    cmd.AddValue("fStart", "first carrier, GHz", fStart);
    cmd.AddValue("fStop", "last carrier, GHz", fStop);
    cmd.AddValue("fStep", "carrier step, GHz", fStep);
    cmd.AddValue("hSat", "satellite altitude, km", hSat);
    cmd.AddValue("steps", "IntegrationSteps attribute (sub-steps per slab)", steps);
    cmd.AddValue("companion", "Run the measured packet companion after the sweep", companion);
    cmd.AddValue("companionFreqGhz", "Companion carrier (GHz)", companionFreqGhz);
    cmd.AddValue("simSeconds", "Companion run length (s)", simSeconds);
    cmd.AddValue("appRateMbps", "Companion offered 5QI 2 load (Mb/s)", appRateMbps);
    cmd.AddValue("outputDir", "Companion output directory", outputDir);
    cmd.Parse(argc, argv);
    std::cout << "[validation-tool] thz-ntn-absorption-sweep evaluates the absorption kernel\n"
                 "over a (frequency, elevation, PWV) grid; its CSV holds no packet KPIs. The\n"
                 "measured packet companion runs after it on the same engine path.\n";

    Ptr<ThzNtnMolecularAbsorption> m = CreateObject<ThzNtnMolecularAbsorption>();
    m->SetHumidityProfile(profile);
    m->SetAttribute("IntegrationSteps", UintegerValue(steps));
    if (mode == "lut" || mode == "lutfile")
    {
        if (!m->LoadAtmosphericLut(lutFile))
        {
            std::cerr << "cannot load LUT " << lutFile << "\n";
            return 1;
        }
    }

    std::vector<double> pwvList = (pwvs == "none") ? std::vector<double>{} : ParseList(pwvs);
    const bool asPublished = pwvList.empty();
    if (asPublished)
    {
        pwvList.push_back(-1.0);
    }

    std::ofstream csv(out);
    csv << "mode,profile,pwv_mm,wv_scale,elev_deg,freq_ghz,range_km,atten_db\n";
    csv << std::setprecision(8);
    std::size_t nEval = 0;
    double seconds = 0.0;
    for (double pwv : pwvList)
    {
        double scale = 1.0;
        if (!asPublished)
        {
            scale = ScaleForColumn(m, pwv);
        }
        m->SetAttribute("WaterVapourScale", DoubleValue(scale));
        const double column = m->GetWaterVapourColumn();
        for (double el : ParseList(elevs))
        {
            const double r = SlantRangeM(el, hSat);
            for (double f = fStart; f <= fStop + 1e-9; f += fStep)
            {
                const auto t0 = std::chrono::steady_clock::now();
                const double a = m->ComputeAbsorptionLoss_dB(f * 1e9, r, 0.0, hSat, el);
                seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                               .count();
                ++nEval;
                csv << mode << "," << profile << "," << column << "," << scale << "," << el << ","
                    << f << "," << r / 1e3 << "," << a << "\n";
            }
        }
    }
    csv.close();
    std::cout << "mode=" << mode << " profile=" << profile << " evaluations=" << nEval
              << " mean_cost_us=" << 1e6 * seconds / std::max<std::size_t>(nEval, 1)
              << " lut_used=" << (m->IsAtmosphericLutLoaded() ? "loaded" : "no")
              << " out=" << out << "\n";

    if (companion)
    {
        thzkit::PassConfig pc;
        pc.runTag = "absorption-sweep-companion";
        pc.outputDir = outputDir;
        pc.simSeconds = simSeconds;
        pc.altitudeKm = hSat;
        pc.appRateMbps = appRateMbps;
        pc.argc = argc;
        pc.argv = argv;
        pc.link.freqHz = companionFreqGhz * 1e9;
        pc.link.bandwidthHz = 1e9;
        bool lutOk = true;
        thzkit::RunPass(pc,
                        [&](Ptr<thzntn::ThzNtnPacketLink> link, Ptr<MobilityModel>,
                            Ptr<MobilityModel>) {
                            // Same engine path as the sweep.
                            Ptr<ThzNtnMolecularAbsorption> a =
                                link->GetAtmosphere()->GetAbsorptionModel();
                            a->SetHumidityProfile(profile);
                            a->SetAttribute("IntegrationSteps", UintegerValue(steps));
                            if (mode == "lut" || mode == "lutfile")
                            {
                                lutOk = a->LoadAtmosphericLut(lutFile);
                            }
                        });
        if (!lutOk)
        {
            std::cerr << "companion: cannot load LUT " << lutFile << "\n";
        }
    }
    return 0;
}

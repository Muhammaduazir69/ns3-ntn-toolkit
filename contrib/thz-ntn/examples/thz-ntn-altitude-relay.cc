/*
 * thz-ntn-altitude-relay.cc
 *
 * Measures how much molecular absorption a terahertz space-to-ground link
 * avoids by lifting its lower endpoint off the ground.
 *
 * The question this answers: water vapour is concentrated in the lowest few
 * kilometres of the atmosphere, so a relay that sits above it should see a much
 * smaller absorption term than a ground station looking through the whole air
 * column. That is a physical claim, and this sweep measures it rather than
 * asserting it. Absorption comes from the module's ITU-R P.676 line-by-line
 * model integrated over the ITU-R P.835 six-layer reference atmosphere; only
 * the lower endpoint altitude and the elevation angle change between rows.
 *
 * Output: one CSV row per (frequency, relay altitude, elevation).
 */

#include "ns3/core-module.h"
#include "ns3/thz-ntn-molecular-absorption.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <vector>

using namespace ns3;
using namespace ns3::thzntn;

NS_LOG_COMPONENT_DEFINE("ThzNtnAltitudeRelay");

namespace
{
constexpr double kEarthRadiusKm = 6371.0;

// Slant range from an endpoint at altitude h_km to a satellite at satAlt_km,
// seen at the given elevation angle. Standard spherical-Earth geometry.
double
SlantRangeKm(double elevationDeg, double hKm, double satAltKm)
{
    const double rl = kEarthRadiusKm + hKm;
    const double rs = kEarthRadiusKm + satAltKm;
    const double e = elevationDeg * M_PI / 180.0;
    return -rl * std::sin(e) + std::sqrt(rl * rl * std::sin(e) * std::sin(e) + rs * rs - rl * rl);
}
} // namespace

int
main(int argc, char* argv[])
{
    double satAltKm = 550.0;
    std::string outputDir = "thz-altitude-relay-output";

    CommandLine cmd(__FILE__);
    cmd.AddValue("satAltKm", "Satellite altitude in km", satAltKm);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    Ptr<ThzNtnMolecularAbsorption> abs = CreateObject<ThzNtnMolecularAbsorption>();

    // The three sub-terahertz atmospheric windows the module reports as usable
    // for space-to-ground, plus 300 GHz as a deliberately harder case.
    const std::vector<double> freqsGHz = {140.0, 220.0, 300.0, 340.0};
    // Ground, a tall mast, a small aircraft, a jet, and the stratospheric band
    // where a solar high-altitude platform actually cruises.
    const std::vector<double> altKm = {0.0, 0.1, 1.0, 3.0, 6.0, 10.0, 14.0, 18.0, 20.0, 25.0};
    const std::vector<double> elevDeg = {5.0, 10.0, 20.0, 30.0, 45.0, 60.0, 90.0};

    std::string path = outputDir;
    if (system(("mkdir -p " + path).c_str()) != 0)
    {
        NS_FATAL_ERROR("cannot create " << path);
    }
    std::ofstream out(path + "/altitude_relay_absorption.csv");
    out << "freq_GHz,relay_alt_km,elevation_deg,slant_km,absorption_dB\n";
    out << std::fixed;

    for (double f : freqsGHz)
    {
        for (double h : altKm)
        {
            for (double e : elevDeg)
            {
                const double d = SlantRangeKm(e, h, satAltKm);
                const double a = abs->ComputeSlantPathAbsorption(f * 1e9, e, h, satAltKm);
                out << std::setprecision(1) << f << "," << std::setprecision(2) << h << "," << e
                    << "," << std::setprecision(2) << d << "," << std::setprecision(4) << a << "\n";
            }
        }
    }
    out.close();

    // Second sweep: absorption against frequency, at the ground and at 20 km,
    // for a fixed elevation. This is what pins the usable windows, and it is
    // also the check that a nearby carrier does not behave completely
    // differently from the one the headline quotes.
    std::ofstream spec(path + "/altitude_relay_spectrum.csv");
    spec << "freq_GHz,elevation_deg,ground_dB,relay20km_dB\n";
    spec << std::fixed;
    for (double e : {10.0, 30.0})
    {
        for (double f = 100.0; f <= 500.0 + 1e-9; f += 1.0)
        {
            const double g = abs->ComputeSlantPathAbsorption(f * 1e9, e, 0.0, satAltKm);
            const double r = abs->ComputeSlantPathAbsorption(f * 1e9, e, 20.0, satAltKm);
            spec << std::setprecision(1) << f << "," << e << "," << std::setprecision(4) << g << ","
                 << r << "\n";
        }
    }
    spec.close();

    // Print the headline comparison so a reader of the log sees it immediately.
    std::printf("# molecular absorption, dB, satellite at %.0f km\n", satAltKm);
    std::printf("# %-8s %-10s %-10s %-10s %s\n",
                "f (GHz)",
                "ground",
                "20 km",
                "saved",
                "at elevation");
    for (double f : freqsGHz)
    {
        for (double e : {10.0, 30.0})
        {
            const double g = abs->ComputeSlantPathAbsorption(f * 1e9, e, 0.0, satAltKm);
            const double r = abs->ComputeSlantPathAbsorption(f * 1e9, e, 20.0, satAltKm);
            std::printf("  %-8.0f %-10.2f %-10.2f %-10.2f %.0f deg\n", f, g, r, g - r, e);
        }
    }
    return 0;
}

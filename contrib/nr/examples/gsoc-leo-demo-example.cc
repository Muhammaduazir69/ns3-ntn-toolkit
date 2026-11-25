// Copyright (c) 2025 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
// Author: Thiago Miyazaki <miyathiago@gmail.com> <t.miyazaki@unesp.br>

#include "math.h"

#include "ns3/core-module.h"
#include "ns3/geocentric-ecef-mobility-model.h"
#include "ns3/geographic-positions.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/leo-orbit-node-helper.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/udp-client-server-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("GsocLeoNrExample");

/**
 * @ingroup examples
 * @file gsoc-leo-demo-example.cc
 * @brief Demonstrates a Non-Terrestrial Network (NTN) link between a LEO satellite constellation
 * and a ground terminal, combining orbital mobility, the 3GPP NTN channel, and the NR stack.
 *
 * This example builds a small NTN scenario in which one or more satellites move along circular LEO
 * orbits and exchange traffic with a node on the ground. The satellites act as NR gNBs and the
 * ground node as an NR UE. Its purpose is to show how to combine, in a single program:
 *
 *  - LEO orbital mobility, using the LeoOrbitNodeHelper and the LeoCircularOrbitMobilityModel (from
 *    the ns-3 mobility module) to create the constellation. The orbit can be the default single
 *    shell, a set of shells loaded from a CSV file (--orbitFile), or tuned from the command line
 *    (e.g. --altitudeKm).
 *  - A ground terminal with a fixed geographic position (GeocentricEcefMobilityModel),
 *    placed beneath the first satellite.
 *  - The 3GPP NTN channel and propagation models, selected per scenario through the NrChannelHelper
 *    (--scenario, default "NTN-Rural").
 *  - Uniform planar array antennas on both ends. The satellite antenna is periodically re-pointed
 *    toward the ground terminal (see UpdateAntennaOrientation), while the ground terminal panel
 *    faces the zenith and steers its beam electronically; when a mobility trace is requested
 *    (--traceFile), the antenna pointing direction is also written out for plotting.
 *  - A bidirectional UDP traffic pattern: a downlink flow (remote host -> ground terminal) and an
 *    uplink flow (ground terminal -> remote host), each transferring 15000 bytes. The example
 *    prints the number of bytes received in each direction at the end of the run.
 *
 * The radio configuration is selected with the --application preset, which sets representative
 * values (frequency, bandwidth, satellite EIRP, terminal transmit power, antenna gains, satellite
 * noise figure and orbit altitude) for three deployments: direct-to-mobile to a handheld ("dtm"),
 * a broadband terminal ("vsat", the default) and a ground gateway/backhaul station ("backhaul").
 * Each preset value can still be overridden individually on the command line.
 *
 * By default the link is intentionally over-driven (the configured EIRP is applied as conducted
 * power and the antenna gains are applied on every array element) so that the example works as a
 * simple connectivity smoke test. Passing --realisticPower compensates for the antenna array gain
 * so that the radiated EIRP and the effective antenna gains match the configured values, yielding
 * a physically representative NTN link budget.
 *
 * Besides the on-screen byte counts, the example writes the standard NR trace files (PHY/MAC/RLC/
 * PDCP statistics, path-loss and SINR traces) to the working directory. Run
 * "./ns3 run 'gsoc-leo-demo-example --PrintHelp'" to list all available parameters.
 *
 * @note This example was developed during the Google Summer of Code 2025 program. The main author
 * is Thiago Miyazaki.
 */

Vector groundNodePosGEO;
Vector groundNodePosECEF;

void
UpdateAntennaOrientation(Ptr<Node> node, Ptr<UniformPlanarArray> satelliteNodeAntenna, Time period)
{
    auto mobility = node->GetObject<MobilityModel>();
    const auto satelliteNodePositionECEF = mobility->GetPosition();

    // convert from ECEF (x, y, z) to Geodetic/Geographic (lon/lat/alt)
    const Vector satelliteNodePositionGEO =
        GeographicPositions::CartesianToGeographicCoordinates(satelliteNodePositionECEF,
                                                              GeographicPositions::SPHERE);

    // makes translation (origin == sat node) and converts to ENU
    const Vector translatedENU =
        GeographicPositions::GeographicToTopocentricCoordinates(groundNodePosGEO,
                                                                satelliteNodePositionGEO,
                                                                GeographicPositions::SPHERE);

    // provide target point in ENU
    const Angles angles(translatedENU);

    // set antenna angles
    satelliteNodeAntenna->SetAlpha(angles.GetAzimuth());
    satelliteNodeAntenna->SetBeta(angles.GetInclination());

    // schedules itself again after a certain period
    Simulator::Schedule(period, &UpdateAntennaOrientation, node, satelliteNodeAntenna, period);
}

Vector
GeneratePoint(const Vector& nodePosECEF, const double azimuth, const double inclination)
{
    // convert node position vector from ECEF (x, y, z) to GEO (Geodesic/Geographic : lon/lat/alt)
    const Vector nodePosGEO =
        GeographicPositions::CartesianToGeographicCoordinates(nodePosECEF,
                                                              GeographicPositions::SPHERE);

    // in the plot, this is the magnitude of the vector that represent the antenna orientation
    // leaving the node and reaching the ground station
    const double orientationVectorLength = (groundNodePosECEF - nodePosECEF).GetLength();

    // pre-calculate sin(inclination)
    double s = std::sin(inclination);

    // newly generated point
    const Vector newPointUnitVectorInTopo(s * std::cos(azimuth),
                                          s * std::sin(azimuth),
                                          std::cos(inclination));

    // apply magnitude to the unit vector
    const Vector newPointWithMagnitudeTopo = newPointUnitVectorInTopo * orientationVectorLength;

    // convert from Topocentric/ENU to Geographic (2nd arg. must be in GEO)
    const Vector newPointWithMagnitudeGEO =
        GeographicPositions::TopocentricToGeographicCoordinates(newPointWithMagnitudeTopo,
                                                                nodePosGEO,
                                                                GeographicPositions::SPHERE);

    // convert from Geographic to Cartesian - the plot uses cartesian coordinates.
    const Vector newPointWithMagnitudeECEF =
        GeographicPositions::GeographicToCartesianCoordinates(newPointWithMagnitudeGEO.x,
                                                              newPointWithMagnitudeGEO.y,
                                                              newPointWithMagnitudeGEO.z,
                                                              GeographicPositions::SPHERE);

    return newPointWithMagnitudeECEF;
}

void
CourseChange(Ptr<const MobilityModel> mob)
{
    const Vector pos = mob->GetPosition();
    Ptr<const Node> node = mob->GetObject<Node>();
    Ptr<UniformPlanarArray> ant = node->GetObject<UniformPlanarArray>();
    const Vector newP = GeneratePoint(pos, ant->GetAlpha(), ant->GetBeta());

    std::cout << Simulator::Now() << ":" << node->GetId() << ":" << pos.x << ":" << pos.y << ":"
              << pos.z << ":" << newP.x << ":" << newP.y << ":" << newP.z << std::endl;
}

/**
 * @brief Apply per-application defaults for a representative NTN deployment.
 *
 * Each preset selects a frequency, bandwidth, satellite EIRP density, terminal transmit power
 * and antenna gains typical of that use case (representative values drawn from 3GPP TR 38.821
 * and public system parameters). These are applied before command-line parsing, so any
 * individual value can still be overridden on the command line.
 *
 * @param application use case: "dtm", "vsat" or "backhaul"
 * @param frequencyHz [out] carrier frequency in Hz
 * @param bandwidthHz [out] bandwidth in Hz
 * @param satEIRP [out] satellite EIRP density in dBW/MHz
 * @param groundTxPower [out] terminal transmit power in dBm
 * @param satAntennaGainDb [out] satellite antenna gain in dBi
 * @param vsatAntennaGainDb [out] terminal antenna gain in dBi
 * @param satNoiseFigureDb [out] satellite (gNB) receiver noise figure in dB
 * @param altitudeKm [out] constellation altitude in km
 */
void
ApplyApplicationPreset(const std::string& application,
                       double& frequencyHz,
                       double& bandwidthHz,
                       double& satEIRP,
                       double& groundTxPower,
                       double& satAntennaGainDb,
                       double& vsatAntennaGainDb,
                       double& satNoiseFigureDb,
                       double& altitudeKm)
{
    if (application == "dtm")
    {
        // Direct-to-mobile: low-band cellular link to a 0 dBi, 23 dBm handheld -- the hardest case,
        // since the return link is power-limited. The parameters mirror how real direct-to-cell
        // systems close the uplink: a low LEO orbit (Starlink direct-to-cell is at ~550 km), a low
        // cellular band, a low-noise satellite receiver and a very large satellite antenna. The
        // satellite gain below is an *effective* value: it stands in for the large array plus the
        // narrowband uplink processing gain that real direct-to-cell systems (NB-IoT-like) use to
        // concentrate the handheld's limited power, which the 5 MHz NR waveform here cannot
        // represent directly. With these values both directions deliver under --realisticPower,
        // the power-limited links possibly only partially.
        frequencyHz = 0.7e9;   // low cellular band (e.g. 600-700 MHz)
        bandwidthHz = 5e6;     // narrow channel (NR minimum that still carries the SSB)
        satEIRP = 50;          // dBW/MHz (very high EIRP to reach a 0 dBi handheld)
        groundTxPower = 23;    // dBm (handheld, UE power class 3)
        satAntennaGainDb = 60; // dBi (effective: large antenna + narrowband uplink processing gain)
        vsatAntennaGainDb = 0; // dBi (handheld, omnidirectional)
        satNoiseFigureDb = 1.5; // dB (low-noise satellite receiver)
        altitudeKm = 550;       // km (low LEO, Starlink direct-to-cell altitude)
    }
    else if (application == "vsat")
    {
        // Broadband flat-panel/dish consumer terminal (Starlink-class), Ka-band.
        frequencyHz = 20e9;      // Ka-band downlink
        bandwidthHz = 100e6;     //
        satEIRP = 24;            // dBW/MHz
        groundTxPower = 33;      // dBm (2 W terminal)
        satAntennaGainDb = 38.5; // dBi
        vsatAntennaGainDb = 40;  // dBi (high-gain steerable terminal)
        satNoiseFigureDb = 5.0;  // dB
        altitudeKm = 1200;       // km
    }
    else if (application == "backhaul")
    {
        // Fixed ground gateway / backhaul station with a large dish, Ka-band.
        frequencyHz = 20e9;      // Ka-band downlink
        bandwidthHz = 400e6;     // wide feeder link
        satEIRP = 20;            // dBW/MHz
        groundTxPower = 40;      // dBm (10 W HPA)
        satAntennaGainDb = 38.5; // dBi
        vsatAntennaGainDb = 50;  // dBi (large gateway dish)
        satNoiseFigureDb = 5.0;  // dB
        altitudeKm = 1200;       // km
    }
    else
    {
        NS_ABORT_MSG("Unknown application '" << application
                                             << "'. Valid values: dtm, vsat, backhaul.");
    }
}

int
main(int argc, char* argv[])
{
    CommandLine cmd;
    std::string orbitFile;
    std::string traceFile;
    uint32_t precision = 1000; // milliseconds
    uint32_t duration = 4;     // seconds
    std::string scenario = "NTN-Rural";

    // Per-application defaults. The use case is read from argv first so that its preset values
    // become the defaults, which any individual --<name> option below can still override.
    std::string application = "vsat";
    const std::string appOpt = "--application=";
    for (int i = 1; i < argc; i++)
    {
        const std::string arg = argv[i];
        if (arg.rfind(appOpt, 0) == 0)
        {
            application = arg.substr(appOpt.size());
        }
    }
    double frequencyHz;
    double bandwidthHz;
    double satEIRP;
    double groundTxPower;
    double satAntennaGainDb;
    double vsatAntennaGainDb;
    double satNoiseFigureDb;
    double altitudeKm;
    ApplyApplicationPreset(application,
                           frequencyHz,
                           bandwidthHz,
                           satEIRP,
                           groundTxPower,
                           satAntennaGainDb,
                           vsatAntennaGainDb,
                           satNoiseFigureDb,
                           altitudeKm);
    bool realisticPower = false; // compensate antenna array gain for a realistic link budget

    cmd.AddValue("application",
                 "NTN use case preset that sets representative frequency, bandwidth, EIRP, power "
                 "and antenna gains: 'dtm' (direct-to-mobile, S-band handheld), 'vsat' (broadband "
                 "terminal, Ka-band) or 'backhaul' (ground gateway dish, Ka-band). The individual "
                 "options below override the preset.",
                 application);
    cmd.AddValue("orbitFile", "CSV file with orbit parameters", orbitFile);
    cmd.AddValue("traceFile", "CSV file to store mobility trace in", traceFile);
    cmd.AddValue("precision", "Mobility model time precision in milliseconds", precision);
    cmd.AddValue("duration", "Duration of the simulation in seconds", duration);
    cmd.AddValue("scenario", "Scenario for the 3GPP Channel Model", scenario);
    cmd.AddValue("frequencyHz",
                 "The operating frequency in Hz (default from --application)",
                 frequencyHz);
    cmd.AddValue("bandwidthHz", "The bandwidth in Hz (default from --application)", bandwidthHz);
    cmd.AddValue("satAntennaGainDb",
                 "Satellite Antenna (gNB) Gain in dBi (default from --application)",
                 satAntennaGainDb);
    cmd.AddValue("vsatAntennaGainDb",
                 "VSAT Antenna (UE) Gain in dBi (default from --application)",
                 vsatAntennaGainDb);
    cmd.AddValue("groundTxPower",
                 "Set Ground Node TxPowerLevel at the PHY in dBm (default from --application)",
                 groundTxPower);
    cmd.AddValue("satEIRP", "Set Satellite EIRP in dBW/MHz (default from --application)", satEIRP);
    cmd.AddValue("satNoiseFigure",
                 "Satellite (gNB) receiver noise figure in dB (default from --application)",
                 satNoiseFigureDb);
    cmd.AddValue("altitudeKm",
                 "Constellation altitude in km (default from --application; ignored when "
                 "--orbitFile is given)",
                 altitudeKm);
    cmd.AddValue("realisticPower",
                 "If true, compensate for the antenna array gain so the configured satEIRP and "
                 "antenna gains are the values actually radiated, yielding a realistic NTN link "
                 "budget. If false (default), the link is intentionally over-driven as a "
                 "connectivity smoke test.",
                 realisticPower);
    cmd.Parse(argc, argv);

    // The time resolution of the mobility model, i.e. how often its position is updated (and a
    // CourseChange is notified), is set through the helper constructor below, which forwards it to
    // the LeoCircularOrbitMobilityModel "Resolution" attribute.
    LeoOrbitNodeHelper orbit(Time(MilliSeconds(precision)));

    // creates the satellite nodes and put them into a container
    NodeContainer satellites;
    if (!orbitFile.empty())
    {
        satellites = orbit.CreateNodesAndInstallMobility(orbitFile);
    }
    else
    {
        satellites = orbit.CreateNodesAndInstallMobility(LeoOrbitalShell(altitudeKm, 30, 1, 2));
    }

    // create node on the ground, create mobility, set position and aggregate mobility
    Ptr<Node> groundNode = CreateObject<Node>();
    Ptr<GeocentricEcefMobilityModel> groundNodeMobility =
        CreateObject<GeocentricEcefMobilityModel>();

    // get the first satellite instantiated and place ground node beneath it
    auto firstSatelliteMobility = satellites.Get(0)->GetObject<MobilityModel>();
    auto firstSatellitePositionGEO =
        GeographicPositions::CartesianToGeographicCoordinates(firstSatelliteMobility->GetPosition(),
                                                              GeographicPositions::SPHERE);
    groundNodeMobility->SetGeographicPosition(
        Vector(firstSatellitePositionGEO.x, firstSatellitePositionGEO.y, 0));

    // set global vars with GEO and ECEF positions for the Ground Node
    groundNodePosGEO = groundNodeMobility->GetGeographicPosition();
    groundNodePosECEF = groundNodeMobility->GetGeocentricPosition();

    groundNode->AggregateObject(groundNodeMobility);

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    // Spectrum configuration. We create a single operational band and configure the scenario
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    constexpr uint8_t numCcPerBand = 1; // 1 CC per Band

    // Create the configuration for the CcBwpHelper.
    // SimpleOperationBandConf creates a single BWP per CC
    CcBwpCreator::SimpleOperationBandConf bandConf(frequencyHz, bandwidthHz, numCcPerBand);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Create the channel helper
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    // Set and configure the channel to the current band
    // A fixed LOS channel condition models the open-sky view of a satellite terminal (TR 38.821
    // rural assumption). With the probabilistic "Default" condition the link frequently draws
    // NLOS, whose clutter loss no realistic NTN budget can absorb, and neither direction delivers.
    channelHelper->ConfigureFactories(scenario, "LOS", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Configure ideal beamforming method
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Configure scheduler
    nrHelper->SetSchedulerTypeId(NrMacSchedulerTdmaRR::GetTypeId());

    // Antenna array geometry. The boresight gain of a UniformPlanarArray is the per-element gain
    // plus the array factor (10*log10(numElements)). By default this example puts the full
    // configured gain on every element and lets the array factor stack on top, which (together
    // with the EIRP handling below) intentionally over-drives the link for a connectivity smoke
    // test. With --realisticPower the configured total gain is split so that the per-element gain
    // plus the array factor equals the configured value.
    const uint32_t ueNumRows = 2;
    const uint32_t ueNumCols = 4;
    const uint32_t gnbNumRows = 8;
    const uint32_t gnbNumCols = 8;
    const double ueArrayFactorDb = 10 * std::log10(ueNumRows * ueNumCols);
    const double gnbArrayFactorDb = 10 * std::log10(gnbNumRows * gnbNumCols);

    double ueElementGainDb = vsatAntennaGainDb;
    double gnbElementGainDb = satAntennaGainDb;
    if (realisticPower)
    {
        ueElementGainDb = vsatAntennaGainDb - ueArrayFactorDb;
        gnbElementGainDb = satAntennaGainDb - gnbArrayFactorDb;
    }

    // Antennas for the UEs. The panel lies flat on the ground with its boresight facing the
    // zenith (the boresight tilted 90 degrees off the horizon), like a flat-panel satellite
    // terminal; the beam itself is steered
    // electronically toward the satellite by the beamforming precoding. Without this the array
    // boresight stays on the horizon while the satellite is near zenith, and both links lose the
    // array gain. Both panels are dual-polarized: with single linear polarization the down-facing
    // satellite panel and the up-facing terminal panel are polarization-mismatched in LOS, which
    // costs several dB in both directions; dual polarization makes the link insensitive to the
    // relative panel orientation, as in real systems.
    nrHelper->SetUeAntennaTypeId("ns3::UniformPlanarArray");
    nrHelper->SetUeAntennaAttribute("DowntiltAngle", DoubleValue(M_PI / 2));
    nrHelper->SetUeAntennaAttribute("IsDualPolarized", BooleanValue(true));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(ueNumRows));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(ueNumCols));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
                                        "Gain",
                                        DoubleValue(ueElementGainDb))));

    // Antennas for the gNbs
    nrHelper->SetGnbAntennaTypeId("ns3::UniformPlanarArray");
    nrHelper->SetGnbAntennaAttribute("IsDualPolarized", BooleanValue(true));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(gnbNumRows));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(gnbNumCols));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
                                         "Gain",
                                         DoubleValue(gnbElementGainDb))));

    // Install nr net devices
    NodeContainer groundNodeContainer;
    groundNodeContainer.Add(groundNode);
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(satellites, allBwps);
    NetDeviceContainer groundNodeNetDev = nrHelper->InstallUeDevice(groundNodeContainer, allBwps);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(groundNodeNetDev, randomStream);

    // satEIRP is an EIRP density (dBW/MHz); convert it to the equivalent total power in dBm
    // (dBm = dBW + 30; spread across the band by adding 10*log10(bandwidth in MHz)).
    // NrPhy::SetTxPower() sets the *conducted* power and the antenna array gain is added on top
    // by the channel, so in the default smoke-test mode the radiated EIRP ends up well above
    // satEIRP. With --realisticPower we subtract the gNB boresight array gain so the
    // actually-radiated EIRP matches the configured satEIRP.
    double satTxPower = (satEIRP + 30) + (10 * std::log10(bandwidthHz / 1e6));
    if (realisticPower)
    {
        satTxPower -= satAntennaGainDb;
    }

    for (uint32_t i = 0; i < gnbNetDev.GetN(); i++)
    {
        NrHelper::GetGnbPhy(gnbNetDev.Get(i), 0)->SetTxPower(satTxPower);
        // The satellite is the uplink receiver; its noise figure drives the return-link budget.
        NrHelper::GetGnbPhy(gnbNetDev.Get(i), 0)->SetNoiseFigure(satNoiseFigureDb);
    }

    // In the default smoke-test mode the forward link is over-driven through the EIRP-to-conducted
    // power conversion above, but the return link would still use the preset's realistic terminal
    // power and stay tens of dB short of closing (all UL TBs corrupt, so the BSR is lost and the
    // uplink flow never starts). Over-drive the terminal symmetrically so that the smoke test
    // delivers in both directions; with --realisticPower the preset terminal power is used as-is.
    double ueTxPower = realisticPower ? groundTxPower : satTxPower;
    NrHelper::GetUePhy(groundNodeNetDev.Get(0), 0)->SetTxPower(ueTxPower);

    // Create the internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, MilliSeconds(10));

    InternetStackHelper internet;
    internet.Install(groundNodeContainer);

    Ipv4InterfaceContainer ueIpInterface;
    ueIpInterface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(groundNodeNetDev));

    // Assign IP address to UEs, and install UDP applications
    uint16_t dlPort = 1234;
    uint16_t ulPort = 1235;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;

    // Server configuration: UDP sink on the ground node (downlink receiver).
    PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    ApplicationContainer sinkContainer = dlPacketSinkHelper.Install(groundNode);
    serverApps.Add(sinkContainer);

    // Client configuration: UDP downlink flow (remote host -> ground node). A downlink UDP flow is
    // used instead of TCP so that delivery does not depend on the return link: for direct-to-mobile
    // the handheld uplink is severely power-limited and could not carry TCP ACKs, which would stall
    // the whole transfer. 10 packets of 1500 bytes make up the 15000 bytes checked at the end.
    UdpClientHelper dlClientHelper(ueIpInterface.GetAddress(0), dlPort);
    dlClientHelper.SetAttribute("MaxPackets", UintegerValue(10));
    dlClientHelper.SetAttribute("PacketSize", UintegerValue(1500));
    dlClientHelper.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    clientApps.Add(dlClientHelper.Install(remoteHost));

    // Uplink: a UDP sink on the remote host fed by a UDP flow from the ground node. A return flow
    // is essential for an NTN link to be meaningful (acknowledgements, telemetry, return traffic).
    // For direct-to-mobile the return link is power-limited, so it may deliver only partially.
    PacketSinkHelper ulPacketSinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), ulPort));
    serverApps.Add(ulPacketSinkHelper.Install(remoteHost));

    UdpClientHelper ulClientHelper(remoteHostIpv4Address, ulPort);
    ulClientHelper.SetAttribute("MaxPackets", UintegerValue(10));
    ulClientHelper.SetAttribute("PacketSize", UintegerValue(1500));
    ulClientHelper.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    clientApps.Add(ulClientHelper.Install(groundNode));

    // Initialize the satellite antenna orientations before attachment so that the initial
    // beamforming vectors are calculated for the oriented panels. Each call also schedules the
    // subsequent periodic orientation updates.
    for (uint32_t i = 0; i < gnbNetDev.GetN(); i++)
    {
        auto satNetDevice = gnbNetDev.Get(i);
        auto gnbNetDevice = satNetDevice->GetObject<NrGnbNetDevice>();
        auto phy = gnbNetDevice->GetPhy(0);
        auto spectrumPhy = phy->GetSpectrumPhy();
        auto satAntenna = spectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>();
        Ptr<Node> satNode = satNetDevice->GetNode();
        satNode->AggregateObject(satAntenna);
        UpdateAntennaOrientation(satNode, satAntenna, MilliSeconds(precision));
    }

    // Attach UEs to the closest gNB
    nrHelper->AttachToClosestGnb(groundNodeNetDev, gnbNetDev);

    // Start server and client apps
    serverApps.Start(MilliSeconds(400));
    clientApps.Start(MilliSeconds(400));
    serverApps.Stop(Seconds(duration));
    clientApps.Stop(Seconds(duration) - MilliSeconds(200));

    // Enable the traces provided by the nr module
    nrHelper->EnableTraces();

    std::streambuf* coutbuf = std::cout.rdbuf();

    // Redirect cout if traceFile is specified
    std::ofstream out;
    out.open(traceFile);
    if (out.is_open())
    {
        Config::ConnectWithoutContextFailSafe("/NodeList/*/$ns3::MobilityModel/CourseChange",
                                              MakeCallback(&CourseChange));
        std::cout.rdbuf(out.rdbuf());
        std::cout << "Time:Satellite:x:y:z:x_2:y_2:z_2" << std::endl;
    }

    Simulator::Stop(Time(Seconds(duration)));
    Simulator::Run();

    out.close();
    std::cout.rdbuf(coutbuf);

    // Report the bytes received in each direction. Both the downlink (satellite -> ground) and the
    // uplink (ground -> satellite) run a 15000-byte UDP flow.
    uint64_t dlRx = serverApps.Get(0)->GetObject<PacketSink>()->GetTotalRx();
    uint64_t ulRx = serverApps.Get(1)->GetObject<PacketSink>()->GetTotalRx();
    std::cout << "Received (downlink) " << dlRx << " bytes" << std::endl;
    std::cout << "Received (uplink) " << ulRx << " bytes" << std::endl;

    Simulator::Destroy();

    // The connectivity smoke test requires both directions to deliver the full flow in the
    // over-driven default mode. Under --realisticPower the budget-limited links are only
    // required to deliver partially, but never nothing.
    const bool connected =
        realisticPower ? (dlRx > 0 && ulRx > 0) : (dlRx == 15000 && ulRx == 15000);
    return connected ? EXIT_SUCCESS : EXIT_FAILURE;
}

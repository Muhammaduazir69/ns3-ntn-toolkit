/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * The tapped-delay-line must reach a spectrum channel and vary ACROSS the band.
 *
 * NtnTdlSpectrumLossModel was correct, complete and unreachable: it is a plain
 * Object, not a SpectrumPropagationLossModel, so nothing could attach it, and
 * its ApplyTo() had two callers in the tree, both inside its own unit test.
 * NtnTdlSpectrumPropagationLossModel is the adapter that closes that. This
 * asserts the adapter actually shapes a PSD, that the shaping is frequency
 * SELECTIVE rather than a flat scale, and that it does not invent energy.
 */
#include "ns3/constant-position-mobility-model.h"
#include "ns3/friis-spectrum-propagation-loss.h"
#include "ns3/node-container.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tdl-spectrum-propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/single-model-spectrum-channel.h"
#include "ns3/spectrum-phy.h"
#include "ns3/spectrum-model.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"
#include "ns3/test.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <vector>

using namespace ns3;

class NtnTdlIsFrequencySelectiveTestCase : public TestCase
{
  public:
    NtnTdlIsFrequencySelectiveTestCase()
        : TestCase("TDL adapter shapes a PSD across the band and conserves mean power")
    {
    }

  private:
    void DoRun() override
    {
        // 100 subcarriers over 20 MHz at S band. The delay spread of the
        // stand-in profile sets a coherence bandwidth well inside this, which is
        // the whole point: a flat model cannot produce variation here.
        const double fc = 2.0e9;
        const double bw = 20e6;
        const uint32_t n = 100;
        Bands bands;
        for (uint32_t i = 0; i < n; ++i)
        {
            BandInfo bi;
            bi.fc = fc - bw / 2 + (bw / n) * (i + 0.5);
            bi.fl = bi.fc - bw / (2 * n);
            bi.fh = bi.fc + bw / (2 * n);
            bands.push_back(bi);
        }
        Ptr<SpectrumModel> sm = Create<SpectrumModel>(bands);
        Ptr<SpectrumValue> psd = Create<SpectrumValue>(sm);
        for (uint32_t i = 0; i < n; ++i)
        {
            (*psd)[i] = 1.0;
        }

        Ptr<SpectrumSignalParameters> params = Create<SpectrumSignalParameters>();
        params->psd = psd;

        // Ground terminal on the globe, satellite 600 km overhead, so the
        // adapter takes the geocentric-up branch as a real scenario would.
        Ptr<ConstantPositionMobilityModel> ue = CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> sat = CreateObject<ConstantPositionMobilityModel>();
        ue->SetPosition(Vector(6371000.0, 0.0, 0.0));
        sat->SetPosition(Vector(6371000.0 + 600e3, 0.0, 0.0));

        Ptr<NtnTdlSpectrumPropagationLossModel> tdl =
            CreateObject<NtnTdlSpectrumPropagationLossModel>();

        Ptr<SpectrumValue> rx = tdl->CalcRxPowerSpectralDensity(params, ue, sat);

        NS_TEST_ASSERT_MSG_EQ(tdl->GetAppliedCount(), 1u,
                              "the adapter must have shaped the PSD, not skipped it");
        NS_TEST_ASSERT_MSG_EQ(tdl->GetSkippedCount(), 0u, "nothing should have been skipped");

        double minG = 1e300;
        double maxG = -1e300;
        double sum = 0.0;
        for (uint32_t i = 0; i < n; ++i)
        {
            const double g = (*rx)[i];
            NS_TEST_ASSERT_MSG_EQ(std::isfinite(g), true, "subcarrier " << i << " is not finite");
            NS_TEST_ASSERT_MSG_GT_OR_EQ(g, 0.0, "a power gain cannot be negative");
            minG = std::min(minG, g);
            maxG = std::max(maxG, g);
            sum += g;
        }

        // Frequency selectivity is the claim. A flat model would give
        // max == min; a tapped-delay-line must not. Measured on the stand-in
        // profile: 2.34 dB of spread across 20 MHz, min 0.738, max 1.265, mean
        // 1.024. Replacing ApplyTo with a flat scale drops the spread to 0 dB
        // and fails this case, so the bound is doing work.
        const double spreadDb = 10.0 * std::log10(maxG / std::max(minG, 1e-12));
        NS_TEST_ASSERT_MSG_GT(spreadDb, 1.0,
                              "the transfer function varies only " << spreadDb
                                  << " dB across 20 MHz, which is a flat channel wearing a "
                                     "tapped-delay-line's name");

        // And it must not invent energy: the profile is normalised to unit mean.
        const double meanG = sum / n;
        NS_TEST_ASSERT_MSG_LT(std::abs(10.0 * std::log10(std::max(meanG, 1e-12))), 3.0,
                              "mean gain " << meanG
                                  << " is more than 3 dB off unity, so the fading model is "
                                     "adding or removing average power");
    }
};

// ---------------------------------------------------------------------------
// In-depth cases (Nov 2026 test campaign). Each runs the adapter inside a live
// SingleModelSpectrumChannel: a probe PHY transmits a flat PSD every 1 ms
// through Friis path loss, a constant-speed delay model and the TDL, and a
// probe PHY at the other end records every received PSD across
// Simulator::Run().
// ---------------------------------------------------------------------------

namespace
{

constexpr double kEarthR = 6371000.0;
constexpr double kC = 299792458.0;

/// A minimal SpectrumPhy: transmits a fixed PSD periodically or records every
/// PSD the channel delivers.
class ProbePhy : public SpectrumPhy
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::NtnTdlTestProbePhy").SetParent<SpectrumPhy>().SetGroupName("Test");
        return tid;
    }

    void SetDevice(Ptr<NetDevice>) override
    {
    }

    Ptr<NetDevice> GetDevice() const override
    {
        return nullptr;
    }

    void SetMobility(Ptr<MobilityModel> m) override
    {
        m_mobility = m;
    }

    Ptr<MobilityModel> GetMobility() const override
    {
        return m_mobility;
    }

    void SetChannel(Ptr<SpectrumChannel> c) override
    {
        m_channel = c;
    }

    Ptr<const SpectrumModel> GetRxSpectrumModel() const override
    {
        return m_model;
    }

    Ptr<Object> GetAntenna() const override
    {
        return nullptr;
    }

    void StartRx(Ptr<SpectrumSignalParameters> params) override
    {
        m_rx.push_back(params->psd->Copy());
        m_rxTimes.push_back(Simulator::Now());
    }

    /// Transmit \p psd every \p period from \p start, \p n times.
    void StartTxTrain(Ptr<SpectrumValue> psd, Time start, Time period, uint32_t n)
    {
        m_model = psd->GetSpectrumModel();
        for (uint32_t k = 0; k < n; ++k)
        {
            Simulator::Schedule(start + period * k, &ProbePhy::TxOnce, this, psd);
        }
    }

    void SetModel(Ptr<const SpectrumModel> m)
    {
        m_model = m;
    }

    std::vector<Ptr<SpectrumValue>> m_rx;
    std::vector<Time> m_rxTimes;
    std::vector<Time> m_txTimes;

  private:
    void TxOnce(Ptr<SpectrumValue> psd)
    {
        Ptr<SpectrumSignalParameters> p = Create<SpectrumSignalParameters>();
        p->psd = psd->Copy();
        p->duration = MicroSeconds(100);
        p->txPhy = this;
        m_txTimes.push_back(Simulator::Now());
        m_channel->StartTx(p);
    }

    Ptr<MobilityModel> m_mobility;
    Ptr<SpectrumChannel> m_channel;
    Ptr<const SpectrumModel> m_model;
};

/// \p n subcarrier bins of width bw / n centered on \p fc.
Ptr<SpectrumModel>
Band(double fc, double bw, uint32_t n)
{
    Bands bands;
    for (uint32_t i = 0; i < n; ++i)
    {
        BandInfo bi;
        bi.fc = fc - bw / 2 + (bw / n) * (i + 0.5);
        bi.fl = bi.fc - bw / (2 * n);
        bi.fh = bi.fc + bw / (2 * n);
        bands.push_back(bi);
    }
    return Create<SpectrumModel>(bands);
}

/// Center frequency of a band; Friis is evaluated there, once per signal.
double
Center(Ptr<const SpectrumModel> sm)
{
    return sm->Begin()->fc + (std::prev(sm->End())->fc - sm->Begin()->fc) / 2;
}

/// TR 38.811 Eq. (6.6-3) slant range for altitude h at elevation e (rad).
double
SlantRange(double h, double e)
{
    return std::sqrt(kEarthR * kEarthR * std::sin(e) * std::sin(e) + h * h + 2 * kEarthR * h) -
           kEarthR * std::sin(e);
}

/// A live link: terminal on the equator at (R, 0, 0), satellite at altitude
/// \p h seen at elevation \p elevDeg, flat unit PSD sent \p n times every 1 ms.
/// Returns the receiver's records (one PSD per transmission).
struct Link
{
    Ptr<ProbePhy> tx;
    Ptr<ProbePhy> rx;
    double distance{0.0};
};

Link
RunLink(Ptr<NtnTdlSpectrumPropagationLossModel> tdl,
        double elevDeg,
        uint32_t n,
        Ptr<SpectrumModel> sm,
        double h = 600e3)
{
    const double e = elevDeg * M_PI / 180.0;
    const double d = SlantRange(h, e);
    Ptr<ConstantPositionMobilityModel> ue = CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> sat = CreateObject<ConstantPositionMobilityModel>();
    ue->SetPosition(Vector(kEarthR, 0.0, 0.0));
    // Up is +x at the terminal, horizontal is +y.
    sat->SetPosition(Vector(kEarthR + d * std::sin(e), d * std::cos(e), 0.0));

    Ptr<SingleModelSpectrumChannel> ch = CreateObject<SingleModelSpectrumChannel>();
    Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel>();
    friis->SetFrequency(Center(sm));
    ch->AddPropagationLossModel(friis);
    ch->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    if (tdl)
    {
        ch->AddSpectrumPropagationLossModel(tdl);
    }

    Link l;
    l.distance = d;
    l.tx = CreateObject<ProbePhy>();
    l.rx = CreateObject<ProbePhy>();
    l.tx->SetMobility(sat);
    l.rx->SetMobility(ue);
    l.tx->SetChannel(ch);
    l.rx->SetChannel(ch);
    l.rx->SetModel(sm);
    ch->AddRx(l.tx);
    ch->AddRx(l.rx);

    Ptr<SpectrumValue> psd = Create<SpectrumValue>(sm);
    *psd = 1.0;
    l.tx->StartTxTrain(psd, MilliSeconds(1), MilliSeconds(1), n);
    Simulator::Stop(MilliSeconds(1) * (n + 60));
    Simulator::Run();
    Simulator::Destroy();
    return l;
}

/// Free-space gain (linear) of TR 38.811 Eq. (6.6-2) at \p d metres, \p fc Hz.
double
FsplLin(double d, double fc)
{
    const double db = 32.45 + 20.0 * std::log10(fc / 1e9) + 20.0 * std::log10(d);
    return std::pow(10.0, -db / 10.0);
}

double
Corr(const std::vector<double>& a, const std::vector<double>& b)
{
    const std::size_t n = std::min(a.size(), b.size());
    double ma = 0;
    double mb = 0;
    for (std::size_t i = 0; i < n; ++i)
    {
        ma += a[i] / n;
        mb += b[i] / n;
    }
    double sab = 0;
    double saa = 0;
    double sbb = 0;
    for (std::size_t i = 0; i < n; ++i)
    {
        sab += (a[i] - ma) * (b[i] - mb);
        saa += (a[i] - ma) * (a[i] - ma);
        sbb += (b[i] - mb) * (b[i] - mb);
    }
    return sab / std::sqrt(saa * sbb);
}

} // namespace

/// Power conservation on a live channel. SetTaps normalizes the profile to
/// unit total power with the relative powers kept, and the RMS delay spread
/// is the power-weighted second central moment (TR 38.901 7.7.3 / TR 38.811
/// 6.9.2 define the TDL delays as normalized to it). Over 2000 transmissions on
/// the 600 km zenith slant, the mean received PSD of every subcarrier equals the
/// TR 38.811 Eq. (6.6-2) free-space gain within 0.25 dB, the band average
/// within 0.05 dB: the fading redistributes power, it does not add or remove it.
/// The signal arrives 600 km / c after it left.
class NtnTdlPowerConservationTest : public TestCase
{
  public:
    NtnTdlPowerConservationTest()
        : TestCase("TDL on a live channel: unit-power profile, mean PSD = TR 38.811 FSPL, delay = d-c")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NtnTdlSpectrumLossModel> m = CreateObject<NtnTdlSpectrumLossModel>();
        m->SetTaps({{0.0, 3.0, true}, {100e-9, -3.0, false}, {400e-9, -9.0, false}});
        double total = 0.0;
        for (const auto& t : m->GetTaps())
        {
            total += std::pow(10.0, t.powerDb / 10.0);
        }
        NS_TEST_ASSERT_MSG_EQ_TOL(total, 1.0, 1e-12, "taps normalized to unit power");
        NS_TEST_ASSERT_MSG_EQ_TOL(m->GetTaps()[0].powerDb - m->GetTaps()[1].powerDb, 6.0, 1e-12,
                                  "relative powers kept");
        // Power-weighted moments by hand.
        const double p[3] = {std::pow(10.0, 0.3), std::pow(10.0, -0.3), std::pow(10.0, -0.9)};
        const double tau[3] = {0.0, 100e-9, 400e-9};
        double s = 0;
        double s1 = 0;
        double s2 = 0;
        for (int i = 0; i < 3; ++i)
        {
            s += p[i];
            s1 += p[i] * tau[i];
            s2 += p[i] * tau[i] * tau[i];
        }
        const double ds = std::sqrt(s2 / s - (s1 / s) * (s1 / s));
        NS_TEST_ASSERT_MSG_EQ_TOL(m->GetRmsDelaySpreadS(), ds, 1e-15, "RMS delay spread");

        Ptr<NtnTdlSpectrumPropagationLossModel> tdl = CreateObject<NtnTdlSpectrumPropagationLossModel>();
        tdl->AssignStreams(100);
        Ptr<SpectrumModel> sm = Band(2.0e9, 20e6, 50);
        const uint32_t n = 2000;
        Link l = RunLink(tdl, 90.0, n, sm);
        NS_TEST_ASSERT_MSG_EQ(l.rx->m_rx.size(), n, "every transmission received");
        NS_TEST_ASSERT_MSG_EQ(tdl->GetAppliedCount(), n, "and shaped once each");
        NS_TEST_ASSERT_MSG_EQ_TOL((l.rx->m_rxTimes.front() - l.tx->m_txTimes.front()).GetSeconds(),
                                  l.distance / kC, 1e-9, "arrival after 600 km / c");
        double bandMean = 0.0;
        for (uint32_t b = 0; b < sm->GetNumBands(); ++b)
        {
            double mean = 0.0;
            for (const auto& psd : l.rx->m_rx)
            {
                mean += (*psd)[b] / n;
            }
            const double errDb = 10.0 * std::log10(mean / FsplLin(l.distance, Center(sm)));
            bandMean += errDb / sm->GetNumBands();
            NS_TEST_ASSERT_MSG_LT(std::abs(errDb), 0.25, "subcarrier " << b << " mean off FSPL by " << errDb << " dB");
        }
        NS_TEST_ASSERT_MSG_LT(std::abs(bandMean), 0.05, "band-average mean gain is the FSPL");
    }
};

/// Frequency correlation against WSSUS theory (Bello 1963), with the delays
/// scaled as TR 38.901 7.7.3 prescribes (tau = tau_norm x DS). Two equal-power
/// Rayleigh taps tau apart give a power correlation |R(df)|^2 = cos^2(pi df tau)
/// between subcarriers df apart. With tau = 500 ns on a 200 kHz grid: about
/// 0.65 at 400 kHz, 0 at 1 MHz and 1 at 2 MHz; halving the delay spread
/// (tau = 250 ns) moves the 1 MHz value to 0.5, i.e. the coherence bandwidth
/// doubles. |H|^2 is exponential (variance = mean^2) as Rayleigh fading must be.
class NtnTdlFrequencyCorrelationTest : public TestCase
{
  public:
    NtnTdlFrequencyCorrelationTest()
        : TestCase("TDL frequency correlation matches cos^2(pi df tau) and scales with the delay spread")
    {
    }

  private:
    std::vector<std::vector<double>> Run(double tau, uint32_t stream)
    {
        Ptr<NtnTdlSpectrumPropagationLossModel> tdl = CreateObject<NtnTdlSpectrumPropagationLossModel>();
        tdl->GetTdl()->SetTaps({{0.0, 0.0, false}, {tau, 0.0, false}});
        tdl->AssignStreams(stream);
        Ptr<SpectrumModel> sm = Band(2.0e9, 20e6, 100);
        Link l = RunLink(tdl, 90.0, 3000, sm);
        std::vector<std::vector<double>> g(sm->GetNumBands());
        for (const auto& psd : l.rx->m_rx)
        {
            for (uint32_t b = 0; b < sm->GetNumBands(); ++b)
            {
                g[b].push_back((*psd)[b] / FsplLin(l.distance, Center(sm)));
            }
        }
        return g;
    }

    static double MeanCorr(const std::vector<std::vector<double>>& g, uint32_t lag)
    {
        double c = 0.0;
        uint32_t k = 0;
        for (uint32_t b = 0; b + lag < g.size(); b += 7)
        {
            c += Corr(g[b], g[b + lag]);
            ++k;
        }
        return c / k;
    }

    void DoRun() override
    {
        const auto g = Run(500e-9, 300);
        const double c2 = MeanCorr(g, 2);
        const double c5 = MeanCorr(g, 5);
        const double c10 = MeanCorr(g, 10);
        std::cout << "[tdl-corr] tau 500 ns: rho(400k) " << c2 << " rho(1M) " << c5 << " rho(2M) " << c10;
        NS_TEST_ASSERT_MSG_EQ_TOL(c2, std::pow(std::cos(M_PI * 400e3 * 500e-9), 2), 0.08, "rho(400 kHz)");
        NS_TEST_ASSERT_MSG_EQ_TOL(c5, 0.0, 0.08, "rho(1 MHz) = 0 at df = 1/(2 tau)");
        NS_TEST_ASSERT_MSG_EQ_TOL(c10, 1.0, 0.08, "rho(2 MHz) = 1 at df = 1/tau");
        // Exponential |H|^2: variance equals the squared mean.
        const auto& x = g[50];
        double m = 0;
        double v = 0;
        for (double y : x)
        {
            m += y / x.size();
        }
        for (double y : x)
        {
            v += (y - m) * (y - m) / (x.size() - 1);
        }
        std::cout << " var/mean^2 " << v / (m * m);
        NS_TEST_ASSERT_MSG_EQ_TOL(v / (m * m), 1.0, 0.15, "Rayleigh: |H|^2 is exponential");

        const auto h = Run(250e-9, 400);
        const double h5 = MeanCorr(h, 5);
        std::cout << "; tau 250 ns: rho(1M) " << h5 << std::endl;
        NS_TEST_ASSERT_MSG_EQ_TOL(h5, 0.5, 0.08, "half the delay spread, twice the coherence bandwidth");
    }
};

/// Rician K against elevation through the adapter's own geometry (geocentric
/// up at the terminal, the convention of Ntn38811ExcessLossModel). A single
/// LOS tap makes |H|^2 flat across the band and Rician in time; the moment
/// estimate K = (1 - v + sqrt(1 - v)) / v (v the variance of |H|^2 at unit
/// mean) must equal the configured table: 8.7 dB at 30 deg, the linear
/// interpolation 9.55 dB at 35 deg, the 10 deg value 4.1 dB clamped at 5 deg.
/// The table is the one the class carries for TR 38.811 6.7.2; the assertion
/// is that the realized fading carries it, within 0.5 dB.
class NtnTdlRicianKElevationTest : public TestCase
{
  public:
    NtnTdlRicianKElevationTest()
        : TestCase("Realized Rician K follows the elevation table through the live geometry")
    {
    }

  private:
    double MeasureKdB(double elevDeg, uint32_t stream)
    {
        Ptr<NtnTdlSpectrumPropagationLossModel> tdl = CreateObject<NtnTdlSpectrumPropagationLossModel>();
        tdl->GetTdl()->SetTaps({{0.0, 0.0, true}});
        tdl->AssignStreams(stream);
        Ptr<SpectrumModel> sm = Band(2.0e9, 1e6, 4);
        Link l = RunLink(tdl, elevDeg, 4000, sm);
        std::vector<double> g;
        double spread = 0.0;
        for (const auto& psd : l.rx->m_rx)
        {
            g.push_back((*psd)[0] / FsplLin(l.distance, Center(sm)));
            spread = std::max(spread, std::abs((*psd)[3] / (*psd)[0] - 1.0));
        }
        NS_TEST_EXPECT_MSG_LT(spread, 1e-9, "one tap: flat across the band");
        double m = 0;
        for (double y : g)
        {
            m += y / g.size();
        }
        double v = 0;
        for (double y : g)
        {
            v += (y / m - 1.0) * (y / m - 1.0) / (g.size() - 1);
        }
        const double k = (1.0 - v + std::sqrt(std::max(0.0, 1.0 - v))) / v;
        return 10.0 * std::log10(k);
    }

    void DoRun() override
    {
        Ptr<NtnTdlSpectrumLossModel> ref = CreateObject<NtnTdlSpectrumLossModel>();
        NS_TEST_ASSERT_MSG_EQ_TOL(ref->RicianKdB(35.0), 9.55, 1e-12, "table interpolation at 35 deg");
        NS_TEST_ASSERT_MSG_EQ_TOL(ref->RicianKdB(5.0), 4.1, 1e-12, "clamped below 10 deg");
        NS_TEST_ASSERT_MSG_EQ_TOL(ref->RicianKdB(95.0), 14.6, 1e-12, "clamped above 90 deg");
        const double k30 = MeasureKdB(30.0, 500);
        const double k35 = MeasureKdB(35.0, 510);
        const double k5 = MeasureKdB(5.0, 520);
        std::cout << "[tdl-k] measured K: 30 deg " << k30 << " dB, 35 deg " << k35 << " dB, 5 deg "
                  << k5 << " dB" << std::endl;
        NS_TEST_ASSERT_MSG_EQ_TOL(k30, 8.7, 0.5, "K at 30 deg");
        NS_TEST_ASSERT_MSG_EQ_TOL(k35, 9.55, 0.5, "K at 35 deg (interpolated)");
        NS_TEST_ASSERT_MSG_EQ_TOL(k5, 4.1, 0.5, "K below the table clamps to the 10 deg value");
        NS_TEST_ASSERT_MSG_GT(k30, k5 + 3.0, "higher elevation, stronger LOS");
    }
};

/// Failure and degenerate paths on a live channel: an empty tap set leaves the
/// PSD at the free-space value; taps that all sit at zero delay give a flat
/// channel (no selectivity); a satellite below the horizon still yields finite,
/// non-negative gains with the K of the lowest table entry; and the adapter
/// called without a mobility model skips (counted) and returns the PSD
/// untouched.
class NtnTdlFailurePathsTest : public TestCase
{
  public:
    NtnTdlFailurePathsTest()
        : TestCase("TDL failure paths: empty taps, zero-delay taps, below horizon, missing mobility")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<SpectrumModel> sm = Band(2.0e9, 20e6, 20);
        {
            Ptr<NtnTdlSpectrumPropagationLossModel> tdl = CreateObject<NtnTdlSpectrumPropagationLossModel>();
            tdl->GetTdl()->SetTaps({});
            Link l = RunLink(tdl, 45.0, 20, sm);
            for (const auto& psd : l.rx->m_rx)
            {
                for (uint32_t b = 0; b < sm->GetNumBands(); ++b)
                {
                    NS_TEST_ASSERT_MSG_EQ_TOL((*psd)[b] / FsplLin(l.distance, Center(sm)), 1.0, 0.003,
                                              "no taps: free space only");
                }
            }
        }
        {
            Ptr<NtnTdlSpectrumPropagationLossModel> tdl = CreateObject<NtnTdlSpectrumPropagationLossModel>();
            tdl->GetTdl()->SetTaps({{0.0, 0.0, true}, {0.0, -3.0, false}});
            tdl->AssignStreams(600);
            NS_TEST_ASSERT_MSG_EQ(tdl->GetTdl()->GetRmsDelaySpreadS(), 0.0, "zero delay spread");
            Link l = RunLink(tdl, 45.0, 50, sm);
            for (const auto& psd : l.rx->m_rx)
            {
                const double r = ((*psd)[19] / FsplLin(l.distance, Center(sm))) /
                                 ((*psd)[0] / FsplLin(l.distance, Center(sm)));
                NS_TEST_ASSERT_MSG_EQ_TOL(r, 1.0, 1e-9, "zero delay spread: flat across the band");
            }
        }
        {
            Ptr<NtnTdlSpectrumPropagationLossModel> tdl = CreateObject<NtnTdlSpectrumPropagationLossModel>();
            tdl->AssignStreams(610);
            Link l = RunLink(tdl, -20.0, 50, sm);
            NS_TEST_ASSERT_MSG_EQ(l.rx->m_rx.size(), 50u, "below the horizon the channel still delivers");
            for (const auto& psd : l.rx->m_rx)
            {
                for (uint32_t b = 0; b < sm->GetNumBands(); ++b)
                {
                    NS_TEST_ASSERT_MSG_EQ(std::isfinite((*psd)[b]) && (*psd)[b] >= 0.0, true,
                                          "finite, non-negative gain");
                }
            }
            NS_TEST_ASSERT_MSG_EQ_TOL(tdl->GetTdl()->RicianKdB(-20.0), 4.1, 1e-12, "lowest K entry");
        }
        {
            Ptr<NtnTdlSpectrumPropagationLossModel> tdl = CreateObject<NtnTdlSpectrumPropagationLossModel>();
            Ptr<SpectrumValue> psd = Create<SpectrumValue>(sm);
            *psd = 2.0;
            Ptr<SpectrumSignalParameters> params = Create<SpectrumSignalParameters>();
            params->psd = psd;
            Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel>();
            Ptr<SpectrumValue> out = tdl->CalcRxPowerSpectralDensity(params, a, nullptr);
            NS_TEST_ASSERT_MSG_EQ(tdl->GetSkippedCount(), 1u, "missing mobility: skipped and counted");
            NS_TEST_ASSERT_MSG_EQ(tdl->GetAppliedCount(), 0u, "not shaped");
            NS_TEST_ASSERT_MSG_EQ((*out)[3], 2.0, "PSD untouched");
        }
    }
};

/// Determinism. AssignStreams fixes the tap draws: two independent channels
/// given the same stream deliver the identical PSD sequence, another stream a
/// different one, and the adapter reports the two streams it used. (It
/// returned 0 and assigned nothing before this campaign, so the same stream
/// gave different channels.)
class NtnTdlDeterminismTest : public TestCase
{
  public:
    NtnTdlDeterminismTest()
        : TestCase("TDL AssignStreams: same stream identical PSD sequence, other stream differs")
    {
    }

  private:
    std::vector<double> Run(int64_t stream)
    {
        Ptr<NtnTdlSpectrumPropagationLossModel> tdl = CreateObject<NtnTdlSpectrumPropagationLossModel>();
        m_used = tdl->AssignStreams(stream);
        Ptr<SpectrumModel> sm = Band(2.0e9, 20e6, 10);
        Link l = RunLink(tdl, 60.0, 30, sm);
        std::vector<double> v;
        for (const auto& psd : l.rx->m_rx)
        {
            for (uint32_t b = 0; b < sm->GetNumBands(); ++b)
            {
                v.push_back((*psd)[b]);
            }
        }
        return v;
    }

    void DoRun() override
    {
        // Create unrelated random variables in between so automatic stream
        // numbering differs between the runs.
        const auto a = Run(700);
        CreateObject<UniformRandomVariable>()->GetValue();
        const auto b = Run(700);
        const auto c = Run(800);
        NS_TEST_ASSERT_MSG_EQ(m_used, 2, "two streams: real and imaginary tap parts");
        NS_TEST_ASSERT_MSG_EQ(a.size(), 300u, "30 receptions x 10 subcarriers");
        NS_TEST_ASSERT_MSG_EQ((a == b), true, "same stream: identical received PSDs");
        NS_TEST_ASSERT_MSG_EQ((a == c), false, "another stream: another fading realization");
    }

    int64_t m_used{0};
};

/// Cross-module: the TDL attached to the NR BWP channel of NtnRealStackHelper
/// through AddSpectrumChannelLoss (the spectrum seam, composed on the 3GPP
/// phased-array channel). On the Set-1 600 km zenith link the adapter shapes
/// every NR transmission, the measured mean DL SINR stays within 3 dB of the
/// flat run (the profile conserves power and the zenith K is 14.6 dB), and the
/// eMBB flow delivers at least 95 % of the flat run's packets.
class NtnTdlOverNrNtnTest : public TestCase
{
  public:
    NtnTdlOverNrNtnTest()
        : TestCase("Cross-module: TDL on the NR-NTN BWP channel shapes every TB, SINR within 3 dB")
    {
    }

  private:
    struct Out
    {
        double sinr{0.0};
        uint64_t rx{0};
        uint64_t applied{0};
        uint64_t tbs{0};
    };

    Out Run(bool withTdl)
    {
        RngSeedManager::ResetNextStreamIndex();
        NodeContainer sats;
        NodeContainer ues;
        sats.Create(1);
        ues.Create(1);
        auto sm = CreateObject<ConstantPositionMobilityModel>();
        sm->SetPosition(NtnRealStackHelper::GeodeticToEcef(40.0, -3.0, 600e3));
        sats.Get(0)->AggregateObject(sm);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(NtnRealStackHelper::GeodeticToEcef(40.0, -3.0, 0.0));
        ues.Get(0)->AggregateObject(um);
        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(1);
        rs.SetSimTime(Seconds(2.0));
        rs.SetOutputDir(CreateTempDirFilename(withTdl ? "tdl-nr" : "flat-nr"));
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.SetTr38811ExcessLoss(false);
        rs.SetSmallScaleFading(false);
        rs.Build(sats, ues);
        Ptr<NtnTdlSpectrumPropagationLossModel> tdl = CreateObject<NtnTdlSpectrumPropagationLossModel>();
        tdl->AssignStreams(900);
        if (withTdl)
        {
            rs.AddSpectrumChannelLoss(tdl);
        }
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, Seconds(0.5), Seconds(1.9));
        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        rs.Collect();
        Out o{rs.GetMeanDlSinrDb(), rs.GetUeRxPackets(0), tdl->GetAppliedCount(), rs.GetPhyRxTb()};
        Simulator::Destroy();
        return o;
    }

    void DoRun() override
    {
        const Out flat = Run(false);
        const Out tdl = Run(true);
        std::cout << "[tdl-nr] flat SINR " << flat.sinr << " dB rx " << flat.rx << "; TDL SINR " << tdl.sinr
                  << " dB rx " << tdl.rx << " applied " << tdl.applied << " TBs " << tdl.tbs << std::endl;
        NS_TEST_ASSERT_MSG_EQ(flat.applied, 0u, "not attached: never called");
        NS_TEST_ASSERT_MSG_GT(tdl.applied, tdl.tbs, "attached: shapes every NR transmission (data and control)");
        NS_TEST_ASSERT_MSG_EQ_TOL(tdl.sinr, flat.sinr, 3.0, "power-conserving fading: mean SINR within 3 dB");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(tdl.rx * 100, flat.rx * 95, "the flow still gets through");
    }
};

class NtnTdlSpectrumTestSuite : public TestSuite
{
  public:
    NtnTdlSpectrumTestSuite()
        : TestSuite("ntn-tdl-spectrum", Type::UNIT)
    {
        AddTestCase(new NtnTdlIsFrequencySelectiveTestCase, Duration::QUICK);
        AddTestCase(new NtnTdlPowerConservationTest, Duration::QUICK);
        AddTestCase(new NtnTdlFrequencyCorrelationTest, Duration::QUICK);
        AddTestCase(new NtnTdlRicianKElevationTest, Duration::QUICK);
        AddTestCase(new NtnTdlFailurePathsTest, Duration::QUICK);
        AddTestCase(new NtnTdlDeterminismTest, Duration::QUICK);
        AddTestCase(new NtnTdlOverNrNtnTest, Duration::QUICK);
    }
};

static NtnTdlSpectrumTestSuite g_ntnTdlSpectrumTestSuite;

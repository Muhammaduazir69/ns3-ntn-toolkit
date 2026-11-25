// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/nr-module.h"
#include "ns3/uniform-planar-array.h"

#include <map>

/**
 * @file
 * @ingroup test
 *
 * Test suite for the spectrum transmit filters installed by
 * NrChannelHelper::AddFilters (NrCsiRsFilter, NrSidelinkFilter and
 * NrCoexistenceFilter).
 */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrTransmitFilterTest");

namespace
{

/**
 * @ingroup test
 *
 * Minimal SpectrumPhy of a non-NR technology, equipped with a phased array.
 *
 * It stands in for a Wi-Fi PHY in coexistence scenarios, while Wi-Fi does not
 * support phased array antennas (required by the 3GPP spectrum propagation
 * loss model). It periodically transmits a signal and counts the signals it
 * receives.
 */
class NonNrPhasedArrayPhy : public SpectrumPhy
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NonNrPhasedArrayPhy")
                                .SetParent<SpectrumPhy>()
                                .SetGroupName("Nr")
                                .AddConstructor<NonNrPhasedArrayPhy>();
        return tid;
    }

    NonNrPhasedArrayPhy()
        : m_array(CreateObject<UniformPlanarArray>())
    {
        m_array->SetBeamformingVector(m_array->GetBeamformingVector(Angles(0, M_PI / 2)));
    }

    void SetDevice(Ptr<NetDevice> d) override
    {
        m_device = d;
    }

    Ptr<NetDevice> GetDevice() const override
    {
        return m_device;
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
        return m_spectrumModel;
    }

    Ptr<Object> GetAntenna() const override
    {
        return m_array;
    }

    void StartRx(Ptr<SpectrumSignalParameters> params) override
    {
        m_rxCount++;
    }

    /**
     * Set the spectrum model used for transmission and reception.
     * @param sm the spectrum model
     */
    void SetSpectrumModel(Ptr<const SpectrumModel> sm)
    {
        m_spectrumModel = sm;
    }

    /**
     * Transmit a signal now and schedule the next transmission.
     * @param period the interval between transmissions
     */
    void TransmitPeriodically(Time period)
    {
        auto params = Create<SpectrumSignalParameters>();
        params->txPhy = this;
        params->psd = Create<SpectrumValue>(m_spectrumModel);
        *params->psd = 1e-9;
        params->duration = MicroSeconds(100);
        m_channel->StartTx(params);
        Simulator::Schedule(period, &NonNrPhasedArrayPhy::TransmitPeriodically, this, period);
    }

    /**
     * @return the number of signals received
     */
    uint64_t GetRxCount() const
    {
        return m_rxCount;
    }

  private:
    Ptr<NetDevice> m_device;                  ///< Device (unused)
    Ptr<MobilityModel> m_mobility;            ///< Mobility model
    Ptr<SpectrumChannel> m_channel;           ///< Spectrum channel
    Ptr<const SpectrumModel> m_spectrumModel; ///< Spectrum model for TX and RX
    Ptr<UniformPlanarArray> m_array;          ///< Phased array antenna
    uint64_t m_rxCount{0};                    ///< Number of signals received
};

/**
 * @ingroup test
 *
 * Transmit filter that never discards a signal, and reports every signal it
 * is asked about. Appended at the tail of the filter chain of a channel, it
 * observes exactly the signals that the other filters let through, which the
 * channel then propagates to the receiver.
 */
class RecordingTransmitFilter : public SpectrumTransmitFilter
{
  public:
    /// Callback invoked for each signal reaching this filter
    using RecordCallback =
        Callback<void, Ptr<const SpectrumSignalParameters>, Ptr<const SpectrumPhy>>;

    /**
     * Constructor
     * @param cb the callback invoked for each signal reaching this filter
     */
    RecordingTransmitFilter(RecordCallback cb)
        : m_cb(cb)
    {
    }

    bool DoFilter(Ptr<const SpectrumSignalParameters> params,
                  Ptr<const SpectrumPhy> receiverPhy) override
    {
        m_cb(params, receiverPhy);
        return false;
    }

  protected:
    int64_t DoAssignStreams(int64_t stream) override
    {
        return 0;
    }

  private:
    RecordCallback m_cb; ///< Callback invoked for each signal
};

/**
 * @ingroup test
 *
 * Checks that the filters selected through NrChannelHelper::AddFilters are
 * installed on the spectrum channel and discard exactly the expected links.
 *
 * Two gNBs, four UEs and two non-NR devices share a 3GPP UMi spectrum channel
 * with CSI-RS enabled. Every signal that passes the installed filters, and is
 * therefore propagated by the channel to its receiver, is recorded by a
 * RecordingTransmitFilter appended to the filter chain, and checked against
 * the installed filters:
 * - NrSidelinkFilter discards gNB-to-gNB and UE-to-UE signals only;
 * - NrCoexistenceFilter discards signals between NR and non-NR devices only;
 * - NrCsiRsFilter discards CSI-RS signals not intended for the receiving UE.
 */
class NrTransmitFilterTestCase : public TestCase
{
  public:
    /**
     * Constructor
     * @param filters bitmask of NrChannelHelper::Filter values to install
     */
    NrTransmitFilterTestCase(uint8_t filters);

  private:
    void DoRun() override;

    /// Role of a SpectrumPhy on the channel
    enum class Role
    {
        GNB,
        UE,
        NON_NR,
    };

    /**
     * Get the role of a SpectrumPhy.
     * @param phy the SpectrumPhy
     * @return the role of the SpectrumPhy
     */
    static Role GetRole(Ptr<const SpectrumPhy> phy);

    /**
     * Get the name of a role.
     * @param role the role
     * @return the name of the role
     */
    static std::string RoleName(Role role);

    /**
     * Record a signal that passed the installed filters.
     * @param params the parameters of the transmitted signal
     * @param rxPhy the receiver SpectrumPhy
     */
    void Record(Ptr<const SpectrumSignalParameters> params, Ptr<const SpectrumPhy> rxPhy);

    uint8_t m_filters; ///< Bitmask of NrChannelHelper::Filter values to install
    std::map<std::pair<Role, Role>, uint64_t> m_linkCount; ///< Signals per (tx, rx) role pair
    uint64_t m_csiRsIntended{0};   ///< CSI-RS signals reaching their intended UE
    uint64_t m_csiRsUnintended{0}; ///< CSI-RS signals reaching any other receiver
};

NrTransmitFilterTestCase::NrTransmitFilterTestCase(uint8_t filters)
    : TestCase("Transmit filters with AddFilters=" + std::to_string(filters)),
      m_filters(filters)
{
}

NrTransmitFilterTestCase::Role
NrTransmitFilterTestCase::GetRole(Ptr<const SpectrumPhy> phy)
{
    auto nrPhy = DynamicCast<const NrSpectrumPhy>(phy);
    if (!nrPhy)
    {
        return Role::NON_NR;
    }
    return nrPhy->IsGnb() ? Role::GNB : Role::UE;
}

std::string
NrTransmitFilterTestCase::RoleName(Role role)
{
    switch (role)
    {
    case Role::GNB:
        return "gNB";
    case Role::UE:
        return "UE";
    default:
        return "non-NR";
    }
}

void
NrTransmitFilterTestCase::Record(Ptr<const SpectrumSignalParameters> params,
                                 Ptr<const SpectrumPhy> rxPhy)
{
    m_linkCount[{GetRole(params->txPhy), GetRole(rxPhy)}]++;

    auto csiRs = DynamicCast<const NrSpectrumSignalParametersCsiRs>(params);
    if (!csiRs)
    {
        return;
    }
    auto nrRxPhy = DynamicCast<const NrSpectrumPhy>(rxPhy);
    if (nrRxPhy && !nrRxPhy->IsGnb() && nrRxPhy->GetCellId() == csiRs->cellId &&
        nrRxPhy->GetRnti() == csiRs->rnti)
    {
        m_csiRsIntended++;
    }
    else
    {
        m_csiRsUnintended++;
    }
}

void
NrTransmitFilterTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    NodeContainer gnbNodes(2);
    NodeContainer ueNodes(4);
    NodeContainer nonNrNodes(2);
    auto positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0, 0, 10));
    positions->Add(Vector(100, 0, 10));
    positions->Add(Vector(10, 10, 1.5));
    positions->Add(Vector(20, -10, 1.5));
    positions->Add(Vector(90, 10, 1.5));
    positions->Add(Vector(110, -10, 1.5));
    positions->Add(Vector(50, 20, 1.5));
    positions->Add(Vector(55, -20, 1.5));
    MobilityHelper mobility;
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);
    mobility.Install(nonNrNodes);

    auto epcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(CreateObject<IdealBeamformingHelper>());
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetAttribute("CsiFeedbackFlags", UintegerValue(CQI_PDSCH_MIMO | CQI_CSI_RS));
    nrHelper->SetGnbPhyAttribute("Pattern", StringValue("DL|S|UL|UL|DL|DL|S|UL|UL|DL|"));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    auto channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->SetAttribute("AddFilters", UintegerValue(m_filters));
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    auto gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    auto ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AssignStreams({.gnbDevs = gnbDevs, .ueDevs = ueDevs});

    InternetStackHelper internet;
    internet.Install(ueNodes);
    epcHelper->AssignUeIpv4Address(ueDevs);
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    auto channel = DynamicCast<MultiModelSpectrumChannel>(band.GetBwpAt(0, 0)->GetChannel());
    NS_TEST_ASSERT_MSG_NE(channel, nullptr, "Expected a MultiModelSpectrumChannel");

    bool hasCsiRsFilter = false;
    bool hasSidelinkFilter = false;
    bool hasCoexistenceFilter = false;
    Ptr<const SpectrumTransmitFilter> tail;
    for (Ptr<const SpectrumTransmitFilter> f = channel->GetSpectrumTransmitFilter(); f;
         f = f->GetNext())
    {
        hasCsiRsFilter |= DynamicCast<const NrCsiRsFilter>(f) != nullptr;
        hasSidelinkFilter |= DynamicCast<const NrSidelinkFilter>(f) != nullptr;
        hasCoexistenceFilter |= DynamicCast<const NrCoexistenceFilter>(f) != nullptr;
        tail = f;
    }
    auto recorder = CreateObject<RecordingTransmitFilter>(
        MakeCallback(&NrTransmitFilterTestCase::Record, this));
    if (tail)
    {
        // Filters added to a channel are prepended, so link the recorder after the last one
        ConstCast<SpectrumTransmitFilter>(tail)->SetNext(recorder);
    }
    else
    {
        channel->AddSpectrumTransmitFilter(recorder);
    }
    const bool csiRsFilterOn = m_filters & NrChannelHelper::CSIRS;
    const bool sidelinkFilterOn = m_filters & NrChannelHelper::SIDELINK;
    const bool coexistenceFilterOn = m_filters & NrChannelHelper::COEXISTENCE;
    NS_TEST_ASSERT_MSG_EQ(hasCsiRsFilter, csiRsFilterOn, "Unexpected NrCsiRsFilter installation");
    NS_TEST_ASSERT_MSG_EQ(hasSidelinkFilter,
                          sidelinkFilterOn,
                          "Unexpected NrSidelinkFilter installation");
    NS_TEST_ASSERT_MSG_EQ(hasCoexistenceFilter,
                          coexistenceFilterOn,
                          "Unexpected NrCoexistenceFilter installation");

    auto spectrumModel =
        NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->GetSpectrumPhy()->GetRxSpectrumModel();
    std::vector<Ptr<NonNrPhasedArrayPhy>> nonNrPhys;
    for (uint32_t i = 0; i < nonNrNodes.GetN(); ++i)
    {
        auto phy = CreateObject<NonNrPhasedArrayPhy>();
        phy->SetSpectrumModel(spectrumModel);
        phy->SetMobility(nonNrNodes.Get(i)->GetObject<MobilityModel>());
        phy->SetChannel(channel);
        channel->AddRx(phy);
        nonNrPhys.push_back(phy);
    }
    Simulator::Schedule(MilliSeconds(20),
                        &NonNrPhasedArrayPhy::TransmitPeriodically,
                        nonNrPhys[0],
                        MilliSeconds(1));

    Simulator::Stop(MilliSeconds(150));
    Simulator::Run();

    for (auto tx : {Role::GNB, Role::UE, Role::NON_NR})
    {
        for (auto rx : {Role::GNB, Role::UE, Role::NON_NR})
        {
            bool sameNrRole = (tx == rx) && (tx != Role::NON_NR);
            bool crossTechnology = (tx == Role::NON_NR) != (rx == Role::NON_NR);
            bool expectFiltered =
                (sameNrRole && sidelinkFilterOn) || (crossTechnology && coexistenceFilterOn);
            auto count = m_linkCount[{tx, rx}];
            NS_LOG_INFO(RoleName(tx) << "->" << RoleName(rx) << ": " << count);
            if (expectFiltered)
            {
                NS_TEST_EXPECT_MSG_EQ(count,
                                      0,
                                      RoleName(tx) << "->" << RoleName(rx)
                                                   << " signals should have been filtered");
            }
            else
            {
                NS_TEST_EXPECT_MSG_GT(count,
                                      0,
                                      RoleName(tx) << "->" << RoleName(rx)
                                                   << " signals should not have been filtered");
            }
        }
    }

    NS_LOG_INFO("CSI-RS intended: " << m_csiRsIntended << " unintended: " << m_csiRsUnintended);
    NS_TEST_EXPECT_MSG_GT(m_csiRsIntended, 0, "CSI-RS should reach its intended UE");
    if (csiRsFilterOn)
    {
        NS_TEST_EXPECT_MSG_EQ(m_csiRsUnintended,
                              0,
                              "CSI-RS should not reach receivers other than its intended UE");
    }
    else
    {
        NS_TEST_EXPECT_MSG_GT(m_csiRsUnintended,
                              0,
                              "Without NrCsiRsFilter, CSI-RS should reach other receivers");
    }

    NS_TEST_EXPECT_MSG_GT(nonNrPhys[1]->GetRxCount(),
                          0,
                          "The non-NR receiver should receive signals");

    Simulator::Destroy();
}

/**
 * @ingroup test
 *
 * Test suite for the NR spectrum transmit filters.
 */
class NrTransmitFilterTestSuite : public TestSuite
{
  public:
    NrTransmitFilterTestSuite()
        : TestSuite("nr-transmit-filter", Type::UNIT)
    {
        // Every combination of CSIRS, SIDELINK and COEXISTENCE
        for (uint8_t filters = 0; filters < 8; ++filters)
        {
            AddTestCase(new NrTransmitFilterTestCase(filters), Duration::QUICK);
        }
    }
};

/// Static instance of the test suite
NrTransmitFilterTestSuite g_nrTransmitFilterTestSuite;

} // namespace

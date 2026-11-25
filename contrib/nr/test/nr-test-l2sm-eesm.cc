// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/boolean.h"
#include "ns3/enum.h"
#include "ns3/nr-eesm-cc-t1.h"
#include "ns3/nr-eesm-cc-t2.h"
#include "ns3/nr-eesm-error-model.h"
#include "ns3/nr-eesm-ir-t1.h"
#include "ns3/nr-eesm-ir-t2.h"
#include "ns3/test.h"

#include <cmath>
#include <vector>

/**
 * @file nr-test-l2sm-eesm.cc
 * @ingroup test
 *
 * @brief This test validates specific functions of the NR PHY abstraction model.
 * The test checks three issues: 1) LDPC base graph (BG) selection works properly, 2)
 * BLER values are properly obtained from the BLER-SINR look up tables for different
 * block sizes, MCS Tables, BG types, and SINR values, and 3) the HARQ-IR combining is not
 * worse than the HARQ-CC combining.
 *
 */
namespace ns3
{

/**
 * @brief NrL2smEesm testcase
 */
class NrL2smEesmTestCase : public TestCase
{
  public:
    NrL2smEesmTestCase(const std::string& name)
        : TestCase(name)
    {
    }

    /**
     * @brief Destroy the object instance
     */
    ~NrL2smEesmTestCase() override
    {
    }

  private:
    void DoRun() override;

    void TestMappingSinrBler1(const Ptr<NrEesmErrorModel>& em);
    void TestMappingSinrBler2(const Ptr<NrEesmErrorModel>& em);
    void TestBgType1(const Ptr<NrEesmErrorModel>& em);
    void TestBgType2(const Ptr<NrEesmErrorModel>& em);

    void TestEesmCcTable1();
    void TestEesmCcTable2();
    void TestEesmIrTable1();
    void TestEesmIrTable2();
};

void
NrL2smEesmTestCase::TestBgType1(const Ptr<NrEesmErrorModel>& em)
{
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 18),
                          NrEesmErrorModel::SECOND,
                          "TestBgType1-a: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3900, 18),
                          NrEesmErrorModel::FIRST,
                          "TestBgType1-b: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(200, 18),
                          NrEesmErrorModel::SECOND,
                          "TestBgType1-c: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(4000, 0),
                          NrEesmErrorModel::SECOND,
                          "TestBgType1-d: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 28),
                          NrEesmErrorModel::FIRST,
                          "TestBgType1-e: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 2),
                          NrEesmErrorModel::SECOND,
                          "TestBgType2-f: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 16),
                          NrEesmErrorModel::SECOND,
                          "TestBgType2-g: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3900, 14),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-h: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
}

void
NrL2smEesmTestCase::TestBgType2(const Ptr<NrEesmErrorModel>& em)
{
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 18),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-a: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3900, 18),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-b: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(200, 18),
                          NrEesmErrorModel::SECOND,
                          "TestBgType2-c: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(4000, 0),
                          NrEesmErrorModel::SECOND,
                          "TestBgType2-d: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 27),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-e: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 2),
                          NrEesmErrorModel::SECOND,
                          "TestBgType2-f: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3200, 16),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-g: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
    NS_TEST_ASSERT_MSG_EQ(em->GetBaseGraphType(3900, 14),
                          NrEesmErrorModel::FIRST,
                          "TestBgType2-h: The calculated value differs from the 3GPP base graph "
                          "selection algorithm.");
}

typedef std::tuple<double, uint8_t, uint32_t, double> MappingTable;

static std::vector<MappingTable> resultTable1 = {
    // sinr (lineal), mcs, cbsize, result

    // MCS 18, all CBS in continuation use BGtype2
    // CBS=3200, in table corresponds to 3104
    MappingTable{19.95, 18, 3200, 0.023},      // sinr 13 db
    MappingTable{15.849, 18, 3200, 0.7567365}, // sinr 12 db
    MappingTable{10, 18, 3200, 1.00},          // sinr 10 db
    // CBS=3500, in table corresponds to 3496
    MappingTable{19.95, 18, 3500, 0.0735},     // sinr 13 db
    MappingTable{15.849, 18, 3500, 0.7908951}, // sinr 12 db
    MappingTable{10, 18, 3500, 1.00},          // sinr 10 db

    // MCS 14, all CBS in continuation use BGtype1
    // CBS=3900, in table corresponds to 3840
    MappingTable{8.9125, 14, 3900, 0.3225703}, // sinr 9.5db
    MappingTable{7.9433, 14, 3900, 0.8827055}, // sinr 9 db
    MappingTable{6.3095, 14, 3900, 1.00},      // sinr 8 db
    // CBS=6300, in table corresponds to 6272
    MappingTable{8.9125, 14, 6300, 0.0237},    // sinr 9.5db
    MappingTable{7.9433, 14, 6300, 0.9990385}, // sinr 9 db
    MappingTable{6.3095, 14, 6300, 1.00}       // sinr 8 db

};
static std::vector<MappingTable> resultTable2 = {
    // sinr (lineal), mcs, cbsize, result

    // MCS 11, all CBS in continuation use BGtype2
    // CBS=3200, in table corresponds to 3104
    MappingTable{19.95, 11, 3200, 0.023},      // sinr 13 db
    MappingTable{15.849, 11, 3200, 0.7567365}, // sinr 12 db
    MappingTable{10, 11, 3200, 1.00},          // sinr 10 db
    // CBS=3500, in table corresponds to 3496
    MappingTable{19.95, 11, 3500, 0.0735},     // sinr 13 db
    MappingTable{15.849, 11, 3500, 0.7908951}, // sinr 12 db
    MappingTable{10, 11, 3500, 1.00},          // sinr 10 db

    // MCS 8, all CBS in continuation use BGtype1
    // CBS=3900, in table corresponds to 3840
    MappingTable{8.9125, 8, 3900, 0.3225703}, // sinr 9.5db
    MappingTable{7.9433, 8, 3900, 0.8827055}, // sinr 9 db
    MappingTable{6.3095, 8, 3900, 1.00},      // sinr 8 db
    // CBS=6300, in table corresponds to 6272
    MappingTable{8.9125, 8, 6300, 0.0237},    // sinr 9.5db
    MappingTable{7.9433, 8, 6300, 0.9990385}, // sinr 9 db
    MappingTable{6.3095, 8, 6300, 1.00}       // sinr 8 db

};

void
NrL2smEesmTestCase::TestMappingSinrBler1(const Ptr<NrEesmErrorModel>& em)
{
    for (auto result : resultTable1)
    {
        NS_TEST_ASSERT_MSG_EQ(
            em->MappingSinrBler(std::get<0>(result), std::get<1>(result), std::get<2>(result)),
            std::get<3>(result),
            "TestMappingSinrBler1: The calculated value differs from "
            " the SINR-BLER table. SINR="
                << std::get<0>(result) << " MCS " << static_cast<uint32_t>(std::get<1>(result))
                << " CBS " << std::get<2>(result));
    }
}

void
NrL2smEesmTestCase::TestMappingSinrBler2(const Ptr<NrEesmErrorModel>& em)
{
    for (auto result : resultTable2)
    {
        NS_TEST_ASSERT_MSG_EQ(
            em->MappingSinrBler(std::get<0>(result), std::get<1>(result), std::get<2>(result)),
            std::get<3>(result),
            "TestMappingSinrBler2: The calculated value differs from "
            " the SINR-BLER table. SINR="
                << std::get<0>(result) << " MCS " << static_cast<uint32_t>(std::get<1>(result))
                << " CBS " << std::get<2>(result));
    }
}

void
NrL2smEesmTestCase::TestEesmCcTable1()
{
    // Create an object of type NrEesmCcT1 and cast it to NrEesmErrorModel
    Ptr<NrEesmErrorModel> em = CreateObject<NrEesmCcT1>();

    // Check that the object was created
    bool ret = em == nullptr;
    NS_TEST_ASSERT_MSG_EQ(ret, false, "Could not create NrEesmCcT1 object");

    // Test here the functions:
    TestBgType1(em);
    TestMappingSinrBler1(em);
}

void
NrL2smEesmTestCase::TestEesmCcTable2()
{
    // Create an object of type NrEesmCcT2 and cast it to NrEesmErrorModel
    Ptr<NrEesmErrorModel> em = CreateObject<NrEesmCcT2>();

    // Check that the object was created
    bool ret = em == nullptr;
    NS_TEST_ASSERT_MSG_EQ(ret, false, "Could not create NrEesmCcT2 object");

    // Test here the functions:
    TestBgType2(em);
    TestMappingSinrBler2(em);
}

void
NrL2smEesmTestCase::TestEesmIrTable1()
{
    // Create an object of type NrEesmIrT1 and cast it to NrEesmErrorModel
    Ptr<NrEesmErrorModel> em = CreateObject<NrEesmIrT1>();

    // Check that the object was created
    bool ret = em == nullptr;
    NS_TEST_ASSERT_MSG_EQ(ret, false, "Could not create NrEesmIrT1 object");

    // Test here the functions:
    TestBgType1(em);
    TestMappingSinrBler1(em);
}

void
NrL2smEesmTestCase::TestEesmIrTable2()
{
    // Create an object of type NrEesmIrT2 and cast it to NrEesmErrorModel
    Ptr<NrEesmErrorModel> em = CreateObject<NrEesmIrT2>();

    // Check that the object was created
    bool ret = em == nullptr;
    NS_TEST_ASSERT_MSG_EQ(ret, false, "Could not create NrEesmIrT2 object");

    // Test here the functions:
    TestBgType2(em);
    TestMappingSinrBler2(em);
}

void
NrL2smEesmTestCase::DoRun()
{
    TestEesmCcTable1();
    TestEesmCcTable2();
    TestEesmIrTable1();
    TestEesmIrTable2();
}

/**
 * @ingroup test
 *
 * @brief Test of the HARQ-IR combining of NrEesmIrT1 against the HARQ-CC combining of NrEesmCcT1.
 *
 * The effective code rate of HARQ-IR is mapped to the MCSs of the modulation order of the
 * transmission, so it stops falling at the lowest code rate of that order: without the rate
 * credit (attribute CreditRateBelowModulationFloor), the 3rd and 4th transmissions of a block
 * gained nothing, and none gained anything when the block started at the lowest MCS of its
 * modulation order, so HARQ-IR was worse than HARQ-CC, while incremental redundancy is never
 * worse than Chase combining. With the same SINR on every RB and every transmission, the test
 * checks that the credit is enabled by default, that it makes the retransmissions decode where
 * they did not without it, and that HARQ-IR is never worse than HARQ-CC for the 16-QAM and
 * 64-QAM MCSs. The QPSK MCSs are excluded: their SINR-BLER curves are closer to each other
 * than their code rates, so the credit falls short of Chase combining there.
 */
class NrL2smEesmIrCreditTestCase : public TestCase
{
  public:
    NrL2smEesmIrCreditTestCase()
        : TestCase("HARQ-IR with the rate credit is not worse than HARQ-CC")
    {
    }

  private:
    void DoRun() override;

    /**
     * TBLER of each of 4 transmissions of a transport block with the same SINR on every RB.
     *
     * @param em the error model
     * @param mcs the MCS of the transport block (Table 1)
     * @param sinrDb the SINR of every RB and transmission [dB]
     * @return the TBLER after the 1st to the 4th transmission
     */
    static std::vector<double> Tbler(Ptr<NrEesmErrorModel> em, uint8_t mcs, double sinrDb);
};

std::vector<double>
NrL2smEesmIrCreditTestCase::Tbler(Ptr<NrEesmErrorModel> em, uint8_t mcs, double sinrDb)
{
    const uint32_t nRb = 26;
    std::vector<int> map(nRb);
    std::vector<double> freqs(nRb);
    for (uint32_t i = 0; i < nRb; ++i)
    {
        map[i] = i;
        freqs[i] = 4e9 + i * 360e3;
    }
    SpectrumValue sinr(Create<SpectrumModel>(freqs));
    sinr = std::pow(10.0, sinrDb / 10);
    const uint32_t tbBytes = em->GetPayloadSize(12 * 11, mcs, 1, nRb, NrErrorModel::DL);

    std::vector<double> tbler;
    NrErrorModel::NrErrorModelHistory history;
    for (int tx = 0; tx < 4; ++tx)
    {
        auto output = em->GetTbDecodificationStats(sinr, map, tbBytes, mcs, history);
        tbler.push_back(output->m_tbler);
        history.push_back(output);
    }
    return tbler;
}

void
NrL2smEesmIrCreditTestCase::DoRun()
{
    Ptr<NrEesmErrorModel> ir = CreateObject<NrEesmIrT1>();
    Ptr<NrEesmErrorModel> irNoCredit = CreateObject<NrEesmIrT1>();
    irNoCredit->SetAttribute("CreditRateBelowModulationFloor", BooleanValue(false));
    Ptr<NrEesmErrorModel> cc = CreateObject<NrEesmCcT1>();

    BooleanValue credit;
    ir->GetAttribute("CreditRateBelowModulationFloor", credit);
    NS_TEST_ASSERT_MSG_EQ(credit.Get(), true, "the rate credit must be enabled by default");

    // MCS 17, the lowest 64-QAM MCS: at 9 dB the first transmission fails; without the credit
    // no retransmission gains anything, with it the second one decodes.
    auto withCredit = Tbler(ir, 17, 9);
    auto withoutCredit = Tbler(irNoCredit, 17, 9);
    NS_TEST_ASSERT_MSG_GT(withCredit[0], 0.9, "MCS 17 at 9 dB: the first transmission fails");
    NS_TEST_ASSERT_MSG_LT(withCredit[1], 0.1, "MCS 17 at 9 dB: the credit decodes the 2nd one");
    NS_TEST_ASSERT_MSG_GT(withoutCredit[3], 0.9, "MCS 17 at 9 dB: no gain without the credit");

    // MCS 22 at 10 dB: the second transmission reaches the floor of 64-QAM and fails; the
    // credit makes the third one decode, while without it the third and fourth gain nothing.
    withCredit = Tbler(ir, 22, 10);
    withoutCredit = Tbler(irNoCredit, 22, 10);
    NS_TEST_ASSERT_MSG_GT(withCredit[1], 0.9, "MCS 22 at 10 dB: the 2nd transmission fails");
    NS_TEST_ASSERT_MSG_LT(withCredit[2], 0.1, "MCS 22 at 10 dB: the credit decodes the 3rd one");
    NS_TEST_ASSERT_MSG_GT(withoutCredit[3], 0.9, "MCS 22 at 10 dB: no gain without the credit");

    // HARQ-IR is never worse than HARQ-CC for the 16-QAM and 64-QAM MCSs.
    for (uint8_t mcs = 10; mcs <= ir->GetMaxMcs(); ++mcs)
    {
        for (double sinrDb = -10; sinrDb <= 30; sinrDb += 0.5)
        {
            auto irTbler = Tbler(ir, mcs, sinrDb);
            auto ccTbler = Tbler(cc, mcs, sinrDb);
            for (size_t tx = 0; tx < irTbler.size(); ++tx)
            {
                NS_TEST_ASSERT_MSG_LT_OR_EQ(irTbler[tx],
                                            ccTbler[tx] + 1e-9,
                                            "HARQ-IR worse than HARQ-CC at MCS "
                                                << +mcs << ", " << sinrDb << " dB, transmission "
                                                << tx + 1);
            }
        }
    }
}

class NrTestL2smEesm : public TestSuite
{
  public:
    NrTestL2smEesm()
        : TestSuite("nr-test-l2sm-eesm", Type::UNIT)
    {
        AddTestCase(new NrL2smEesmTestCase("First test"), Duration::QUICK);
        AddTestCase(new NrL2smEesmIrCreditTestCase(), Duration::QUICK);
    }
};

static NrTestL2smEesm NrTestL2smEesmTestSuite; //!< Nr test suite

} // namespace ns3

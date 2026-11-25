/*
 * Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Gabriel Ferreira <gabrielcarvfer@gmail.com>
 */

/**
 * @ingroup test
 * @file nr-amc-tb-size-test.cc
 *
 * @brief Test suite for the code block segmentation in NrAmc::CalculateTbSize.
 */

#include "ns3/nr-amc.h"
#include "ns3/nr-eesm-cc-t1.h"
#include "ns3/nr-eesm-ir-t1.h"
#include "ns3/nr-eesm-ir-t2.h"
#include "ns3/nr-error-model.h"
#include "ns3/object-factory.h"
#include "ns3/test.h"

#include <cmath>

using namespace ns3;

/**
 * @ingroup test
 *
 * @brief Check that NrAmc::CalculateTbSize segments the transport block into
 * the number of code blocks mandated by TS 38.212 Section 5.2.2.
 *
 * The number of code blocks is C = ceil(B / (Kcb - L)), a real-valued
 * division. Sweeping MCS and number of PRBs, the transport block size
 * returned by CalculateTbSize() must equal the payload minus one CRC per
 * resulting code block, with C computed in real arithmetic: an integer
 * division truncates the ratio before ceil is applied, undercounting the
 * code blocks whenever the transport block is not an exact multiple of the
 * code block size (e.g. 590 / 480 must give C = 2, not 1).
 */
class NrAmcTbSizeSegmentationTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     * @param errorModelType The TypeId name of the error model to configure.
     */
    explicit NrAmcTbSizeSegmentationTestCase(const std::string& errorModelType)
        : TestCase("TB size code block segmentation (" + errorModelType + ")"),
          m_errorModelType(errorModelType)
    {
    }

  private:
    void DoRun() override;

    std::string m_errorModelType; //!< Error model to test against
};

void
NrAmcTbSizeSegmentationTestCase::DoRun()
{
    // The TB and CB CRC length used by NrAmc, in bytes (24 bits).
    constexpr uint32_t crcLen = 24 / 8;
    constexpr uint8_t rank = 1;

    auto amc = CreateObject<NrAmc>();
    amc->SetErrorModelType(TypeId::LookupByName(m_errorModelType));
    amc->SetDlMode();

    ObjectFactory emFactory;
    emFactory.SetTypeId(TypeId::LookupByName(m_errorModelType));
    auto errorModel = DynamicCast<NrErrorModel>(emFactory.Create());
    NS_TEST_ASSERT_MSG_NE(errorModel, nullptr, "Could not create the error model");

    // Count the segmentation cases in which integer division would truncate
    // the code block ratio, to make sure the sweep actually exercises the
    // TS 38.212 rounding.
    uint32_t truncatingCases = 0;

    for (uint8_t mcs = 0; mcs <= errorModel->GetMaxMcs(); mcs += 4)
    {
        for (uint32_t nprb = 1; nprb <= 275; nprb += 3)
        {
            const uint32_t payload = amc->GetPayloadSize(mcs, rank, nprb);
            const uint32_t cbSize = errorModel->GetMaxCbSize(payload, mcs);

            uint32_t expected = payload;
            if (payload >= crcLen)
            {
                expected = payload - crcLen;
            }
            if (expected > cbSize)
            {
                // TS 38.212 Section 5.2.2: C = ceil(B / (Kcb - L)), computed
                // with a real-valued division.
                const auto c =
                    static_cast<uint32_t>(std::ceil(static_cast<double>(expected) / cbSize));
                expected = payload - c * crcLen;
                if (payload - crcLen != c * cbSize)
                {
                    truncatingCases++;
                }
            }

            NS_TEST_ASSERT_MSG_EQ(amc->CalculateTbSize(mcs, rank, nprb),
                                  expected,
                                  "Wrong TB size for mcs " << +mcs << " nprb " << nprb
                                                           << " (payload " << payload
                                                           << ", CB size " << cbSize << ")");
        }
    }

    NS_TEST_ASSERT_MSG_GT(truncatingCases,
                          0,
                          "The sweep did not exercise any non-integer code block ratio");
}

/**
 * @ingroup test
 *
 * @brief Test suite for the code block segmentation in NrAmc::CalculateTbSize.
 */
class NrAmcTbSizeTestSuite : public TestSuite
{
  public:
    NrAmcTbSizeTestSuite()
        : TestSuite("nr-amc-tb-size", Type::UNIT)
    {
        AddTestCase(new NrAmcTbSizeSegmentationTestCase("ns3::NrEesmIrT1"),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrAmcTbSizeSegmentationTestCase("ns3::NrEesmIrT2"),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrAmcTbSizeSegmentationTestCase("ns3::NrEesmCcT1"),
                    TestCase::Duration::QUICK);
    }
};

/// Static variable for test initialization
static NrAmcTbSizeTestSuite g_nrAmcTbSizeTestSuite;

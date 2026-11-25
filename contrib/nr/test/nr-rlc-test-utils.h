// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_RLC_TEST_UTILS_H
#define NR_RLC_TEST_UTILS_H

#include "ns3/nr-mac-sap.h"
#include "ns3/nr-rlc-sap.h"
#include "ns3/packet.h"
#include "ns3/test.h"

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

/**
 * @ingroup tests
 * @file nr-rlc-test-utils.h
 *
 * @brief Helpers shared by the white-box RLC test suites (TM, UM and AM): a recording
 * RLC SAP user, a capturing MAC SAP provider, identifiable-payload builders and
 * checkers, and a TestCase base holding the trace sinks and the delivery and structure
 * verification routines used by more than one suite.
 */

namespace ns3
{

/**
 * @ingroup tests
 *
 * @brief RLC SAP user recording every SDU delivered by the RLC entity under test.
 *
 * NrTestPdcp only keeps the last delivered SDU, so the tests that need the full
 * ordered sequence of deliveries (count and content) use this sink instead.
 */
class NrRlcTestSduSink
{
  public:
    NrRlcTestSduSink()
        : m_sapUser(new NrRlcSpecificNrRlcSapUser<NrRlcTestSduSink>(this))
    {
    }

    ~NrRlcTestSduSink()
    {
        delete m_sapUser;
    }

    /**
     * Receive and record an SDU delivered by the RLC entity.
     *
     * @param p the delivered SDU
     */
    void DoReceivePdcpPdu(Ptr<Packet> p)
    {
        m_sdus.push_back(p);
    }

    /**
     * @return the RLC SAP user to install on the RLC entity under test
     */
    NrRlcSapUser* GetSapUser() const
    {
        return m_sapUser;
    }

    std::vector<Ptr<Packet>> m_sdus; ///< delivered SDUs in delivery order

  private:
    NrRlcSapUser* m_sapUser; ///< the RLC SAP user
};

/**
 * @ingroup tests
 *
 * @brief MAC SAP provider capturing every PDU handed down by the RLC entity under test.
 */
class NrRlcTestCaptureMac : public NrMacSapProvider
{
  public:
    void TransmitPdu(TransmitPduParameters params) override
    {
        m_pdus.push_back(params.pdu);
    }

    void BufferStatusReport(BufferStatusReportParameters params) override
    {
    }

    std::vector<Ptr<Packet>> m_pdus; ///< captured PDUs in transmission order
};

/// One SDU of a test stream: fill byte (identifying the SDU) and size in bytes
using NrRlcSduSpec = std::pair<uint8_t, uint32_t>;

/**
 * Create an SDU of the given size filled with the given byte.
 *
 * @param fill the fill byte identifying the SDU
 * @param size the SDU size in bytes
 * @return the SDU
 */
inline Ptr<Packet>
MakeUniformSdu(uint8_t fill, uint32_t size)
{
    std::vector<uint8_t> buf(size, fill);
    return Create<Packet>(buf.data(), buf.size());
}

/**
 * Check that a delivered SDU has the given size and uniform fill byte.
 *
 * @param sdu the delivered SDU
 * @param fill the expected fill byte
 * @param size the expected size
 * @return true if size and every byte match
 */
inline bool
IsUniformSdu(Ptr<Packet> sdu, uint8_t fill, uint32_t size)
{
    if (sdu->GetSize() != size)
    {
        return false;
    }
    std::vector<uint8_t> buf(sdu->GetSize());
    sdu->CopyData(buf.data(), buf.size());
    return std::all_of(buf.begin(), buf.end(), [fill](uint8_t byte) { return byte == fill; });
}

/**
 * Collect the distinct SDU fill bytes present in the data field of a captured PDU.
 *
 * @tparam HeaderT the RLC header type of the PDU (NrRlcHeader or NrRlcAmHeader)
 * @param pdu the captured PDU
 * @return the set of fill bytes it carries
 */
template <class HeaderT>
std::set<uint8_t>
FillsInPdu(Ptr<Packet> pdu)
{
    Ptr<Packet> copy = pdu->Copy();
    HeaderT header;
    copy->RemoveHeader(header);
    std::vector<uint8_t> buf(copy->GetSize());
    copy->CopyData(buf.data(), buf.size());
    return std::set<uint8_t>(buf.begin(), buf.end());
}

/**
 * Deliver a copy of the given PDU to the RLC entity through the MAC SAP.
 *
 * @tparam RlcT the RLC entity type
 * @param rlc the receiving RLC entity
 * @param pdu the PDU to deliver
 */
template <class RlcT>
void
DeliverPduToRlc(Ptr<RlcT> rlc, Ptr<Packet> pdu)
{
    NrMacSapUser::ReceivePduParameters rxPduParams = {};
    rxPduParams.p = pdu->Copy();
    rxPduParams.rnti = 0;
    rxPduParams.lcid = 0;
    rlc->DoReceivePdu(rxPduParams);
}

/**
 * Queue the given SDUs in the transmission buffer of the RLC entity.
 *
 * @tparam RlcT the RLC entity type
 * @param rlc the transmitting RLC entity
 * @param sdus the SDUs to queue
 */
template <class RlcT>
void
QueueSdus(Ptr<RlcT> rlc, const std::vector<NrRlcSduSpec>& sdus)
{
    for (const auto& [fill, size] : sdus)
    {
        rlc->DoTransmitPdcpPdu(MakeUniformSdu(fill, size));
    }
}

/**
 * Grant a number of equally sized transmission opportunities to the RLC entity.
 *
 * @tparam RlcT the RLC entity type
 * @param rlc the transmitting RLC entity
 * @param bytes the size in bytes of each transmission opportunity
 * @param count the number of transmission opportunities
 */
template <class RlcT>
void
GrantTxOpportunities(Ptr<RlcT> rlc, uint32_t bytes, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
    {
        rlc->DoNotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters(bytes, 0, 0, 0, 0, 3));
    }
}

/**
 * Transmit the given SDUs one per transmission opportunity, so that every produced
 * PDU is SDU-aligned.
 *
 * @tparam RlcT the RLC entity type
 * @param rlc the transmitting RLC entity
 * @param sdus the SDUs to transmit
 * @param bytes the size in bytes of each transmission opportunity
 */
template <class RlcT>
void
TransmitSdusAligned(Ptr<RlcT> rlc, const std::vector<NrRlcSduSpec>& sdus, uint32_t bytes)
{
    for (const auto& [fill, size] : sdus)
    {
        rlc->DoTransmitPdcpPdu(MakeUniformSdu(fill, size));
        GrantTxOpportunities(rlc, bytes, 1);
    }
}

/**
 * @ingroup tests
 *
 * @brief Base of the white-box RLC test cases, holding the trace sinks and the
 * verification routines shared by the TM, UM and AM suites.
 */
class NrRlcTestCaseBase : public TestCase
{
  public:
    /**
     * TxPDU trace sink.
     *
     * @param rnti the RNTI
     * @param lcid the logical channel id
     * @param bytes the PDU size in bytes
     */
    void HandleTxPdu(uint16_t rnti, uint8_t lcid, uint32_t bytes)
    {
        m_pduSizes.push_back(bytes);
    }

    /**
     * TxDrop trace sink.
     *
     * @param p the dropped packet
     */
    void HandleTxDrop(Ptr<const Packet> p)
    {
        m_dropCount++;
    }

    std::vector<uint32_t> m_pduSizes; ///< sizes of the PDUs handed to the MAC
    uint32_t m_dropCount{0};          ///< count of TxDrop trace invocations

  protected:
    /**
     * Constructor.
     *
     * @param name the test case name
     */
    NrRlcTestCaseBase(std::string name)
        : TestCase(name)
    {
    }

    /**
     * Verify the delivered SDUs are exactly the expected ones, in order, byte-exact.
     *
     * @param sink the sink holding the delivered SDUs
     * @param expected the expected SDUs in expected delivery order
     * @param label scenario label for the assertion messages
     */
    void VerifyDelivered(const NrRlcTestSduSink& sink,
                         const std::vector<NrRlcSduSpec>& expected,
                         const std::string& label)
    {
        NS_TEST_ASSERT_MSG_EQ(sink.m_sdus.size(),
                              expected.size(),
                              label << ": number of delivered SDUs");
        for (uint32_t i = 0; i < std::min(sink.m_sdus.size(), expected.size()); i++)
        {
            NS_TEST_ASSERT_MSG_EQ(
                IsUniformSdu(sink.m_sdus.at(i), expected[i].first, expected[i].second),
                true,
                label << ": SDU " << i << " ('" << (char)expected[i].first << "', "
                      << expected[i].second << " bytes) must be delivered byte-exact");
        }
    }

    /**
     * Verify the captured PDUs exercise both segmentation (an SDU spanning several
     * PDUs) and concatenation (a PDU carrying several SDUs), so that a data-integrity
     * scenario built on them is actually meaningful.
     *
     * @tparam HeaderT the RLC header type of the PDUs (NrRlcHeader or NrRlcAmHeader)
     * @param pdus the captured PDUs
     */
    template <class HeaderT>
    void CheckSegmentationAndConcatenation(const std::vector<Ptr<Packet>>& pdus)
    {
        std::map<uint8_t, uint32_t> pdusPerFill;
        bool anyConcatenation = false;
        for (const auto& pdu : pdus)
        {
            std::set<uint8_t> fills = FillsInPdu<HeaderT>(pdu);
            anyConcatenation |= (fills.size() > 1);
            for (uint8_t fill : fills)
            {
                pdusPerFill[fill]++;
            }
        }
        bool anySegmentation = false;
        for (const auto& [fill, count] : pdusPerFill)
        {
            anySegmentation |= (count > 1);
        }
        NS_TEST_ASSERT_MSG_EQ(anySegmentation, true, "the PDUs must exercise segmentation");
        NS_TEST_ASSERT_MSG_EQ(anyConcatenation, true, "the PDUs must exercise concatenation");
    }
};

} // namespace ns3

#endif // NR_RLC_TEST_UTILS_H

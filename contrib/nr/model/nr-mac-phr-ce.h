// Copyright (c) 2026 Muhammad Uzair
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_MAC_PHR_CE_H
#define NR_MAC_PHR_CE_H

#include "nr-mac-header-fs-ul.h"

#include "ns3/header.h"
#include "ns3/tag.h"

namespace ns3
{

/**
 * \ingroup ue-mac
 * \brief Single Entry PHR MAC CE (TS 38.321 6.1.3.8), with its fixed-size
 * subheader (LCID 57).
 *
 * Two octets after the one-octet subheader:
\verbatim
 +-----+-----+-----------------------------------+
 |  R  |  R  |  PH (type 1, PCell)               |   Oct 1
 +-----+-----+-----------------------------------+
 |  R  |  R  |  P_CMAX,f,c                       |   Oct 2
 +-----+-----+-----------------------------------+
\endverbatim
 *
 * PH is the TS 38.133 Table 10.1.17.1-1 report index (0..63) of the type 1
 * power headroom of TS 38.213 7.7.1 for the PUSCH carrying the CE; P_CMAX,f,c
 * is the TS 38.133 Table 10.1.18.1-1 index (0..63) of the configured maximum
 * output power used for that PH.
 */
class NrMacSingleEntryPhrCe : public Header
{
  public:
    /**
     * \brief GetTypeId
     * \return the type id of the object
     */
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;

    NrMacSingleEntryPhrCe();

    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    uint32_t GetSerializedSize() const override;
    void Print(std::ostream& os) const override;

    /**
     * \brief TS 38.133 Table 10.1.17.1-1: power headroom (dB) to report index.
     * \param phDb power headroom in dB
     * \return POWER_HEADROOM_n index n, 0..63
     */
    static uint8_t PhDbToIndex(double phDb);
    /**
     * \brief Lower edge (dB) of the PH range a report index stands for
     * (index 0, PH < -32 dB, is decoded as -33 dB).
     * \param index report index 0..63
     * \return the power headroom in dB the network may rely on
     */
    static double PhIndexToDb(uint8_t index);
    /**
     * \brief TS 38.133 Table 10.1.18.1-1: P_CMAX,f,c (dBm) to report index.
     * \param pcmaxDbm configured maximum output power in dBm
     * \return PCMAX_C_n index n, 0..63
     */
    static uint8_t PcmaxDbmToIndex(double pcmaxDbm);
    /**
     * \brief Lower edge (dBm) of the P_CMAX,f,c range a report index stands for.
     * \param index report index 0..63
     * \return P_CMAX,f,c in dBm
     */
    static double PcmaxIndexToDbm(uint8_t index);

    uint8_t m_phIndex{0};    //!< PH field (6 bits)
    uint8_t m_pcmaxIndex{0}; //!< P_CMAX,f,c field (6 bits)

  private:
    NrMacHeaderFsUl m_header; //!< Fixed-size subheader (LCID 57)
};

/**
 * \ingroup gnb-phy
 * \brief Number of PRBs of the PUSCH a packet was received on.
 *
 * Added by the gNB NrSpectrumPhy to a decoded uplink packet that carries a
 * PHR MAC CE. It stands for knowledge the gNB has in any case: it scheduled
 * that PUSCH, so it knows the M_RB the UE's type 1 power headroom refers to
 * (TS 38.213 7.7.1). Not transmitted over the air.
 */
class NrPuschAllocationTag : public Tag
{
  public:
    /**
     * \brief GetTypeId
     * \return the type id of the object
     */
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;

    NrPuschAllocationTag() = default;
    /**
     * \brief Constructor
     * \param numRb number of PRBs of the PUSCH
     */
    explicit NrPuschAllocationTag(uint16_t numRb);

    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;

    /**
     * \return the number of PRBs of the PUSCH
     */
    uint16_t GetNumRb() const;

  private:
    uint16_t m_numRb{0}; //!< PRBs of the PUSCH
};

} // namespace ns3

#endif /* NR_MAC_PHR_CE_H */

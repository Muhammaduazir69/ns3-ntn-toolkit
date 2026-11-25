// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Jaume Nin <jnin@cttc.cat>

#ifndef NR_EPS_GTPU_V1_H
#define NR_EPS_GTPU_V1_H

#include "ns3/header.h"
#include "ns3/ipv4-header.h"
#include "ns3/nr-export.h"
#include "ns3/ptr.h"

#include <vector>

namespace ns3
{

class Packet;

/**
 * @ingroup nr
 *
 * Implementation of the GPRS Tunnelling Protocol header according to
 * GTPv1-U Release 10 as per 3Gpp TS 29.281 document
 *
 */
class NR_EXPORT NrGtpuHeader : public Header
{
  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    NrGtpuHeader();
    ~NrGtpuHeader() override;
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    /**
     * Get the extension header flag, set when the header has extension headers.
     * @returns the extension header flag
     */
    bool GetExtensionHeaderFlag() const;
    /**
     * Get length function
     * @returns the length
     */
    uint16_t GetLength() const;
    /**
     * Get message type function
     * @returns the message type
     */
    uint8_t GetMessageType() const;
    /**
     * Get NPDU number function
     * @returns the NPDU number
     */
    uint8_t GetNPduNumber() const;
    /**
     * Get type of GTP-U message function
     * @returns the NPDU number flag
     */
    bool GetNPduNumberFlag() const;
    /**
     * Get the type of the first extension header.
     * @returns the next extension type, 0 if there is no extension header
     */
    uint8_t GetNextExtensionType() const;
    /**
     * Get protocol type function
     * @returns the protocol type
     */
    bool GetProtocolType() const;
    /**
     * Get protocol type function
     * @returns the sequence number
     */
    uint16_t GetSequenceNumber() const;
    /**
     * Get sequence number flag function
     * @returns the sequence number flag
     */
    bool GetSequenceNumberFlag() const;
    /**
     * Get a tunnel endpoint identificator (TEID)
     * @returns the TEID
     */
    uint32_t GetTeid() const;
    /**
     * Get version function
     * @returns the version
     */
    uint8_t GetVersion() const;
    /**
     * Set the length in octets of the payload
     * @param length the length
     */
    void SetLength(uint16_t length);
    /**
     * Set message type function
     * @param messageType the message type
     */
    void SetMessageType(uint8_t messageType);
    /**
     * Set NPDU number function
     * @param nPduNumber the NPDU number
     */
    void SetNPduNumber(uint8_t nPduNumber);
    /**
     * Sets the flag that indicates the presence of a meaningful value of the N-PDU Number field.
     * @param nPduNumberFlag the NPDU number flag
     */
    void SetNPduNumberFlag(bool nPduNumberFlag);
    /**
     * Set protocol type function
     * @param protocolType the protocol type
     */
    void SetProtocolType(bool protocolType);
    /**
     * Set sequence number function
     * @param sequenceNumber the sequence number
     */
    void SetSequenceNumber(uint16_t sequenceNumber);
    /**
     * Set sequence number flag function
     * @param sequenceNumberFlag the sequence number flag
     */
    void SetSequenceNumberFlag(bool sequenceNumberFlag);
    /**
     * Set TEID function
     * @param teid the TEID
     */
    void SetTeid(uint32_t teid);
    /**
     * Set version function
     * @param version the version
     */
    void SetVersion(uint8_t version);

    /// An extension header (TS 29.281, section 5.2)
    struct ExtensionHeader
    {
        uint8_t type; ///< the type of the extension header
        /// the content of the extension header, padded with zeros so that the extension header
        /// (length, content and next extension header type) is a multiple of 4 octets
        std::vector<uint8_t> content;

        /**
         * Equality operator.
         * @param other another extension header
         * @return true if both are equal
         */
        bool operator==(const ExtensionHeader& other) const = default;
    };

    /**
     * Add an extension header, after the ones already added.
     * @param type the type of the extension header (not 0)
     * @param content the content of the extension header, which is padded with zeros if needed
     */
    void AddExtensionHeader(uint8_t type, const std::vector<uint8_t>& content);

    /**
     * @return the extension headers, in order
     */
    const std::vector<ExtensionHeader>& GetExtensionHeaders() const;

    /// Type of the PDU Session Container extension header (TS 29.281, section 5.2.1)
    static constexpr uint8_t PDU_SESSION_CONTAINER{0x85};

    /// PDU type of the PDU Session Container (TS 38.415, section 5.5.3.1)
    enum PduType : uint8_t
    {
        DL_PDU_SESSION_INFORMATION = 0, ///< downlink PDU session information
        UL_PDU_SESSION_INFORMATION = 1, ///< uplink PDU session information
    };

    /**
     * Set the PDU Session Container extension header (TS 38.415, section 5.5.2), which carries
     * the QFI of the packet on the N3 interface, replacing the existing one if any.
     * @param pduType the PDU type
     * @param qfi the QoS flow identifier
     */
    void SetPduSessionContainer(PduType pduType, uint8_t qfi);

    /**
     * @return true if the header has a PDU Session Container extension header
     */
    bool HasPduSessionContainer() const;

    /**
     * @return the PDU type of the PDU Session Container extension header, which must exist
     */
    PduType GetPduType() const;

    /**
     * @return the QFI of the PDU Session Container extension header, which must exist
     */
    uint8_t GetQfi() const;

    /**
     * Equality operator.
     *
     * @param b NrGtpuHeader object to compare
     * @returns true of equal
     */
    bool operator==(const NrGtpuHeader& b) const;

  private:
    /**
     * @return whether the optional fields (sequence number, N-PDU number and next extension
     * header type) are present, i.e., whether any of the E, S and PN flags is set
     */
    bool HasOptionalFields() const;

    /**
     * @return the PDU Session Container extension header, or nullptr if there is none
     */
    const ExtensionHeader* FindPduSessionContainer() const;

    /**
     * This field is used to determine the version of the GTPU-U protocol. The version number shall
     * be set to 1.
     */
    uint8_t m_version; // really a 3 uint3_t

    /**
     * This bit is used as a protocol discriminator between GTP (when PT is '1') and GTP' (when
     * PT is '0').
     */
    bool m_protocolType;

    /**
     * This flag indicates the presence of a meaningful value of the Sequence Number field.
     * When it is set to '0', the Sequence Number field either is not present or, if present,
     * shall not be interpreted. When it is set to '1', the Sequence Number field is present,
     * and shall be interpreted
     */
    bool m_sequenceNumberFlag;
    /**
     * This flag indicates the presence of a meaningful value of the N-PDU Number field.
     * When it is set to '0', the N-PDU Number field either is not present, or, if present,
     * shall not be interpreted. When it is set to '1', the N-PDU Number field is present,
     * and shall be interpreted
     */
    bool m_nPduNumberFlag;
    /**
     *  This field indicates the type of GTP-U message
     */
    uint8_t m_messageType;
    /**
     *  This field indicates the length in octets of the payload, i.e. the rest of the packet
     *  following the mandatory part of the GTP header (that is the first 8 octets).
     *  The Sequence Number, the N-PDU Number or any Extension headers shall be considered
     *  to be part of the payload, i.e. included in the length count
     */
    uint16_t m_length;

    /**
     * This field unambiguously identifies a tunnel endpoint in the receiving GTP-U
     * protocol entity. The receiving end side of a GTP tunnel locally assigns the
     * TEID value the transmitting side has to use.
     */
    uint32_t m_teid;
    /**
     * If Sequence Number field is used for G-PDUs (T-PDUs+headers), an increasing
     * sequence number for T-PDUs is transmitted via GTP-U tunnels, when transmission
     * order must be preserved
     */
    uint16_t m_sequenceNumber;
    /**
     * This field is used at the Inter SGSN Routeing Area Update procedure and some
     * inter-system handover procedures (e.g. between 2G and 3G radio access networks).
     * This field is used to co-ordinate the data transmission for acknowledged mode
     * of communication between the MS and the SGSN
     */
    uint8_t m_nPduNumber;
    std::vector<ExtensionHeader> m_extensionHeaders; ///< the extension headers, in order
};

} // namespace ns3

#endif /* NR_EPS_GTPU_V1_H */

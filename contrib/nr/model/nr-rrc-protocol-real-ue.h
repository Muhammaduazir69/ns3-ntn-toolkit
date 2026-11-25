// Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nicola Baldo <nbaldo@cttc.es>
//          Lluis Parcerisa <lparcerisa@cttc.cat>

#ifndef NR_RRC_PROTOCOL_REAL_UE_H
#define NR_RRC_PROTOCOL_REAL_UE_H

#include "nr-pdcp-sap.h"
#include "nr-rlc-sap.h"
#include "nr-rrc-sap.h"

#include "ns3/nr-export.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <map>
#include <stdint.h>

namespace ns3
{

class NrUeRrcSapProvider;
class NrUeRrcSapUser;
class NrUeRrc;
class NrGnbNetDevice;

namespace nr
{
/**
 * @ingroup nr
 *
 * Models the transmission of RRC messages from the UE to the gNB in
 * a real fashion, by creating real RRC PDUs and transmitting them
 * over Signaling Radio Bearers using radio resources allocated by the
 * NR MAC scheduler.
 *
 */
class NR_EXPORT UeRrcProtocolReal : public Object
{
    /// allow MemberNrUeRrcSapUser<UeRrcProtocolReal> class friend access
    friend class MemberNrUeRrcSapUser<UeRrcProtocolReal>;
    /// allow NrRlcSpecificNrRlcSapUser<UeRrcProtocolReal> class friend access
    friend class NrRlcSpecificNrRlcSapUser<UeRrcProtocolReal>;
    /// allow NrPdcpSpecificNrPdcpSapUser<UeRrcProtocolReal> class friend access
    friend class NrPdcpSpecificNrPdcpSapUser<UeRrcProtocolReal>;

  public:
    UeRrcProtocolReal();
    ~UeRrcProtocolReal() override;

    // inherited from Object
    void DoDispose() override;
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * Set NR UE RRC SAP provider function
     *
     * @param p the NR UE RRC SAP provider
     */
    void SetNrUeRrcSapProvider(NrUeRrcSapProvider* p);
    /**
     * Get NR UE RRC SAP user function
     *
     * @returns NR UE RRC SAP user
     */
    NrUeRrcSapUser* GetNrUeRrcSapUser();

    /**
     * Set UE RRC function
     *
     * @param rrc the NR UE RRC
     */
    void SetUeRrc(Ptr<NrUeRrc> rrc);

  private:
    // methods forwarded from NrUeRrcSapUser
    /**
     * Setup function
     *
     * @param params NrUeRrcSapUser::SetupParameters
     */
    void DoSetup(NrUeRrcSapUser::SetupParameters params);
    /**
     * Send RRC connection request function
     *
     * @param msg NrRrcSap::RrcConnectionRequest
     */
    void DoSendRrcConnectionRequest(NrRrcSap::RrcConnectionRequest msg);
    /**
     * Send RRC connection setup completed function
     *
     * @param msg NrRrcSap::RrcConnectionSetupCompleted
     */
    void DoSendRrcConnectionSetupCompleted(NrRrcSap::RrcConnectionSetupCompleted msg) const;
    /**
     * Send RRC connection reconfiguration setup completed function
     *
     * @param msg NrRrcSap::RrcConnectionReconfigurationCompleted
     */
    void DoSendRrcConnectionReconfigurationCompleted(
        NrRrcSap::RrcConnectionReconfigurationCompleted msg);
    /**
     * Send RRC connection reestablishment request function
     *
     * @param msg NrRrcSap::RrcConnectionReestablishmentRequest
     */
    void DoSendRrcConnectionReestablishmentRequest(
        NrRrcSap::RrcConnectionReestablishmentRequest msg) const;
    /**
     * Send RRC connection reestablishment complete function
     *
     * @param msg NrRrcSap::RrcConnectionReestablishmentComplete
     */
    void DoSendRrcConnectionReestablishmentComplete(
        NrRrcSap::RrcConnectionReestablishmentComplete msg) const;
    /**
     * Send measurement report function
     *
     * @param msg NrRrcSap::MeasurementReport
     */
    void DoSendMeasurementReport(NrRrcSap::MeasurementReport msg);
    /**
     * @brief Send ideal UE context remove request function
     *
     * Notify eNodeB to release UE context once radio link failure
     * or random access failure is detected. It is needed since no
     * RLF detection mechanism at eNodeB is implemented
     *
     * @param rnti the RNTI of the UE
     */
    void DoSendIdealUeContextRemoveRequest(uint16_t rnti);

    /**
     * @brief Send the UE's same-cell primary-BWP switch indication to the gNB.
     * @param rnti  the RNTI of the UE
     * @param bwpId the new primary BWP/CC index the UE switched to
     */
    void DoSendIdealBwpSwitchIndication(uint16_t rnti, uint8_t bwpId);

    /// Set gNB RRC SAP provider
    void SetGnbRrcSapProvider();
    /**
     * Receive PDCP PDU function
     *
     * @param p the packet
     */
    void DoReceivePdcpPdu(Ptr<Packet> p);
    /**
     * Receive PDCP SDU function
     *
     * @param params NrPdcpSapUser::ReceivePdcpSduParameters
     */
    void DoReceivePdcpSdu(NrPdcpSapUser::ReceivePdcpSduParameters params);

    Ptr<NrUeRrc> m_rrc;                       ///< the RRC
    uint16_t m_rnti;                          ///< the RNTI
    NrUeRrcSapProvider* m_ueRrcSapProvider;   ///< UE RRC SAP provider
    NrUeRrcSapUser* m_ueRrcSapUser;           ///< UE RRC SAP user
    NrGnbRrcSapProvider* m_gnbRrcSapProvider; ///< gNB RRC SAP provider

    NrUeRrcSapUser::SetupParameters m_setupParameters; ///< setup parameters
    NrUeRrcSapProvider::CompleteSetupParameters
        m_completeSetupParameters; ///< complete setup parameters
    std::unordered_map<uint16_t, Ptr<NrGnbNetDevice>> m_knownGnb;
};

} // namespace nr
} // namespace ns3

#endif // NR_RRC_PROTOCOL_REAL_UE_H

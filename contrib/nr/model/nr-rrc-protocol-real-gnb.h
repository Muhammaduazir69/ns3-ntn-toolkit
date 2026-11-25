// Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nicola Baldo <nbaldo@cttc.es>
//          Lluis Parcerisa <lparcerisa@cttc.cat>

#ifndef NR_RRC_PROTOCOL_REAL_GNB_H
#define NR_RRC_PROTOCOL_REAL_GNB_H

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

class NrGnbRrcSapProvider;
class NrGnbNetDevice;

namespace nr
{

/**
 * Models the transmission of RRC messages from the UE to the gNB in
 * a real fashion, by creating real RRC PDUs and transmitting them
 * over Signaling Radio Bearers using radio resources allocated by the
 * NR MAC scheduler.
 *
 */
class NR_EXPORT NrGnbRrcProtocolReal : public Object
{
    /// allow MemberNrGnbRrcSapUser<NrGnbRrcProtocolReal> class friend access
    friend class MemberNrGnbRrcSapUser<NrGnbRrcProtocolReal>;
    /// allow NrPdcpSpecificNrPdcpSapUser<NrGnbRrcProtocolReal> class friend access
    friend class NrPdcpSpecificNrPdcpSapUser<NrGnbRrcProtocolReal>;
    /// allow NrRlcSpecificNrRlcSapUser<NrGnbRrcProtocolReal> class friend access
    friend class NrRlcSpecificNrRlcSapUser<NrGnbRrcProtocolReal>;
    /// allow RealProtocolRlcSapUser class friend access
    friend class RealProtocolRlcSapUser;

  public:
    NrGnbRrcProtocolReal();
    ~NrGnbRrcProtocolReal() override;

    // inherited from Object
    void DoDispose() override;
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * Set NR gNB RRC SAP provider function
     *
     * @param p NrGnbRrcSapProvider *
     */
    void SetNrGnbRrcSapProvider(NrGnbRrcSapProvider* p);
    /**
     * Get NR gNB RRC SAP user function
     *
     * @returns NrGnbRrcSapUser *
     */
    NrGnbRrcSapUser* GetNrGnbRrcSapUser();

    /**
     * Set cell ID function
     *
     * @param cellId the cell ID
     */
    void SetCellId(uint16_t cellId);

    /**
     * Get UE RRC SAP provider function
     *
     * @param rnti the RNTI
     * @returns NrUeRrcSapProvider *
     */
    NrUeRrcSapProvider* GetUeRrcSapProvider(uint16_t rnti);
    /**
     * Set UE RRC SAP provider function
     *
     * @param rnti the RNTI
     * @param p NrUeRrcSapProvider *
     */
    void SetUeRrcSapProvider(uint16_t rnti, NrUeRrcSapProvider* p);

  private:
    // methods forwarded from NrGnbRrcSapUser
    /**
     * Setup UE function
     *
     * @param rnti the RNTI
     * @param params NrGnbRrcSapUser::SetupUeParameters
     */
    void DoSetupUe(uint16_t rnti, NrGnbRrcSapUser::SetupUeParameters params);
    /**
     * Remove UE function
     *
     * @param rnti the RNTI
     */
    void DoRemoveUe(uint16_t rnti);
    /**
     * Send system information function
     *
     * @param cellId cell ID
     * @param msg NrRrcSap::SystemInformation
     */
    void DoSendSystemInformation(uint16_t cellId, NrRrcSap::SystemInformation msg);
    /**
     * Send system information function
     *
     * @param cellId cell ID
     * @param msg NrRrcSap::SystemInformation
     */
    void SendSystemInformation(uint16_t cellId, NrRrcSap::SystemInformation msg);
    /**
     * Send RRC connection setup function
     *
     * @param rnti the RNTI
     * @param msg NrRrcSap::RrcConnectionSetup
     */
    void DoSendRrcConnectionSetup(uint16_t rnti, NrRrcSap::RrcConnectionSetup msg);
    /**
     * Send RRC connection reconfiguration function
     *
     * @param rnti the RNTI
     * @param msg NrRrcSap::RrcConnectionReconfiguration
     */
    void DoSendRrcConnectionReconfiguration(uint16_t rnti,
                                            NrRrcSap::RrcConnectionReconfiguration msg);
    /**
     * Send RRC connection reestabishment function
     *
     * @param rnti the RNTI
     * @param msg NrRrcSap::RrcConnectionReestablishment
     */
    void DoSendRrcConnectionReestablishment(uint16_t rnti,
                                            NrRrcSap::RrcConnectionReestablishment msg);
    /**
     * Send RRC connection reestabishment reject function
     *
     * @param rnti the RNTI
     * @param msg NrRrcSap::RrcConnectionReestablishmentReject
     */
    void DoSendRrcConnectionReestablishmentReject(uint16_t rnti,
                                                  NrRrcSap::RrcConnectionReestablishmentReject msg);
    /**
     * Send RRC connection release function
     *
     * @param rnti the RNTI
     * @param msg NrRrcSap::RrcConnectionRelease
     */
    void DoSendRrcConnectionRelease(uint16_t rnti, NrRrcSap::RrcConnectionRelease msg);
    /**
     * Send RRC connection reject function
     *
     * @param rnti the RNTI
     * @param msg NrRrcSap::RrcConnectionReject
     */
    void DoSendRrcConnectionReject(uint16_t rnti, NrRrcSap::RrcConnectionReject msg);
    /**
     * Encode handover preparation information function
     *
     * @param msg NrRrcSap::HandoverPreparationInfo
     * @returns the packet
     */
    Ptr<Packet> DoEncodeHandoverPreparationInformation(NrRrcSap::HandoverPreparationInfo msg);
    /**
     * Decode handover preparation information function
     *
     * @param p the packet
     * @returns NrRrcSap::HandoverPreparationInfo
     */
    NrRrcSap::HandoverPreparationInfo DoDecodeHandoverPreparationInformation(Ptr<Packet> p);
    /**
     * Encode handover command function
     *
     * @param msg NrRrcSap::RrcConnectionReconfiguration
     * @returns the packet
     */
    Ptr<Packet> DoEncodeHandoverCommand(NrRrcSap::RrcConnectionReconfiguration msg);
    /**
     * Decode handover command function
     *
     * @param p the packet
     * @returns NrRrcSap::RrcConnectionReconfiguration
     */
    NrRrcSap::RrcConnectionReconfiguration DoDecodeHandoverCommand(Ptr<Packet> p);

    /**
     * Receive PDCP SDU function
     *
     * @param params NrPdcpSapUser::ReceivePdcpSduParameters
     */
    void DoReceivePdcpSdu(NrPdcpSapUser::ReceivePdcpSduParameters params);
    /**
     * Receive PDCP PDU function
     *
     * @param rnti the RNTI
     * @param p the packet
     */
    void DoReceivePdcpPdu(uint16_t rnti, Ptr<Packet> p);

    uint16_t m_rnti;                                                ///< the RNTI
    uint16_t m_cellId;                                              ///< the cell ID
    NrGnbRrcSapProvider* m_gnbRrcSapProvider;                       ///< gNB RRC SAP provider
    NrGnbRrcSapUser* m_gnbRrcSapUser;                               ///< gNB RRC SAP user
    std::map<uint16_t, NrUeRrcSapProvider*> m_gnbRrcSapProviderMap; ///< gNB RRC SAP provider map
    std::map<uint16_t, NrGnbRrcSapUser::SetupUeParameters>
        m_setupUeParametersMap; ///< setup UE parameters map
    std::map<uint16_t, NrGnbRrcSapProvider::CompleteSetupUeParameters>
        m_completeSetupUeParametersMap; ///< complete setup UE parameters map
};

/// RealProtocolRlcSapUser class
class NR_EXPORT RealProtocolRlcSapUser : public NrRlcSapUser
{
  public:
    /**
     * Real protocol RC SAP user
     *
     * @param pdcp NrGnbRrcProtocolReal *
     * @param rnti the RNTI
     */
    RealProtocolRlcSapUser(NrGnbRrcProtocolReal* pdcp, uint16_t rnti);

    // Interface implemented from NrRlcSapUser
    void ReceivePdcpPdu(Ptr<Packet> p) override;

  private:
    RealProtocolRlcSapUser();
    NrGnbRrcProtocolReal* m_pdcp; ///< PDCP
    uint16_t m_rnti;              ///< RNTI
};

} // namespace nr
} // namespace ns3

#endif // NR_RRC_PROTOCOL_REAL_GNB_H

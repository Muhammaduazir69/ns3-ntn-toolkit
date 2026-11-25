// Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nicola Baldo <nbaldo@cttc.es>
//          Lluis Parcerisa <lparcerisa@cttc.cat>

#include "nr-rrc-protocol-real-gnb.h"

#include "nr-gnb-net-device.h"
#include "nr-gnb-rrc.h"
#include "nr-rrc-header.h"
#include "nr-ue-net-device.h"
#include "nr-ue-rrc.h"

#include "ns3/fatal-error.h"
#include "ns3/log.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"

namespace ns3
{
namespace nr
{

NS_LOG_COMPONENT_DEFINE("NrRrcProtocolRealGnb");

/// RRC real message delay
const Time RRC_REAL_MSG_DELAY = MilliSeconds(0);

NS_OBJECT_ENSURE_REGISTERED(NrGnbRrcProtocolReal);

NrGnbRrcProtocolReal::NrGnbRrcProtocolReal()
    : m_gnbRrcSapProvider(nullptr)
{
    NS_LOG_FUNCTION(this);
    m_gnbRrcSapUser = new MemberNrGnbRrcSapUser<NrGnbRrcProtocolReal>(this);
}

NrGnbRrcProtocolReal::~NrGnbRrcProtocolReal()
{
    NS_LOG_FUNCTION(this);
}

void
NrGnbRrcProtocolReal::DoDispose()
{
    NS_LOG_FUNCTION(this);
    delete m_gnbRrcSapUser;
    for (auto it = m_completeSetupUeParametersMap.begin();
         it != m_completeSetupUeParametersMap.end();
         ++it)
    {
        delete it->second.srb0SapUser;
        delete it->second.srb1SapUser;
    }
    m_completeSetupUeParametersMap.clear();
}

TypeId
NrGnbRrcProtocolReal::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrGnbRrcProtocolReal")
                            .SetParent<Object>()
                            .SetGroupName("Nr")
                            .AddConstructor<NrGnbRrcProtocolReal>();
    return tid;
}

void
NrGnbRrcProtocolReal::SetNrGnbRrcSapProvider(NrGnbRrcSapProvider* p)
{
    m_gnbRrcSapProvider = p;
}

NrGnbRrcSapUser*
NrGnbRrcProtocolReal::GetNrGnbRrcSapUser()
{
    return m_gnbRrcSapUser;
}

void
NrGnbRrcProtocolReal::SetCellId(uint16_t cellId)
{
    m_cellId = cellId;
}

NrUeRrcSapProvider*
NrGnbRrcProtocolReal::GetUeRrcSapProvider(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    auto it = m_gnbRrcSapProviderMap.find(rnti);
    NS_ASSERT_MSG(it != m_gnbRrcSapProviderMap.end(), "could not find RNTI = " << rnti);
    return it->second;
}

void
NrGnbRrcProtocolReal::SetUeRrcSapProvider(uint16_t rnti, NrUeRrcSapProvider* p)
{
    NS_LOG_FUNCTION(this << rnti);
    auto it = m_gnbRrcSapProviderMap.find(rnti);
    // assign UE RRC only if the RNTI is found at gNB
    if (it != m_gnbRrcSapProviderMap.end())
    {
        it->second = p;
    }
}

void
NrGnbRrcProtocolReal::DoSetupUe(uint16_t rnti, NrGnbRrcSapUser::SetupUeParameters params)
{
    NS_LOG_FUNCTION(this << rnti);

    // // walk list of all nodes to get the peer UE RRC SAP Provider
    // Ptr<NrUeRrc> ueRrc;
    // NodeList::Iterator listEnd = NodeList::End ();
    // bool found = false;
    // for (NodeList::Iterator i = NodeList::Begin (); (i != listEnd) && (found == false); i++)
    //   {
    //     Ptr<Node> node = *i;
    //     int nDevs = node->GetNDevices ();
    //     for (int j = 0; j < nDevs; j++)
    //       {
    //         Ptr<NrUeNetDevice> ueDev = node->GetDevice (j)->GetObject <NrUeNetDevice> ();
    //         if (!ueDev)
    //           {
    //             continue;
    //           }
    //         else
    //           {
    //             ueRrc = ueDev->GetRrc ();
    //             if ((ueRrc->GetRnti () == rnti) && (ueRrc->GetCellId () == m_cellId))
    //               {
    //              found = true;
    //              break;
    //               }
    //           }
    //       }
    //   }
    // NS_ASSERT_MSG (found , " Unable to find UE with RNTI=" << rnti << " cellId=" << m_cellId);
    // m_gnbRrcSapProviderMap[rnti] = ueRrc->GetNrUeRrcSapProvider ();

    // just create empty entry, the UeRrcSapProvider will be set by the
    // ue upon connection request or connection reconfiguration
    // completed
    m_gnbRrcSapProviderMap[rnti] = nullptr;

    // Store SetupUeParameters
    m_setupUeParametersMap[rnti] = params;

    NrGnbRrcSapProvider::CompleteSetupUeParameters completeSetupUeParameters;
    auto csupIt = m_completeSetupUeParametersMap.find(rnti);
    if (csupIt == m_completeSetupUeParametersMap.end())
    {
        // Create NrRlcSapUser, NrPdcpSapUser
        NrRlcSapUser* srb0SapUser = new RealProtocolRlcSapUser(this, rnti);
        NrPdcpSapUser* srb1SapUser = new NrPdcpSpecificNrPdcpSapUser<NrGnbRrcProtocolReal>(this);
        completeSetupUeParameters.srb0SapUser = srb0SapUser;
        completeSetupUeParameters.srb1SapUser = srb1SapUser;
        // Store NrRlcSapUser, NrPdcpSapUser
        m_completeSetupUeParametersMap[rnti] = completeSetupUeParameters;
    }
    else
    {
        completeSetupUeParameters = csupIt->second;
    }
    m_gnbRrcSapProvider->CompleteSetupUe(rnti, completeSetupUeParameters);
}

void
NrGnbRrcProtocolReal::DoRemoveUe(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    auto it = m_completeSetupUeParametersMap.find(rnti);
    NS_ASSERT(it != m_completeSetupUeParametersMap.end());
    delete it->second.srb0SapUser;
    delete it->second.srb1SapUser;
    m_completeSetupUeParametersMap.erase(it);
    m_gnbRrcSapProviderMap.erase(rnti);
    m_setupUeParametersMap.erase(rnti);
}

void
NrGnbRrcProtocolReal::DoSendSystemInformation(uint16_t cellId, NrRrcSap::SystemInformation msg)
{
    NS_LOG_FUNCTION(this << cellId);
    // walk list of all nodes to get UEs with this cellId
    Ptr<NrUeRrc> ueRrc;
    for (auto i = NodeList::Begin(); i != NodeList::End(); ++i)
    {
        Ptr<Node> node = *i;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; ++j)
        {
            Ptr<NrUeNetDevice> ueDev = node->GetDevice(j)->GetObject<NrUeNetDevice>();
            if (ueDev)
            {
                Ptr<NrUeRrc> ueRrc = ueDev->GetRrc();
                NS_LOG_LOGIC("considering UE IMSI " << ueDev->GetImsi() << " that has cellId "
                                                    << ueRrc->GetCellId());
                if (ueRrc->GetCellId() == cellId)
                {
                    NS_LOG_LOGIC("sending SI to IMSI " << ueDev->GetImsi());

                    Simulator::ScheduleWithContext(node->GetId(),
                                                   RRC_REAL_MSG_DELAY,
                                                   &NrUeRrcSapProvider::RecvSystemInformation,
                                                   ueRrc->GetNrUeRrcSapProvider(),
                                                   msg);
                }
            }
        }
    }
}

void
NrGnbRrcProtocolReal::DoSendRrcConnectionSetup(uint16_t rnti, NrRrcSap::RrcConnectionSetup msg)
{
    NS_LOG_FUNCTION(this << rnti);
    Ptr<Packet> packet = Create<Packet>();

    NrRrcConnectionSetupHeader rrcConnectionSetupHeader{};
    rrcConnectionSetupHeader.SetMessage(msg);

    packet->AddHeader(rrcConnectionSetupHeader);

    NrRlcSapProvider::TransmitPdcpPduParameters transmitPdcpPduParameters{};
    transmitPdcpPduParameters.pdcpPdu = packet;
    transmitPdcpPduParameters.rnti = rnti;
    transmitPdcpPduParameters.lcid = 0;

    m_setupUeParametersMap.at(rnti).srb0SapProvider->TransmitPdcpPdu(transmitPdcpPduParameters);
}

void
NrGnbRrcProtocolReal::DoSendRrcConnectionReject(uint16_t rnti, NrRrcSap::RrcConnectionReject msg)
{
    NS_LOG_FUNCTION(this);
    Ptr<Packet> packet = Create<Packet>();

    NrRrcConnectionRejectHeader rrcConnectionRejectHeader;
    rrcConnectionRejectHeader.SetMessage(msg);

    packet->AddHeader(rrcConnectionRejectHeader);

    NrRlcSapProvider::TransmitPdcpPduParameters transmitPdcpPduParameters;
    transmitPdcpPduParameters.pdcpPdu = packet;
    transmitPdcpPduParameters.rnti = rnti;
    transmitPdcpPduParameters.lcid = 0;

    m_setupUeParametersMap[rnti].srb0SapProvider->TransmitPdcpPdu(transmitPdcpPduParameters);
}

void
NrGnbRrcProtocolReal::DoSendRrcConnectionReconfiguration(uint16_t rnti,
                                                         NrRrcSap::RrcConnectionReconfiguration msg)
{
    NS_LOG_FUNCTION(this << rnti);

    Ptr<Packet> packet = Create<Packet>();

    NrRrcConnectionReconfigurationHeader rrcConnectionReconfigurationHeader{};
    rrcConnectionReconfigurationHeader.SetMessage(msg);

    packet->AddHeader(rrcConnectionReconfigurationHeader);

    NrPdcpSapProvider::TransmitPdcpSduParameters transmitPdcpSduParameters{};
    transmitPdcpSduParameters.pdcpSdu = packet;
    transmitPdcpSduParameters.rnti = rnti;
    transmitPdcpSduParameters.lcid = 1;

    m_setupUeParametersMap[rnti].srb1SapProvider->TransmitPdcpSdu(transmitPdcpSduParameters);
}

void
NrGnbRrcProtocolReal::DoSendRrcConnectionReestablishment(uint16_t rnti,
                                                         NrRrcSap::RrcConnectionReestablishment msg)
{
    NS_LOG_FUNCTION(this);
    Ptr<Packet> packet = Create<Packet>();

    NrRrcConnectionReestablishmentHeader rrcConnectionReestablishmentHeader;
    rrcConnectionReestablishmentHeader.SetMessage(msg);

    packet->AddHeader(rrcConnectionReestablishmentHeader);

    NrRlcSapProvider::TransmitPdcpPduParameters transmitPdcpPduParameters;
    transmitPdcpPduParameters.pdcpPdu = packet;
    transmitPdcpPduParameters.rnti = rnti;
    transmitPdcpPduParameters.lcid = 0;

    m_setupUeParametersMap[rnti].srb0SapProvider->TransmitPdcpPdu(transmitPdcpPduParameters);
}

void
NrGnbRrcProtocolReal::DoSendRrcConnectionReestablishmentReject(
    uint16_t rnti,
    NrRrcSap::RrcConnectionReestablishmentReject msg)
{
    NS_LOG_FUNCTION(this);
    Ptr<Packet> packet = Create<Packet>();

    NrRrcConnectionReestablishmentRejectHeader rrcConnectionReestablishmentRejectHeader;
    rrcConnectionReestablishmentRejectHeader.SetMessage(msg);

    packet->AddHeader(rrcConnectionReestablishmentRejectHeader);

    NrRlcSapProvider::TransmitPdcpPduParameters transmitPdcpPduParameters;
    transmitPdcpPduParameters.pdcpPdu = packet;
    transmitPdcpPduParameters.rnti = rnti;
    transmitPdcpPduParameters.lcid = 0;

    m_setupUeParametersMap[rnti].srb0SapProvider->TransmitPdcpPdu(transmitPdcpPduParameters);
}

void
NrGnbRrcProtocolReal::DoSendRrcConnectionRelease(uint16_t rnti, NrRrcSap::RrcConnectionRelease msg)
{
    // The code below is commented so RRC connection release can be sent in an ideal way
    /*
    Ptr<Packet> packet = Create<Packet> ();

    NrRrcConnectionReleaseHeader rrcConnectionReleaseHeader;
    rrcConnectionReleaseHeader.SetMessage (msg);

    packet->AddHeader (rrcConnectionReleaseHeader);

    NrPdcpSapProvider::TransmitPdcpSduParameters transmitPdcpSduParameters;
    transmitPdcpSduParameters.pdcpSdu = packet;
    transmitPdcpSduParameters.rnti = rnti;
    transmitPdcpSduParameters.lcid = 1;

    m_setupUeParametersMap[rnti].srb1SapProvider->TransmitPdcpSdu (transmitPdcpSduParameters);
    */
    /**
     * Send RRC connection release in an idle way to ensure UE goes
     * to idle mode during handover failure and connection setup timeout.
     * Implemented to avoid unnecessary triggering of assert msgs due to reception of
     * msgs (SRS CQI reports) from UE after UE context is deleted at gNB.
     * TODO: Detection of handover failure and connection setup timeout at UE,
     * so that the RRC connection release can be sent through the physical channel again.
     */
    NS_LOG_FUNCTION(this << rnti);
    Simulator::Schedule(RRC_REAL_MSG_DELAY,
                        &NrUeRrcSapProvider::RecvRrcConnectionRelease,
                        GetUeRrcSapProvider(rnti),
                        msg);
}

void
NrGnbRrcProtocolReal::DoReceivePdcpPdu(uint16_t rnti, Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << rnti);
    // Get type of message received
    NrRrcUlCcchMessage rrcUlCcchMessage;
    p->PeekHeader(rrcUlCcchMessage);

    // Declare possible headers to receive
    NrRrcConnectionReestablishmentRequestHeader rrcConnectionReestablishmentRequestHeader;
    NrRrcConnectionRequestHeader rrcConnectionRequestHeader;

    // Deserialize packet and call member recv function with appropriate structure
    switch (rrcUlCcchMessage.GetMessageType())
    {
    case 0:
        p->RemoveHeader(rrcConnectionReestablishmentRequestHeader);
        NrRrcSap::RrcConnectionReestablishmentRequest rrcConnectionReestablishmentRequestMsg;
        rrcConnectionReestablishmentRequestMsg =
            rrcConnectionReestablishmentRequestHeader.GetMessage();
        m_gnbRrcSapProvider->RecvRrcConnectionReestablishmentRequest(
            rnti,
            rrcConnectionReestablishmentRequestMsg);
        break;
    case 1:
        p->RemoveHeader(rrcConnectionRequestHeader);
        NrRrcSap::RrcConnectionRequest rrcConnectionRequestMsg;
        rrcConnectionRequestMsg = rrcConnectionRequestHeader.GetMessage();
        m_gnbRrcSapProvider->RecvRrcConnectionRequest(rnti, rrcConnectionRequestMsg);
        break;
    }
}

void
NrGnbRrcProtocolReal::DoReceivePdcpSdu(NrPdcpSapUser::ReceivePdcpSduParameters params)
{
    NS_LOG_FUNCTION(this << params.rnti << params.lcid);
    // Get type of message received
    NrRrcUlDcchMessage rrcUlDcchMessage{};
    params.pdcpSdu->PeekHeader(rrcUlDcchMessage);

    // Declare possible headers to receive
    NrMeasurementReportHeader measurementReportHeader{};
    NrRrcConnectionReconfigurationCompleteHeader rrcConnectionReconfigurationCompleteHeader{};
    NrRrcConnectionReestablishmentCompleteHeader rrcConnectionReestablishmentCompleteHeader{};
    NrRrcConnectionSetupCompleteHeader rrcConnectionSetupCompleteHeader{};

    // Declare possible messages to receive
    NrRrcSap::MeasurementReport measurementReportMsg{};
    NrRrcSap::RrcConnectionReconfigurationCompleted rrcConnectionReconfigurationCompleteMsg{};
    NrRrcSap::RrcConnectionReestablishmentComplete rrcConnectionReestablishmentCompleteMsg{};
    NrRrcSap::RrcConnectionSetupCompleted rrcConnectionSetupCompletedMsg{};

    // Deserialize packet and call member recv function with appropriate structure
    switch (rrcUlDcchMessage.GetMessageType())
    {
    case 1:
        params.pdcpSdu->RemoveHeader(measurementReportHeader);
        measurementReportMsg = measurementReportHeader.GetMessage();
        m_gnbRrcSapProvider->RecvMeasurementReport(params.rnti, measurementReportMsg);
        break;
    case 2:
        params.pdcpSdu->RemoveHeader(rrcConnectionReconfigurationCompleteHeader);
        rrcConnectionReconfigurationCompleteMsg =
            rrcConnectionReconfigurationCompleteHeader.GetMessage();
        m_gnbRrcSapProvider->RecvRrcConnectionReconfigurationCompleted(
            params.rnti,
            rrcConnectionReconfigurationCompleteMsg);
        break;
    case 3:
        params.pdcpSdu->RemoveHeader(rrcConnectionReestablishmentCompleteHeader);
        rrcConnectionReestablishmentCompleteMsg =
            rrcConnectionReestablishmentCompleteHeader.GetMessage();
        m_gnbRrcSapProvider->RecvRrcConnectionReestablishmentComplete(
            params.rnti,
            rrcConnectionReestablishmentCompleteMsg);
        break;
    case 4:
        params.pdcpSdu->RemoveHeader(rrcConnectionSetupCompleteHeader);
        rrcConnectionSetupCompletedMsg = rrcConnectionSetupCompleteHeader.GetMessage();
        m_gnbRrcSapProvider->RecvRrcConnectionSetupCompleted(params.rnti,
                                                             rrcConnectionSetupCompletedMsg);
        break;
    }
}

Ptr<Packet>
NrGnbRrcProtocolReal::DoEncodeHandoverPreparationInformation(NrRrcSap::HandoverPreparationInfo msg)
{
    NS_LOG_FUNCTION(this);
    NrHandoverPreparationInfoHeader h;
    h.SetMessage(msg);

    Ptr<Packet> p = Create<Packet>();
    p->AddHeader(h);
    return p;
}

NrRrcSap::HandoverPreparationInfo
NrGnbRrcProtocolReal::DoDecodeHandoverPreparationInformation(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this);
    NrHandoverPreparationInfoHeader h;
    p->RemoveHeader(h);
    NrRrcSap::HandoverPreparationInfo msg = h.GetMessage();
    return msg;
}

Ptr<Packet>
NrGnbRrcProtocolReal::DoEncodeHandoverCommand(NrRrcSap::RrcConnectionReconfiguration msg)
{
    NS_LOG_FUNCTION(this);
    NrRrcConnectionReconfigurationHeader h;
    h.SetMessage(msg);
    Ptr<Packet> p = Create<Packet>();
    p->AddHeader(h);
    return p;
}

NrRrcSap::RrcConnectionReconfiguration
NrGnbRrcProtocolReal::DoDecodeHandoverCommand(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this);
    NrRrcConnectionReconfigurationHeader h;
    p->RemoveHeader(h);
    NrRrcSap::RrcConnectionReconfiguration msg = h.GetMessage();
    return msg;
}

//////////////////////////////////////////////////////

RealProtocolRlcSapUser::RealProtocolRlcSapUser(NrGnbRrcProtocolReal* pdcp, uint16_t rnti)
    : m_pdcp(pdcp),
      m_rnti(rnti)
{
}

RealProtocolRlcSapUser::RealProtocolRlcSapUser()
{
}

void
RealProtocolRlcSapUser::ReceivePdcpPdu(Ptr<Packet> p)
{
    m_pdcp->DoReceivePdcpPdu(m_rnti, p);
}

} // namespace nr
} // namespace ns3

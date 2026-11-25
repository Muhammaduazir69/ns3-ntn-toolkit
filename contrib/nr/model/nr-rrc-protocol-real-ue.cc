// Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Authors: Nicola Baldo <nbaldo@cttc.es>
//          Lluis Parcerisa <lparcerisa@cttc.cat>

#include "nr-rrc-protocol-real-ue.h"

#include "nr-gnb-net-device.h"
#include "nr-gnb-rrc.h"
#include "nr-rrc-header.h"
#include "nr-rrc-protocol-real-gnb.h"
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
NS_LOG_COMPONENT_DEFINE("NrRrcProtocolRealUe");

/// RRC real message delay
const Time RRC_REAL_MSG_DELAY = MilliSeconds(0);

NS_OBJECT_ENSURE_REGISTERED(UeRrcProtocolReal);

UeRrcProtocolReal::UeRrcProtocolReal()
    : m_ueRrcSapProvider(nullptr),
      m_gnbRrcSapProvider(nullptr)
{
    m_ueRrcSapUser = new MemberNrUeRrcSapUser<UeRrcProtocolReal>(this);
    m_completeSetupParameters.srb0SapUser = new NrRlcSpecificNrRlcSapUser<UeRrcProtocolReal>(this);
    m_completeSetupParameters.srb1SapUser =
        new NrPdcpSpecificNrPdcpSapUser<UeRrcProtocolReal>(this);
}

UeRrcProtocolReal::~UeRrcProtocolReal()
{
}

void
UeRrcProtocolReal::DoDispose()
{
    NS_LOG_FUNCTION(this);
    delete m_ueRrcSapUser;
    delete m_completeSetupParameters.srb0SapUser;
    delete m_completeSetupParameters.srb1SapUser;
    m_rrc = nullptr;
}

TypeId
UeRrcProtocolReal::GetTypeId()
{
    static TypeId tid = TypeId("ns3::UeRrcProtocolReal")
                            .SetParent<Object>()
                            .SetGroupName("Nr")
                            .AddConstructor<UeRrcProtocolReal>();
    return tid;
}

void
UeRrcProtocolReal::SetNrUeRrcSapProvider(NrUeRrcSapProvider* p)
{
    m_ueRrcSapProvider = p;
}

NrUeRrcSapUser*
UeRrcProtocolReal::GetNrUeRrcSapUser()
{
    return m_ueRrcSapUser;
}

void
UeRrcProtocolReal::SetUeRrc(Ptr<NrUeRrc> rrc)
{
    m_rrc = rrc;
}

void
UeRrcProtocolReal::DoSetup(NrUeRrcSapUser::SetupParameters params)
{
    NS_LOG_FUNCTION(this);

    m_setupParameters.srb0SapProvider = params.srb0SapProvider;
    m_setupParameters.srb1SapProvider = params.srb1SapProvider;
    m_ueRrcSapProvider->CompleteSetup(m_completeSetupParameters);
}

void
UeRrcProtocolReal::DoSendRrcConnectionRequest(NrRrcSap::RrcConnectionRequest msg)
{
    NS_LOG_FUNCTION(this << m_rnti);
    // initialize the RNTI and get the GnbNrRrcSapProvider for the
    // gNB we are currently attached to
    m_rnti = m_rrc->GetRnti();
    SetGnbRrcSapProvider();

    Ptr<Packet> packet = Create<Packet>();

    NrRrcConnectionRequestHeader rrcConnectionRequestHeader{};
    rrcConnectionRequestHeader.SetMessage(msg);

    packet->AddHeader(rrcConnectionRequestHeader);

    NrRlcSapProvider::TransmitPdcpPduParameters transmitPdcpPduParameters{};
    transmitPdcpPduParameters.pdcpPdu = packet;
    transmitPdcpPduParameters.rnti = m_rnti;
    transmitPdcpPduParameters.lcid = 0;

    m_setupParameters.srb0SapProvider->TransmitPdcpPdu(transmitPdcpPduParameters);
}

void
UeRrcProtocolReal::DoSendRrcConnectionSetupCompleted(
    NrRrcSap::RrcConnectionSetupCompleted msg) const
{
    NS_LOG_FUNCTION(this << m_rnti);
    Ptr<Packet> packet = Create<Packet>();

    NrRrcConnectionSetupCompleteHeader rrcConnectionSetupCompleteHeader;
    rrcConnectionSetupCompleteHeader.SetMessage(msg);

    packet->AddHeader(rrcConnectionSetupCompleteHeader);

    NrPdcpSapProvider::TransmitPdcpSduParameters transmitPdcpSduParameters;
    transmitPdcpSduParameters.pdcpSdu = packet;
    transmitPdcpSduParameters.rnti = m_rnti;
    transmitPdcpSduParameters.lcid = 1;

    if (m_setupParameters.srb1SapProvider)
    {
        m_setupParameters.srb1SapProvider->TransmitPdcpSdu(transmitPdcpSduParameters);
    }
}

void
UeRrcProtocolReal::DoSendRrcConnectionReconfigurationCompleted(
    NrRrcSap::RrcConnectionReconfigurationCompleted msg)
{
    NS_LOG_FUNCTION(this);
    // re-initialize the RNTI and get the GnbNrRrcSapProvider for the
    // gNB we are currently attached to
    m_rnti = m_rrc->GetRnti();
    SetGnbRrcSapProvider();

    Ptr<Packet> packet = Create<Packet>();

    NrRrcConnectionReconfigurationCompleteHeader rrcConnectionReconfigurationCompleteHeader{};
    rrcConnectionReconfigurationCompleteHeader.SetMessage(msg);

    packet->AddHeader(rrcConnectionReconfigurationCompleteHeader);

    NrPdcpSapProvider::TransmitPdcpSduParameters transmitPdcpSduParameters;
    transmitPdcpSduParameters.pdcpSdu = packet;
    transmitPdcpSduParameters.rnti = m_rnti;
    transmitPdcpSduParameters.lcid = 1;

    m_setupParameters.srb1SapProvider->TransmitPdcpSdu(transmitPdcpSduParameters);
}

void
UeRrcProtocolReal::DoSendMeasurementReport(NrRrcSap::MeasurementReport msg)
{
    NS_LOG_FUNCTION(this);
    // re-initialize the RNTI and get the GnbNrRrcSapProvider for the
    // gNB we are currently attached to
    m_rnti = m_rrc->GetRnti();
    SetGnbRrcSapProvider();

    Ptr<Packet> packet = Create<Packet>();

    NrMeasurementReportHeader measurementReportHeader;
    measurementReportHeader.SetMessage(msg);

    packet->AddHeader(measurementReportHeader);

    NrPdcpSapProvider::TransmitPdcpSduParameters transmitPdcpSduParameters;
    transmitPdcpSduParameters.pdcpSdu = packet;
    transmitPdcpSduParameters.rnti = m_rnti;
    transmitPdcpSduParameters.lcid = 1;

    m_setupParameters.srb1SapProvider->TransmitPdcpSdu(transmitPdcpSduParameters);
}

void
UeRrcProtocolReal::DoSendIdealUeContextRemoveRequest(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);
    uint16_t cellId = m_rrc->GetCellId();
    // re-initialize the RNTI and get the GnbNrRrcSapProvider for the
    // gNB we are currently attached to or attempting random access to
    // a target gNB
    m_rnti = m_rrc->GetRnti();

    NS_LOG_DEBUG("RNTI " << rnti << " sending UE context remove request to cell id " << cellId);
    NS_ABORT_MSG_IF(m_rnti != rnti, "RNTI mismatch");

    SetGnbRrcSapProvider(); // the provider has to be reset since the cell might
                            //  have changed due to handover
    // ideally informing gNB
    Simulator::Schedule(RRC_REAL_MSG_DELAY,
                        &NrGnbRrcSapProvider::RecvIdealUeContextRemoveRequest,
                        m_gnbRrcSapProvider,
                        rnti);
}

void
UeRrcProtocolReal::DoSendIdealBwpSwitchIndication(uint16_t rnti, uint8_t bwpId)
{
    NS_LOG_FUNCTION(this << rnti << +bwpId);
    m_rnti = m_rrc->GetRnti();
    NS_ABORT_MSG_IF(m_rnti != rnti, "RNTI mismatch");

    SetGnbRrcSapProvider();
    Simulator::Schedule(RRC_REAL_MSG_DELAY,
                        &NrGnbRrcSapProvider::RecvIdealBwpSwitchIndication,
                        m_gnbRrcSapProvider,
                        rnti,
                        bwpId);
}

void
UeRrcProtocolReal::DoSendRrcConnectionReestablishmentRequest(
    NrRrcSap::RrcConnectionReestablishmentRequest msg) const
{
    NS_LOG_FUNCTION(this << m_rnti);
    Ptr<Packet> packet = Create<Packet>();

    NrRrcConnectionReestablishmentRequestHeader rrcConnectionReestablishmentRequestHeader;
    rrcConnectionReestablishmentRequestHeader.SetMessage(msg);

    packet->AddHeader(rrcConnectionReestablishmentRequestHeader);

    NrRlcSapProvider::TransmitPdcpPduParameters transmitPdcpPduParameters;
    transmitPdcpPduParameters.pdcpPdu = packet;
    transmitPdcpPduParameters.rnti = m_rnti;
    transmitPdcpPduParameters.lcid = 0;

    m_setupParameters.srb0SapProvider->TransmitPdcpPdu(transmitPdcpPduParameters);
}

void
UeRrcProtocolReal::DoSendRrcConnectionReestablishmentComplete(
    NrRrcSap::RrcConnectionReestablishmentComplete msg) const
{
    NS_LOG_FUNCTION(this << m_rnti);
    Ptr<Packet> packet = Create<Packet>();

    NrRrcConnectionReestablishmentCompleteHeader rrcConnectionReestablishmentCompleteHeader;
    rrcConnectionReestablishmentCompleteHeader.SetMessage(msg);

    packet->AddHeader(rrcConnectionReestablishmentCompleteHeader);

    NrPdcpSapProvider::TransmitPdcpSduParameters transmitPdcpSduParameters;
    transmitPdcpSduParameters.pdcpSdu = packet;
    transmitPdcpSduParameters.rnti = m_rnti;
    transmitPdcpSduParameters.lcid = 1;

    m_setupParameters.srb1SapProvider->TransmitPdcpSdu(transmitPdcpSduParameters);
}

void
UeRrcProtocolReal::SetGnbRrcSapProvider()
{
    NS_LOG_FUNCTION(this);

    uint16_t cellId = m_rrc->GetCellId();

    NS_LOG_DEBUG("RNTI " << m_rnti << " connected to cell " << cellId);

    if (m_knownGnb.find(cellId) == m_knownGnb.end())
    {
        // walk list of all nodes to get the peer gNB
        Ptr<NrGnbNetDevice> gnbDev;
        auto listEnd = NodeList::End();
        for (auto i = NodeList::Begin(); i != listEnd; ++i)
        {
            Ptr<Node> node = *i;
            int nDevs = node->GetNDevices();
            for (int j = 0; j < nDevs; j++)
            {
                gnbDev = node->GetDevice(j)->GetObject<NrGnbNetDevice>();
                if (!gnbDev)
                {
                    continue;
                }
                // Populate a table to avoid repeating this
                m_knownGnb[gnbDev->GetCellId()] = gnbDev;
            }
        }
        NS_ABORT_MSG_IF(m_knownGnb.find(cellId) == m_knownGnb.end(),
                        " Unable to find gNB with CellId =" << cellId);
    }
    m_gnbRrcSapProvider = m_knownGnb.at(cellId)->GetRrc()->GetNrGnbRrcSapProvider();
    Ptr<NrGnbRrcProtocolReal> gnbRrcProtocolReal =
        m_knownGnb.at(cellId)->GetRrc()->GetObject<NrGnbRrcProtocolReal>();
    gnbRrcProtocolReal->SetUeRrcSapProvider(m_rnti, m_ueRrcSapProvider);
}

void
UeRrcProtocolReal::DoReceivePdcpPdu(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this << m_rnti);
    // Get type of message received
    NrRrcDlCcchMessage rrcDlCcchMessage;
    p->PeekHeader(rrcDlCcchMessage);

    // Declare possible headers to receive
    NrRrcConnectionReestablishmentHeader rrcConnectionReestablishmentHeader;
    NrRrcConnectionReestablishmentRejectHeader rrcConnectionReestablishmentRejectHeader;
    NrRrcConnectionSetupHeader rrcConnectionSetupHeader;
    NrRrcConnectionRejectHeader rrcConnectionRejectHeader;

    // Declare possible messages
    NrRrcSap::RrcConnectionReestablishment rrcConnectionReestablishmentMsg;
    NrRrcSap::RrcConnectionReestablishmentReject rrcConnectionReestablishmentRejectMsg;
    NrRrcSap::RrcConnectionSetup rrcConnectionSetupMsg;
    NrRrcSap::RrcConnectionReject rrcConnectionRejectMsg;

    // Deserialize packet and call member recv function with appropriate structure
    switch (rrcDlCcchMessage.GetMessageType())
    {
    case 0:
        // RrcConnectionReestablishment
        p->RemoveHeader(rrcConnectionReestablishmentHeader);
        rrcConnectionReestablishmentMsg = rrcConnectionReestablishmentHeader.GetMessage();
        m_ueRrcSapProvider->RecvRrcConnectionReestablishment(rrcConnectionReestablishmentMsg);
        break;
    case 1:
        // RrcConnectionReestablishmentReject
        p->RemoveHeader(rrcConnectionReestablishmentRejectHeader);
        rrcConnectionReestablishmentRejectMsg =
            rrcConnectionReestablishmentRejectHeader.GetMessage();
        // m_ueRrcSapProvider->RecvRrcConnectionReestablishmentReject
        // (rrcConnectionReestablishmentRejectMsg);
        break;
    case 2:
        // RrcConnectionReject
        p->RemoveHeader(rrcConnectionRejectHeader);
        rrcConnectionRejectMsg = rrcConnectionRejectHeader.GetMessage();
        m_ueRrcSapProvider->RecvRrcConnectionReject(rrcConnectionRejectMsg);
        break;
    case 3:
        // RrcConnectionSetup
        p->RemoveHeader(rrcConnectionSetupHeader);
        rrcConnectionSetupMsg = rrcConnectionSetupHeader.GetMessage();
        m_ueRrcSapProvider->RecvRrcConnectionSetup(rrcConnectionSetupMsg);
        break;
    }
}

void
UeRrcProtocolReal::DoReceivePdcpSdu(NrPdcpSapUser::ReceivePdcpSduParameters params)
{
    NS_LOG_FUNCTION(this << m_rnti << params.lcid);
    // Get type of message received
    NrRrcDlDcchMessage rrcDlDcchMessage;
    params.pdcpSdu->PeekHeader(rrcDlDcchMessage);

    // Declare possible headers to receive
    NrRrcConnectionReconfigurationHeader rrcConnectionReconfigurationHeader;
    NrRrcConnectionReleaseHeader rrcConnectionReleaseHeader;

    // Declare possible messages to receive
    NrRrcSap::RrcConnectionReconfiguration rrcConnectionReconfigurationMsg;
    NrRrcSap::RrcConnectionRelease rrcConnectionReleaseMsg;

    // Deserialize packet and call member recv function with appropriate structure
    switch (rrcDlDcchMessage.GetMessageType())
    {
    case 4:
        params.pdcpSdu->RemoveHeader(rrcConnectionReconfigurationHeader);
        rrcConnectionReconfigurationMsg = rrcConnectionReconfigurationHeader.GetMessage();
        m_ueRrcSapProvider->RecvRrcConnectionReconfiguration(rrcConnectionReconfigurationMsg);
        break;
    case 5:
        params.pdcpSdu->RemoveHeader(rrcConnectionReleaseHeader);
        rrcConnectionReleaseMsg = rrcConnectionReleaseHeader.GetMessage();
        // m_ueRrcSapProvider->RecvRrcConnectionRelease (rrcConnectionReleaseMsg);
        break;
    }
}
} // namespace nr
} // namespace ns3

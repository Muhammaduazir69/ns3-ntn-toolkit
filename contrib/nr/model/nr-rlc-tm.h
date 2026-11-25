// Copyright (c) 2011,2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>
//         Nicola Baldo <nbaldo@cttc.es>

#ifndef NR_RLC_TM_H
#define NR_RLC_TM_H

#include "nr-rlc.h"

#include "ns3/event-id.h"
#include "ns3/nr-export.h"

#include <map>

namespace ns3
{

/**
 * LTE RLC Transparent Mode (TM), see 3GPP TS 36.322
 *
 * Please note that, as in TM it is not possible to add any header, the delay
 * measurements gathered from the trace source "RxPDU" of NrRlc are invalid
 * (they will be always 0)
 */
class NR_EXPORT NrRlcTm : public NrRlc
{
  public:
    NrRlcTm();
    ~NrRlcTm() override;
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();
    void DoDispose() override;

    /**
     * RLC SAP
     *
     * @param p packet
     */
    void DoTransmitPdcpPdu(Ptr<Packet> p) override;

    /**
     * MAC SAP
     *
     * @param txOpParams the NrMacSapUser::TxOpportunityParameters
     */
    void DoNotifyTxOpportunity(NrMacSapUser::TxOpportunityParameters txOpParams) override;
    /**
     * Notify HARQ deliver failure
     */
    void DoNotifyHarqDeliveryFailure() override;
    void DoReceivePdu(NrMacSapUser::ReceivePduParameters rxPduParams) override;

  private:
    /// Expire BSR timer function
    void ExpireBsrTimer();
    /**
     * @brief Restart the buffer status report timer, or stop it if there is nothing left to send.
     *
     * Called whenever the buffer status is reported to the MAC. While data is waiting the timer
     * keeps running, so its expiry triggers a new report (the equivalent of retxBSR-Timer of 3GPP
     * TS 38.321, clause 5.4.5, whose expiry triggers a regular buffer status report); once the
     * buffer is empty there is nothing to report and the timer is left stopped.
     */
    void RestartBsrTimer();
    /// Buffer status report
    void DoTransmitBufferStatusReport();

  private:
    /// Grant the transparent-mode test access to private transmit buffer state
    friend class NrRlcTmTestCase;

    /**
     * @brief Store an incoming (from layer above us) PDU, waiting to transmit it
     */
    struct NR_EXPORT TxPdu
    {
        /**
         * @brief TxPdu default constructor
         * @param pdu the PDU
         * @param time the arrival time
         */
        TxPdu(const Ptr<Packet>& pdu, const Time& time)
            : m_pdu(pdu),
              m_waitingSince(time)
        {
        }

        TxPdu() = delete;

        Ptr<Packet> m_pdu;   ///< PDU
        Time m_waitingSince; ///< Layer arrival time
    };

    std::vector<TxPdu> m_txBuffer; ///< Transmission buffer

    uint32_t m_maxTxBufferSize; ///< maximum transmit buffer size
    uint32_t m_txBufferSize;    ///< transmit buffer size

    EventId m_bsrTimer;   ///< BSR timer
    Time m_bsrTimerValue; ///< BSR timer value
};

} // namespace ns3

#endif // NR_RLC_TM_H

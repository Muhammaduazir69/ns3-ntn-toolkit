// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
// Copyright (c) 2018 Fraunhofer ESK : RLF extensions
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Nicola Baldo <nbaldo@cttc.es>
//         Budiarto Herman <budiarto.herman@magister.fi>
// Modified by:
//          Danilo Abrignani <danilo.abrignani@unibo.it> (Carrier Aggregation - GSoC 2015)
//          Biljana Bojovic <biljana.bojovic@cttc.es> (Carrier Aggregation)
//          Vignesh Babu <ns3-dev@esk.fraunhofer.de> (RLF extensions)

#ifndef NR_UE_RRC_H
#define NR_UE_RRC_H

#include "nr-as-sap.h"
#include "nr-pdcp-sap.h"
#include "nr-rrc-sap.h"
#include "nr-ue-ccm-rrc-sap.h"
#include "nr-ue-cmac-sap.h"
#include "nr-ue-cphy-sap.h"

#include "ns3/nr-export.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/traced-callback.h"

#include <map>
#include <set>
#include <vector>

namespace ns3
{

/**
 * @brief Artificial delay of UE measurements procedure.
 *
 * i.e. the period between the time layer-1-filtered measurements from PHY
 * layer is received and the earliest time the actual measurement report
 * submission to the serving cell is invoked.
 *
 * This delay exists because of racing condition between several UE measurements
 * functions which happen to be scheduled at the same time. The delay ensures
 * that:
 *  - measurements (e.g., layer-3 filtering) are always performed before
 *    reporting, thus the latter always use the latest measured RSRP and RSRQ;
 *    and
 *  - time-to-trigger check is always performed before the reporting, so there
 *    would still be chance for it to cancel the reporting if necessary.
 */
extern NR_EXPORT const Time NR_UE_MEASUREMENT_REPORT_DELAY;

class NrRlc;
class NrMacSapProvider;
class NrUeCmacSapUser;
class NrUeCmacSapProvider;
class NrDataRadioBearerInfo;
class NrSignalingRadioBearerInfo;

/**
 *
 *
 */
class NR_EXPORT NrUeRrc : public Object
{
    /// allow UeMemberNrUeCmacSapUser class friend access
    friend class UeMemberNrUeCmacSapUser;
    /// allow UeRrcMemberNrGnbCmacSapUser class friend access
    friend class UeRrcMemberNrGnbCmacSapUser;
    /// allow NrPdcpSpecificNrPdcpSapUser<NrUeRrc> class friend access
    friend class NrPdcpSpecificNrPdcpSapUser<NrUeRrc>;
    /// allow MemberNrAsSapProvider<NrUeRrc> class friend access
    friend class MemberNrAsSapProvider<NrUeRrc>;
    /// allow MemberNrUeCphySapUser<NrUeRrc> class friend access
    friend class MemberNrUeCphySapUser<NrUeRrc>;
    /// allow MemberNrUeRrcSapProvider<NrUeRrc> class friend access
    friend class MemberNrUeRrcSapProvider<NrUeRrc>;
    /// allow MemberNrUeCcmRrcSapUser<NrUeRrc> class friend access
    friend class MemberNrUeCcmRrcSapUser<NrUeRrc>;

  public:
    /**
     * The states of the UE RRC entity
     *
     */
    enum State
    {
        IDLE_START = 0,
        IDLE_CELL_SEARCH,
        IDLE_WAIT_MIB_SIB1,
        IDLE_WAIT_MIB,
        IDLE_WAIT_SIB1,
        IDLE_CAMPED_NORMALLY,
        IDLE_WAIT_SIB2,
        IDLE_RANDOM_ACCESS,
        IDLE_CONNECTING,
        CONNECTED_NORMALLY,
        CONNECTED_HANDOVER,
        CONNECTED_PHY_PROBLEM,
        CONNECTED_REESTABLISHING,
        NUM_STATES
    };

    /**
     * The cause of a transition into the CONNECTED_PHY_PROBLEM state. Recorded at
     * the instant of failure so downstream analysis does not have to infer the
     * cause from the state-machine timeline (which conflates, e.g., a network
     * connection release with a genuine radio link failure). Mirrors the 3GPP
     * TR 36.839 mobility-failure taxonomy where applicable.
     */
    enum RadioLinkFailureCause
    {
        RLF_NONE = 0,        ///< not in a failure (default)
        RLF_T310_EXPIRY,     ///< T310 ran to completion: DL stayed below Qout (genuine RLF)
        RLF_HO_COMMAND_LATE, ///< handover command arrived while T310 active (too-late handover)
        RLF_DURING_HANDOVER, ///< failure while in CONNECTED_HANDOVER (handover execution failure)
        RLF_CONNECTION_RELEASE, ///< network-initiated RRC connection release (not a radio failure)
        RLF_CONNECTION_TIMEOUT, ///< RRC connection (re)establishment/setup attempts exhausted
        RLF_RLC_MAX_RETX ///< RLC-AM reached maxRetxThreshold (TS 38.331 5.3.10.3 RLF trigger)
    };

    /**
     * create an RRC instance for use within an ue
     *
     */
    NrUeRrc();

    /**
     * Destructor
     */
    ~NrUeRrc() override;

    // inherited from Object
  private:
    void DoDispose() override;

  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    /// Initialize SAP
    void InitializeSap();

    /**
     * @brief Initialize the UE side of SRB0
     */
    void InitializeSrb0();

    /**
     * set the CPHY SAP this RRC should use to interact with the PHY
     *
     * @param s the CPHY SAP Provider
     */
    void SetNrUeCphySapProvider(NrUeCphySapProvider* s);
    /**
     * set the CPHY SAP this RRC should use to interact with the PHY
     *
     * @param s the CPHY SAP Provider
     * @param index the index
     */
    void SetNrUeCphySapProvider(NrUeCphySapProvider* s, uint8_t index);

    /**
     *
     *
     * @return s the CPHY SAP User interface offered to the PHY by this RRC
     */
    NrUeCphySapUser* GetNrUeCphySapUser();
    /**
     *
     * @param index the index
     * @return s the CPHY SAP User interface offered to the PHY by this RRC
     */
    NrUeCphySapUser* GetNrUeCphySapUser(uint8_t index);

    /**
     * set the CMAC SAP this RRC should interact with
     * @brief This function is overloaded to maintain backward compatibility
     * @param s the CMAC SAP Provider to be used by this RRC
     */
    void SetNrUeCmacSapProvider(NrUeCmacSapProvider* s);
    /**
     * set the CMAC SAP this RRC should interact with
     * @brief This function is overloaded to maintain backward compatibility
     * @param s the CMAC SAP Provider to be used by this RRC
     * @param index the index
     */
    void SetNrUeCmacSapProvider(NrUeCmacSapProvider* s, uint8_t index);

    /**
     * @brief This function is overloaded to maintain backward compatibility
     * @return s the CMAC SAP User interface offered to the MAC by this RRC
     */
    NrUeCmacSapUser* GetNrUeCmacSapUser();
    /**
     * @brief This function is overloaded to maintain backward compatibility
     * @param index the index
     * @return s the CMAC SAP User interface offered to the MAC by this RRC
     */
    NrUeCmacSapUser* GetNrUeCmacSapUser(uint8_t index);

    /**
     * set the RRC SAP this RRC should interact with
     *
     * @param s the RRC SAP User to be used by this RRC
     */
    void SetNrUeRrcSapUser(NrUeRrcSapUser* s);

    /**
     *
     *
     * @return s the RRC SAP Provider interface offered to the MAC by this RRC
     */
    NrUeRrcSapProvider* GetNrUeRrcSapProvider();

    /**
     * Get the RRC SAP user interface used by this RRC.
     *
     * @return s the RRC SAP User interface
     */
    NrUeRrcSapUser* GetNrUeRrcSapUser();

    /**
     * set the MAC SAP provider. The ue RRC does not use this
     * directly, but it needs to provide it to newly created RLC instances.
     *
     * @param s the MAC SAP provider that will be used by all
     * newly created RLC instances
     */
    void SetNrMacSapProvider(NrMacSapProvider* s);

    /**
     * Set the AS SAP user to interact with the NAS entity
     *
     * @param s the AS SAP user
     */
    void SetAsSapUser(NrAsSapUser* s);

    /**
     *
     *
     * @return the AS SAP provider exported by this RRC
     */
    NrAsSapProvider* GetAsSapProvider();

    /**
     * set the Component Carrier Management SAP this RRC should interact with
     *
     * @param s the Component Carrier Management SAP Provider to be used by this RRC
     */
    void SetNrCcmRrcSapProvider(NrUeCcmRrcSapProvider* s);

    /**
     * Get the Component Carrier Management SAP offered by this RRC
     * @return s the Component Carrier Management SAP User interface offered to the
     *           carrier component selection algorithm by this RRC
     */
    NrUeCcmRrcSapUser* GetNrCcmRrcSapUser();

    /**
     *
     * @param imsi the unique UE identifier
     */
    void SetImsi(uint64_t imsi);

    /**
     * @brief Store the previous cell id
     *
     * @param cellId The cell id of the previous cell the UE was attached to
     */
    void StorePreviousCellId(uint16_t cellId);

    /**
     *
     * @return imsi the unique UE identifier
     */
    uint64_t GetImsi() const;

    /**
     *
     * @return the C-RNTI of the user
     */
    uint16_t GetRnti() const;

    /**
     *
     * @return the CellId of the attached Gnb
     */
    uint16_t GetCellId() const;

    /**
     * @param cellId cell identifier
     * @return true if cellId is the serving cell for this UE
     */
    bool IsServingCell(uint16_t cellId) const;

    /**
     * @return the uplink bandwidth in RBs
     */
    uint8_t GetUlBandwidth() const;

    /**
     * @return the downlink bandwidth in RBs
     */
    uint8_t GetDlBandwidth() const;

    /**
     * @return the downlink carrier frequency (ARFCN)
     */
    uint32_t GetArfcn() const;

    /**
     *
     * @return the current state
     */
    State GetState() const;

    /**
     * @brief Get the previous cell id
     *
     * @return The cell Id of the previous cell the UE was attached to.
     */
    uint16_t GetPreviousCellId() const;

    /**
     *
     *
     * @param val true if RLC SM is to be used, false if RLC UM/AM are to be used
     */
    void SetUseRlcSm(bool val);

    /**
     * @brief Sets the index of the UE PHY/MAC that will be used as the primary UL PHY/MAC
     * @param primaryIndex Sets the primaryIndex
     */
    void SetPrimaryUlIndex(uint16_t primaryIndex);

    /**
     * @brief Returns the primary index
     */
    uint16_t GetPrimaryUlIndex() const;

    /**
     * @brief Sets the index of the UE PHY/MAC that will be used as the primary UL PHY/MAC
     * @param primaryIndex Sets the primaryIndex
     */
    void SetPrimaryDlIndex(uint16_t primaryIndex);

    /**
     * @brief Returns the primary index
     */
    uint16_t GetPrimaryDlIndex() const;

    /**
     * @brief Install the callback used to update the BWP manager routing when
     * the primary DL/UL BWP pair changes (e.g. on handover to a cell whose
     * UL/DL carriers are paired differently)
     * @param fn callback taking (source BWP, output BWP), same semantics as
     *           BwpManagerUe::SetOutputLink
     */
    void SetUpdateBwpOutputLinkFn(std::function<void(uint32_t, uint32_t)> fn);

    /**
     * @brief Install the callback used to program the BWP manager's QoS flow to
     * BWP mapping from the one the serving cell advertises over RRC
     * @param fn callback taking (5QI, BWP index)
     */
    void SetUpdateQosFlowBwpFn(std::function<void(uint8_t, uint8_t)> fn)
    {
        m_updateQosFlowBwpFn = std::move(fn);
    }

    /**
     * @brief Install the callback used to drop all BWP manager output links
     * when the UE moves to a different cell, whose UL/DL carrier pairing may
     * differ from the previous one's
     * @param fn callback taking no arguments
     */
    void SetClearBwpOutputLinksFn(std::function<void()> fn)
    {
        m_clearBwpOutputLinksFn = std::move(fn);
    }

    /**
     * @brief Set the cell-individual offset (Ocn/Ocp, in dB) applied to the given
     * cell during event A3 evaluation, per 3GPP TS 36.331 Section 5.5.4.4.
     *
     * A positive value biases handover towards that cell: it is added to the
     * cell's measured level both when the cell is the A3 neighbour (Ocn) and when
     * it is the serving PCell (Ocp). This implements cell-range expansion (CRE),
     * e.g. applying a positive bias to pico cells so UEs offload to them earlier
     * and remain attached longer.
     *
     * @param cellId the cell the offset applies to
     * @param offsetDb the offset in dB (0 removes any previously set offset)
     */
    void SetCellIndividualOffset(uint16_t cellId, double offsetDb);

    /**
     * @brief Get the cell-individual offset (in dB) configured for a cell.
     * @param cellId the cell to query
     * @return the configured offset in dB, or 0.0 if none was set
     */
    double GetCellIndividualOffset(uint16_t cellId) const;

    /**
     * Configures the UE physical layer based on the SIB1 information corresponding to a given cell
     * ID and BWP ID.
     *
     * This function retrieves the necessary configuration parameters from a gNB associated with the
     * specified cell ID and bandwidth part (BWP) ID. These parameters include DL/UL control
     * symbols, numerology, resource block overhead, and other physical layer settings. The
     * retrieved values are then set in the provided UE PHY SAP provider.
     *
     * This method assumes that the mapped gNB, its PHY, MAC, and scheduler for the specified BWP
     * ID, exist and are valid.
     *
     */
    void ReconfigureFromSib1(const uint8_t bwpId,
                             const uint16_t cellId,
                             const uint8_t dlCtrlSym,
                             const uint8_t ulCtrlSym,
                             const uint32_t symPerSlot,
                             const uint16_t numerology,
                             const std::string& tddPattern,
                             const uint8_t numRbsPerRbg);

    /**
     * TracedCallback signature for imsi, cellId and rnti events.
     *
     * @param [in] imsi
     * @param [in] cellId
     */
    typedef void (*CellSelectionTracedCallback)(uint64_t imsi, uint16_t cellId);

    /**
     * TracedCallback signature for imsi, cellId and rnti events.
     *
     * @param [in] imsi
     * @param [in] cellId
     * @param [in] rnti
     */
    typedef void (*ImsiCidRntiTracedCallback)(uint64_t imsi, uint16_t cellId, uint16_t rnti);

    /**
     * TracedCallback signature for MIBReceived, Sib1Received and
     * HandoverStart events.
     *
     * @param [in] imsi
     * @param [in] cellId
     * @param [in] rnti
     * @param [in] otherCid
     */
    typedef void (*MibSibHandoverTracedCallback)(uint64_t imsi,
                                                 uint16_t cellId,
                                                 uint16_t rnti,
                                                 uint16_t otherCid);

    /**
     * TracedCallback signature for state transition events.
     *
     * @param [in] imsi
     * @param [in] cellId
     * @param [in] rnti
     * @param [in] oldState
     * @param [in] newState
     */
    typedef void (*StateTracedCallback)(uint64_t imsi,
                                        uint16_t cellId,
                                        uint16_t rnti,
                                        State oldState,
                                        State newState);

    /**
     * TracedCallback signature for secondary carrier configuration events.
     *
     * @param [in] Pointer to UE RRC
     * @param [in] List of NrRrcSap::SCellToAddMod
     */
    typedef void (*SCarrierConfiguredTracedCallback)(Ptr<NrUeRrc>,
                                                     std::list<NrRrcSap::SCellToAddMod>);

    /**
     * TracedCallback signature for in-sync and out-of-sync detection events.
     *
     *
     * @param [in] imsi
     * @param [in] rnti
     * @param [in] cellId
     * @param [in] type
     * @param [in] count
     */
    typedef void (*PhySyncDetectionTracedCallback)(uint64_t imsi,
                                                   uint16_t rnti,
                                                   uint16_t cellId,
                                                   std::string type,
                                                   uint16_t count);

    /**
     * TracedCallback signature for imsi, cellId, rnti and counter for
     * random access events.
     *
     * @param [in] imsi
     * @param [in] cellId
     * @param [in] rnti
     * @param [in] count
     */
    typedef void (*ImsiCidRntiCountTracedCallback)(uint64_t imsi,
                                                   uint16_t cellId,
                                                   uint16_t rnti,
                                                   uint8_t count);

    /**
     * TracedCallback signature for radio-link-failure events with cause and the
     * timing context known at the instant of failure.
     *
     * @param [in] imsi UE IMSI
     * @param [in] cellId serving cell at failure
     * @param [in] rnti UE RNTI
     * @param [in] oldState RRC state immediately before the failure (NrUeRrc::State value)
     * @param [in] cause failure cause token (see ns3::ToString(NrUeRrc::RadioLinkFailureCause))
     * @param [in] t310ElapsedMs time the DL had been below Qout (T310 elapsed), or -1 if T310 was
     *             not running
     * @param [in] msSinceLastHoSuccess time since this UE's last successful handover, or -1 if it
     *             has not handed over yet
     */
    typedef void (*RlfCauseTracedCallback)(uint64_t imsi,
                                           uint16_t cellId,
                                           uint16_t rnti,
                                           uint16_t oldState,
                                           std::string cause,
                                           int64_t t310ElapsedMs,
                                           int64_t msSinceLastHoSuccess);

  private:
    // PDCP SAP methods
    /**
     * Receive PDCP SDU function
     *
     * @param params NrPdcpSapUser::ReceivePdcpSduParameters
     */
    void DoReceivePdcpSdu(NrPdcpSapUser::ReceivePdcpSduParameters params);

    // CMAC SAP methods
    /**
     * Set temporary cell rnti function
     *
     * @param rnti RNTI
     */
    void DoSetTemporaryCellRnti(uint16_t rnti);
    /// Notify random access successful function
    void DoNotifyRandomAccessSuccessful();
    /// Notify random access failed function
    void DoNotifyRandomAccessFailed();

    // NR AS SAP methods
    /**
     * Set CSG white list function
     *
     * @param csgId CSG ID
     */
    void DoSetCsgWhiteList(uint32_t csgId);
    /**
     * Force camped on gNB function
     *
     * @param cellId the cell ID
     * @param arfcn the DL ARFCN
     */
    void DoForceCampedOnGnb(uint16_t cellId, uint32_t arfcn);
    /**
     * Start cell selection function
     *
     * @param arfcn the DL ARFCN
     */
    void DoStartCellSelection(uint32_t arfcn);

    /**
     * Function to start cell selection in all configured BWPs
     */
    void DoStartCellSelection();

    /// Connect function
    void DoConnect();
    /**
     * Send data function
     *
     * @param packet the packet
     * @param qfi the QoS flow ID
     */
    void DoSendData(Ptr<Packet> packet, uint8_t qfi);
    /// Disconnect function
    void DoDisconnect();

    // CPHY SAP methods
    /**
     * Receive master information block function
     *
     * @param cellId the cell ID
     * @param arfcn the ARFCN of the carrier (BWP) on which the MIB was received
     * @param msg NrRrcSap::MasterInformationBlock
     */
    void DoRecvMasterInformationBlock(uint16_t cellId,
                                      uint32_t arfcn,
                                      NrRrcSap::MasterInformationBlock msg);
    /**
     * Receive system information block type 1 function
     *
     * @param cellId the cell ID
     * @param msg NrRrcSap::SystemInformationBlockType1
     */
    void DoRecvSystemInformationBlockType1(uint16_t cellId,
                                           uint32_t arfcn,
                                           NrRrcSap::SystemInformationBlockType1 msg);
    /**
     * Report UE measurements function
     *
     * @param params NrUeCphySapUser::UeMeasurementsParameters
     */
    void DoReportUeMeasurements(NrUeCphySapUser::UeMeasurementsParameters params);

    // RRC SAP methods

    /**
     * Part of the RRC protocol. Implement the NrUeRrcSapProvider::CompleteSetup interface.
     * @param params the NrUeRrcSapProvider::CompleteSetupParameters
     */
    void DoCompleteSetup(NrUeRrcSapProvider::CompleteSetupParameters params);
    /**
     * Part of the RRC protocol. Implement the NrUeRrcSapProvider::RecvSystemInformation interface.
     * @param msg the NrRrcSap::SystemInformation
     */
    void DoRecvSystemInformation(NrRrcSap::SystemInformation msg);
    /**
     * Part of the RRC protocol. Implement the NrUeRrcSapProvider::RecvRrcConnectionSetup
     * interface.
     * @param msg the NrRrcSap::RrcConnectionSetup
     */
    void DoRecvRrcConnectionSetup(NrRrcSap::RrcConnectionSetup msg);
    /**
     * Part of the RRC protocol. Implement the NrUeRrcSapProvider::RecvRrcConnectionReconfiguration
     * interface.
     * @param msg the NrRrcSap::RrcConnectionReconfiguration
     */
    void DoRecvRrcConnectionReconfiguration(NrRrcSap::RrcConnectionReconfiguration msg);
    /**
     * Part of the RRC protocol. Implement the NrUeRrcSapProvider::RecvRrcConnectionReestablishment
     * interface.
     * @param msg NrRrcSap::RrcConnectionReestablishment
     */
    void DoRecvRrcConnectionReestablishment(NrRrcSap::RrcConnectionReestablishment msg);
    /**
     * Part of the RRC protocol. Implement the
     * NrUeRrcSapProvider::RecvRrcConnectionReestablishmentReject interface.
     * @param msg NrRrcSap::RrcConnectionReestablishmentReject
     */
    void DoRecvRrcConnectionReestablishmentReject(NrRrcSap::RrcConnectionReestablishmentReject msg);
    /**
     * Part of the RRC protocol. Implement the NrUeRrcSapProvider::RecvRrcConnectionRelease
     * interface.
     * @param msg NrRrcSap::RrcConnectionRelease
     */
    void DoRecvRrcConnectionRelease(NrRrcSap::RrcConnectionRelease msg);
    /**
     * Part of the RRC protocol. Implement the NrUeRrcSapProvider::RecvRrcConnectionReject
     * interface.
     * @param msg the NrRrcSap::RrcConnectionReject
     */
    void DoRecvRrcConnectionReject(NrRrcSap::RrcConnectionReject msg);

    /**
     * RRC CCM SAP USER Method
     * @param noOfComponentCarriers the number of component carriers
     */
    void DoSetNumberOfComponentCarriers(uint16_t noOfComponentCarriers);

    // INTERNAL METHODS

    /**
     * @brief Go through the list of measurement results, choose the one with the
     *        strongest RSRP, and tell PHY to synchronize to it.
     *
     * @warning This function is a part of the *initial cell selection* procedure,
     *          hence must be only executed during IDLE mode.
     */
    void SynchronizeToStrongestCell();

    /**
     * @brief Tune the UE to a cell and register with its MAC.
     *
     * Points the primary DL BWP at the given cell/ARFCN (numerology, PHY
     * synchronization) and registers the UE MAC to the cell. Shared by the
     * force-camp path (DoForceCampedOnGnb) and the MIB-wait reselection fallback
     * (MibWaitReselect). Does not change the RRC state.
     *
     * @param cellId cell to camp on.
     * @param arfcn ARFCN of the cell's carrier.
     */
    void CampOnGnb(uint16_t cellId, uint32_t arfcn);

    /**
     * @brief Arm (or re-arm) the MIB-wait reselection timer.
     *
     * When #m_mibWaitReselectTimeout is non-zero, schedules MibWaitReselect() to
     * fire after that interval. Used to rescue a force-camped UE that never
     * decodes its target cell's MIB. A no-op when the timeout is zero (disabled).
     */
    void ArmMibWaitReselect();

    /**
     * @brief MIB-wait timeout handler: reselect the strongest measured cell.
     *
     * If the UE is still in IDLE_WAIT_MIB, reselects the strongest cell it has
     * measured (excluding cells already tried) and re-camps on it, then re-arms
     * the timer. Mimics idle-mode cell reselection so a UE force-camped on an
     * unreachable cell is not stuck waiting for a MIB indefinitely.
     */
    void MibWaitReselect();

    /**
     * @brief Performs cell selection evaluation to the current serving cell.
     *
     * @warning This function is a part of the *initial cell selection* procedure,
     *          hence must be only executed during IDLE mode and specifically
     *          during the state when the UE just received the first SIB1 message
     *          from the serving cell.
     *
     * This function assumes that the required information for the evaluation
     * procedure have been readily gathered, such as *measurement results*, MIB,
     * and SIB1. Please refer to the NR module's Design Documentation for more
     * details on the evaluation process.
     *
     * If the cell passes the evaluation, the UE will immediately camp to it.
     * Otherwise, the UE will pick another cell and restart the cell selection
     * procedure.
     */
    void EvaluateCellForSelection();

    /**
     * @brief Update the current measurement configuration #m_varMeasConfig.
     * @param mc measurements to be performed by the UE
     *
     * Implements Section 5.5.2 "Measurement configuration" of 3GPP TS 36.331.
     * The supported subfunctions are:
     * - Measurement object removal
     * - Measurement object addition/ modification
     * - Reporting configuration removal
     * - Reporting configuration addition/ modification
     * - Quantity configuration
     * - Measurement identity removal
     * - Measurement identity addition/ modification
     *
     * The subfunctions that will be invoked are determined by the content of
     * the given measurement configuration.
     *
     * Note the existence of some chain reaction behaviours:
     * - Removal of measurement object or reporting configuration also removes any
     *   impacted measurement identities.
     * - Removal of measurement identity also removes any associated *reporting
     *   entry* from #m_varMeasReportList.
     * - Modification to measurement object or reporting configuration also
     *   removes any reporting entries of the impacted measurement identities
     *   from #m_varMeasReportList.
     * - Modification to quantity configuration also removes all existing
     *   reporting entries from #m_varMeasReportList, regardless of measurement
     *   identity.
     *
     * Some unsupported features:
     * - List of neighbouring cells
     * - List of black cells
     * - CGI reporting
     * - Periodical reporting configuration
     * - Measurement gaps
     * - s-Measure
     * - Speed-dependent scaling
     *
     * @warning There is a possibility that the input argument (of type
     *          NrRrcSap::MeasConfig) may contain information in fields related
     *          to the unsupported features. In such case, the function will raise
     *          an error.
     *
     * The measurement configuration given as an argument is typically provided by
     * the serving gNB. It is transmitted through the RRC protocol when the UE
     * joins the cell, e.g., by connection establishment or by incoming handover.
     * The information inside the argument can be configured from the gNB side,
     * which would then equally apply to all other UEs attached to the same
     * gNB. See the NR module's User Documentation for more information on
     * configuring this.
     *
     * \sa NrRrcSap::MeasConfig, NrUeRrc::m_varMeasReportList
     */
    void ApplyMeasConfig(NrRrcSap::MeasConfig mc);

    /**
     * @brief Keep the given measurement result as the latest measurement figures,
     *        to be utilised by UE RRC functions.
     * @param cellId the cell ID of the measured cell
     * @param rsrp measured RSRP value to be saved (in dBm)
     * @param rsrq measured RSRQ value to be saved (in dB)
     * @param useLayer3Filtering
     * @param componentCarrierId
     * @todo Remove the useLayer3Filtering argument
     *
     * Implements Section 5.5.3.2 "Layer 3 filtering" of 3GPP TS 36.331. *Layer-3
     * filtering* is applied to the given measurement results before saved to
     * #m_storedMeasValues. The filtering is however disabled when the UE is in
     * IDLE mode, i.e., saving unfiltered values.
     *
     * Layer-3 filtering is influenced by a filter coefficient, which determines
     * the strength of the filtering. This coefficient is provided by the active
     * *quantity configuration* in #m_varMeasConfig, which is configured by the
     * NrUeRrc::ApplyMeasConfig. Details on how the coefficient works and how to
     * modify it can be found in NR module's Design Documentation.
     *
     * \sa NrUeRrc::m_storedMeasValues
     */
    void SaveUeMeasurements(uint16_t cellId,
                            double rsrp,
                            double rsrq,
                            bool useLayer3Filtering,
                            uint8_t componentCarrierId);

    /**
     * @brief Evaluate the reporting criteria of a measurement identity and
     *        invoke some reporting actions based on the result.
     * @param measId the measurement identity to be evaluated
     *
     * Implements Section 5.5.4.1 "Measurement report triggering - General" of
     * 3GPP TS 36.331. This function take into use the latest measurement results
     * and evaluate them against the *entering condition* and the *leaving
     * condition* of the measurement identity's reporting criteria. The evaluation
     * also take into account other defined criteria, such as *hysteresis* and
     * *time-to-trigger*.
     *
     * The entering and leaving condition to be evaluated are determined by the
     * *event type* of the measurement identity's reporting criteria. As defined
     * in NrRrcSap::ReportConfigEutra, there 5 supported events. The gore details
     * of these events can be found in Section 5.5.4 of 3GPP TS 36.331.
     *
     * An applicable entering condition (i.e., the condition evaluates to true)
     * will insert a new *reporting entry* to #m_varMeasReportList, so
     * *measurement reports* would be produced and submitted to gNB. On the
     * other hand, an applicable leaving condition will remove the related
     * reporting entry from #m_varMeasReportList, so submission of related
     * measurement reports to eNodeB will be suspended.
     */
    void MeasurementReportTriggering(uint8_t measId);

    /**
     * @brief Produce a proper measurement report from the given measurement
     *        identity's reporting entry in #m_varMeasReportList and then submit
     *        it to the serving eNodeB.
     * @param measId the measurement identity which report is to be submitted.
     *
     * Implements Section 5.5.5 "Measurement reporting" of 3GPP TS 36.331.
     * Producing a *measurement report* involves several tasks such as:
     * - including the measurement results of the serving cell into the report;
     * - selecting some neighbour cells which triggered the reporting (i.e., those
     *   in *cellsTriggeredList*) to be included in the report;
     * - sorting the order of neighbour cells in the report by their RSRP or RSRQ
     *   measurement results (the highest comes first); and
     * - ensuring the number of neighbour cells in the report is under the
     *   *maxReportCells* limit defined by the measurement identity's reporting
     *   configuration.
     *
     * The RSRP and RSRQ measurement results included in the report are expressed
     * in 3GPP-specified range format. They are converted from dBm and dB units
     * using nr::EutranMeasurementMapping::Dbm2RsrpRange and
     * nr::EutranMeasurementMapping::Db2RsrqRange functions.
     *
     * Measurement report is submitted to the serving eNodeB through the *RRC
     * protocol*. The NrUeRrcSapUser::SendMeasurementReport method of the *UE RRC
     * SAP* facilitates this submission.
     *
     * After the submission, the function will repeat itself after a certain
     * interval. The interval length may vary from 120 ms to 60 minutes and is
     * determined by the *report interval* parameter specified by the measurement
     * identity's reporting configuration.
     */
    void SendMeasurementReport(uint8_t measId);

    /**
     * Apply radio resource config dedicated.
     * @param rrcd NrRrcSap::RadioResourceConfigDedicated
     */
    void ApplyRadioResourceConfigDedicated(NrRrcSap::RadioResourceConfigDedicated rrcd);
    /**
     * Apply the serving-cell BWP configurations and QoS flow to BWP mappings
     * received in dedicated RRC config: reconfigures the local BWPs, installs
     * the UL pairing of DL-only (FDD) carriers as output links, and programs
     * the BWP manager algorithm.
     * @param rrcd NrRrcSap::RadioResourceConfigDedicated
     */
    void ApplyServingCellBwpConfig(const NrRrcSap::RadioResourceConfigDedicated& rrcd);
    /**
     * Apply radio resource config dedicated secondary carrier.
     * @param nonCec NrRrcSap::NonCriticalExtensionConfiguration
     */
    void ApplyRadioResourceConfigDedicatedSecondaryCarrier(
        NrRrcSap::NonCriticalExtensionConfiguration nonCec);
    /// Start connection function
    void StartConnection();
    /**
     * @brief Leave connected mode method
     * Resets the UE back to an appropriate state depending
     * on the nature of cause. For example, the UE is move
     * to the IDLE_START state upon radio link failure. At
     * RRC, all radio bearers except SRB 0 are removed,
     * measurement reports are cleared and the appropriate
     * flags are reset to their default values. This method
     * in turn triggers the reset methods of UE PHY and MAC layers.
     */
    void LeaveConnectedMode();
    /// Dispose old SRB1
    void DisposeOldSrb1();
    /**
     * Convert QFI to DRB ID
     * @param qfi the QoS Flow ID
     * @returns the DR bid
     */
    uint8_t Qfi2Drbid(uint8_t qfi);
    /**
     * Switch the UE RRC to the given state.
     * @param s the destination state
     */
    void SwitchToState(State s);

    // QFI to DRBID mapping, maintained for defensive programming purposes. While the
    // current ns-3 implementation enforces DRBID = QFI + 2 (which could be computed with
    // a formula), this explicit map allows for future changes to the identifier mapping
    // scheme without requiring code modifications elsewhere in the UE RRC layer.
    std::map<uint8_t, uint8_t> m_qfi2DrbidMap; ///< QFI to DR bearer id map

    std::vector<NrUeCphySapUser*> m_cphySapUser;         ///< UE CPhy SAP user
    std::vector<NrUeCphySapProvider*> m_cphySapProvider; ///< UE CPhy SAP provider

    std::vector<NrUeCmacSapUser*> m_cmacSapUser;         ///< UE CMac SAP user
    std::vector<NrUeCmacSapProvider*> m_cmacSapProvider; ///< UE CMac SAP provider
    /// Notify the MAC/PHY of every BWP that the connection succeeded (ntn-patch 24).
    void NotifyConnectionSuccessfulAllBwps();

    NrUeRrcSapUser* m_rrcSapUser;         ///< RRC SAP user
    NrUeRrcSapProvider* m_rrcSapProvider; ///< RRC SAP provider

    NrMacSapProvider* m_macSapProvider; ///< MAC SAP provider
    NrPdcpSapUser* m_drbPdcpSapUser;    ///< DRB PDCP SAP user

    NrAsSapProvider* m_asSapProvider; ///< AS SAP provider
    NrAsSapUser* m_asSapUser;         ///< AS SAP user

    // Receive API calls from the NrUeComponentCarrierManager  instance.
    // NrCcmRrcSapUser* m_ccmRrcSapUser;
    /// Interface to the NrUeComponentCarrierManage instance.
    NrUeCcmRrcSapProvider* m_ccmRrcSapProvider; ///< CCM RRC SAP provider
    NrUeCcmRrcSapUser* m_ccmRrcSapUser;         ///< CCM RRC SAP user

    /// The current UE RRC state.
    State m_state;

    /// The unique UE identifier.
    uint64_t m_imsi;
    /// Per-cell individual offset (Ocn/Ocp) in dB applied during event A3
    /// evaluation, used to bias handover (e.g. pico cell-range expansion).
    std::map<uint16_t, double> m_cellIndividualOffsetDb;
    /**
     * The `C-RNTI` attribute. Cell Radio Network Temporary Identifier.
     */
    uint16_t m_rnti;
    /**
     * The `CellId` attribute. Serving cell identifier.
     */
    uint16_t m_cellId;

    /**
     * The index of primary UL PHY/MAC instances
     */
    /**
     * @brief Push the current primary DL->UL pairing into the BWP manager
     * routing, overwriting links left over from a previous serving cell
     */
    void SyncBwpOutputLinks();

    std::function<void(uint32_t, uint32_t)>
        m_updateBwpOutputLinkFn; ///< updates the BWP manager output links

    std::function<void(uint8_t, uint8_t)>
        m_updateQosFlowBwpFn; ///< programs the BWP manager 5QI to BWP mapping

    std::function<void()> m_clearBwpOutputLinksFn; ///< drops all BWP manager output links

    /**
     * @brief UL carrier pairing of DL-only BWPs, advertised by the serving cell
     * in dedicated RRC config (local source BWP -> local UL BWP).
     *
     * Replaced on every application of a serving-cell BWP configuration, and
     * consulted by SyncBwpOutputLinks() so primary index changes do not break
     * the FDD pairing of secondary carriers.
     */
    std::map<uint32_t, uint32_t> m_rrcBwpPairings;

    uint16_t m_primaryUlIndex{0};

    /**
     * The index of primary DL PHY/MAC instances
     */
    uint16_t m_primaryDlIndex{0};

    /**
     * The `Srb0` attribute. SignalingRadioBearerInfo for SRB0.
     */
    Ptr<NrSignalingRadioBearerInfo> m_srb0;
    /**
     * The `Srb1` attribute. SignalingRadioBearerInfo for SRB1.
     */
    Ptr<NrSignalingRadioBearerInfo> m_srb1;
    /**
     * SRB1 configuration before RRC connection reconfiguration. To be deleted
     * soon by DisposeOldSrb1().
     */
    Ptr<NrSignalingRadioBearerInfo> m_srb1Old;
    /**
     * The `DataRadioBearerMap` attribute. List of UE RadioBearerInfo for Data
     * Radio Bearers by LCID.
     */
    std::map<uint8_t, Ptr<NrDataRadioBearerInfo>> m_drbMap;

    /**
     * True if RLC SM is to be used, false if RLC UM/AM are to be used.
     * Can be modified using SetUseRlcSm().
     */
    bool m_useRlcSm;

    /**
     * True if RRC connection reestablishment is enabled, false if the UE
     * should directly clear its context upon radio link failure.
     * Can be modified using SetUseRrcReestablishment().
     */
    bool m_useRrcReestablishment; ///< RRC reestablishment enabled flag
    bool m_rlcMaxRetxTriggersRlf; ///< if true, RLC-AM max-retx declares RLF (TS 38.331 5.3.10.3)
    bool m_rlcMaxRetxRlfDeclared{
        false}; ///< guard: RLC-max-retx RLF already declared this connection
    /// TR 36.839 (5.3.2) handover-failure model: fail the handover if the HO
    /// command arrives while the source link is below Qout (T310 running).
    bool m_tr36839HandoverFailure{false};
    /// Graded threshold for the above: a late HO command is only failed if T310
    /// has been running for at least this long; otherwise the command rescues the
    /// link. 0 ms = original binary behaviour. @see Tr36839HoFailureMinT310Elapsed
    Time m_tr36839HoFailureMinT310Elapsed{MilliSeconds(0)};

    /**
     * @brief Set whether RRC connection reestablishment is enabled.
     * @param val true to enable reestablishment, false to disable it
     */
    void SetUseRrcReestablishment(bool val);

    uint8_t m_lastRrcTransactionIdentifier; ///< last RRC transaction identifier

    NrRrcSap::PdschConfigDedicated m_pdschConfigDedicated; ///< the PDSCH config dedicated

    uint16_t m_dlBandwidth; /**< Downlink bandwidth in RBs. */
    uint16_t m_ulBandwidth; /**< Uplink bandwidth in RBs. */

    uint32_t m_initDlArfcn; /**< Downlink carrier frequency to listen for gNBs. */
    uint32_t m_initUlArfcn; /**< Initial BWP UL carrier frequency set by SIB2. */

    std::list<NrRrcSap::SCellToAddMod> m_sCellToAddModList; /**< Secondary carriers. */

    /**
     * The `MibReceived` trace source. Fired upon reception of Master Information
     * Block. Exporting IMSI, the serving cell ID, RNTI, and the source cell ID.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t, uint16_t> m_mibReceivedTrace;
    /**
     * The `Sib1Received` trace source. Fired upon reception of System
     * Information Block Type 1. Exporting IMSI, the serving cell ID, RNTI, and
     * the source cell ID.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t, uint16_t> m_sib1ReceivedTrace;
    /**
     * The `Sib2Received` trace source. Fired upon reception of System
     * Information Block Type 2. Exporting IMSI, the serving cell ID, RNTI.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t> m_sib2ReceivedTrace;
    /**
     * The `StateTransition` trace source. Fired upon every UE RRC state
     * transition. Exporting IMSI, the serving cell ID, RNTI, old state, and new
     * state.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t, State, State> m_stateTransitionTrace;
    /**
     * The `InitialCellSelectionEndOk` trace source. Fired upon successful
     * initial cell selection procedure. Exporting IMSI and the selected cell ID.
     */
    TracedCallback<uint64_t, uint16_t> m_initialCellSelectionEndOkTrace;
    /**
     * The `InitialCellSelectionEndError` trace source. Fired upon failed initial
     * cell selection procedure. Exporting IMSI and the cell ID under evaluation.
     */
    TracedCallback<uint64_t, uint16_t> m_initialCellSelectionEndErrorTrace;
    /**
     * The `RandomAccessSuccessful` trace source. Fired upon successful
     * completion of the random access procedure. Exporting IMSI, cell ID, and
     * RNTI.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t> m_randomAccessSuccessfulTrace;
    /**
     * The `RandomAccessError` trace source. Fired upon failure of the random
     * access procedure. Exporting IMSI, cell ID, and RNTI.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t> m_randomAccessErrorTrace;
    /**
     * The `ConnectionEstablished` trace source. Fired upon successful RRC
     * connection establishment. Exporting IMSI, cell ID, and RNTI.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t> m_connectionEstablishedTrace;
    /**
     * The `ConnectionTimeout` trace source. Fired upon timeout RRC connection
     * establishment because of T300. Exporting IMSI, cell ID, and RNTI.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t, uint8_t> m_connectionTimeoutTrace;
    /**
     * The `ConnectionReconfiguration` trace source. Fired upon RRC connection
     * reconfiguration. Exporting IMSI, cell ID, and RNTI.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t> m_connectionReconfigurationTrace;
    /**
     * The `HandoverStart` trace source. Fired upon start of a handover
     * procedure. Exporting IMSI, source cell ID, RNTI, and target cell ID.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t, uint16_t> m_handoverStartTrace;
    /**
     * The `HandoverEndOk` trace source. Fired upon successful termination of a
     * handover procedure. Exporting IMSI, cell ID, and RNTI.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t> m_handoverEndOkTrace;
    /**
     * The `HandoverEndError` trace source. Fired upon failure of a handover
     * procedure. Exporting IMSI, cell ID, and RNTI.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t> m_handoverEndErrorTrace;
    /**
     * The `RadioLinkFailureCause` trace source. Fired at the instant the UE
     * enters CONNECTED_PHY_PROBLEM, carrying the cause and timing context.
     * Exporting IMSI, cellId, RNTI, oldState, cause token, T310-elapsed ms, and
     * ms since the last successful handover.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t, uint16_t, std::string, int64_t, int64_t>
        m_radioLinkFailureCauseTrace;
    /**
     * The `SCarrierConfigured` trace source. Fired after the configuration
     * of secondary carriers received through RRC Connection Reconfiguration
     * message.
     */
    TracedCallback<Ptr<NrUeRrc>, std::list<NrRrcSap::SCellToAddMod>> m_sCarrierConfiguredTrace;
    /**
     * The `Srb1Created` trace source. Fired when SRB1 is created, i.e.
     * the RLC and PDCP entities are created for logical channel = 1.
     * Exporting IMSI, cell ID, and RNTI
     */
    TracedCallback<uint64_t, uint16_t, uint16_t> m_srb1CreatedTrace;
    /**
     * The `DrbCreated` trace source. Fired when DRB is created, i.e.
     * the RLC and PDCP entities are created for one logical channel.
     * Exporting IMSI, cell ID, RNTI, LCID.
     */
    TracedCallback<uint64_t, uint16_t, uint16_t, uint8_t> m_drbCreatedTrace;
    /**
     * The 'PhySyncDetection' trace source. Fired when UE RRC
     * receives in-sync or out-of-sync indications from UE PHY
     *
     */
    TracedCallback<uint64_t, uint16_t, uint16_t, std::string, uint8_t> m_phySyncDetectionTrace;
    /**
     * The 'RadioLinkFailure' trace source. Fired when T310 timer expires.
     *
     */
    TracedCallback<uint64_t, uint16_t, uint16_t> m_radioLinkFailureTrace;

    /// True if a connection request by upper layers is pending.
    bool m_connectionPending;
    /// True if MIB was received for the current cell.
    bool m_hasReceivedMib;
    /// True if SIB1 was received for the current cell.
    bool m_hasReceivedSib1;
    /// True if SIB2 was received for the current cell.
    bool m_hasReceivedSib2;

    /// Stored content of the last SIB1 received.
    NrRrcSap::SystemInformationBlockType1 m_lastSib1;

    /// List of cell ID of acceptable cells for cell selection that have been detected.
    std::set<uint16_t> m_acceptableCell;

    /// List of CSG ID which this UE entity has access to.
    uint32_t m_csgWhiteList;

    // INTERNAL DATA STRUCTURE RELATED TO UE MEASUREMENTS

    /**
     * @brief Includes the accumulated configuration of the measurements to be
     *        performed by the UE.
     *
     * Based on 3GPP TS 36.331 section 7.1. Also note that some optional variables
     * in the specification are omitted.
     */
    struct NR_EXPORT VarMeasConfig
    {
        std::map<uint8_t, NrRrcSap::MeasIdToAddMod> measIdList;             ///< measure ID list
        std::map<uint8_t, NrRrcSap::MeasObjectToAddMod> measObjectList;     ///< measure object list
        std::map<uint8_t, NrRrcSap::ReportConfigToAddMod> reportConfigList; ///< report config list
        NrRrcSap::QuantityConfig quantityConfig;                            ///< quantity config
        // Layer-3 filter coefficients; 1.0 (no filtering, F_n = M_n) until a
        // measurement configuration with a quantityConfig is applied
        double aRsrp{1.0}; ///< RSRP
        double aRsrq{1.0}; ///< RSRQ
    };

    /**
     * @brief Includes the accumulated configuration of the measurements to be
     *        performed by the UE.
     *
     * Based on 3GPP TS 36.331 section 7.1.
     */
    VarMeasConfig m_varMeasConfig;

    /**
     * @brief Represents a single measurement reporting entry., which includes
     *        information about a measurement for which the triggering conditions
     *        have been met.
     *
     * Based on 3GPP TS 36.331 section 7.1.
     */
    struct NR_EXPORT VarMeasReport
    {
        uint8_t measId;                        ///< measure ID
        std::set<uint16_t> cellsTriggeredList; ///< note: only E-UTRA is supported.
        uint32_t numberOfReportsSent;          ///< number of reports sent
        EventId periodicReportTimer;           ///< periodic report timer
    };

    /**
     * @brief The list of active reporting entries, indexed by the measurement
     *        identity which triggered the reporting. Includes information about
     *        measurements for which the triggering conditions have been met.
     */
    std::map<uint8_t, VarMeasReport> m_varMeasReportList;

    /**
     * @brief List of cell IDs which are responsible for a certain trigger.
     */
    typedef std::list<uint16_t> ConcernedCells_t;

    /**
     * @brief Compose a new reporting entry of the given measurement identity,
     *        insert it into #m_varMeasReportList, and set it up for submission
     *        to eNodeB.
     * @param measId the measurement identity which the new reporting entry will
     *               be based upon
     * @param enteringCells the cells which are responsible for triggering the
     *                      reporting (i.e., successfully fulfilling the entering
     *                      condition of the measurement identity) and will be
     *                      included in the measurement report.
     *
     * @note If an existing reporting entry with the same measurement identity has
     *       already existed in #m_varMeasReportList, the function will update it
     *       by adding the entering cells into the existing reporting entry.
     * @note When time-to-trigger is enabled for this measurement identity, the
     *       function will also remove the related trigger from the
     *       #m_enteringTriggerQueue.
     */
    void VarMeasReportListAdd(uint8_t measId, ConcernedCells_t enteringCells);

    /**
     * @brief Remove some cells from an existing reporting entry in
     *        #m_varMeasReportList.
     * @param measId the measurement identity to be removed from
     *               #m_varMeasReportList, must already exists there, otherwise
     *               an error would be raised
     * @param leavingCells the cells which are about to be removed
     * @param reportOnLeave when true, will make the function send one last
     *                      measurement report to eNodeB before removing it
     *
     * @note If a given cell is not found in the reporting entry, the function
     *       will quietly continue.
     * @note If the removal has removed all the cells in the reporting entry, the
     *       function will remove the reporting entry as well.
     * @note When time-to-trigger is enabled for this measurement identity, the
     *       function will also remove the related trigger from the
     *       #m_leavingTriggerQueue.
     */
    void VarMeasReportListErase(uint8_t measId, ConcernedCells_t leavingCells, bool reportOnLeave);

    /**
     * @brief Remove the reporting entry of the given measurement identity from
     *        #m_varMeasReportList.
     * @param measId the measurement identity to be removed from
     *               #m_varMeasReportList, must already exists there, otherwise
     *               an error would be raised
     *
     * Any events or triggers related with this measurement identity will be
     * canceled as well.
     */
    void VarMeasReportListClear(uint8_t measId);

    /**
     * @brief Represents a measurement result from a certain cell.
     */
    struct NR_EXPORT MeasValues
    {
        double rsrp;          ///< Measured RSRP in dBm.
        double rsrq;          ///< Measured RSRQ in dB.
        uint32_t carrierFreq; ///< Measurement object frequency
    };

    /**
     * @brief Internal storage of the latest measurement results from all detected
     *        detected cells, indexed by the cell ID where the measurement was
     *        taken from.
     *
     * Each *measurement result* comprises of RSRP (in dBm) and RSRQ (in dB).
     *
     * In IDLE mode, the measurement results are used by the *initial cell
     * selection* procedure. While in CONNECTED mode, *layer-3 filtering* is
     * applied to the measurement results and they are used by *UE measurements*
     * function (NrUeRrc::MeasurementReportTriggering and
     * NrUeRrc::SendMeasurementReport).
     */
    std::map<uint16_t, MeasValues> m_storedMeasValues;

    /**
     * @brief Cell<->carrier bookkeeping: maps a discovered cell ID to the
     *        carrier (ARFCN) on which it was found.
     *
     * The UE keeps multiple BWPs tuned (one per carrier) so it can receive SSB
     * and measure RSRP on neighbour frequencies. This map lets the RRC route a
     * received broadcast (MIB) to the BWP actually tuned to the originating
     * cell's carrier (via GetArfcnBwpId), instead of unconditionally applying
     * it to the primary serving BWP. Populated by TrackCellArfcn() as cells are
     * discovered/measured/synchronized.
     *
     * A single cell can legitimately span SEVERAL carriers (the same cellId on
     * several BWPs, differentiated by ARFCN alone). Therefore the value is a
     * SET of carriers, not a single carrier. This is what lets same-cell BWP
     * switching tell apart "another BWP of my serving cell" (accept) from "a
     * different cell" (reject; that would be handover/DC).
     */
    std::map<uint16_t, std::set<uint32_t>> m_cellIdToArfcn;

    /**
     * @brief Per-carrier RSRP, keyed by ARFCN, layer-3 filtered.
     *
     * Because all BWPs of a single gNB share one cellId, the cellId-keyed
     * #m_storedMeasValues store (used by A3 / SynchronizeToStrongestCell) cannot
     * tell one BWP from another of the same cell: the last-measured carrier
     * simply overwrites the previous one. To drive a SAME-CELL primary-BWP
     * switch we therefore track RSRP SEPARATELY per carrier (ARFCN). This is
     * updated additively (same alpha as #m_storedMeasValues) inside
     * SaveUeMeasurements and consulted only by the same-cell switch policy; it
     * never feeds the cellId-keyed handover machinery.
     */
    std::map<uint32_t, double> m_rsrpPerArfcn;

    /**
     * @brief Per-BWP uplink capability, keyed by BWP index.
     *
     * Recorded in ReconfigureFromSib1() from the TDD pattern applied to the BWP:
     * a BWP whose pattern has no UL-capable slot (e.g., the DL carrier of an FDD
     * pair) cannot host the primary UL. Consulted by SwitchPrimaryBwpSameCell()
     * to avoid moving the primary UL onto a DL-only BWP. BWPs with no recorded
     * entry are assumed UL-capable (the common TDD case).
     */
    std::map<std::size_t, bool> m_bwpUlCapable;

    /**
     * @brief Stored measure values per carrier.
     */
    std::map<uint16_t, std::map<uint8_t, MeasValues>> m_storedMeasValuesPerCarrier;

    /**
     * @brief Internal storage of the latest measurement results from all detected
     *        detected Secondary carrier component, indexed by the carrier component ID
     *        where the measurement was taken from.
     *
     * Each *measurement result* comprises of RSRP (in dBm) and RSRQ (in dB).
     *
     * In IDLE mode, the measurement results are used by the *initial cell
     * selection* procedure. While in CONNECTED mode, *layer-3 filtering* is
     * applied to the measurement results and they are used by *UE measurements*
     * function:
     * - NrUeRrc::MeasurementReportTriggering: in this case it is not set any
     *   measurement related to secondary carrier components since the
     *   A6 event is not implemented
     * - NrUeRrc::SendMeasurementReport: in this case the report are sent.
     */
    std::map<uint16_t, MeasValues> m_storedScellMeasValues;

    /**
     * @brief Represents a single triggered event from a measurement identity
     *        which reporting criteria have been fulfilled, but delayed by
     *        time-to-trigger.
     */
    struct NR_EXPORT PendingTrigger_t
    {
        uint8_t measId;                  ///< The measurement identity which raised the trigger.
        ConcernedCells_t concernedCells; ///< The list of cells responsible for this trigger.
        EventId
            timer; ///< The pending reporting event, scheduled at the end of the time-to-trigger.
    };

    /**
     * @brief List of triggers that were raised because entering condition have
     *        been true, but are still delayed from reporting it by
     *        time-to-trigger.
     *
     * The list is indexed by the measurement identity where the trigger
     * originates from. The enclosed event will run at the end of the
     * time-to-trigger and insert a *reporting entry* to #m_varMeasReportList.
     */
    std::map<uint8_t, std::list<PendingTrigger_t>> m_enteringTriggerQueue;

    /**
     * @brief List of triggers that were raised because leaving condition have
     *        been true, but are still delayed from stopping the reporting by
     *        time-to-trigger.
     *
     * The list is indexed by the measurement identity where the trigger
     * originates from. The enclosed event will run at the end of the
     * time-to-trigger and remove the associated *reporting entry* from
     * #m_varMeasReportList.
     */
    std::map<uint8_t, std::list<PendingTrigger_t>> m_leavingTriggerQueue;

    /**
     * @brief Clear all the waiting triggers in #m_enteringTriggerQueue which are
     *        associated with the given measurement identity.
     * @param measId the measurement identity to be processed, must already exists
     *               in #m_enteringTriggerQueue, otherwise an error would be
     *               raised
     *
     * @note The function may conclude that there is nothing to be removed. In
     *       this case, the function will simply ignore quietly.
     *
     * This function is used when the entering condition of the measurement
     * identity becomes no longer true. Therefore all the waiting triggers for
     * this measurement identity in #m_enteringTriggerQueue have become invalid
     * and must be canceled.
     *
     * \sa NrUeRrc::m_enteringTriggerQueue
     */
    void CancelEnteringTrigger(uint8_t measId);

    /**
     * @brief Remove a specific cell from the waiting triggers in
     *        #m_enteringTriggerQueue which belong to the given measurement
     *        identity.
     * @param measId the measurement identity to be processed, must already exists
     *               in #m_enteringTriggerQueue, otherwise an error would be
     *               raised
     * @param cellId the cell ID to be removed from the waiting triggers
     *
     * @note The function may conclude that there is nothing to be removed. In
     *       this case, the function will simply ignore quietly.
     *
     * This function is used when a specific neighbour cell no longer fulfills
     * the entering condition of the measurement identity. Thus the cell must be
     * removed from all the waiting triggers for this measurement identity in
     * #m_enteringTriggerQueue.
     *
     * \sa NrUeRrc::m_enteringTriggerQueue
     */
    void CancelEnteringTrigger(uint8_t measId, uint16_t cellId);

    /**
     * @brief Clear all the waiting triggers in #m_leavingTriggerQueue which are
     *        associated with the given measurement identity.
     * @param measId the measurement identity to be processed, must already exists
     *               in #m_leavingTriggerQueue, otherwise an error would be
     *               raised
     *
     * @note The function may conclude that there is nothing to be removed. In
     *       this case, the function will simply ignore quietly.
     *
     * This function is used when the leaving condition of the measurement
     * identity becomes no longer true. Therefore all the waiting triggers for
     * this measurement identity in #m_leavingTriggerQueue have become invalid
     * and must be canceled.
     *
     * \sa NrUeRrc::m_leavingTriggerQueue
     */
    void CancelLeavingTrigger(uint8_t measId);

    /**
     * @brief Remove a specific cell from the waiting triggers in
     *        #m_leavingTriggerQueue which belong to the given measurement
     *        identity.
     * @param measId the measurement identity to be processed, must already exists
     *               in #m_leavingTriggerQueue, otherwise an error would be
     *               raised
     * @param cellId the cell ID to be removed from the waiting triggers
     *
     * @note The function may conclude that there is nothing to be removed. In
     *       this case, the function will simply ignore quietly.
     *
     * This function is used when a specific neighbour cell no longer fulfills
     * the leaving condition of the measurement identity. Thus the cell must be
     * removed from all the waiting triggers for this measurement identity in
     * #m_leavingTriggerQueue.
     *
     * \sa NrUeRrc::m_leavingTriggerQueue
     */
    void CancelLeavingTrigger(uint8_t measId, uint16_t cellId);

    /**
     * The `T300` attribute. Timer for RRC connection establishment procedure
     * (i.e., the procedure is deemed as failed if it takes longer than this).
     * See Section 7.3 of 3GPP TS 36.331.
     */
    Time m_t300;

    /**
     * @brief Invokes ConnectionEstablishmentTimeout() if RRC connection
     *        establishment procedure for this UE takes longer than T300.
     */
    EventId m_connectionTimeout;

    /**
     * @brief Invoked after timer T300 expires, notifying upper layers that RRC
     *        connection establishment procedure has failed.
     */
    void ConnectionTimeout();

    /**
     * The 'T310' attribute. After detecting N310 out-of-sync indications,
     * if number of in-sync indications detected is less than N311 before this
     * time, then the radio link is considered to have failed and the UE
     * transitions to state CONNECTED_PHY_PROMLEM and eventually IDLE_START
     * and UE context at eNodeB is destroyed. RRC connection re-establishment
     * is not initiated after this time. See 3GPP TS 36.331 7.3.
     */
    Time m_t310;

    /**
     * The 'N310' attribute. This specifies the maximum
     * consecutive out-of-sync indications from lower layers.
     */
    uint8_t m_n310;

    /**
     *  The 'N311' attribute. This specifies the minimum
     *  consecutive in-sync indications from lower layers.
     */
    uint8_t m_n311;

    /**
     * Time limit (given by m_t310) before the radio link is considered to have failed.
     * It is set upon detecting physical layer problems i.e. upon receiving
     * N310 consecutive out-of-sync indications from lower layers. Calling
     * NrUeRrc::RadioLinkFailureDetected() when it expires.
     * It is cancelled upon receiving N311 consecutive in-sync indications. Upon
     * expiry, the UE transitions to RRC_IDLE and no RRC connection
     * re-establishment is initiated.
     */
    EventId m_radioLinkFailureDetected;

    uint8_t m_noOfSyncIndications; ///< number of in-sync or out-of-sync indications coming from PHY
                                   ///< layer

    bool m_leaveConnectedMode; ///< true if UE NAS ask UE RRC to leave connected mode, e.g., after
                               ///< RLF, i.e. T310 has expired

    uint16_t m_previousCellId; ///< the cell id of the previous cell UE was attached to

    uint8_t m_connEstFailCountLimit; ///< the counter value for T300 timer expiration received from
                                     ///< the gNB

    uint8_t m_connEstFailCount; ///< the counter to count T300 timer expiration

    Time m_lastHoSuccessTime{
        Seconds(0)}; ///< time of this UE's last successful handover (HandoverEndOk);
                     ///< Seconds(0) means it has not handed over yet

    // Mobility State Estimation (TR 36.839 / TS 36.331 5.5.6.2, TS 36.304 5.2.4.3). When
    // enabled, the UE counts its recent handovers over a sliding window to classify itself
    // as Normal/Medium/High mobility and scales the measurement time-to-trigger accordingly,
    // so a fast UE does not chase a small cell's transient peak (or fails to hand over in time).
    bool m_mseEnable{false};             ///< enable Mobility State Estimation
    Time m_mseCountWindow{Seconds(1)};   ///< sliding window over which handovers are counted
    Time m_mseHystNormal{Seconds(1)};    ///< keep the elevated state at least this long
    uint32_t m_mseThreshMedium{2};       ///< handover count for Medium mobility
    uint32_t m_mseThreshHigh{4};         ///< handover count for High mobility
    double m_mseSfMedium{1.0};           ///< TTT scale factor in Medium mobility
    double m_mseSfHigh{1.0};             ///< TTT scale factor in High mobility
    std::list<Time> m_mseHandoverTimes;  ///< completion times of recent handovers
    double m_mseTttScaleFactor{1.0};     ///< current TTT scale factor (Normal = 1.0)
    Time m_mseElevatedUntil{Seconds(0)}; ///< hysteresis: hold >= current elevated state until here
    double m_mseFixedScale{
        0.0}; ///< if >0, apply this TTT scale from t=0 (bypass handover counting)
    /// Re-estimate the mobility state from recent handover count and update the TTT scale factor.
    void UpdateMobilityState();
    /// Current time-to-trigger scale factor: the fixed override if set, else the mobility-state
    /// estimate (MseEnable), else 1.0.
    double GetTttScale();

    RadioLinkFailureCause m_rlfCause{
        RLF_NONE}; ///< cause of the most recent CONNECTED_PHY_PROBLEM entry
    /**
     * @brief Radio link failure detected function
     *
     * Upon detection of radio link failure, the UE is reverted
     * back to idle state and the UE context at eNodeB and EPC
     * is deleted, thus releasing the RRC connection. The eNodeB is notified
     * in an ideal way since there is no radio link failure detection
     * implemented at the eNodeB. If the deletion process is not synchronous,
     * then errors occur due to triggering of assert messages.
     *
     * @param cause why the failure was declared (RLF_T310_EXPIRY when invoked by the
     *        expiring T310 timer, RLF_HO_COMMAND_LATE when invoked because a handover
     *        command arrived while T310 was already running)
     */
    void RadioLinkFailureDetected(RadioLinkFailureCause cause = RLF_T310_EXPIRY);

    /**
     * @brief Single entry point for transitioning into CONNECTED_PHY_PROBLEM.
     *
     * Captures the failure cause and the timing context known at this instant
     * (how long the DL had been below Qout, and how long since the last
     * successful handover), fires the RadioLinkFailureCause trace, then performs
     * the state switch. All sites that move the UE into CONNECTED_PHY_PROBLEM go
     * through here so the recorded cause is authoritative.
     *
     * @param cause the failure cause to record
     */
    void EnterPhyProblemState(RadioLinkFailureCause cause);

    /**
     * @brief Callback target invoked when one of this UE's RLC-AM entities reaches
     *        maxRetxThreshold. When RlcMaxRetxTriggersRlf is enabled and the UE is in a
     *        connected state, this declares a radio link failure (TS 38.331 5.3.10.3);
     *        otherwise it is ignored (legacy behaviour).
     */
    void DoNotifyRlcMaxRetx();

    /**
     * @brief Do notify in sync function
     *
     * Triggered upon receiving an in sync indication from UE PHY.
     * When the count equals N311, then T310 is cancelled.
     */
    void DoNotifyInSync();

    /**
     * @brief Do notify out of sync function
     *
     * Triggered upon receiving an out of sync indication from UE PHY.
     * When the count equals N310, then T310 is started.
     */
    void DoNotifyOutOfSync();

    /**
     * @brief Do reset sync indication counter function
     *
     * Reset the sync indication counter
     * if the Qin or Qout condition at PHY
     * is not fulfilled for the number of
     * consecutive frames.
     */
    void DoResetSyncIndicationCounter();

    /**
     * @brief Reset radio link failure parameters
     *
     * RLF timers and counters should be rest upon:
     *
     * - If the UE received N311 in Sync indications from the UE
     *   PHY.
     * - If the UE receives RRCConnectionReconfiguration including
     *   the mobilityControlInfo (TS 36.331 sec 5.3.5.4)
     *
     * Inside this method the UE RRC also instructs the UE PHY to reset the
     * RLF parameters so, it can start RLF detection again.
     *
     */
    void ResetRlfParams();
    std::size_t GetArfcnBwpId(uint32_t arfcn) const;

    /**
     * @brief Record the carrier (ARFCN) on which a given cell was discovered.
     *
     * Maintains the cell<->BWP bookkeeping used to route per-cell broadcasts
     * (MIB) to the BWP that is actually tuned to that cell's carrier. Called
     * whenever a cell's SSB is measured/synchronized, so the UE always knows
     * which of its tuned BWPs corresponds to a given cell.
     *
     * @param cellId Cell whose carrier is being recorded.
     * @param arfcn  Carrier (ARFCN) the cell was discovered on.
     */
    void TrackCellArfcn(uint16_t cellId, uint32_t arfcn);

    /**
     * @brief Resolve the BWP/CC index tuned to a given cell's carrier.
     *
     * Uses the cell<->carrier bookkeeping (TrackCellArfcn) plus the BWP ARFCN
     * lookup (GetArfcnBwpId) to find which tuned BWP received/can receive that
     * cell's broadcast. Returns false if the cell's carrier is unknown or no
     * BWP is tuned to it.
     *
     * @param cellId Cell to resolve.
     * @param[out] bwpId BWP/CC index tuned to that cell, if found.
     * @return true if a tuned BWP was found for the cell.
     */
    bool GetCellBwpId(uint16_t cellId, std::size_t& bwpId) const;

    /**
     * @brief Switch the primary (serving) DL/UL BWP to another BWP of the
     *        SAME serving cell.
     *
     * This is the mechanism for same-cell BWP switching. It only re-points the
     * primary DL (and, when they coincide, UL) index to @p targetBwpId and
     * re-binds the RNTI on the new PHY/MAC; it does NOT implement any policy
     * for *when* to switch. It is guarded so it can only ever move between BWPs
     * that belong to the current serving cell (@a m_cellId): moving to a BWP of
     * a different cell would be a handover (and, if kept simultaneously, dual
     * connectivity), which is out of scope.
     *
     * @param targetBwpId BWP/CC index to make the new primary serving BWP.
     * @return true if the switch was performed; false if rejected (e.g. the
     *         target BWP is not tuned to the serving cell).
     */
    bool SwitchPrimaryBwpSameCell(std::size_t targetBwpId);

    /**
     * @brief Same-cell BWP-switch POLICY: pick the best same-cell BWP by RSRP.
     *
     * Examines the per-carrier RSRP (#m_rsrpPerArfcn) of every BWP tuned to the
     * CURRENT serving cell (#m_cellId). If a non-serving same-cell BWP beats the
     * serving BWP by at least #m_bwpSwitchHysteresisDb, it requests a switch to
     * it via SwitchPrimaryBwpSameCell. Does nothing unless connected. This is
     * the minimal RSRP-driven trigger; it deliberately does NOT use the
     * cellId-keyed #m_storedMeasValues (which cannot tell same-cell BWPs apart).
     */
    void EvaluateSameCellBwpSwitch();

    /// Hysteresis margin (dB) a non-serving same-cell BWP must exceed the
    /// serving BWP's RSRP by before the UE switches its primary BWP to it.
    double m_bwpSwitchHysteresisDb{3.0};

    /// True once the UE has reported its current primary BWP to the gNB; used
    /// to avoid resending the same indication every measurement.
    uint32_t m_reportedPrimaryArfcn{0};

    // Multi-BWP RACH lock
    /** True while a RACH procedure is in progress on any BWP. */
    bool m_rachInProgress{false};
    /** BWP/CC index that owns the running RACH procedure. */
    uint8_t m_rachBwpId{255};
    /** Simulation time after which the lock expires unconditionally. */
    Time m_rachDeadline{Seconds(0)};
    /** Fallback timer: force-clears the lock if RACH never completes.
     *  Fires at m_rachDeadline + 10 ms. */
    EventId m_rachTimeoutEvent;
    /**
     * Maximum expected duration of one full RACH procedure.
     */
    Time m_rachLockDuration{MilliSeconds(120)};

    // IDLE_WAIT_MIB reselection fallback
    /** If non-zero, a force-camped UE that receives no MIB within this time
     *  reselects the strongest measured cell (0 = disabled, wait forever). */
    Time m_mibWaitReselectTimeout{Seconds(0)};
    /** Maximum number of cells tried during MIB-wait reselection. */
    uint32_t m_mibWaitReselectMaxAttempts{8};
    /** Pending MIB-wait reselection timer. */
    EventId m_mibWaitTimeoutEvent;
    /** MIB-wait reselection attempts made since the current camp started. */
    uint32_t m_mibWaitAttempts{0};
    /** Cells already tried during the current MIB-wait reselection sequence. */
    std::set<uint16_t> m_mibCampTried;
    /**
     * @brief Clears the RACH (Random Access Channel) lock state for the UE
     *
     * This function resets the state variables related to the RACH procedure. It
     * marks the RACH as not in progress, resets the BWP (Bandwidth Part) ID
     * associated with the RACH, clears any configured RACH deadline, and cancels
     * the pending RACH timeout event if one exists.
     */
    void ClearRachLock();

    uint8_t m_rachAttempts{0}; ///< number of RACH attempts before looking for a new cell to camp on
    uint8_t m_rachAttemptsLimit{
        2}; ///< the maximum number of RACH attempts before looking for a new cell to camp on
  public:
    /**
     * The number of component carriers.
     */
    uint16_t m_numberOfComponentCarriers;

}; // end of class NrUeRrc

/**
 * Converts NrUeRrc::State to a string
 * @param state enum value of the state
 * @return string value of the state
 */
NR_EXPORT const std::string ToString(NrUeRrc::State state);
/**
 * Converts NrUeRrc::RadioLinkFailureCause to a stable upper-case token
 * (e.g. "T310_EXPIRY"), used for logging and database storage.
 * @param cause enum value of the failure cause
 * @return string token for the cause
 */
NR_EXPORT std::string ToString(NrUeRrc::RadioLinkFailureCause cause);
/**
 * Prints to the output stream the NrUeRrc::State
 * @param os output stream
 * @param state the enum value of the state
 * @return the output stream
 */
NR_EXPORT std::ostream& operator<<(std::ostream& os, NrUeRrc::State state);

} // namespace ns3

#endif // NR_UE_RRC_H

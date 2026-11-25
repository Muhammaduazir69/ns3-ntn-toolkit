// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef BWP_MANAGER_UE_H
#define BWP_MANAGER_UE_H

#include "nr-phy-mac-common.h"
#include "nr-qos-flow.h"
#include "nr-simple-ue-component-carrier-manager.h"
#include "nr-ue-ccm-rrc-sap.h"

#include "ns3/nr-export.h"

#include <unordered_set>

namespace ns3
{

class BwpManagerAlgorithm;
class NrControlMessage;

/**
 * @ingroup ue-bwp
 * @brief The BwpManagerUe class
 */
class NR_EXPORT BwpManagerUe : public NrSimpleUeComponentCarrierManager
{
  public:
    /**
     * @brief GetTypeId
     * @return the type id for the object
     */
    static TypeId GetTypeId();

    /**
     * @brief BwpManagerUe constructor
     */
    BwpManagerUe();
    /**
     * @brief ~BwpManagerUe
     */
    ~BwpManagerUe() override;

    /**
     * @brief Set the algorithm
     * @param algorithm pointer to the algorithm
     */
    void SetBwpManagerAlgorithm(const Ptr<BwpManagerAlgorithm>& algorithm);

    /**
     * @brief The UE received a HARQ feedback from spectrum. Where this feedback
     * should be forwarded?
     *
     * @param m the feedback
     * @return the BWP index in which the feedback can be transmitted to the gNB.
     */
    uint8_t RouteDlHarqFeedback(const DlHarqInfo& m) const;

    /**
     * @brief Decide the BWP for the control message received.
     * @param msg Message
     * @param sourceBwpArfcn BWP arfcn from which this message come from.
     *
     * The routing is made following the bandwidth part reported in the message.
     *
     * @return the BWP arfcn to which this message should be routed to.
     */
    uint32_t RouteIngoingCtrlMsg(const Ptr<NrControlMessage>& msg, uint32_t sourceBwpArfcn) const;

    /**
     * @brief Route the outgoing messages to the right BWP
     * @param msg the message
     * @param sourceBwpId the source bwp of the message
     *
     * The routing is made by following the mapping provided through the function
     * SetOutputLink. If no mapping has been installed, or if the sourceBwpId
     * provided is not in the mapping, then forward the message back to the
     * originating BWP.
     *
     * @see SetOutputLink
     *
     * @return the bwp to which the ctrl messages should be redirected
     */
    uint8_t RouteOutgoingCtrlMsg(const Ptr<NrControlMessage>& msg, uint8_t sourceBwpId) const;

    /**
     * @brief Set a mapping between two BWP.
     * @param sourceBwp The messages that comes from this value...
     * @param outputBwp ... will get routed in this bandwidth part.
     *
     * Call it for each mapping you want to install.
     *
     * Mappings installed through this method are explicit: they are never
     * overwritten by the automatically derived primary DL/UL BWP pair links
     * (@see SetDefaultOutputLink).
     */
    void SetOutputLink(uint32_t sourceBwp, uint32_t outputBwp);

    /**
     * @brief Set a mapping between two BWP, unless one was explicitly configured.
     * @param sourceBwp The messages that comes from this value...
     * @param outputBwp ... will get routed in this bandwidth part.
     *
     * Used for mappings derived automatically from the primary DL/UL BWP pair,
     * which must not override mappings explicitly installed with SetOutputLink()
     * (e.g., the pairing of secondary FDD carriers in mixed TDD/FDD setups).
     */
    void SetDefaultOutputLink(uint32_t sourceBwp, uint32_t outputBwp);

    /**
     * @brief Remove all output link mappings, explicit or not.
     *
     * Output links express the UL/DL carrier pairing of a serving cell, so
     * they are dropped when the UE moves to a different cell: the new cell's
     * configuration (primary pair sync or dedicated RRC config) installs the
     * links appropriate for it.
     */
    void ClearOutputLinks();

    /**
     * @brief Get the output BWP a source BWP's messages are routed to
     * @param sourceBwp the source BWP
     * @return the output BWP, or sourceBwp itself if no mapping is installed
     */
    uint32_t GetOutputLink(uint32_t sourceBwp) const
    {
        auto it = m_outputLinks.find(sourceBwp);
        return it != m_outputLinks.end() ? it->second : sourceBwp;
    }

    /**
     * @brief Set the BWP serving a 5QI at runtime
     * @param fiveQi the 5QI
     * @param bwpIndex the BWP index serving flows of this 5QI
     *
     * Programs the UE's BWP manager algorithm from the mapping the serving
     * cell advertises over RRC, so no manual UE-side attribute configuration
     * is needed.
     */
    void SetBwpForQosFlow(uint8_t fiveQi, uint8_t bwpIndex);

    /**
     * @brief Set a callback function to retrieve the primary uplink index.
     *
     * This function assigns a callback that is used to obtain the primary
     * uplink index. The callback must be a callable object or function
     * returning a uint8_t value, defining the primary uplink index to be
     * used by the system.
     *
     * @param fn The callback function returning a uint8_t value.
     */
    void SetGetPrimaryUlFn(std::function<uint8_t()> fn);

  protected:
    void DoDispose() override;
    void DoTransmitBufferStatusReport(
        NrMacSapProvider::BufferStatusReportParameters params) override;
    std::vector<NrUeCcmRrcSapProvider::LcsConfig> DoAddLc(
        uint8_t lcId,
        NrUeCmacSapProvider::LogicalChannelConfig lcConfig,
        NrMacSapUser* msu) override;
    NrMacSapUser* DoConfigureSignalBearer(uint8_t lcId,
                                          NrUeCmacSapProvider::LogicalChannelConfig lcConfig,
                                          NrMacSapUser* msu) override;
    /**
     * Allow subclasses to access BwpManagerAlgorithm
     * @return const pointer to the algorithm
     */
    Ptr<const BwpManagerAlgorithm> GetAlgorithm() const;

  private:
    Ptr<BwpManagerAlgorithm> m_algorithm;
    std::unordered_map<uint8_t, NrQosFlow::FiveQi> m_lcToFlowMap; //!< Map from LCID to QoS flow ID

    std::unordered_map<uint32_t, uint32_t> m_outputLinks; //!< Mapping between BWP.
    std::unordered_set<uint32_t>
        m_explicitOutputLinks; //!< Sources of mappings installed with SetOutputLink
    std::function<uint8_t()>
        m_getPrimaryUlFn; //!< Callback to retrieve primary UL BWP index from RRC
};

} // namespace ns3
#endif // BWP_MANAGER_UE_H

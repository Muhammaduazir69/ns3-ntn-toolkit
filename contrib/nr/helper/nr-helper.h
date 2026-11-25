// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NR_HELPER_H
#define NR_HELPER_H

#include "cc-bwp-helper.h"
#include "ideal-beamforming-helper.h"
#include "nr-bearer-stats-connector.h"
#include "nr-epc-helper.h"
#include "nr-mac-scheduling-stats.h"

#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nr-component-carrier.h"
#include "ns3/nr-control-messages.h"
#include "ns3/nr-export.h"
#include "ns3/nr-qos-flow.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-ue-cphy-sap.h"
#include "ns3/object-factory.h"

#include <map>
#include <set>
#include <utility>

namespace ns3
{

class NrUePhy;
class NrGnbPhy;
class SpectrumChannel;
class NrSpectrumValueHelper;
class NrGnbMac;
class NrEpcHelper;
class NrQosRule;
class NrBearerStatsCalculator;
class NrMacRxTrace;
class NrPhyRxTrace;
class NrMacScheduler;
class NrGnbNetDevice;
class NrUeNetDevice;
class NrUeMac;
class BwpManagerGnb;
class BwpManagerUe;
class NrFhControl;
class NodeDistributionScenarioInterface;
class BeamformingHelperBase;

/**
 * @ingroup helper
 * @brief Helper to set up single- or multi-cell scenarios with NR
 *
 * @section helper_pre Pre-requisite: Node creation and placement
 *
 * The `NrHelper` installation API accepts an `ns3::NodeContainer`.
 * Users are advised to create gNB nodes in one or more node containers,
 * and UE nodes in one or more additional node containers, because the
 * installation method is different for gNB and UE nodes. Additionally,
 * any position or mobility models must be installed on the nodes
 * outside of the NrHelper. For simple cases, we provide helpers that
 * position nodes on rectangular and hexagonal grids. Please take a look at
 * the GridScenarioHelper documentation in that case.
 *
 * @section helper_creating Creating the helper
 *
 * The NrHelper inherits from `ns3::Object` and therefore should be created
 * with `CreateObject()`.  The helper should remain in scope until the
 * simulation program ends.  More than one `NrHelper` can be created.
 *
\verbatim
  Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper> ();
\endverbatim
 *
 * @section helper_additional Adding additional helpers
 *
 * The `NrHelper` accepts two other optional helpers, a beamforming helper
 * and an Evolved Packet Core (EPC) helper.  Both of these helpers have
 * different subclasses (the `Ideal` and `Realistic` beamforming helpers,
 * and several EPC helpers that vary on the basis of backhaul technology.
 * The following code shows an example of these additional helpers.
 *
\verbatim
  Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();
  nrHelper->SetBeamformingHelper (idealBeamformingHelper);
  nrHelper->SetEpcHelper (nrEpcHelper);
\endverbatim
 *
 * @section helper_dividing Dividing the spectrum and creating the channels
 *
 * The spectrum management is delegated to the class CcBwpHelper. Please
 * refer to its documentation to have more information. After having divided
 * the spectrum in component carriers and bandwidth part, use the method
 * InitializeOperationBand() to create the channels and other things that will
 * be used by the channel modeling part.
 *
 * @section helper_configuring Configuring the cells
 *
 * After the spectrum part is ready, it is time to check how to configure the
 * cells. The configuration is done in the most typical ns-3 way, through the
 * Attributes. We assume that you already know what an attribute is; if not,
 * please read the ns-3 documentation about objects and attributes.
 *
 *
 * We have two different ways to configure the attributes of our objects
 * (and, therefore, configure the cell). The first is before the object are
 * created: as this class is responsible to create all the object that are needed
 * in the module, we provide many methods (prefixed with "Set") that can
 * store the attribute values for the objects, until they are created and then
 * these values applied. For instance, we can set the antenna dimensions for
 * all the UEs with:
 *
\verbatim
  // Antennas for all the UEs
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (4));
\endverbatim
 *
 * These attributes will be stored, and then used inside the creation methods
 * InstallGnbDevice() and InstallUeDevice(). In this case, the antenna dimensions
 * will be taken inside the InstallUeDevice() method, and all the UEs created
 * will have these dimensions.
 *
 * Of course, it is possible to divide the nodes, and install them separately
 * with different attributes. For example:
 *
\verbatim
  NodeContainer ueFirstSet;
  NodeContainer ueSecondSet;
  ...
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (4));
  ...
  NetDeviceContainer ueFirstSetNetDevs = nrHelper->InstallUeDevice (ueFirstSet, allBwps);
  ...
  // Then, prepare the second set:
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (4));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (8));
  ...
  NetDeviceContainer ueSecondSetNetDevs = nrHelper->InstallUeDevice (ueSecondSet, allBwps);
  ...
\endverbatim
 *
 * In this way, you can configure different sets of nodes with different properties.
 *
 * The second configuration option is setting the attributes after the object are
 * created. Once you have a pointer to the object, you can use the Object::SetAttribute()
 * method on it. To get the pointer, use one of the helper methods to retrieve
 * a pointer to the PHY, MAC, or scheduler, given the index of the bandwidth part:
 *
\verbatim
  ...
  NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice (gnbContainer, allBwps);
  NrHelper::GetGnbPhy (gnbNetDev.Get (0), 0)->SetAttribute ("Numerology", UintegerValue (2));
\endverbatim
 *
 * In the snippet, we are selecting the first gnb (`gnbNetDev.Get (0)`) and
 * then selecting the first BWP (`, 0`). We are using the method GetGnbPhy()
 * to obtain the PHY pointer, and then setting the `Numerology` attribute
 * on it. The list of attributes is present in the class description, as well
 * as some reminder on how configure it through the helper.
 *
 * @section helper_installing Installing UEs and GNBs
 *
 * The installation part is done through two methods: InstallGnbDevice()
 * and InstallUeDevice(). Pass to these methods the container of the nodes,
 * plus the spectrum configuration that comes from CcBwpHelper.
 *
 * @section helper_finishing Finishing the configuration
 *
 * After you finish the configuration, please remember to call UpdateConfig()
 * on all the NetDevices:
 *
\verbatim
  for (auto it = gnbNetDev.Begin (); it != gnbNetDev.End (); ++it)
    {
      DynamicCast<NrGnbNetDevice> (*it)->UpdateConfig ();
    }

  for (auto it = ueNetDev.Begin (); it != ueNetDev.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }
\endverbatim
 *
 * The call to UpdateConfig() will finish the configuration, and update the
 * RRC layer.
 *
 * @section helper_bearers Dedicated QoS flows
 *
 * We have methods to open dedicated UE QosFlows: ActivateDedicatedQosFlow()
 * and DeActivateDedicatedQosFlow(). Please take a look at their documentation
 * for more information.
 *
 * @section helper_attachment Attachment of UEs to GNBs
 *
 * We provide three methods to attach a set of UE to a GNB: AttachToClosestGnb(),
AttachToMaxRsrpGnb(),
 * and AttachToGnb(). Through these function, you will manually attach one or
 * more UEs to a specified GNB.
 *
 * @section helper_Traces Traces
 *
 * We provide a method that enables the generation of files that include among
 * others information related to the received packets at the gNB and UE side,
 * the control messages transmitted and received from/at the gNB and UE side,
 * the SINR, as well as RLC and PDCP statistics such as the packet size.
 * Please refer to their documentation for more information.
 * Enabling the traces is done by simply calling the method `EnableTraces()` in the
 * scenario.
 *
 */
class NR_EXPORT NrHelper : public Object
{
  public:
    /**
     * @brief NrHelper constructor
     */
    NrHelper();
    /**
     * @brief ~NrHelper
     */
    ~NrHelper() override;

    /**
     * @brief GetTypeId
     * @return the type id of the object
     */
    static TypeId GetTypeId();

    /**
     * @brief Install one (or more) UEs
     * @param c Node container with the UEs
     * @param allBwps The spectrum configuration that comes from CcBwpHelper
     * @return a NetDeviceContainer with the net devices that have been installed.
     *
     */
    NetDeviceContainer InstallUeDevice(
        const NodeContainer& c,
        const std::vector<std::reference_wrapper<BandwidthPartInfoPtr>>& allBwps);
    /**
     * @brief Install one (or more) GNBs
     * @param c Node container with the GNB
     * @param allBwps The spectrum configuration that comes from CcBwpHelper
     * @return a NetDeviceContainer with the net devices that have been installed.
     */
    NetDeviceContainer InstallGnbDevice(
        const NodeContainer& c,
        const std::vector<std::reference_wrapper<BandwidthPartInfoPtr>> allBwps);

    /**
     * @brief Get the number of configured BWP for a specific GNB NetDevice
     * @param gnbDevice The GNB NetDevice, obtained from InstallGnbDevice()
     * @return the number of BWP installed, or 0 if there are errors
     */
    static uint32_t GetNumberBwp(const Ptr<const NetDevice>& gnbDevice);
    /**
     * @brief Get a pointer to the PHY of the GNB at the specified BWP
     * @param gnbDevice The GNB NetDevice, obtained from InstallGnbDevice()
     * @param bwpIndex The index of the BWP required
     * @return A pointer to the PHY layer of the GNB, or nullptr if there are errors
     */
    static Ptr<NrGnbPhy> GetGnbPhy(const Ptr<NetDevice>& gnbDevice, uint32_t bwpIndex);
    /**
     * @brief Get a pointer to the MAC of the GNB at the specified BWP
     * @param gnbDevice The GNB NetDevice, obtained from InstallGnbDevice()
     * @param bwpIndex The index of the BWP required
     * @return A pointer to the MAC layer of the GNB, or nullptr if there are errors
     */
    static Ptr<NrGnbMac> GetGnbMac(const Ptr<NetDevice>& gnbDevice, uint32_t bwpIndex);
    /**
     * @brief Get a pointer to the MAC of the UE at the specified BWP
     * @param ueDevice The UE NetDevice, obtained from InstallUeDevice()
     * @param bwpIndex The index of the BWP required
     * @return A pointer to the MAC layer of the UE, or nullptr if there are errors
     */
    static Ptr<NrUeMac> GetUeMac(const Ptr<NetDevice>& ueDevice, uint32_t bwpIndex);
    /**
     * @brief Get a pointer to the PHY of the UE at the specified BWP
     * @param ueDevice The UE NetDevice, obtained from InstallUeDevice()
     * @param bwpIndex The index of the BWP required
     * @return A pointer to the PHY layer of the UE, or nullptr if there are errors
     */
    static Ptr<NrUePhy> GetUePhy(const Ptr<NetDevice>& ueDevice, uint32_t bwpIndex);
    /**
     * @brief Get the BwpManager of the GNB
     * @param gnbDevice the GNB NetDevice, obtained from InstallGnbDevice()
     * @return A pointer to the BwpManager of the GNB, or nullptr if there are errors
     */
    static Ptr<BwpManagerGnb> GetBwpManagerGnb(const Ptr<NetDevice>& gnbDevice);
    /**
     * @brief Get the BwpManager of the UE
     * @param ueDevice the UE NetDevice, obtained from InstallGnbDevice()
     * @return A pointer to the BwpManager of the UE, or nullptr if there are errors
     */
    static Ptr<BwpManagerUe> GetBwpManagerUe(const Ptr<NetDevice>& ueDevice);
    /**
     * @brief Get the Scheduler from the GNB specified
     * @param gnbDevice The GNB NetDevice, obtained from InstallGnbDevice()
     * @param bwpIndex The index of the BWP required
     * @return A pointer to the scheduler, or nullptr if there are errors
     */
    static Ptr<NrMacScheduler> GetScheduler(const Ptr<NetDevice>& gnbDevice, uint32_t bwpIndex);

    /**
     * @brief Attach the UE specified to the max RSRP associated GNB
     * @param ueDevices UE devices to attach
     * @param gnbDevices GNB devices from which the algorithm has to select the RSRP
     */
    void AttachToMaxRsrpGnb(const NetDeviceContainer& ueDevices,
                            const NetDeviceContainer& gnbDevices);
    /**
     * @brief Attach the UE specified to the closest GNB
     * @param ueDevices UE devices to attach
     * @param gnbDevices GNB devices from which the algorithm has to select the closest
     */
    void AttachToClosestGnb(const NetDeviceContainer& ueDevices,
                            const NetDeviceContainer& gnbDevices);
    /**
     * @brief Attach a UE to a particular GNB
     *
     * If the AttachWindow attribute is non-zero the attachment is deferred, spreading
     * successive calls over that window instead of attaching every UE at the same instant.
     * The whole attachment is deferred as one unit, registration and connection trigger
     * together: deferring only the connection trigger would leave the UE processing a random
     * access response for a procedure it has not started yet.
     *
     * @param ueDevice the UE device
     * @param gnbDevice the GNB device to which attach the UE
     */
    void AttachToGnb(const Ptr<NetDevice>& ueDevice, const Ptr<NetDevice>& gnbDevice);

    /**
     * @brief Enables the following traces:
     * Transmitted/Received Control Messages
     * DL/UL Phy Traces
     * RLC traces
     * PDCP traces
     *
     */
    void EnableTraces();

    /**
     * @brief Activate a Data Radio Bearer on a given UE devices
     *
     * @param ueDevices the set of UE devices
     * @param flow the QoS characteristics of the flow to be activated
     */
    void ActivateDataRadioBearer(NetDeviceContainer ueDevices, NrQosFlow flow);
    /**
     * @brief Activate a Data Radio Bearer on a UE device.
     *
     * This method will schedule the actual activation
     * the bearer so that it happens after the UE got connected.
     *
     * @param ueDevice the UE device
     * @param bearer the characteristics of the bearer to be activated
     */
    void ActivateDataRadioBearer(Ptr<NetDevice> ueDevice, NrQosFlow flow);
    /**
     * Set the NrEpcHelper to be used to setup the EPC network in
     * conjunction with the setup of the NR radio access network.
     *
     * @note if no NrEpcHelper is ever set, then NrHelper will default
     * to creating a simulation with no EPC, using NrRlcSm as
     * the RLC model, and without supporting any IP networking. In other
     * words, it will be a radio-level simulation involving only NR PHY
     * and MAC and the Scheduler, with a saturation traffic model for
     * the RLC.
     *
     * Several core networks can be used in a simulation: the gNBs are connected to the core
     * network of the NrEpcHelper set when they are installed, and the UEs are subscribers of the
     * core network of the NrEpcHelper set when they are installed (their home network).
     *
     * @param NrEpcHelper a pointer to the NrEpcHelper to be used
     */
    void SetEpcHelper(Ptr<NrEpcHelper> NrEpcHelper);

    /**
     * @brief Set an ideal beamforming helper
     * @param beamformingHelper a pointer to the beamforming helper
     *
     */
    void SetBeamformingHelper(Ptr<BeamformingHelperBase> beamformingHelper);

    /**
     * @brief SetSnrTest
     * @param snrTest
     *
     * We never really tested this function, so please be careful when using it.
     */
    void SetSnrTest(bool snrTest);
    /**
     * @brief GetSnrTest
     * @return the value of SnrTest variable
     */
    bool GetSnrTest() const;

    /**
     * Activate a dedicated QoS flow on a given set of UE devices.
     *
     * @param ueDevices the set of UE devices
     * @param flow the QoS characteristics of the flow to be activated
     * @param rule the QoS rule that identifies the traffic to go on this flow
     * @returns QoS flow ID
     */
    uint8_t ActivateDedicatedQosFlow(NetDeviceContainer ueDevices,
                                     NrQosFlow flow,
                                     Ptr<NrQosRule> rule);

    /**
     * Activate a dedicated QoS flow on a given UE device.
     *
     * @param ueDevice the UE device
     * @param flow the QoS characteristics of the flow to be activated
     * @param rule the QoS rule that identifies the traffic to go on this flow.
     * @returns QoS flow ID
     */
    uint8_t ActivateDedicatedQosFlow(Ptr<NetDevice> ueDevice, NrQosFlow flow, Ptr<NrQosRule> rule);

    /**
     * Activate an unstructured PDU session on a given UE device, carrying a single
     * network layer protocol whose payload the network does not parse.
     *
     * The session must be activated before the UE connects, and the device through
     * which it reaches the external network is obtained from the EPC helper with
     * GetUnstructuredSessionDevice(), using the returned QFI.
     *
     * @param ueDevice the UE device
     * @param protocolNumber the network layer protocol carried by the session
     * @param flow the QoS characteristics of the flow to be activated
     * @returns QoS flow ID
     */
    uint8_t ActivateUnstructuredQosFlow(Ptr<NetDevice> ueDevice,
                                        uint16_t protocolNumber,
                                        NrQosFlow flow);

    /**
     *  @brief Manually trigger dedicated flow de-activation at specific simulation time
     *  @param ueDevice the UE on which dedicated flow to be de-activated must be of the type
     * NrUeNetDevice
     *  @param gnbDevice gNB, must be of the type NrGnbNetDevice
     *  @param qosFlowId QoS flow ID which is to be de-activated
     *
     *  @warning Requires the use of EPC mode. See SetEpcHelper() method.
     */

    void DeActivateDedicatedQosFlow(Ptr<NetDevice> ueDevice,
                                    Ptr<NetDevice> gnbDevice,
                                    uint8_t qosFlowId);

    /**
     * @brief Set an attribute for the UE MAC, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     * @see NrUeMac
     */
    void SetUeMacAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set an attribute for the GNB MAC, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     * @see NrGnbMac
     */
    void SetGnbMacAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set a different TypeId for the UE antenna device
     * @param typeId the antenna TypeId
     */
    void SetUeAntennaTypeId(const std::string&);

    /**
     * @brief Set a different TypeId for the GNB antenna device
     * @param typeId the antenna TypeId
     */
    void SetGnbAntennaTypeId(const std::string&);

    /**
     * @brief Set an attribute for the GNB spectrum, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     * @see NrSpectrumPhy
     */
    void SetGnbSpectrumAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set an attribute for the UE spectrum, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     * @see NrSpectrumPhy
     */
    void SetUeSpectrumAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set an attribute for the UE channel access manager, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     *
     * @see NrChAccessManager
     */
    void SetUeChannelAccessManagerAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set an attribute for the GNB channel access manager, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     *
     * @see NrChAccessManager
     */
    void SetGnbChannelAccessManagerAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set an attribute for the scheduler, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     * @see NrMacSchedulerNs3
     */
    void SetSchedulerAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set an attribute for the UE PHY, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     *
     * @see NrUePhy
     */
    void SetUePhyAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set an attribute for the GNB PHY, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     *
     * @see NrGnbPhy
     */
    void SetGnbPhyAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set an attribute for the UE antenna, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     *
     * @see UniformPlanarArray (in ns-3 documentation)
     */
    void SetUeAntennaAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set an attribute for the GNB antenna, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     *
     * @see UniformPlanarArray (in ns-3 documentation)
     */
    void SetGnbAntennaAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set the TypeId of the UE Channel Access Manager. Works only before it is created.
     *
     * @param typeId the type of the object
     *
     * @see NrChAccessManager
     * @see NrAlwaysOnAccessManager
     */
    void SetUeChannelAccessManagerTypeId(const TypeId& typeId);

    /**
     * @brief Set the TypeId of the GNB Channel Access Manager. Works only before it is created.
     *
     * @param typeId the type of the object
     *
     * @see NrChAccessManager
     * @see NrAlwaysOnAccessManager
     */
    void SetGnbChannelAccessManagerTypeId(const TypeId& typeId);

    /**
     * @brief Set the Scheduler TypeId. Works only before it is created.
     * @param typeId The scheduler type
     *
     * @see NrMacSchedulerOfdmaPF
     * @see NrMacSchedulerOfdmaRR
     * @see NrMacSchedulerOfdmaMR
     * @see NrMacSchedulerTdmaPF
     * @see NrMacSchedulerTdmaRR
     * @see NrMacSchedulerTdmaMR
     */
    void SetSchedulerTypeId(const TypeId& typeId);

    /**
     * @brief Use another scheduler type on one BWP (ntn-patch 17). Under paired
     * FDD each BWP carries one direction and has its own scheduler, so the UL
     * BWP can run an OFDMA scheduler (FDM of power-limited UEs in a symbol)
     * while the DL BWP keeps the default. The attributes set with
     * SetSchedulerAttribute apply to every BWP. Works only before the devices
     * are installed.
     * @param bwpId the BWP index on the gNB device
     * @param typeId the scheduler type for that BWP
     */
    void SetSchedulerTypeIdForBwp(uint16_t bwpId, const TypeId& typeId);

    /**
     * @brief Set the TypeId of the GNB BWP Manager. Works only before it is created.
     * @param typeId Type of the object
     *
     * @see BwpManagerAlgorithm
     */
    void SetGnbBwpManagerAlgorithmTypeId(const TypeId& typeId);

    /**
     * @brief Set an attribute for the GNB BWP Manager, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     */
    void SetGnbBwpManagerAlgorithmAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set the TypeId of the UE BWP Manager. Works only before it is created.
     * @param typeId Type of the object
     *
     * @see BwpManagerAlgorithm
     */
    void SetUeBwpManagerAlgorithmTypeId(const TypeId& typeId);

    /**
     * @brief Set an attribute for the GNB BWP Manager, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     */
    void SetUeBwpManagerAlgorithmAttribute(const std::string& n, const AttributeValue& v);

    /**
     * Set an attribute for the GNB DL AMC, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     *
     * @see NrAmc
     */
    void SetGnbDlAmcAttribute(const std::string& n, const AttributeValue& v);

    /**
     * Set an attribute for the GNB UL AMC, before it is created.
     *
     * @param n the name of the attribute
     * @param v the value of the attribute
     *
     * @see NrAmc
     */
    void SetGnbUlAmcAttribute(const std::string& n, const AttributeValue& v);

    /*
     * @brief Sets beam managers attribute.
     * @param n the name of the attribute
     * @param v the value of the attribute
     */
    void SetGnbBeamManagerAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Set the TypeId of the beam manager
     * @param typeId the type of the object
     *
     */
    void SetGnbBeamManagerTypeId(const TypeId& typeId);

    /**
     * @brief Set the ErrorModel for UL AMC and UE spectrum at the same time
     * @param errorModelTypeId The TypeId of the error model
     *
     * Equivalent to the calls to
     *
     * * SetGnbUlAmcAttribute ("ErrorModelType", ....
     * * SetGnbSpectrumAttribute ("ErrorModelType", ...
     *
     * @see NrErrorModel
     * @see NrEesmIrT2
     * @see NrEesmIrT1
     * @see NrEesmCcT1
     * @see NrEesmCcT2
     * @see NrLteMiErrorModel
     *
     */
    void SetUlErrorModel(const std::string& errorModelTypeId);

    /**
     * @brief Set the ErrorModel for DL AMC and GNB spectrum at the same time
     * @param errorModelTypeId The TypeId of the error model
     *
     * Equivalent to the calls to
     *
     * * SetGnbDlAmcAttribute ("ErrorModelType", ....
     * * SetUeSpectrumAttribute ("ErrorModelType", ...
     *
     * @see NrErrorModel
     * @see NrEesmIrT2
     * @see NrEesmIrT1
     * @see NrEesmCcT1
     * @see NrEesmCcT2
     * @see NrLteMiErrorModel
     */
    void SetDlErrorModel(const std::string& errorModelTypeId);

    /**
     * @brief Enable FH Control
     *
     * If enabled, the MAC scheduler operation is constrained by the
     * FH Capacity.
     */
    void EnableFhControl();

    /**
     * @brief Configure FH Control of each cell
     *
     * It sets the numerology as configured in the BWP to which this
     * FH Control instance belongs to.
     *
     * @param gnbNetDevices The gNB Net Devices for which we want
     *        to configure the FH Capacity Control.
     */
    void ConfigureFhControl(NetDeviceContainer gnbNetDevices);

    /*
     * @brief Sets the FH Control attributes.
     * @param n the name of the attribute
     * @param v the value of the attribute
     */
    void SetFhControlAttribute(const std::string& n, const AttributeValue& v);

    /**
     * @brief Enable DL DATA PHY traces
     */
    void EnableDlDataPhyTraces();

    /**
     * @brief Enable DL CTRL PHY traces
     */
    void EnableDlCtrlPhyTraces();

    /**
     * @brief Enable UL PHY traces
     */
    void EnableUlPhyTraces();

    /**
     * @brief Get the phy traces object
     *
     * Creates the NrPhyRxTrace object upon first call.
     *
     * @return The NrPhyRxTrace object to write PHY traces
     */
    Ptr<NrPhyRxTrace> GetPhyRxTrace();

    /**
     * @brief Get the mac stats trace object
     *
     * Creates the NrMacRxTrace object upon first call.
     *
     * @return The NrMacRxTrace object to write MAC traces
     */
    Ptr<NrMacRxTrace> GetMacRxTrace();

    /**
     * @brief Enable gNB packet count trace
     */
    void EnableGnbPacketCountTrace();

    /**
     * @brief Enable UE packet count trace
     *
     */
    void EnableUePacketCountTrace();

    /**
     * @brief Enable transport block trace
     *
     * At the time of writing this documentation
     * this method only connect the ReportDownlinkTbSize
     * of NrUePhy.
     */
    void EnableTransportBlockTrace();

    /**
     * @brief Enable gNB PHY CTRL TX and RX traces
     */
    void EnableGnbPhyCtrlMsgsTraces();

    /**
     * @brief Enable UE PHY CTRL TX and RX traces
     */
    void EnableUePhyCtrlMsgsTraces();

    /**
     * @brief Enable gNB MAC CTRL TX and RX traces
     */
    void EnableGnbMacCtrlMsgsTraces();

    /**
     * @brief Enable UE MAC CTRL TX and RX traces
     */
    void EnableUeMacCtrlMsgsTraces();

    /**
     * @brief Get the RLC stats calculator object
     *
     * @return The NrBearerStatsCalculator stats calculator object to write RLC traces
     */
    Ptr<NrBearerStatsCalculator> GetRlcStatsCalculator();

    /**
     * @brief Enable RLC simple traces (DL RLC TX, DL RLC RX, UL DL TX, UL DL RX)
     */
    void EnableRlcSimpleTraces();

    /**
     * @brief Enable PDCP traces (DL PDCP TX, DL PDCP RX, UL PDCP TX, UL PDCP RX)
     */
    void EnablePdcpSimpleTraces();

    /**
     * @brief Enable RLC calculator and end-to-end RCL traces to file
     */
    void EnableRlcE2eTraces();

    /**
     * @brief Enable PDCP calculator and end-to-end PDCP traces to file
     */
    void EnablePdcpE2eTraces();

    /**
     * @brief Get the PDCP stats calculator object
     *
     * @return The NrBearerStatsCalculator stats calculator object to write PDCP traces
     */
    Ptr<NrBearerStatsCalculator> GetPdcpStatsCalculator();

    /**
     * Enable trace sinks for DL MAC layer scheduling.
     */
    void EnableDlMacSchedTraces();

    /**
     * Enable trace sinks for UL MAC layer scheduling.
     */
    void EnableUlMacSchedTraces();

    /**
     * @brief Enable trace sinks for DL and UL pathloss
     */
    void EnablePathlossTraces();

    /*
     * @brief Enable DL CTRL pathloss trace from a serving cell (this trace connects
     * to NrSpectrumPhy trace, which is implementation wise different from
     * EnablePathlossTrace function which is implemented in by using
     * MultiModelSpectrumChannel trace and which
     * generates pathloss traces for both DL and UL
     */
    void EnableDlCtrlPathlossTraces(NetDeviceContainer& netDeviceContainer);

    /*
     * @brief Enable DL CTRL pathloss trace from a serving cell (this trace connects
     * to NrSpectrumPhy trace, which is implementation wise different from
     * EnablePathlossTrace function which is implemented in by using
     * MultiModelSpectrumChannel trace and which generates pathloss traces for
     * both DL and UL)
     */
    void EnableDlDataPathlossTraces(NetDeviceContainer& netDeviceContainer);

    /**
     * Assign a fixed random variable stream number to the random variables used.
     *
     * The InstallGnbDevice() or InstallUeDevice method should have previously
     * been called by the user on the given devices.
     *
     *
     * @param c NetDeviceContainer of the set of net devices for which the
     *          NrNetDevice should be modified to use a fixed stream
     * @param stream first stream index to use
     * @return the number of stream indices (possibly zero) that have been assigned
     */
    int64_t AssignStreams(NetDeviceContainer c, int64_t stream);

    /**
     * @name Recommended RNG stream base offsets
     *
     * Default first-stream index for each kind of simulation object handled by
     * AssignStreams(const StreamAssignmentParams&). The offsets are spaced far
     * apart so that the (much smaller) blocks of streams consumed by distinct
     * objects never overlap, regardless of scenario size. These are the values
     * historically replicated across the nr tests and examples, now defined in
     * a single place.
     * @{
     */
    static constexpr int64_t EPC_STREAM_BASE = 0;            //!< EPC core nodes
    static constexpr int64_t REMOTE_HOST_STREAM_BASE = 1000; //!< Remote host internet stacks
    static constexpr int64_t UE_NODE_STREAM_BASE = 2000;     //!< UE node internet stacks
    static constexpr int64_t GNB_NODE_STREAM_BASE = 3000;    //!< gNB node internet stacks
    static constexpr int64_t GNB_DEV_STREAM_BASE = 5000;     //!< gNB NR net devices
    static constexpr int64_t UE_DEV_STREAM_BASE = 6000;      //!< UE NR net devices
    static constexpr int64_t SCENARIO_STREAM_BASE = 8000;    //!< Scenario/mobility models
    static constexpr int64_t BEAMFORMING_STREAM_BASE = 9000; //!< Beamforming helper
    /// Reserved base for the per-UE initial association RNG (offset by the UE node id);
    /// kept far above the other offsets so it never collides with them.
    static constexpr int64_t INIT_ASSOC_STREAM_BASE = 0x10000000;

    /** @} */

    /**
     * @brief Parameters describing which simulation objects should receive a
     *        deterministic RNG stream assignment, and the base stream for each.
     *
     * This is the single point of entry for assigning RNG streams in an nr
     * simulation. Every member is optional: empty containers, null pointers and
     * false flags are skipped, so a caller passes only the objects it actually
     * created. Each object is assigned a contiguous block of streams starting at
     * its (overridable) base offset; the defaults follow the recommended
     * large-gap scheme so that distinct objects never share a stream.
     *
     * @see NrHelper::AssignStreams(const StreamAssignmentParams&)
     */
    struct NR_EXPORT StreamAssignmentParams
    {
        // Objects to assign streams to (optional).
        bool assignEpc{false}; //!< also assign streams to the EPC helper set on this NrHelper
        NodeContainer remoteHostNodes{}; //!< remote host nodes whose internet stacks get streams
        NodeContainer ueNodes{};         //!< UE nodes whose internet stacks get streams
        NodeContainer gnbNodes{};        //!< gNB nodes whose internet stacks get streams
        //!< scenario/mobility helper (a stack object, hence a raw, non-owning pointer)
        NodeDistributionScenarioInterface* scenario{nullptr};
        Ptr<BeamformingHelperBase> beamformingHelper{nullptr}; //!< beamforming helper
        NetDeviceContainer gnbDevs{}; //!< gNB NR net devices (PHY, scheduler, channel objects)
        NetDeviceContainer ueDevs{};  //!< UE NR net devices

        // First stream index used for each object (defaults to the recommended scheme).
        int64_t epcStream{EPC_STREAM_BASE};                 //!< base stream for the EPC helper
        int64_t remoteHostStream{REMOTE_HOST_STREAM_BASE};  //!< base stream for remote host stacks
        int64_t ueNodeStream{UE_NODE_STREAM_BASE};          //!< base stream for UE node stacks
        int64_t gnbNodeStream{GNB_NODE_STREAM_BASE};        //!< base stream for gNB node stacks
        int64_t scenarioStream{SCENARIO_STREAM_BASE};       //!< base stream for the scenario
        int64_t beamformingStream{BEAMFORMING_STREAM_BASE}; //!< base stream for beamforming
        int64_t gnbDevStream{GNB_DEV_STREAM_BASE};          //!< base stream for gNB net devices
        int64_t ueDevStream{UE_DEV_STREAM_BASE};            //!< base stream for UE net devices
    };

    /**
     * @brief Assign deterministic RNG streams to all objects in a simulation.
     *
     * Single point of entry that replaces the repeated sequences of
     * AssignStreams() calls previously scattered across nr tests and examples.
     * Only the objects present in @p params are touched; each is assigned a
     * block of streams starting at its configured base offset (see
     * StreamAssignmentParams). Assigning the EPC helper requires that one has
     * been set via SetEpcHelper().
     *
     * @param params the objects to assign streams to and their base offsets
     * @return the total number of stream indices that have been assigned
     */
    int64_t AssignStreams(const StreamAssignmentParams& params);

    /// @brief parameters of the gNB or UE antenna arrays
    struct NR_EXPORT AntennaParams
    {
        std::string antennaElem{"ns3::IsotropicAntennaModel"}; ///< Antenna type
        size_t nAntCols{1};          ///< Number of antenna element columns (horizontal width)
        size_t nAntRows{1};          ///< Number of antenna element rows (vertical height)
        bool isDualPolarized{false}; ///< true if antennas are cross-polarized (dual-polarized)
        size_t nHorizPorts{1};       ///< Number of antenna ports in horizontal direction
        size_t nVertPorts{1};        ///< Number of antenna ports in vertical direction
        double bearingAngle{0.0};    ///< Bearing angle in radians
        double polSlantAngle{0.0};   ///< Polarization slant angle in radians
        double downtiltAngle{0.0};   ///<  Downtilt angle in radians
    };

    /// @brief parameters for the search of optimal rank and precoding matrix indicator (RI, PMI)
    struct NR_EXPORT MimoPmiParams
    {
        std::string pmSearchMethod{"ns3::NrPmSearchFull"}; ///< Precoding matrix search algorithm
        std::string fullSearchCb{"ns3::NrCbTwoPort"}; ///< Codebook when using full-search algorithm
        uint8_t rankLimit{UINT8_MAX}; ///< Limit maximum MIMO rank to model limited UE capabilities.
        /// Limits the selection of ranks determined by SVD decomposition.
        double rankThreshold{0.0};
        /// Select technique that determines ranks in non-exhaustive search.
        std::string rankTechnique{"Sasaoka"};
        uint8_t subbandSize{1}; ///< Number of PRBs per subband for downsampling
        std::string downsamplingTechnique{"FirstPRB"}; ///< Sub-band compression technique
    };

    /// @brief Parameters for initial attachment association
    struct NR_EXPORT InitialAssocParams
    {
        std::vector<double> rowAngles{0, 90}; ///< vector of angles to set in initial assocc
        std::vector<double> colAngles{0, 90}; ///< vector of angles to set in initial assocc
        double handoffMargin{0};              ///< Handoff margin for Initial assocc
        uint8_t primaryCarrierIndex{0};       ///< primary carrier index for Initial assocc
    };

    /// @brief Set TypeId of the precoding matrix search algorithm
    /// @param typeId Class TypeId
    void SetPmSearchTypeId(const TypeId& typeId);

    /// @brief Set attribute of the precoding matrix search algorithm
    /// @param name attribute to set
    /// @param value value of the attribute
    void SetPmSearchAttribute(const std::string& name, const AttributeValue& value);

    /// @brief Set TypeId of the initial attachment algorithm
    /// @param typeId Class TypeId
    void SetInitialAssocTypeId(const TypeId& typeId);

    /// @brief Set attribute of the initial attachment algorithm
    /// @param name attribute to set
    /// @param value value of the attribute
    void SetInitialAssocAttribute(const std::string& name, const AttributeValue& value);

    /// @brief Set parameters for gNB and UE antenna arrays
    /// @param ap the struct with antenna parameters
    void SetupGnbAntennas(const AntennaParams& ap);

    /// @brief Set parameters for gNB and UE antenna arrays
    /// @param ap the struct with antenna parameters
    void SetupUeAntennas(const AntennaParams& ap);

    /// @brief Set parameters for PMI search in MIMO operation
    /// @param mp the struct with MIMO PMI parameters
    void SetupMimoPmi(const MimoPmiParams& mp);

    /// @brief Set parameters for max RSRP based Initial Association
    /// @param params the struct with initial association parameters
    void SetupInitialAssoc(const InitialAssocParams& params);

    /// @brief Create BandwidthParts from a vector of band configurations
    /// @param bandConfs the vector with operation band configurations
    /// @param scenario the scenario to be used to create the bandwidth parts
    /// @param channelCondition the channel condition to be used to create the bandwidth parts
    /// @param channelModel the channel model to be used to create the bandwidth parts
    /// @return a pair with total bandwidth and vector of bandwidth parts
    std::pair<double, BandwidthPartInfoPtrVector> CreateBandwidthParts(
        std::vector<CcBwpCreator::SimpleOperationBandConf> bandConfs,
        const std::string& scenario = "RMa",
        const std::string& channelCondition = "Default",
        const std::string& channelModel = "ThreeGpp");

    /**
     * @brief Update NetDevice configuration of one or more devices
     *
     * This method finishes cell configuration in the RRC once PHY
     * configuration is finished.
     *
     * After NrHelper::Install() is called on gNB nodes, either this method
     * or the NrGnbNetDevice::UpdateConfig() method must be called exactly
     * once, @b after any post-install PHY configuration is done (if any),
     * and @b before any call is made (if any) to attach UEs to gNBs
     * such as AttachToGnb() and AttachToClosestGnb().
     *
     * This method will assert if called twice on the same container.
     *
     * This method will cause a deprecation warning to be emitted if
     * called on NrUeNetDevice types, and will have no effect on other
     * ns-3 NetDevice types.
     *
     * @param netDevs NetDevice container with the gNBs
     *
     * This method is deprecated and no longer needed and will be removed
     * from future versions of this helper.
     */
    NS_DEPRECATED("Obsolete method")
    void UpdateDeviceConfigs(const NetDeviceContainer& netDevs);

    // Handover related functions
    void AddX2Interface(NodeContainer gnbNodes);
    void AddX2Interface(Ptr<Node> gnbNode1, Ptr<Node> gnbNode2);

    std::string GetHandoverAlgorithmType() const;
    void SetHandoverAlgorithmType(std::string type);
    void SetHandoverAlgorithmAttribute(std::string n, const AttributeValue& v);
    void HandoverRequest(Time hoTime,
                         Ptr<NetDevice> ueDev,
                         Ptr<NetDevice> sourceGnbDev,
                         Ptr<NetDevice> targetGnbDev);
    void HandoverRequest(Time hoTime,
                         Ptr<NetDevice> ueDev,
                         Ptr<NetDevice> sourceGnbDev,
                         uint16_t targetCellId);

  private:
    bool IsMimoFeedbackEnabled() const; ///< Let UE compute MIMO feedback with PMI and RI
    ObjectFactory m_pmSearchFactory;    ///< Factory for precoding matrix search algorithm
    uint8_t m_csiFeedbackFlags{
        CQI_PDSCH_SISO}; //!< CSI feedback flags that indicate whether for CSI feedback is used
                         //!< CSI-RS, CSI-IM, PDSCH MIMO, or only PDSCH SISO.
    /**
     * Assign a fixed random variable stream number to the channel and propagation
     * objects. This function will save the objects to which it has assigned stream
     * to not overwrite assignment, because these objects are shared by gNB and UE
     * devices.
     *
     * The InstallGnbDevice() or InstallUeDevice method should have previously
     * been called by the user on the given devices.
     *
     *
     * @param c NetDeviceContainer of the set of net devices for which the
     *          NrNetDevice should be modified to use a fixed stream
     * @param stream first stream index to use
     * @return the number of stream indices (possibly zero) that have been assigned
     */
    int64_t DoAssignStreamsToChannelObjects(Ptr<NrSpectrumPhy> phy, int64_t currentStream);

    /**
     *  @brief The actual function to trigger a manual flow de-activation
     *  @param ueDevice the UE on which flow to be de-activated must be of the type NrUeNetDevice
     *  @param gnbDevice gNB, must be of the type NrGnbNetDevice
     *  @param qosFlowId QoS flow ID which is to be de-activated
     *
     *  This method is normally scheduled by DeActivateDedicatedQosFlow() to run at a specific
     *  time when a manual flow de-activation is desired by the simulation user.
     */
    void DoDeActivateDedicatedQosFlow(Ptr<NetDevice> ueDevice,
                                      Ptr<NetDevice> gnbDevice,
                                      uint8_t qosFlowId);

    Ptr<NrGnbPhy> CreateGnbPhy(const Ptr<Node>& n,
                               const BandwidthPartInfoPtr& bwp,
                               const Ptr<NrGnbNetDevice>& dev,
                               const NrSpectrumPhy::NrPhyRxCtrlEndOkCallback& phyEndCtrlCallback);
    Ptr<NrMacScheduler> CreateGnbSched(uint16_t bwpId = UINT16_MAX);
    Ptr<NrGnbMac> CreateGnbMac();
    Ptr<NrFhControl> CreateNrFhControl();

    Ptr<NrUeMac> CreateUeMac() const;
    Ptr<NrUePhy> CreateUePhy(const Ptr<Node>& n,
                             const BandwidthPartInfoPtr& bwp,
                             const Ptr<NrUeNetDevice>& dev,
                             const NrSpectrumPhy::NrPhyDlHarqFeedbackCallback& dlHarqCallback,
                             const NrSpectrumPhy::NrPhyRxCtrlEndOkCallback& phyRxCtrlCallback);

    Ptr<NetDevice> InstallSingleUeDevice(
        const Ptr<Node>& n,
        const std::vector<std::reference_wrapper<BandwidthPartInfoPtr>> allBwps);
    Ptr<NetDevice> InstallSingleGnbDevice(
        const Ptr<Node>& n,
        const std::vector<std::reference_wrapper<BandwidthPartInfoPtr>> allBwps);

    void DoHandoverRequest(Ptr<NetDevice> ueDev,
                           Ptr<NetDevice> sourceGnbDev,
                           uint16_t targetCellId);
    void AttachToClosestGnb(const Ptr<NetDevice>& ueDevice, const NetDeviceContainer& gnbDevices);

    void AttachToMaxRsrpGnb(const Ptr<NetDevice>& ueDevice, const NetDeviceContainer& gnbDevices);

    /**
     * @brief Perform the actual attachment of a UE to a GNB.
     *
     * Holds the body of AttachToGnb(), which either calls this directly or defers it to
     * implement the AttachWindow staggering.
     *
     * @param ueDevice the UE device
     * @param gnbDevice the GNB device to which attach the UE
     */
    void DoAttachToGnb(const Ptr<NetDevice>& ueDevice, const Ptr<NetDevice>& gnbDevice);

    /**
     * @brief Register a UE at the gNB that serves it: the gNB PHYs (CSI-RS and beam
     *        selection), the UE PMI search, and the beamforming task of the pair.
     *
     * @param ueNetDev the UE device
     * @param gnbNetDev the gNB device that serves the UE
     */
    void RegisterUeAtGnb(const Ptr<NrUeNetDevice>& ueNetDev, const Ptr<NrGnbNetDevice>& gnbNetDev);

    /**
     * @brief Move the registration of a UE to the cell to which its RRC connected.
     *
     * Connected to the ConnectionEstablished and HandoverEndOk traces of the UE RRCs.
     * After a radio link failure or a handover, a UE connects to a cell without
     * AttachToGnb(); without a registration at its new gNB, it would get neither CSI-RS
     * nor a beam from it.
     *
     * @param imsi the IMSI of the UE
     * @param cellId the cell to which the UE connected
     * @param rnti the RNTI of the UE in that cell
     */
    void UeConnectedToCell(uint64_t imsi, uint16_t cellId, uint16_t rnti);

    std::map<uint16_t, Ptr<NrGnbNetDevice>> m_gnbDevices; //!< gNB devices of this helper by cell
    std::map<uint64_t, Ptr<NrUeNetDevice>> m_ueDevices;   //!< UE devices of this helper by IMSI
    /// gNB device at which each UE, by IMSI, is registered
    std::map<uint64_t, Ptr<NrGnbNetDevice>> m_ueRegistrations;
    /// Pairs of gNB and UE devices with a beamforming task
    std::set<std::pair<Ptr<NrGnbNetDevice>, Ptr<NrUeNetDevice>>> m_beamformingTasks;

    ObjectFactory m_gnbNetDeviceFactory;            //!< NetDevice factory for gnb
    ObjectFactory m_ueNetDeviceFactory;             //!< NetDevice factory for ue
    ObjectFactory m_channelFactory;                 //!< Channel factory
    ObjectFactory m_ueMacFactory;                   //!< UE MAC factory
    ObjectFactory m_gnbMacFactory;                  //!< GNB MAC factory
    ObjectFactory m_ueSpectrumFactory;              //!< UE Spectrum factory
    ObjectFactory m_gnbSpectrumFactory;             //!< GNB spectrum factory
    ObjectFactory m_uePhyFactory;                   //!< UE PHY factory
    ObjectFactory m_gnbPhyFactory;                  //!< GNB PHY factory
    ObjectFactory m_ueChannelAccessManagerFactory;  //!< UE Channel access manager factory
    ObjectFactory m_gnbChannelAccessManagerFactory; //!< GNB Channel access manager factory
    ObjectFactory m_schedFactory;                   //!< Scheduler factory
    std::map<uint16_t, TypeId> m_schedTypeIdPerBwp; //!< per-BWP scheduler type (ntn-patch 17)
    ObjectFactory m_ueAntennaFactory;               //!< UE antenna factory
    ObjectFactory m_gnbAntennaFactory;              //!< UE antenna factory
    ObjectFactory m_gnbBwpManagerAlgoFactory;       //!< BWP manager algorithm factory
    ObjectFactory m_ueBwpManagerAlgoFactory;        //!< BWP manager algorithm factory
    ObjectFactory m_channelConditionModelFactory;   //!< Channel condition factory
    ObjectFactory m_spectrumPropagationFactory;     //!< Spectrum Factory
    ObjectFactory m_pathlossModelFactory;           //!< Pathloss factory
    ObjectFactory m_gnbDlAmcFactory;                //!< DL AMC factory
    ObjectFactory m_gnbUlAmcFactory;                //!< UL AMC factory
    ObjectFactory m_gnbBeamManagerFactory;          //!< gNb Beam manager factory
    ObjectFactory m_ueBeamManagerFactory;           //!< UE beam manager factory
    ObjectFactory m_handoverAlgorithmFactory;       //!< Handover algorithm factory
    ObjectFactory m_fhControlFactory;
    ObjectFactory m_initialAttachmentFactory; //!< initial attachment factory

    uint16_t m_cellIdCounter{1}; //!< CellId Counter

    Ptr<NrEpcHelper> m_nrEpcHelper{nullptr}; //!< Ptr to the EPC helper (optional)
    /// EPC helper of the home network of each UE device, set when the UE is installed
    std::map<Ptr<NetDevice>, Ptr<NrEpcHelper>> m_ueEpcHelpers;
    Ptr<BeamformingHelperBase> m_beamformingHelper{nullptr}; //!< Ptr to the beamforming helper

    bool m_snrTest{false};
    bool m_fhEnabled{false};

    Ptr<NrPhyRxTrace> m_phyStats; //!< Pointer to the PhyRx stats
    Ptr<NrMacRxTrace> m_macStats; //!< Pointer to the MacRx stats

    NrBearerStatsConnector
        m_radioBearerStatsConnectorSimpleTraces; //!< RLC and PDCP statistics connector for simple
                                                 //!< file statistics
    NrBearerStatsConnector
        m_radioBearerStatsConnectorCalculator; //!< RLC and PDCP statistics connector for complex
                                               //!< calculator statistics

    std::map<uint8_t, NrComponentCarrier> m_componentCarrierPhyParams; //!< component carrier map
    std::vector<Ptr<Object>>
        m_channelObjectsWithAssignedStreams; //!< channel and propagation objects to which NrHelper
    //!< has assigned streams in order to avoid double
    //!< assignments
    Ptr<NrMacSchedulingStats> m_macSchedStats; //!<< Pointer to NrMacStatsCalculator
    bool m_useIdealRrc;
    std::vector<OperationBandInfo> m_bands;

    InitialAssocParams
        m_initialParams; //!<< Initial attachment parameters to pass from example to setup

    double m_rbOverhead;   //!< Overhead used by gNBs and UEs when calculating the usable RB number
    uint8_t m_numRbPerRbg; //!< The standard determines this via bandwidth and numerology. We are
                           //!< not prepared to handle that just yet
    Time m_attachWindow;   //!< If non-zero, spread initial UE attachment over this window
    uint32_t m_attachCount{0}; //!< Number of AttachToGnb() calls so far, indexes the staggering
};

} // namespace ns3

#endif /* NR_HELPER_H */

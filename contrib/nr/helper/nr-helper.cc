// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-helper.h"

#include "node-distribution-scenario-interface.h"
#include "nr-bearer-stats-calculator.h"
#include "nr-channel-helper.h"
#include "nr-epc-helper.h"
#include "nr-mac-rx-trace.h"
#include "nr-phy-rx-trace.h"
#include "realistic-beamforming-helper.h"

#include "ns3/bandwidth-part-gnb.h"
#include "ns3/bandwidth-part-ue.h"
#include "ns3/beam-manager.h"
#include "ns3/buildings-channel-condition-model.h"
#include "ns3/bwp-manager-algorithm.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/config.h"
#include "ns3/deprecated.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/names.h"
#include "ns3/nr-ch-access-manager.h"
#include "ns3/nr-chunk-processor.h"
#include "ns3/nr-epc-ue-nas.h"
#include "ns3/nr-fh-control.h"
#include "ns3/nr-gnb-mac.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-handover-algorithm.h"
#include "ns3/nr-initial-association.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-pm-search-full.h"
#include "ns3/nr-rrc-protocol-ideal.h"
#include "ns3/nr-rrc-protocol-real-gnb.h"
#include "ns3/nr-rrc-protocol-real-ue.h"
#include "ns3/nr-ue-mac.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/nr-wraparound-utils.h"
#include "ns3/nyu-channel-model.h"
#include "ns3/nyu-propagation-loss-model.h"
#include "ns3/nyu-spectrum-propagation-loss-model.h"
#include "ns3/pointer.h"
#include "ns3/spectrum-channel.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/three-gpp-v2v-channel-condition-model.h"
#include "ns3/three-gpp-v2v-propagation-loss-model.h"
#include "ns3/two-ray-spectrum-propagation-loss-model.h"
#include "ns3/uniform-planar-array.h"

#include <algorithm>

namespace ns3
{

/* ... */
NS_LOG_COMPONENT_DEFINE("NrHelper");

NS_OBJECT_ENSURE_REGISTERED(NrHelper);

NrHelper::NrHelper()
{
    NS_LOG_FUNCTION(this);
    m_channelFactory.SetTypeId(MultiModelSpectrumChannel::GetTypeId());
    m_gnbNetDeviceFactory.SetTypeId(NrGnbNetDevice::GetTypeId());
    m_ueNetDeviceFactory.SetTypeId(NrUeNetDevice::GetTypeId());
    m_ueMacFactory.SetTypeId(NrUeMac::GetTypeId());
    m_gnbMacFactory.SetTypeId(NrGnbMac::GetTypeId());
    m_ueSpectrumFactory.SetTypeId(NrSpectrumPhy::GetTypeId());
    m_gnbSpectrumFactory.SetTypeId(NrSpectrumPhy::GetTypeId());
    m_uePhyFactory.SetTypeId(NrUePhy::GetTypeId());
    m_gnbPhyFactory.SetTypeId(NrGnbPhy::GetTypeId());
    m_ueChannelAccessManagerFactory.SetTypeId(NrAlwaysOnAccessManager::GetTypeId());
    m_gnbChannelAccessManagerFactory.SetTypeId(NrAlwaysOnAccessManager::GetTypeId());
    m_schedFactory.SetTypeId(NrMacSchedulerTdmaRR::GetTypeId());
    m_ueAntennaFactory.SetTypeId(UniformPlanarArray::GetTypeId());
    m_gnbAntennaFactory.SetTypeId(UniformPlanarArray::GetTypeId());
    m_gnbBwpManagerAlgoFactory.SetTypeId(BwpManagerAlgorithmStatic::GetTypeId());
    m_ueBwpManagerAlgoFactory.SetTypeId(BwpManagerAlgorithmStatic::GetTypeId());
    m_gnbUlAmcFactory.SetTypeId(NrAmc::GetTypeId());
    m_gnbDlAmcFactory.SetTypeId(NrAmc::GetTypeId());
    m_gnbBeamManagerFactory.SetTypeId(BeamManager::GetTypeId());
    m_ueBeamManagerFactory.SetTypeId(BeamManager::GetTypeId());
    m_spectrumPropagationFactory.SetTypeId(ThreeGppSpectrumPropagationLossModel::GetTypeId());
    m_initialAttachmentFactory.SetTypeId(NrInitialAssociation::GetTypeId());

    // Initialization that is there just because the user can configure attribute
    // through the helper methods without making it sad that no TypeId is set.
    // When the TypeId is changed, the user-set attribute will be maintained.
    m_pathlossModelFactory.SetTypeId(ThreeGppPropagationLossModel::GetTypeId());
    m_channelConditionModelFactory.SetTypeId(ThreeGppChannelConditionModel::GetTypeId());
    m_fhControlFactory.SetTypeId(NrFhControl::GetTypeId());
}

NrHelper::~NrHelper()
{
    NS_LOG_FUNCTION(this);
    if (m_beamformingHelper)
    {
        m_beamformingHelper->Dispose();
    }
    m_beamformingHelper = nullptr;
    for (const auto& [imsi, ueNetDev] : m_ueDevices)
    {
        if (auto rrc = ueNetDev->GetRrc())
        {
            rrc->TraceDisconnectWithoutContext("ConnectionEstablished",
                                               MakeCallback(&NrHelper::UeConnectedToCell, this));
            rrc->TraceDisconnectWithoutContext("HandoverEndOk",
                                               MakeCallback(&NrHelper::UeConnectedToCell, this));
        }
    }
}

TypeId
NrHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrHelper")
            .SetParent<Object>()
            .AddConstructor<NrHelper>()
            .AddAttribute(
                "CsiFeedbackFlags",
                "Signals and measurements that will be used for CQI feedback if available."
                "CQI_PDSCH_SISO imply SISO feedback."
                "CQI_PSDCH_MIMO and CQI_CSI_IM imply MIMO feedback."
                "Supported configurations are: CQI_PDSCH_MIMO = 1, CQI_CSI_RS = 2, "
                "CQI_PDSCH_MIMO|CQI_CSI_RS = 3, "
                "CQI_CSI_RS|CQI_CSI_IM = 6, CQI_PDSCH_MIMO|CQI_CSI_RS|CQI_CSI_IM = 7, and "
                "CQI_PDSCH_SISO = 8.",
                UintegerValue(CQI_PDSCH_SISO),
                MakeUintegerAccessor(&NrHelper::m_csiFeedbackFlags),
                MakeUintegerChecker<uint8_t>(0x0, 0x08))
            .AddAttribute("PmSearchMethod",
                          "Type of the precoding matrix search method.",
                          TypeIdValue(NrPmSearchFull::GetTypeId()),
                          MakeTypeIdAccessor(&NrHelper::SetPmSearchTypeId),
                          MakeTypeIdChecker())
            .AddAttribute("UseIdealRrc",
                          "If true, NrRrcProtocolIdeal will be used for RRC signaling. "
                          "If false, NrRrcProtocolReal will be used.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrHelper::m_useIdealRrc),
                          MakeBooleanChecker())
            .AddAttribute("HandoverAlgorithm",
                          "The type of handover algorithm to be used for gNBs. "
                          "The allowed values for this attributes are the type names "
                          "of any class inheriting from ns3::LteHandoverAlgorithm.",
                          StringValue("ns3::NrNoOpHandoverAlgorithm"),
                          MakeStringAccessor(&NrHelper::SetHandoverAlgorithmType,
                                             &NrHelper::GetHandoverAlgorithmType),
                          MakeStringChecker())
            .AddAttribute("RbOverhead",
                          "Overhead when calculating the usable RB number",
                          DoubleValue(0.04),
                          MakeDoubleAccessor(&NrHelper::m_rbOverhead),
                          MakeDoubleChecker<double>(0, 0.5))
            .AddAttribute("NumRbPerRbg",
                          "Number of resource blocks per resource block group.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrHelper::m_numRbPerRbg),
                          MakeUintegerChecker<uint32_t>(1, 16))
            .AddAttribute("AttachWindow",
                          "If non-zero, initial UE attachment is spread deterministically "
                          "over this time window instead of every UE attaching at once: the "
                          "k-th call to AttachToGnb() is deferred by AttachWindow times the "
                          "radical inverse of k in base 2. De-synchronises the "
                          "RACH/connection-setup thundering herd at initial association.",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NrHelper::m_attachWindow),
                          MakeTimeChecker());
    return tid;
}

std::pair<double, BandwidthPartInfoPtrVector>
NrHelper::CreateBandwidthParts(std::vector<CcBwpCreator::SimpleOperationBandConf> bandConfs,
                               const std::string& scenario,
                               const std::string& channelCondition,
                               const std::string& channelModel)
{
    CcBwpCreator ccBwpCreator;
    double totalBandwidth = 0.0;
    auto channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories(scenario, channelCondition, channelModel);
    for (auto& bandConf : bandConfs)
    {
        m_bands.push_back(ccBwpCreator.CreateOperationBandContiguousCc(bandConf));
        totalBandwidth += bandConf.m_channelBandwidth;
    }
    std::vector<std::reference_wrapper<OperationBandInfo>> bandsRefs(m_bands.rbegin(),
                                                                     m_bands.rbegin() +
                                                                         bandConfs.size());

    channelHelper->AssignChannelsToBands(bandsRefs);
    return std::make_pair(totalBandwidth, CcBwpCreator::GetAllBwps(bandsRefs));
}

uint32_t
NrHelper::GetNumberBwp(const Ptr<const NetDevice>& gnbDevice)
{
    NS_LOG_FUNCTION(gnbDevice);
    Ptr<const NrGnbNetDevice> netDevice = DynamicCast<const NrGnbNetDevice>(gnbDevice);
    if (netDevice == nullptr)
    {
        return 0;
    }
    return netDevice->GetCcMapSize();
}

Ptr<NrGnbPhy>
NrHelper::GetGnbPhy(const Ptr<NetDevice>& gnbDevice, uint32_t bwpIndex)
{
    NS_LOG_FUNCTION(gnbDevice << bwpIndex);
    NS_ASSERT(bwpIndex < UINT8_MAX);
    Ptr<NrGnbNetDevice> netDevice = DynamicCast<NrGnbNetDevice>(gnbDevice);
    if (netDevice == nullptr)
    {
        return nullptr;
    }
    return netDevice->GetPhy(static_cast<uint8_t>(bwpIndex));
}

Ptr<NrGnbMac>
NrHelper::GetGnbMac(const Ptr<NetDevice>& gnbDevice, uint32_t bwpIndex)
{
    NS_LOG_FUNCTION(gnbDevice << bwpIndex);
    NS_ASSERT(bwpIndex < UINT8_MAX);
    Ptr<NrGnbNetDevice> netDevice = DynamicCast<NrGnbNetDevice>(gnbDevice);
    if (netDevice == nullptr)
    {
        return nullptr;
    }
    return netDevice->GetMac(static_cast<uint8_t>(bwpIndex));
}

Ptr<NrUeMac>
NrHelper::GetUeMac(const Ptr<NetDevice>& ueDevice, uint32_t bwpIndex)
{
    NS_LOG_FUNCTION(ueDevice << bwpIndex);
    NS_ASSERT(bwpIndex < UINT8_MAX);
    Ptr<NrUeNetDevice> netDevice = DynamicCast<NrUeNetDevice>(ueDevice);
    if (netDevice == nullptr)
    {
        return nullptr;
    }
    return netDevice->GetMac(static_cast<uint8_t>(bwpIndex));
}

Ptr<NrUePhy>
NrHelper::GetUePhy(const Ptr<NetDevice>& ueDevice, uint32_t bwpIndex)
{
    NS_LOG_FUNCTION(ueDevice << bwpIndex);
    NS_ASSERT(bwpIndex < UINT8_MAX);
    Ptr<NrUeNetDevice> netDevice = DynamicCast<NrUeNetDevice>(ueDevice);
    if (netDevice == nullptr)
    {
        return nullptr;
    }
    return netDevice->GetPhy(static_cast<uint8_t>(bwpIndex));
}

Ptr<BwpManagerGnb>
NrHelper::GetBwpManagerGnb(const Ptr<NetDevice>& gnbDevice)
{
    NS_LOG_FUNCTION(gnbDevice);

    Ptr<NrGnbNetDevice> netDevice = DynamicCast<NrGnbNetDevice>(gnbDevice);
    if (netDevice == nullptr)
    {
        return nullptr;
    }

    return netDevice->GetBwpManager();
}

Ptr<BwpManagerUe>
NrHelper::GetBwpManagerUe(const Ptr<NetDevice>& ueDevice)
{
    NS_LOG_FUNCTION(ueDevice);

    Ptr<NrUeNetDevice> netDevice = DynamicCast<NrUeNetDevice>(ueDevice);
    if (netDevice == nullptr)
    {
        return nullptr;
    }

    return netDevice->GetBwpManager();
}

Ptr<NrMacScheduler>
NrHelper::GetScheduler(const Ptr<NetDevice>& gnbDevice, uint32_t bwpIndex)
{
    NS_LOG_FUNCTION(gnbDevice << bwpIndex);

    Ptr<NrGnbNetDevice> netDevice = DynamicCast<NrGnbNetDevice>(gnbDevice);
    if (netDevice == nullptr)
    {
        return nullptr;
    }

    return netDevice->GetScheduler(bwpIndex);
}

void
NrHelper::SetSnrTest(bool snrTest)
{
    m_snrTest = snrTest;
}

bool
NrHelper::GetSnrTest() const
{
    return m_snrTest;
}

NetDeviceContainer
NrHelper::InstallUeDevice(const NodeContainer& c,
                          const std::vector<std::reference_wrapper<BandwidthPartInfoPtr>>& allBwps)
{
    NS_LOG_FUNCTION(this);
    NetDeviceContainer devices;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        Ptr<Node> node = *i;
        Ptr<NetDevice> device = InstallSingleUeDevice(node, allBwps);
        device->SetAddress(Mac48Address::Allocate());
        devices.Add(device);
    }
    return devices;
}

NetDeviceContainer
NrHelper::InstallGnbDevice(const NodeContainer& c,
                           const std::vector<std::reference_wrapper<BandwidthPartInfoPtr>> allBwps)
{
    NS_LOG_FUNCTION(this);
    NetDeviceContainer devices;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        Ptr<Node> node = *i;
        Ptr<NetDevice> device = InstallSingleGnbDevice(node, allBwps);
        device->SetAddress(Mac48Address::Allocate());
        devices.Add(device);
    }
    return devices;
}

void
NrHelper::UpdateDeviceConfigs(const NetDeviceContainer& netDevs)
{
    for (uint32_t i = 0; i < netDevs.GetN(); i++)
    {
        auto ueNetDev = DynamicCast<NrUeNetDevice>(netDevs.Get(i));
        auto gnbNetDev = DynamicCast<NrGnbNetDevice>(netDevs.Get(i));
        if (ueNetDev)
        {
            std::cerr << "Deprecation warning: UpdateConfig is no longer needed for UE device types"
                      << std::endl;
        }
        if (gnbNetDev)
        {
            std::cerr
                << "Deprecation warning: UpdateConfig is no longer needed for gNB device types"
                << std::endl;
        }
    }
}

Ptr<NrUeMac>
NrHelper::CreateUeMac() const
{
    NS_LOG_FUNCTION(this);
    Ptr<NrUeMac> mac = m_ueMacFactory.Create<NrUeMac>();
    return mac;
}

Ptr<NrUePhy>
NrHelper::CreateUePhy(const Ptr<Node>& n,
                      const BandwidthPartInfoPtr& bwp,
                      const Ptr<NrUeNetDevice>& dev,
                      const NrSpectrumPhy::NrPhyDlHarqFeedbackCallback& dlHarqCallback,
                      const NrSpectrumPhy::NrPhyRxCtrlEndOkCallback& phyRxCtrlCallback)
{
    NS_LOG_FUNCTION(this);

    Ptr<NrUePhy> phy = m_uePhyFactory.Create<NrUePhy>();

    NS_ASSERT(bwp->GetChannel() != nullptr);

    phy->InstallCentralFrequency(bwp->m_centralFrequency);

    phy->ScheduleStartEventLoop(n->GetId(), 0, 0, 0);

    // connect CAM and PHY
    Ptr<NrChAccessManager> cam =
        DynamicCast<NrChAccessManager>(m_ueChannelAccessManagerFactory.Create());
    phy->SetCam(cam);
    // set device
    phy->SetDevice(dev);
    // Set CSI feedback type to UE device
    phy->SetCsiFeedbackType(m_csiFeedbackFlags);

    Ptr<MobilityModel> mm = n->GetObject<MobilityModel>();
    NS_ASSERT_MSG(
        mm,
        "MobilityModel needs to be set on node before calling NrHelper::InstallUeDevice ()");

    Ptr<NrSpectrumPhy> channelPhy =
        m_ueSpectrumFactory.Create<NrSpectrumPhy>(); // Create NrSpectrumPhy

    channelPhy->SetPhyDlHarqFeedbackCallback(dlHarqCallback);

    channelPhy->SetIsGnb(false);
    channelPhy->SetDevice(dev); // each NrSpectrumPhy should have a pointer to device

    bool usingUniformPlanarArray =
        m_ueAntennaFactory.GetTypeId() == UniformPlanarArray::GetTypeId();
    // Create n antenna panels and beam manager for Ue
    for (auto i = 0; i < channelPhy->GetNumPanels(); i++)
    {
        auto antenna = m_ueAntennaFactory.Create(); // Create antenna object per panel
        channelPhy->AddPanel(antenna);
        // Check if the antenna is a uniform planar array type
        if (usingUniformPlanarArray)
        {
            Ptr<BeamManager> beamManager = m_ueBeamManagerFactory.Create<BeamManager>();
            auto uniformPlanarArray = DynamicCast<UniformPlanarArray>(antenna);
            beamManager->Configure(uniformPlanarArray);
            channelPhy->AddBeamManager(beamManager);
        }
    }
    if (usingUniformPlanarArray)
    {
        // Config bearing angles for all panels installed in NrSpectrumPhy
        channelPhy->ConfigPanelsBearingAngles();
    }

    cam->SetNrSpectrumPhy(channelPhy); // connect CAM

    Ptr<NrChunkProcessor> pData = Create<NrChunkProcessor>();
    pData->AddCallback(MakeCallback(&NrSpectrumPhy::UpdateSinrPerceived, channelPhy));
    channelPhy->AddDataSinrChunkProcessor(pData);

    Ptr<NrMimoChunkProcessor> pDataMimo{nullptr};
    auto phasedChannel = bwp->GetChannel()->GetPhasedArraySpectrumPropagationLossModel();
    if (phasedChannel)
    {
        pDataMimo = Create<NrMimoChunkProcessor>();
        pDataMimo->AddCallback(MakeCallback(&NrSpectrumPhy::UpdateMimoSinrPerceived, channelPhy));
        channelPhy->AddDataMimoChunkProcessor(pDataMimo);

        if (m_csiFeedbackFlags & CQI_PDSCH_MIMO)
        {
            // Report DL CQI, PMI, RI (channel quality, MIMO precoding matrix and rank indicators)
            pDataMimo->AddCallback(MakeCallback(&NrUePhy::PdschMimoReceived, phy));
        }

        if (m_csiFeedbackFlags & CQI_CSI_RS)
        {
            Ptr<NrMimoChunkProcessor> pCsiRs = Create<NrMimoChunkProcessor>();
            pCsiRs->AddCallback(MakeCallback(&NrUePhy::CsiRsReceived, phy));
            channelPhy->AddCsiRsMimoChunkProcessor(pCsiRs);
            // currently, CSI_IM can be enabled only if CSI-RS is enabled
            if (m_csiFeedbackFlags & CQI_CSI_IM)
            {
                Ptr<NrMimoChunkProcessor> pCsiIm = Create<NrMimoChunkProcessor>();
                pCsiIm->AddCallback(MakeCallback(&NrUePhy::CsiImEnded, phy));
                channelPhy->AddCsiImMimoChunkProcessor(pCsiIm);
            }
        }
    }
    if (!phasedChannel || (m_csiFeedbackFlags == CQI_PDSCH_SISO))
    {
        // SISO CQI feedback
        pData->AddCallback(MakeCallback(&NrUePhy::GenerateDlCqiReport, phy));
    }

    Ptr<NrChunkProcessor> pRs = Create<NrChunkProcessor>();
    pRs->AddCallback(MakeCallback(&NrUePhy::ReportRsReceivedPower, phy));
    channelPhy->AddRsPowerChunkProcessor(pRs);

    Ptr<NrChunkProcessor> pSinr = Create<NrChunkProcessor>();
    pSinr->AddCallback(MakeCallback(&NrSpectrumPhy::ReportDlCtrlSinr, channelPhy));
    channelPhy->AddDlCtrlSinrChunkProcessor(pSinr);

    channelPhy->SetChannel(bwp->GetChannel());
    channelPhy->InstallPhy(phy);
    channelPhy->SetMobility(mm);
    channelPhy->SetPhyRxDataEndOkCallback(MakeCallback(&NrUePhy::PhyDataPacketReceived, phy));
    channelPhy->SetPhyRxCtrlEndOkCallback(phyRxCtrlCallback);
    channelPhy->SetPhyRxPssCallback(MakeCallback(&NrUePhy::ReceivePss, phy));
    phy->InstallSpectrumPhy(channelPhy);
    return phy;
}

Ptr<NetDevice>
NrHelper::InstallSingleUeDevice(
    const Ptr<Node>& n,
    const std::vector<std::reference_wrapper<BandwidthPartInfoPtr>> allBwps)
{
    NS_LOG_FUNCTION(this);

    Ptr<NrUeNetDevice> dev = m_ueNetDeviceFactory.Create<NrUeNetDevice>();
    dev->SetNode(n);

    std::map<uint8_t, Ptr<BandwidthPartUe>> ueCcMap;

    // Create, for each ue, its bandwidth parts
    for (uint32_t bwpId = 0; bwpId < allBwps.size(); ++bwpId)
    {
        Ptr<BandwidthPartUe> cc = CreateObject<BandwidthPartUe>();
        double bwInKhz = allBwps[bwpId].get()->m_channelBandwidth / 1000.0;
        NS_ABORT_MSG_IF(bwInKhz / 100.0 > 65535.0,
                        "A bandwidth of " << bwInKhz / 100.0 << " kHz cannot be represented");
        cc->SetUlBandwidth(static_cast<uint16_t>(bwInKhz / 100));
        cc->SetDlBandwidth(static_cast<uint16_t>(bwInKhz / 100));
        cc->SetArfcn(NrPhy::FrequencyHzToArfcn(allBwps[bwpId].get()->m_centralFrequency));

        auto mac = CreateUeMac();
        cc->SetMac(mac);

        auto phy = CreateUePhy(n,
                               allBwps[bwpId].get(),
                               dev,
                               MakeCallback(&NrUeNetDevice::EnqueueDlHarqFeedback, dev),
                               std::bind(&NrUeNetDevice::RouteIngoingCtrlMsgs,
                                         dev,
                                         std::placeholders::_1,
                                         cc->GetArfcn()));

        phy->SetBwpId(bwpId);
        cc->SetPhy(phy);

        if (bwpId == 0)
        {
            cc->SetAsPrimary(true);
        }
        else
        {
            cc->SetAsPrimary(false);
        }
        phy->SetDlAmc(m_gnbDlAmcFactory.Create<NrAmc>());
        phy->SetRbOverhead(m_rbOverhead);
        ueCcMap.insert(std::make_pair(bwpId, cc));
    }

    Ptr<NrUeComponentCarrierManager> ccmUe =
        DynamicCast<NrUeComponentCarrierManager>(CreateObject<BwpManagerUe>());
    DynamicCast<BwpManagerUe>(ccmUe)->SetBwpManagerAlgorithm(
        m_ueBwpManagerAlgoFactory.Create<BwpManagerAlgorithm>());

    UintegerValue primaryUlIndex;
    dev->GetAttribute("PrimaryUlIndex", primaryUlIndex);
    NS_ASSERT_MSG(primaryUlIndex.Get() < ueCcMap.size(),
                  "UL primary index out of bounds. Configure PrimaryUlIndex attribute of "
                  "NrUeNetDevice correctly.");

    Ptr<NrUeRrc> rrc = CreateObject<NrUeRrc>();
    rrc->SetPrimaryUlIndex(primaryUlIndex.Get());
    rrc->m_numberOfComponentCarriers = ueCcMap.size();
    // run InitializeSap to create the proper number of sap provider/users
    rrc->InitializeSap();
    rrc->SetNrMacSapProvider(ccmUe->GetNrMacSapProvider());
    // setting ComponentCarrierManager SAP
    rrc->SetNrCcmRrcSapProvider(ccmUe->GetNrCcmRrcSapProvider());
    ccmUe->SetNrCcmRrcSapUser(rrc->GetNrCcmRrcSapUser());
    ccmUe->SetNumberOfComponentCarriers(ueCcMap.size());
    DynamicCast<BwpManagerUe>(ccmUe)->SetGetPrimaryUlFn(
        [rrc]() { return rrc->GetPrimaryUlIndex(); });
    Ptr<BwpManagerUe> bwpManagerUe = DynamicCast<BwpManagerUe>(ccmUe);
    rrc->SetUpdateBwpOutputLinkFn([bwpManagerUe](uint32_t sourceBwp, uint32_t outputBwp) {
        bwpManagerUe->SetDefaultOutputLink(sourceBwp, outputBwp);
    });
    rrc->SetUpdateQosFlowBwpFn([bwpManagerUe](uint8_t fiveQi, uint8_t bwpIndex) {
        bwpManagerUe->SetBwpForQosFlow(fiveQi, bwpIndex);
    });
    rrc->SetClearBwpOutputLinksFn([bwpManagerUe]() { bwpManagerUe->ClearOutputLinks(); });

    if (m_useIdealRrc)
    {
        Ptr<NrUeRrcProtocolIdeal> rrcProtocol = CreateObject<NrUeRrcProtocolIdeal>();
        rrcProtocol->SetUeRrc(rrc);
        rrc->AggregateObject(rrcProtocol);
        rrcProtocol->SetNrUeRrcSapProvider(rrc->GetNrUeRrcSapProvider());
        rrc->SetNrUeRrcSapUser(rrcProtocol->GetNrUeRrcSapUser());
    }
    else
    {
        Ptr<nr::UeRrcProtocolReal> rrcProtocol = CreateObject<nr::UeRrcProtocolReal>();
        rrcProtocol->SetUeRrc(rrc);
        rrc->AggregateObject(rrcProtocol);
        rrcProtocol->SetNrUeRrcSapProvider(rrc->GetNrUeRrcSapProvider());
        rrc->SetNrUeRrcSapUser(rrcProtocol->GetNrUeRrcSapUser());
    }

    if (m_nrEpcHelper != nullptr)
    {
        rrc->SetUseRlcSm(false);
    }
    else
    {
        rrc->SetUseRlcSm(true);
    }
    Ptr<NrEpcUeNas> nas = CreateObject<NrEpcUeNas>();

    nas->SetAsSapProvider(rrc->GetAsSapProvider());
    nas->SetDevice(dev);
    nas->SetForwardUpCallback(MakeCallback(&NrUeNetDevice::Receive, dev));

    rrc->SetAsSapUser(nas->GetAsSapUser());

    for (auto& it : ueCcMap)
    {
        rrc->SetNrUeCmacSapProvider(it.second->GetMac()->GetUeCmacSapProvider(), it.first);
        it.second->GetMac()->SetUeCmacSapUser(rrc->GetNrUeCmacSapUser(it.first));

        it.second->GetPhy()->SetUeCphySapUser(rrc->GetNrUeCphySapUser());
        rrc->SetNrUeCphySapProvider(it.second->GetPhy()->GetUeCphySapProvider(), it.first);

        it.second->GetPhy()->SetPhySapUser(it.second->GetMac()->GetPhySapUser());
        it.second->GetMac()->SetPhySapProvider(it.second->GetPhy()->GetPhySapProvider());

        bool ccmTest =
            ccmUe->SetComponentCarrierMacSapProviders(it.first,
                                                      it.second->GetMac()->GetUeMacSapProvider());

        if (!ccmTest)
        {
            NS_FATAL_ERROR("Error in SetComponentCarrierMacSapProviders");
        }
    }

    dev->SetCcMap(ueCcMap);
    dev->SetAttribute("nrUeRrc", PointerValue(rrc));
    dev->SetAttribute("NrEpcUeNas", PointerValue(nas));
    dev->SetAttribute("NrUeComponentCarrierManager", PointerValue(ccmUe));
    dev->SetAttribute("Imsi", UintegerValue(n->GetId()));

    n->AddDevice(dev);

    if (m_nrEpcHelper != nullptr)
    {
        m_nrEpcHelper->AddUe(dev, dev->GetImsi());
        m_ueEpcHelpers[dev] = m_nrEpcHelper;
    }

    rrc->InitializeSrb0();

    m_ueDevices[dev->GetImsi()] = dev;
    rrc->TraceConnectWithoutContext("ConnectionEstablished",
                                    MakeCallback(&NrHelper::UeConnectedToCell, this));
    rrc->TraceConnectWithoutContext("HandoverEndOk",
                                    MakeCallback(&NrHelper::UeConnectedToCell, this));
    return dev;
}

Ptr<NrGnbPhy>
NrHelper::CreateGnbPhy(const Ptr<Node>& n,
                       const BandwidthPartInfoPtr& bwp,
                       const Ptr<NrGnbNetDevice>& dev,
                       const NrSpectrumPhy::NrPhyRxCtrlEndOkCallback& phyEndCtrlCallback)
{
    NS_LOG_FUNCTION(this);

    Ptr<NrGnbPhy> phy = m_gnbPhyFactory.Create<NrGnbPhy>();

    DoubleValue frequency;
    phy->SetRbOverhead(m_rbOverhead);
    phy->InstallCentralFrequency(bwp->m_centralFrequency);

    phy->ScheduleStartEventLoop(n->GetId(), 0, 0, 0);

    // PHY <--> CAM
    Ptr<NrChAccessManager> cam =
        DynamicCast<NrChAccessManager>(m_gnbChannelAccessManagerFactory.Create());
    phy->SetCam(cam);
    phy->SetDevice(dev);

    Ptr<MobilityModel> mm = n->GetObject<MobilityModel>();
    NS_ASSERT_MSG(
        mm,
        "MobilityModel needs to be set on node before calling NrHelper::InstallGnbDevice ()");

    Ptr<NrSpectrumPhy> channelPhy = m_gnbSpectrumFactory.Create<NrSpectrumPhy>();
    auto antenna = m_gnbAntennaFactory.Create();
    channelPhy->SetAntenna(antenna);
    cam->SetNrSpectrumPhy(channelPhy);

    channelPhy->SetIsGnb(true);
    channelPhy->SetDevice(dev); // each NrSpectrumPhy should have a pointer to device
    channelPhy->SetChannel(
        bwp->GetChannel()); // each NrSpectrumPhy needs to have a pointer to the SpectrumChannel
    // object of the corresponding spectrum part
    channelPhy->InstallPhy(phy); // each NrSpectrumPhy should have a pointer to its NrPhy

    Ptr<NrChunkProcessor> pData =
        Create<NrChunkProcessor>(); // create pData chunk processor per NrSpectrumPhy
    Ptr<NrChunkProcessor> pSrs =
        Create<NrChunkProcessor>(); // create pSrs per processor per NrSpectrumPhy
    auto phasedChannel = bwp->GetChannel()->GetPhasedArraySpectrumPropagationLossModel();
    if (!m_snrTest)
    {
        if (!phasedChannel)
        {
            // Without a MIMO channel, the PUSCH CQI comes from the SISO SINR
            pData->AddCallback(MakeCallback(&NrGnbPhy::GenerateDataCqiReport, phy));
        }
        pData->AddCallback(MakeCallback(&NrSpectrumPhy::UpdateSinrPerceived,
                                        channelPhy)); // connect DATA chunk processor that will
        // call UpdateSinrPerceived function
        pSrs->AddCallback(MakeCallback(&NrSpectrumPhy::UpdateSrsSinrPerceived,
                                       channelPhy)); // connect SRS chunk processor that will
                                                     // call UpdateSrsSinrPerceived function
        if (phasedChannel)
        {
            auto pDataMimo = Create<NrMimoChunkProcessor>();
            pDataMimo->AddCallback(
                MakeCallback(&NrSpectrumPhy::UpdateMimoSinrPerceived, channelPhy));
            // Use the post-MMSE SINR used for decoding as the PUSCH CQI
            pDataMimo->AddCallback(MakeCallback(&NrGnbPhy::GenerateMimoDataCqiReport, phy));
            channelPhy->AddDataMimoChunkProcessor(pDataMimo);
        }
    }
    channelPhy->AddDataSinrChunkProcessor(pData); // set DATA chunk processor to NrSpectrumPhy
    channelPhy->AddSrsSinrChunkProcessor(pSrs);   // set SRS chunk processor to NrSpectrumPhy
    channelPhy->SetMobility(mm);                  // set mobility model to this NrSpectrumPhy
    channelPhy->SetPhyRxDataEndOkCallback(
        MakeCallback(&NrGnbPhy::PhyDataPacketReceived, phy));  // connect PhyRxDataEndOk callback
    channelPhy->SetPhyRxCtrlEndOkCallback(phyEndCtrlCallback); // connect PhyRxCtrlEndOk
                                                               // callback
    channelPhy->SetPhyUlHarqFeedbackCallback(
        MakeCallback(&NrGnbPhy::ReportUlHarqFeedback, phy)); // PhyUlHarqFeedback callback
    // Check if the antenna is a uniform planar array type
    auto uniformPlanarArray = DynamicCast<UniformPlanarArray>(antenna);
    if (uniformPlanarArray)
    {
        Ptr<BeamManager> beamManager = m_gnbBeamManagerFactory.Create<BeamManager>();
        beamManager->Configure(uniformPlanarArray);
        channelPhy->SetBeamManager(beamManager);
    }
    phy->InstallSpectrumPhy(channelPhy); // finally let know phy that there is this spectrum phy
    if ((m_csiFeedbackFlags & CQI_CSI_RS) && phasedChannel)
    {
        phy->EnableCsiRs();
    }
    return phy;
}

Ptr<NrGnbMac>
NrHelper::CreateGnbMac()
{
    NS_LOG_FUNCTION(this);

    Ptr<NrGnbMac> mac = m_gnbMacFactory.Create<NrGnbMac>();
    return mac;
}

Ptr<NrMacScheduler>
NrHelper::CreateGnbSched(uint16_t bwpId)
{
    NS_LOG_FUNCTION(this);

    Ptr<NrMacSchedulerNs3> sched;
    auto perBwp = m_schedTypeIdPerBwp.find(bwpId);
    if (perBwp != m_schedTypeIdPerBwp.end())
    {
        ObjectFactory f = m_schedFactory; // same attributes, another type
        f.SetTypeId(perBwp->second);
        sched = f.Create<NrMacSchedulerNs3>();
    }
    else
    {
        sched = m_schedFactory.Create<NrMacSchedulerNs3>();
    }
    auto dlAmc = m_gnbDlAmcFactory.Create<NrAmc>();
    auto ulAmc = m_gnbUlAmcFactory.Create<NrAmc>();

    sched->InstallDlAmc(dlAmc);
    sched->InstallUlAmc(ulAmc);

    return sched;
}

Ptr<NrFhControl>
NrHelper::CreateNrFhControl()
{
    NS_LOG_FUNCTION(this);

    Ptr<NrFhControl> fhControl = m_fhControlFactory.Create<NrFhControl>();
    return fhControl;
}

Ptr<NetDevice>
NrHelper::InstallSingleGnbDevice(
    const Ptr<Node>& n,
    const std::vector<std::reference_wrapper<BandwidthPartInfoPtr>> allBwps)
{
    NS_ABORT_MSG_IF(m_cellIdCounter == 65535, "max num gNBs exceeded");

    Ptr<NrGnbNetDevice> dev = m_gnbNetDeviceFactory.Create<NrGnbNetDevice>();
    Ptr<NrHandoverAlgorithm> handoverAlgorithm =
        m_handoverAlgorithmFactory.Create<NrHandoverAlgorithm>();

    NS_LOG_DEBUG("Creating gNB, cellId = " << m_cellIdCounter);
    uint16_t cellId = m_cellIdCounter++; // New cellId
    dev->SetCellId(cellId);
    dev->SetNode(n);
    m_gnbDevices[cellId] = dev;

    // create component carrier map for this gNB device
    std::map<uint8_t, Ptr<BandwidthPartGnb>> ccMap;

    auto fhControl = CreateNrFhControl();
    fhControl->SetPhysicalCellId(cellId);

    if (m_fhEnabled)
    {
        dev->SetNrFhControl(fhControl);
    }

    for (uint32_t bwpId = 0; bwpId < allBwps.size(); ++bwpId)
    {
        NS_LOG_DEBUG("Creating BandwidthPart, id = " << bwpId);
        Ptr<BandwidthPartGnb> cc = CreateObject<BandwidthPartGnb>();
        double bwInKhz = allBwps[bwpId].get()->m_channelBandwidth / 1000.0;
        NS_ABORT_MSG_IF(bwInKhz / 100.0 > 65535.0,
                        "A bandwidth of " << bwInKhz / 100.0 << " kHz cannot be represented");

        cc->SetUlBandwidth(static_cast<uint16_t>(bwInKhz / 100));
        cc->SetDlBandwidth(static_cast<uint16_t>(bwInKhz / 100));
        cc->SetArfcn(NrPhy::FrequencyHzToArfcn(allBwps[bwpId].get()->m_centralFrequency));
        cc->SetCellId(cellId); // CellId is set just for easier debugging
        cc->SetBwpId(bwpId);   // CC BwpId is used to map BWPs to the correct CC PHY/MAC
        cc->SetCsgId(0);       // Assume single group

        auto phy = CreateGnbPhy(
            n,
            allBwps[bwpId].get(),
            dev,
            std::bind(&NrGnbNetDevice::RouteIngoingCtrlMsgs, dev, std::placeholders::_1, bwpId));
        phy->SetBwpId(bwpId);
        cc->SetPhy(phy);

        auto mac = CreateGnbMac();
        mac->SetNumRbPerRbg(m_numRbPerRbg);

        cc->SetMac(mac);
        phy->GetCam()->SetNrGnbMac(mac);

        auto sched = CreateGnbSched(bwpId);
        cc->SetNrMacScheduler(sched);

        if (bwpId == 0)
        {
            cc->SetAsPrimary(true);
        }
        else
        {
            cc->SetAsPrimary(false);
        }

        ccMap.insert(std::make_pair(bwpId, cc));
    }

    Ptr<NrGnbRrc> rrc = CreateObject<NrGnbRrc>();
    Ptr<NrGnbComponentCarrierManager> ccmGnbManager =
        DynamicCast<NrGnbComponentCarrierManager>(CreateObject<BwpManagerGnb>());
    DynamicCast<BwpManagerGnb>(ccmGnbManager)
        ->SetBwpManagerAlgorithm(m_gnbBwpManagerAlgoFactory.Create<BwpManagerAlgorithm>());

    rrc->SetNrHandoverManagementSapProvider(
        handoverAlgorithm->GetNrHandoverManagementSapProvider());
    handoverAlgorithm->SetNrHandoverManagementSapUser(rrc->GetNrHandoverManagementSapUser());

    // Convert Gnb carrier map to only PhyConf map
    // we want to make RRC to be generic, to be able to work with any type of carriers, not only
    // strictly LTE carriers
    std::map<uint8_t, Ptr<BandwidthPartGnb>> ccPhyConfMap;
    for (const auto& i : ccMap)
    {
        Ptr<BandwidthPartGnb> c = i.second;
        ccPhyConfMap.insert(std::make_pair(i.first, c));
    }

    // ComponentCarrierManager SAP
    rrc->SetNrCcmRrcSapProvider(ccmGnbManager->GetNrCcmRrcSapProvider());
    ccmGnbManager->SetNrCcmRrcSapUser(rrc->GetNrCcmRrcSapUser());
    // Set number of component carriers. Note: gNB CCM would also set the
    // number of component carriers in gNB RRC

    ccmGnbManager->SetNumberOfComponentCarriers(ccMap.size());
    rrc->ConfigureCarriers(ccPhyConfMap);

    // nr module currently uses only RRC ideal mode
    if (m_useIdealRrc)
    {
        Ptr<NrGnbRrcProtocolIdeal> rrcProtocol = CreateObject<NrGnbRrcProtocolIdeal>();
        rrcProtocol->SetNrGnbRrcSapProvider(rrc->GetNrGnbRrcSapProvider());
        rrc->SetNrGnbRrcSapUser(rrcProtocol->GetNrGnbRrcSapUser());
        rrc->AggregateObject(rrcProtocol);
    }
    else
    {
        Ptr<nr::NrGnbRrcProtocolReal> rrcProtocol = CreateObject<nr::NrGnbRrcProtocolReal>();
        rrcProtocol->SetNrGnbRrcSapProvider(rrc->GetNrGnbRrcSapProvider());
        rrc->SetNrGnbRrcSapUser(rrcProtocol->GetNrGnbRrcSapUser());
        rrc->AggregateObject(rrcProtocol);
    }

    if (m_nrEpcHelper != nullptr)
    {
        EnumValue<NrGnbRrc::NrQosFlowToRlcMapping_t> qosFlowToRlcMapping;
        rrc->GetAttribute("QosFlowToRlcMapping", qosFlowToRlcMapping);
        // it does not make sense to use RLC/SM when also using the EPC
        if (qosFlowToRlcMapping.Get() == NrGnbRrc::RLC_SM_ALWAYS)
        {
            rrc->SetAttribute("QosFlowToRlcMapping", EnumValue(NrGnbRrc::RLC_UM_ALWAYS));
        }
    }

    // This RRC attribute is used to connect each new RLC instance with the MAC layer
    // (for function such as TransmitPdu, BufferStatusReportReport).
    // Since in this new architecture, the component carrier manager acts a proxy, it
    // will have its own NrMacSapProvider interface, RLC will see it as through original MAC
    // interface NrMacSapProvider, but the function call will go now through
    // NrGnbComponentCarrierManager instance that needs to implement functions of this interface,
    // and its task will be to forward these calls to the specific MAC of some of the instances of
    // component carriers. This decision will depend on the specific implementation of the component
    // carrier manager.
    rrc->SetNrMacSapProvider(ccmGnbManager->GetNrMacSapProvider());
    rrc->SetForwardUpCallback(MakeCallback(&NrGnbNetDevice::Receive, dev));

    for (auto& it : ccMap)
    {
        it.second->GetPhy()->SetGnbCphySapUser(rrc->GetNrGnbCphySapUser(it.first));
        rrc->SetNrGnbCphySapProvider(it.second->GetPhy()->GetGnbCphySapProvider(), it.first);

        rrc->SetNrGnbCmacSapProvider(it.second->GetMac()->GetGnbCmacSapProvider(), it.first);
        it.second->GetMac()->SetGnbCmacSapUser(rrc->GetNrGnbCmacSapUser(it.first));

        // PHY <--> MAC SAP
        it.second->GetPhy()->SetPhySapUser(it.second->GetMac()->GetPhySapUser());
        it.second->GetMac()->SetPhySapProvider(it.second->GetPhy()->GetPhySapProvider());
        // PHY <--> MAC SAP END

        // Scheduler SAP
        it.second->GetMac()->SetNrMacSchedSapProvider(
            it.second->GetScheduler()->GetMacSchedSapProvider());
        it.second->GetMac()->SetNrMacCschedSapProvider(
            it.second->GetScheduler()->GetMacCschedSapProvider());

        it.second->GetScheduler()->SetMacSchedSapUser(it.second->GetMac()->GetNrMacSchedSapUser());
        it.second->GetScheduler()->SetMacCschedSapUser(
            it.second->GetMac()->GetNrMacCschedSapUser());
        // Scheduler SAP END

        it.second->GetMac()->SetNrCcmMacSapUser(ccmGnbManager->GetNrCcmMacSapUser());
        ccmGnbManager->SetCcmMacSapProviders(it.first,
                                             it.second->GetMac()->GetNrCcmMacSapProvider());

        // insert the pointer to the NrMacSapProvider interface of the MAC layer of the specific
        // component carrier
        ccmGnbManager->SetMacSapProvider(it.first, it.second->GetMac()->GetMacSapProvider());

        // FH Control SAPs
        if (m_fhEnabled)
        {
            // Multiple sched/phy instances (as many as BWPs) - 1 NrFhControl instance (1 per cell)
            it.second->GetScheduler()->SetNrFhSchedSapProvider(
                dev->GetNrFhControl()->GetNrFhSchedSapProvider());
            dev->GetNrFhControl()->SetNrFhSchedSapUser(
                it.first,
                it.second->GetScheduler()->GetNrFhSchedSapUser());
            it.second->GetPhy()->SetNrFhPhySapProvider(
                dev->GetNrFhControl()->GetNrFhPhySapProvider());
            dev->GetNrFhControl()->SetNrFhPhySapUser(it.first,
                                                     it.second->GetPhy()->GetNrFhPhySapUser());
        }
    }

    dev->SetAttribute("NrGnbComponentCarrierManager", PointerValue(ccmGnbManager));
    dev->SetCcMap(ccMap);
    dev->SetAttribute("NrGnbRrc", PointerValue(rrc));
    dev->SetAttribute("NrHandoverAlgorithm", PointerValue(handoverAlgorithm));

    n->AddDevice(dev);

    if (m_nrEpcHelper != nullptr)
    {
        NS_LOG_INFO("adding this gNB to the EPC");
        m_nrEpcHelper->AddGnb(n, dev, cellId);
    }
    return dev;
}

std::string
NrHelper::GetHandoverAlgorithmType() const
{
    return m_handoverAlgorithmFactory.GetTypeId().GetName();
}

void
NrHelper::SetHandoverAlgorithmType(std::string type)
{
    NS_LOG_FUNCTION(this << type);
    m_handoverAlgorithmFactory = ObjectFactory();
    m_handoverAlgorithmFactory.SetTypeId(type);
}

void
NrHelper::SetHandoverAlgorithmAttribute(std::string n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this << n);
    m_handoverAlgorithmFactory.Set(n, v);
}

void
NrHelper::AddX2Interface(NodeContainer gnbNodes)
{
    NS_LOG_FUNCTION(this);

    NS_ASSERT_MSG(m_nrEpcHelper, "X2 interfaces cannot be set up when the EPC is not used");

    for (auto i = gnbNodes.Begin(); i != gnbNodes.End(); ++i)
    {
        for (auto j = i + 1; j != gnbNodes.End(); ++j)
        {
            AddX2Interface(*i, *j);
        }
    }
}

void
NrHelper::AddX2Interface(Ptr<Node> gnbNode1, Ptr<Node> gnbNode2)
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("setting up the X2 interface");

    m_nrEpcHelper->AddX2Interface(gnbNode1, gnbNode2);
}

void
NrHelper::HandoverRequest(Time hoTime,
                          Ptr<NetDevice> ueDev,
                          Ptr<NetDevice> sourceGnbDev,
                          Ptr<NetDevice> targetGnbDev)
{
    NS_LOG_FUNCTION(this << ueDev << sourceGnbDev << targetGnbDev);
    NS_ASSERT_MSG(m_nrEpcHelper,
                  "Handover requires the use of the EPC - did you forget to call "
                  "NrHelper::SetEpcHelper () ?");
    uint16_t targetCellId = targetGnbDev->GetObject<NrGnbNetDevice>()->GetCellId();
    Simulator::Schedule(hoTime,
                        &NrHelper::DoHandoverRequest,
                        this,
                        ueDev,
                        sourceGnbDev,
                        targetCellId);
}

void
NrHelper::HandoverRequest(Time hoTime,
                          Ptr<NetDevice> ueDev,
                          Ptr<NetDevice> sourceGnbDev,
                          uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << ueDev << sourceGnbDev << targetCellId);
    NS_ASSERT_MSG(m_nrEpcHelper,
                  "Handover requires the use of the EPC - did you forget to call "
                  "NrHelper::SetEpcHelper () ?");
    Simulator::Schedule(hoTime,
                        &NrHelper::DoHandoverRequest,
                        this,
                        ueDev,
                        sourceGnbDev,
                        targetCellId);
}

void
NrHelper::DoHandoverRequest(Ptr<NetDevice> ueDev,
                            Ptr<NetDevice> sourceGnbDev,
                            uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << ueDev << sourceGnbDev << targetCellId);

    Ptr<NrGnbRrc> sourceRrc = sourceGnbDev->GetObject<NrGnbNetDevice>()->GetRrc();
    uint16_t rnti = ueDev->GetObject<NrUeNetDevice>()->GetRrc()->GetRnti();
    sourceRrc->SendHandoverRequest(rnti, targetCellId);
}

void
NrHelper::AttachToMaxRsrpGnb(const NetDeviceContainer& ueDevices,
                             const NetDeviceContainer& gnbDevices)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(gnbDevices.GetN() > 0, "gNB container should not be empty");
    for (auto i = ueDevices.Begin(); i != ueDevices.End(); i++)
    {
        // Since UE may not be attached to any gNB, it won't be properly configured via MIB
        // so we configure its numerology manually here. All gNBs numerology must match.
        {
            auto ueNetDevCast = DynamicCast<NrUeNetDevice>(*i);
            auto gnbNetDevCast = DynamicCast<NrGnbNetDevice>(gnbDevices.Get(0));
            ueNetDevCast->GetPhy(0)->SetNumerology(gnbNetDevCast->GetPhy(0)->GetNumerology());
        }

        // attach the UE to the highest RSRP gNB (this will change with active panel).
        // Any AttachWindow staggering is applied by AttachToGnb, which this path reaches
        // through AttachToMaxRsrpGnb.
        Simulator::ScheduleNow([=, this]() { AttachToMaxRsrpGnb(*i, gnbDevices); });
    }
}

void
NrHelper::AttachToMaxRsrpGnb(const Ptr<NetDevice>& ueDevice, const NetDeviceContainer& gnbDevices)
{
    NS_LOG_FUNCTION(this);

    NS_ASSERT_MSG(gnbDevices.GetN() > 0, "empty gNB device container");

    auto nrInitAssoc = m_initialAttachmentFactory.Create<NrInitialAssociation>();
    ueDevice->GetObject<NrUeNetDevice>()->SetInitAssoc(nrInitAssoc);

    nrInitAssoc->AssignStreams(INIT_ASSOC_STREAM_BASE + ueDevice->GetNode()->GetId());

    nrInitAssoc->SetUeDevice(ueDevice);
    nrInitAssoc->SetGnbDevices(gnbDevices);
    nrInitAssoc->SetColBeamAngles(m_initialParams.colAngles);
    nrInitAssoc->SetRowBeamAngles(m_initialParams.rowAngles);
    nrInitAssoc->FindAssociatedGnb();
    auto maxRsrpGnbDevice = nrInitAssoc->GetAssociatedGnb();
    NS_ASSERT(maxRsrpGnbDevice);

    AttachToGnb(ueDevice, maxRsrpGnbDevice);
}

void
NrHelper::AttachToClosestGnb(const NetDeviceContainer& ueDevices,
                             const NetDeviceContainer& gnbDevices)
{
    NS_LOG_FUNCTION(this);

    for (auto i = ueDevices.Begin(); i != ueDevices.End(); i++)
    {
        AttachToClosestGnb(*i, gnbDevices);
    }
}

void
NrHelper::AttachToClosestGnb(const Ptr<NetDevice>& ueDevice, const NetDeviceContainer& gnbDevices)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(gnbDevices.GetN() > 0, "empty gnb device container");
    double minDistance = std::numeric_limits<double>::infinity();
    Ptr<NetDevice> closestGnbDevice;
    Ptr<MobilityModel> ueMm = ueDevice->GetNode()->GetObject<MobilityModel>();
    Ptr<SpectrumChannel> channel = GetUePhy(ueDevice, 0)->GetSpectrumPhy()->GetSpectrumChannel();
    for (auto i = gnbDevices.Begin(); i != gnbDevices.End(); ++i)
    {
        auto gnbMm =
            GetVirtualMobilityModel(channel, (*i)->GetNode()->GetObject<MobilityModel>(), ueMm);
        Vector gnbpos = gnbMm->GetPosition();
        Vector uepos = ueDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();

        double distance = CalculateDistance(uepos, gnbpos);
        if (distance < minDistance)
        {
            minDistance = distance;
            closestGnbDevice = *i;
        }
    }
    NS_ASSERT(closestGnbDevice);

    AttachToGnb(ueDevice, closestGnbDevice);
}

void
NrHelper::AttachToGnb(const Ptr<NetDevice>& ueDevice, const Ptr<NetDevice>& gnbDevice)
{
    const uint32_t index = m_attachCount++;

    if (m_attachWindow.IsZero())
    {
        DoAttachToGnb(ueDevice, gnbDevice);
        return;
    }

    // Spread this attachment across the window by its position in the call sequence, using the
    // radical inverse of the index in base 2 (the van der Corput sequence): 0, 1/2, 1/4, 3/4,
    // 1/8, ... Any prefix of that sequence covers the window roughly uniformly, so a handful of
    // UEs is spread just as well as a large deployment and the helper never needs to know the
    // total number of UEs in advance.
    double fraction = 0.0;
    double weight = 0.5;
    for (uint32_t v = index; v != 0; v >>= 1)
    {
        if (v & 1u)
        {
            fraction += weight;
        }
        weight *= 0.5;
    }

    // The whole attachment is deferred as one unit rather than just the connection trigger,
    // because a UE that has registered but not started random access would receive a random
    // access response for a procedure it never began.
    Simulator::Schedule(m_attachWindow * fraction,
                        &NrHelper::DoAttachToGnb,
                        this,
                        ueDevice,
                        gnbDevice);
}

void
NrHelper::DoAttachToGnb(const Ptr<NetDevice>& ueDevice, const Ptr<NetDevice>& gnbDevice)
{
    Ptr<NrGnbNetDevice> gnbNetDev = gnbDevice->GetObject<NrGnbNetDevice>();
    Ptr<NrUeNetDevice> ueNetDev = ueDevice->GetObject<NrUeNetDevice>();

    NS_ABORT_IF(gnbNetDev == nullptr || ueNetDev == nullptr);

    if (!gnbNetDev->IsCellConfigured())
    {
        gnbNetDev->ConfigureCell();
    }

    // Derive the UE primary DL/UL BWP pair from the gNB configuration. The DL
    // is the gNB's primary carrier (the one broadcasting MIB/SIB); for an FDD
    // cell (DL-only primary) the UL is the cell's dedicated UL-only carrier,
    // matching what SIB1 advertises in ulCarrierFreq.
    if (!gnbNetDev->GetPhy(0)->HasDlSlot())
    {
        NS_FATAL_ERROR("The gNB primary carrier (CC0) cannot be UL-only: it broadcasts MIB/SIB");
    }
    ueNetDev->GetRrc()->SetPrimaryDlIndex(ueNetDev->GetArfcnBwpId(gnbNetDev->GetBwpArfcn(0)));
    if (!gnbNetDev->GetPhy(0)->HasUlSlot())
    {
        for (uint32_t i = 0; i < gnbNetDev->GetCcMapSize(); ++i)
        {
            if (!gnbNetDev->GetPhy(i)->HasDlSlot())
            {
                ueNetDev->GetRrc()->SetPrimaryUlIndex(
                    ueNetDev->GetArfcnBwpId(gnbNetDev->GetBwpArfcn(i)));
                break;
            }
        }
    }

    for (uint32_t i = 0; i < gnbNetDev->GetCcMapSize(); ++i)
    {
        ueNetDev->GetPhy(i)->RegisterToGnb(gnbNetDev->GetCellId());
        const auto& gnbSched = gnbNetDev->GetScheduler(i);
        const auto& gnbPhy = gnbNetDev->GetPhy(i);

        // Search BWP (j) with equivalent frequency on UE that of the configured gNB (i)
        auto gnbArfcn = gnbNetDev->GetBwpArfcn(i);
        auto ueBwpId = ueNetDev->GetArfcnBwpId(gnbArfcn);

        ueNetDev->GetRrc()->ReconfigureFromSib1(ueBwpId,
                                                gnbNetDev->GetCellId(),
                                                gnbSched->GetDlCtrlSyms(),
                                                gnbSched->GetUlCtrlSyms(),
                                                gnbPhy->GetSymbolsPerSlot(),
                                                gnbPhy->GetNumerology(),
                                                gnbPhy->GetPattern(),
                                                gnbPhy->GetNumRbPerRbg());

        Ptr<NrEpcUeNas> ueNas = ueNetDev->GetNas();
        ueNas->Connect(gnbNetDev->GetCellId(), gnbNetDev->GetBwpArfcn(i));
    }

    // The default QoS flow is an IP PDU session, whose rule classifies packets by
    // their IP headers, so it is activated only for a UE that has an IP stack. A UE
    // reaching the network over an unstructured session alone has none.
    Ptr<Node> ueNode = ueDevice->GetNode();
    if (m_nrEpcHelper && (ueNode->GetObject<Ipv4>() || ueNode->GetObject<Ipv6>()))
    {
        // activate default QoS flow, in the home network of the UE
        m_ueEpcHelpers.at(ueDevice)->ActivateQosFlow(ueDevice,
                                                     ueNetDev->GetImsi(),
                                                     NrQosRule::Default(),
                                                     NrQosFlow(NrQosFlow::NGBR_VIDEO_TCP_DEFAULT));
    }

    RegisterUeAtGnb(ueNetDev, gnbNetDev);
}

void
NrHelper::RegisterUeAtGnb(const Ptr<NrUeNetDevice>& ueNetDev, const Ptr<NrGnbNetDevice>& gnbNetDev)
{
    NS_LOG_FUNCTION(this << ueNetDev->GetImsi() << gnbNetDev->GetCellId());

    for (uint32_t i = 0; i < gnbNetDev->GetCcMapSize(); ++i)
    {
        gnbNetDev->GetPhy(i)->RegisterUe(ueNetDev->GetImsi(), ueNetDev);
        if (IsMimoFeedbackEnabled())
        {
            // Initialize parameters for MIMO precoding matrix search (PMI feedback)
            auto pmSearch = m_pmSearchFactory.Create<NrPmSearch>();
            // ntn-patch 25: TS 38.214 Table 5.2.1.4-2 ties the CSI subband size to the BWP
            // width, and one helper-wide SubbandSize cannot fit BWPs of different widths (e.g. a
            // wide primary BWP next to narrow per-slice BWPs). Keep the configured size where it
            // is allowed, otherwise take the largest size this BWP allows.
            {
                const uint32_t prbs = gnbNetDev->GetPhy(i)->GetRbNum();
                const size_t sb = pmSearch->GetSubbandSize();
                const auto allowed = prbs < 24    ? std::vector<size_t>{1}
                                     : prbs <= 72  ? std::vector<size_t>{4, 8}
                                     : prbs <= 144 ? std::vector<size_t>{8, 16}
                                     : prbs <= 275 ? std::vector<size_t>{16, 32}
                                                   : std::vector<size_t>{32};
                if (prbs > 0 && std::find(allowed.begin(), allowed.end(), sb) == allowed.end())
                {
                    pmSearch->SetAttribute("SubbandSize", UintegerValue(allowed.back()));
                }
            }
            ueNetDev->GetPhy(i)->SetPmSearch(pmSearch);
            auto gnbAnt =
                gnbNetDev->GetPhy(i)->GetSpectrumPhy()->GetAntenna()->GetObject<PhasedArrayModel>();
            auto ueAnt =
                ueNetDev->GetPhy(i)->GetSpectrumPhy()->GetAntenna()->GetObject<PhasedArrayModel>();
            pmSearch->SetGnbParams(gnbAnt->IsDualPol(),
                                   gnbAnt->GetNumHorizontalPorts(),
                                   gnbAnt->GetNumVerticalPorts());
            pmSearch->SetUeParams(ueAnt->GetNumPorts());
            pmSearch->InitCodebooks();
        }
    }

    // tricks needed for the simplified LTE-only simulations
    // if (m_nrEpcHelper == 0)
    //{
    ueNetDev->SetTargetGnb(gnbNetDev);
    //}
    m_ueRegistrations[ueNetDev->GetImsi()] = gnbNetDev;

    // A UE returning to a cell keeps the beamforming task of its earlier connection
    if (m_beamformingHelper && m_beamformingTasks.emplace(gnbNetDev, ueNetDev).second)
    {
        m_beamformingHelper->AddBeamformingTask(gnbNetDev, ueNetDev);
    }
}

void
NrHelper::UeConnectedToCell(uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);

    auto itUe = m_ueDevices.find(imsi);
    auto itGnb = m_gnbDevices.find(cellId);
    if (itUe == m_ueDevices.end() || itGnb == m_gnbDevices.end())
    {
        return;
    }
    const auto& ueNetDev = itUe->second;
    const auto& gnbNetDev = itGnb->second;
    // The UE RRC updates the target gNB of the UE device itself, so the cell where the
    // UE is registered is tracked here
    auto itRegistered = m_ueRegistrations.find(imsi);
    if (itRegistered != m_ueRegistrations.end())
    {
        if (itRegistered->second == gnbNetDev)
        {
            return;
        }
        for (uint32_t i = 0; i < itRegistered->second->GetCcMapSize(); ++i)
        {
            itRegistered->second->GetPhy(i)->DeregisterUe(imsi);
        }
    }
    NS_LOG_INFO("UE " << imsi << " connected to cell " << cellId << " with RNTI " << rnti
                      << ", registering it at its gNB");
    RegisterUeAtGnb(ueNetDev, gnbNetDev);
}

uint8_t
NrHelper::ActivateDedicatedQosFlow(NetDeviceContainer ueDevices,
                                   NrQosFlow flow,
                                   Ptr<NrQosRule> rule)
{
    NS_LOG_FUNCTION(this);
    for (auto i = ueDevices.Begin(); i != ueDevices.End(); ++i)
    {
        uint8_t qosFlowId = ActivateDedicatedQosFlow(*i, flow, rule);

        return qosFlowId;
    }
    return 0;
}

uint8_t
NrHelper::ActivateDedicatedQosFlow(Ptr<NetDevice> ueDevice, NrQosFlow flow, Ptr<NrQosRule> rule)
{
    NS_LOG_FUNCTION(this);

    NS_ASSERT_MSG(m_nrEpcHelper, "dedicated QoS flows cannot be set up when the EPC is not used");

    uint64_t imsi = ueDevice->GetObject<NrUeNetDevice>()->GetImsi();
    uint8_t qosFlowId = m_ueEpcHelpers.at(ueDevice)->ActivateQosFlow(ueDevice, imsi, rule, flow);
    return qosFlowId;
}

uint8_t
NrHelper::ActivateUnstructuredQosFlow(Ptr<NetDevice> ueDevice,
                                      uint16_t protocolNumber,
                                      NrQosFlow flow)
{
    NS_LOG_FUNCTION(this << ueDevice << protocolNumber);

    NS_ASSERT_MSG(m_nrEpcHelper,
                  "unstructured PDU sessions cannot be set up when the EPC is not used");

    // in the home network of the UE, as the other QoS flows
    uint64_t imsi = ueDevice->GetObject<NrUeNetDevice>()->GetImsi();
    return m_ueEpcHelpers.at(ueDevice)->ActivateUnstructuredQosFlow(ueDevice,
                                                                    imsi,
                                                                    protocolNumber,
                                                                    flow);
}

void
NrHelper::DeActivateDedicatedQosFlow(Ptr<NetDevice> ueDevice,
                                     Ptr<NetDevice> gnbDevice,
                                     uint8_t qosFlowId)
{
    NS_LOG_FUNCTION(this << ueDevice << qosFlowId);
    NS_ASSERT_MSG(m_nrEpcHelper != nullptr,
                  "Dedicated QoS flows cannot be de-activated when the EPC is not used");
    NS_ASSERT_MSG(qosFlowId != 1,
                  "Default QoS flow cannot be de-activated until and unless and UE is released");

    DoDeActivateDedicatedQosFlow(ueDevice, gnbDevice, qosFlowId);
}

void
NrHelper::SetUeMacAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_ueMacFactory.Set(n, v);
}

void
NrHelper::SetGnbMacAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_gnbMacFactory.Set(n, v);
}

void
NrHelper::SetGnbSpectrumAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_gnbSpectrumFactory.Set(n, v);
}

void
NrHelper::SetUeSpectrumAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_ueSpectrumFactory.Set(n, v);
}

void
NrHelper::SetUeChannelAccessManagerAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_ueChannelAccessManagerFactory.Set(n, v);
}

void
NrHelper::SetGnbChannelAccessManagerAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_gnbChannelAccessManagerFactory.Set(n, v);
}

void
NrHelper::SetSchedulerAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_schedFactory.Set(n, v);
}

void
NrHelper::SetUePhyAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_uePhyFactory.Set(n, v);
}

void
NrHelper::SetGnbPhyAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_gnbPhyFactory.Set(n, v);
}

void
NrHelper::SetUeAntennaAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_ueAntennaFactory.Set(n, v);
}

void
NrHelper::SetGnbAntennaAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_gnbAntennaFactory.Set(n, v);
}

void
NrHelper::SetUeAntennaTypeId(const std::string& typeId)
{
    NS_LOG_FUNCTION(this);
    m_ueAntennaFactory.SetTypeId(typeId);
}

void
NrHelper::SetGnbAntennaTypeId(const std::string& typeId)
{
    NS_LOG_FUNCTION(this);
    m_gnbAntennaFactory.SetTypeId(typeId);
}

void
NrHelper::SetUeChannelAccessManagerTypeId(const TypeId& typeId)
{
    NS_LOG_FUNCTION(this);
    m_ueChannelAccessManagerFactory.SetTypeId(typeId);
}

void
NrHelper::SetGnbChannelAccessManagerTypeId(const TypeId& typeId)
{
    NS_LOG_FUNCTION(this);
    m_gnbChannelAccessManagerFactory.SetTypeId(typeId);
}

void
NrHelper::SetSchedulerTypeId(const TypeId& typeId)
{
    NS_LOG_FUNCTION(this);
    m_schedFactory.SetTypeId(typeId);
}

void
NrHelper::SetSchedulerTypeIdForBwp(uint16_t bwpId, const TypeId& typeId)
{
    NS_LOG_FUNCTION(this);
    m_schedTypeIdPerBwp[bwpId] = typeId;
}

void
NrHelper::SetUeBwpManagerAlgorithmTypeId(const TypeId& typeId)
{
    NS_LOG_FUNCTION(this);
    m_ueBwpManagerAlgoFactory.SetTypeId(typeId);
}

void
NrHelper::SetUeBwpManagerAlgorithmAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_ueBwpManagerAlgoFactory.Set(n, v);
}

void
NrHelper::SetGnbDlAmcAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_gnbDlAmcFactory.Set(n, v);
}

void
NrHelper::SetGnbUlAmcAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_gnbUlAmcFactory.Set(n, v);
}

void
NrHelper::SetGnbBeamManagerAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_gnbBeamManagerFactory.Set(n, v);
}

void
NrHelper::SetGnbBeamManagerTypeId(const TypeId& typeId)
{
    NS_LOG_FUNCTION(this);
    m_gnbBeamManagerFactory.SetTypeId(typeId);
}

void
NrHelper::SetFhControlAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_fhControlFactory.Set(n, v);
}

void
NrHelper::SetUlErrorModel(const std::string& errorModelTypeId)
{
    NS_LOG_FUNCTION(this);

    SetGnbUlAmcAttribute("ErrorModelType", TypeIdValue(TypeId::LookupByName(errorModelTypeId)));
    SetGnbSpectrumAttribute("ErrorModelType", TypeIdValue(TypeId::LookupByName(errorModelTypeId)));
}

void
NrHelper::SetDlErrorModel(const std::string& errorModelTypeId)
{
    NS_LOG_FUNCTION(this);

    SetGnbDlAmcAttribute("ErrorModelType", TypeIdValue(TypeId::LookupByName(errorModelTypeId)));
    SetUeSpectrumAttribute("ErrorModelType", TypeIdValue(TypeId::LookupByName(errorModelTypeId)));
}

void
NrHelper::EnableFhControl()
{
    m_fhEnabled = true;
}

void
NrHelper::ConfigureFhControl(NetDeviceContainer gnbNetDevices)
{
    for (auto i = gnbNetDevices.Begin(); i != gnbNetDevices.End(); ++i)
    {
        Ptr<NrGnbNetDevice> gnbNetDev = DynamicCast<NrGnbNetDevice>(*i);

        for (uint32_t j = 0; j < gnbNetDev->GetCcMapSize(); j++)
        {
            gnbNetDev->GetNrFhControl()->SetFhNumerology(j, gnbNetDev->GetPhy(j)->GetNumerology());

            gnbNetDev->GetNrFhControl()->SetErrorModelType(
                DynamicCast<NrMacSchedulerNs3>(gnbNetDev->GetScheduler(j))
                    ->GetDlAmc()
                    ->GetErrorModelType()
                    .GetName());
        }
    }
}

int64_t
NrHelper::AssignStreams(NetDeviceContainer c, int64_t stream)
{
    int64_t currentStream = stream;
    Ptr<NetDevice> netDevice;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        netDevice = (*i);
        Ptr<NrGnbNetDevice> nrGnb = DynamicCast<NrGnbNetDevice>(netDevice);
        if (nrGnb)
        {
            for (uint32_t bwp = 0; bwp < nrGnb->GetCcMapSize(); bwp++)
            {
                currentStream += nrGnb->GetPhy(bwp)->AssignStreams(currentStream);
                currentStream += nrGnb->GetScheduler(bwp)->AssignStreams(currentStream);
                currentStream +=
                    DoAssignStreamsToChannelObjects(nrGnb->GetPhy(bwp)->GetSpectrumPhy(),
                                                    currentStream);
            }
        }

        Ptr<NrUeNetDevice> nrUe = DynamicCast<NrUeNetDevice>(netDevice);
        if (nrUe)
        {
            for (uint32_t bwp = 0; bwp < nrUe->GetCcMapSize(); bwp++)
            {
                Ptr<NrPmSearch> pmSearch = nrUe->GetPhy(bwp)->GetPmSearch();
                if (pmSearch)
                {
                    currentStream += nrUe->GetPhy(bwp)->GetPmSearch()->AssignStreams(currentStream);
                }
                currentStream += nrUe->GetPhy(bwp)->GetSpectrumPhy()->AssignStreams(currentStream);
                currentStream += nrUe->GetMac(bwp)->AssignStreams(currentStream);
                currentStream +=
                    DoAssignStreamsToChannelObjects(nrUe->GetPhy(bwp)->GetSpectrumPhy(),
                                                    currentStream);
            }
        }
    }

    return (currentStream - stream);
}

int64_t
NrHelper::AssignStreams(const StreamAssignmentParams& params)
{
    int64_t assigned = 0;

    if (params.assignEpc)
    {
        NS_ABORT_MSG_UNLESS(m_nrEpcHelper,
                            "AssignStreams: assignEpc requested but no EPC helper was set");
        assigned += m_nrEpcHelper->AssignStreams(params.epcStream);
    }

    // InternetStackHelper holds no relevant state for AssignStreams; it merely
    // walks the IPv4/IPv6 protocols installed on the given nodes.
    InternetStackHelper internet;
    if (params.remoteHostNodes.GetN() > 0)
    {
        assigned += internet.AssignStreams(params.remoteHostNodes, params.remoteHostStream);
    }
    if (params.ueNodes.GetN() > 0)
    {
        assigned += internet.AssignStreams(params.ueNodes, params.ueNodeStream);
    }
    if (params.gnbNodes.GetN() > 0)
    {
        assigned += internet.AssignStreams(params.gnbNodes, params.gnbNodeStream);
    }

    if (params.scenario)
    {
        assigned += params.scenario->AssignStreams(params.scenarioStream);
    }

    if (params.beamformingHelper)
    {
        // Only the realistic beamforming helper draws random variables.
        if (auto realisticBf = DynamicCast<RealisticBeamformingHelper>(params.beamformingHelper))
        {
            assigned += realisticBf->AssignStreams(params.beamformingStream);
        }
    }

    if (params.gnbDevs.GetN() > 0)
    {
        assigned += AssignStreams(params.gnbDevs, params.gnbDevStream);
    }
    if (params.ueDevs.GetN() > 0)
    {
        assigned += AssignStreams(params.ueDevs, params.ueDevStream);
    }

    return assigned;
}

int64_t
NrHelper::DoAssignStreamsToChannelObjects(Ptr<NrSpectrumPhy> phy, int64_t currentStream)
{
    int64_t initialStream = currentStream;

    auto assignConditionModelStreams = [&](Ptr<ChannelConditionModel> channelConditionModel) {
        if (channelConditionModel &&
            std::find(m_channelObjectsWithAssignedStreams.begin(),
                      m_channelObjectsWithAssignedStreams.end(),
                      channelConditionModel) == m_channelObjectsWithAssignedStreams.end())
        {
            currentStream += channelConditionModel->AssignStreams(currentStream);
            m_channelObjectsWithAssignedStreams.emplace_back(channelConditionModel);
        }
    };

    auto propagationLossModel = phy->GetSpectrumChannel()->GetPropagationLossModel();
    Ptr<ChannelConditionModel> channelConditionModel;
    if (propagationLossModel &&
        std::find(m_channelObjectsWithAssignedStreams.begin(),
                  m_channelObjectsWithAssignedStreams.end(),
                  propagationLossModel) == m_channelObjectsWithAssignedStreams.end())
    {
        auto prop3gppModel = DynamicCast<ThreeGppPropagationLossModel>(propagationLossModel);
        auto propNyuModel = DynamicCast<NYUPropagationLossModel>(propagationLossModel);
        if (prop3gppModel)
        {
            channelConditionModel = prop3gppModel->GetChannelConditionModel();
        }
        else if (propNyuModel)
        {
            channelConditionModel = propNyuModel->GetChannelConditionModel();
        }
        else
        {
            channelConditionModel = nullptr;
        }
        currentStream += propagationLossModel->AssignStreams(currentStream);
        m_channelObjectsWithAssignedStreams.emplace_back(propagationLossModel);
    }
    assignConditionModelStreams(channelConditionModel);

    auto spectrumLossModel =
        phy->GetSpectrumChannel()->GetPhasedArraySpectrumPropagationLossModel();

    if (spectrumLossModel)
    {
        if (std::find(m_channelObjectsWithAssignedStreams.begin(),
                      m_channelObjectsWithAssignedStreams.end(),
                      spectrumLossModel) == m_channelObjectsWithAssignedStreams.end())
        {
            auto tgppSpecModel =
                DynamicCast<ThreeGppSpectrumPropagationLossModel>(spectrumLossModel);
            auto nyuSpecModel = DynamicCast<NYUSpectrumPropagationLossModel>(spectrumLossModel);
            auto twoRaySpectrumModel =
                DynamicCast<TwoRaySpectrumPropagationLossModel>(spectrumLossModel);
            Ptr<Object> fadingModel = spectrumLossModel;
            if (twoRaySpectrumModel)
            {
                currentStream += twoRaySpectrumModel->AssignStreams(currentStream);
            }
            else if (tgppSpecModel)
            {
                fadingModel = tgppSpecModel->GetChannelModel();
                // ntn-patch 09: another MatrixBasedChannelModel (e.g. a deterministic
                // LOS model) may be plugged in; it has no random streams to assign.
                if (auto channel = DynamicCast<ThreeGppChannelModel>(fadingModel))
                {
                    currentStream += channel->AssignStreams(currentStream);
                }
            }
            else if (nyuSpecModel)
            {
                fadingModel = nyuSpecModel->GetChannelModel();
                currentStream +=
                    StaticCast<NYUChannelModel>(fadingModel)->AssignStreams(currentStream);
            }
            m_channelObjectsWithAssignedStreams.emplace_back(spectrumLossModel);

            // The fading model shares the channel condition model of the propagation loss model,
            // but it is the only way to reach it when no propagation loss model is installed
            PointerValue fadingConditionModel;
            if (fadingModel->GetAttributeFailSafe("ChannelConditionModel", fadingConditionModel))
            {
                assignConditionModelStreams(fadingConditionModel.Get<ChannelConditionModel>());
            }
        }
    }

    return currentStream - initialStream;
}

void
NrHelper::SetGnbBwpManagerAlgorithmTypeId(const TypeId& typeId)
{
    NS_LOG_FUNCTION(this);
    m_gnbBwpManagerAlgoFactory.SetTypeId(typeId);
}

void
NrHelper::SetGnbBwpManagerAlgorithmAttribute(const std::string& n, const AttributeValue& v)
{
    NS_LOG_FUNCTION(this);
    m_gnbBwpManagerAlgoFactory.Set(n, v);
}

void
NrHelper::DoDeActivateDedicatedQosFlow(Ptr<NetDevice> ueDevice,
                                       Ptr<NetDevice> gnbDevice,
                                       uint8_t qosFlowId)
{
    NS_LOG_FUNCTION(this << ueDevice << qosFlowId);

    // Extract IMSI and rnti
    uint64_t imsi = ueDevice->GetObject<NrUeNetDevice>()->GetImsi();
    uint16_t rnti = ueDevice->GetObject<NrUeNetDevice>()->GetRrc()->GetRnti();

    Ptr<NrGnbRrc> gnbRrc = gnbDevice->GetObject<NrGnbNetDevice>()->GetRrc();

    gnbRrc->DoSendReleaseQosFlow(imsi, rnti, qosFlowId);
}

void
NrHelper::SetEpcHelper(Ptr<NrEpcHelper> NrEpcHelper)
{
    m_nrEpcHelper = NrEpcHelper;
}

void
NrHelper::SetBeamformingHelper(Ptr<BeamformingHelperBase> beamformingHelper)
{
    m_beamformingHelper = beamformingHelper;
    m_beamformingHelper->Initialize();
}

class NrDrbActivator : public SimpleRefCount<NrDrbActivator>
{
  public:
    NrDrbActivator(Ptr<NetDevice> ueDevice, NrQosFlow flow);
    static void ActivateCallback(Ptr<NrDrbActivator> a,
                                 std::string context,
                                 uint64_t imsi,
                                 uint16_t cellId,
                                 uint16_t rnti);
    void ActivateDrb(uint64_t imsi, uint16_t cellId, uint16_t rnti);

  private:
    bool m_active;
    Ptr<NetDevice> m_ueDevice;
    NrQosFlow m_flow;
    uint64_t m_imsi;
};

NrDrbActivator::NrDrbActivator(Ptr<NetDevice> ueDevice, NrQosFlow flow)
    : m_active(false),
      m_ueDevice(ueDevice),
      m_flow(flow),
      m_imsi(m_ueDevice->GetObject<NrUeNetDevice>()->GetImsi())
{
}

void
NrDrbActivator::ActivateCallback(Ptr<NrDrbActivator> a,
                                 std::string context,
                                 uint64_t imsi,
                                 uint16_t cellId,
                                 uint16_t rnti)
{
    NS_LOG_FUNCTION(a << context << imsi << cellId << rnti);
    a->ActivateDrb(imsi, cellId, rnti);
}

void
NrDrbActivator::ActivateDrb(uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti << m_active);
    if ((!m_active) && (imsi == m_imsi))
    {
        Ptr<NrUeRrc> ueRrc = m_ueDevice->GetObject<NrUeNetDevice>()->GetRrc();
        NS_ASSERT(ueRrc->GetState() == NrUeRrc::CONNECTED_NORMALLY);
        NS_ASSERT(rnti == ueRrc->GetRnti());
        Ptr<const NrGnbNetDevice> nrGnbDevice =
            m_ueDevice->GetObject<NrUeNetDevice>()->GetTargetGnb();
        Ptr<NrGnbRrc> gnbRrc = nrGnbDevice->GetObject<NrGnbNetDevice>()->GetRrc();
        NS_ASSERT(gnbRrc->HasCellId(ueRrc->GetCellId()));
        Ptr<NrUeManager> ueManager = gnbRrc->GetUeManager(rnti);
        NS_ASSERT(ueManager->GetState() == NrUeManager::CONNECTED_NORMALLY ||
                  ueManager->GetState() == NrUeManager::CONNECTION_RECONFIGURATION);
        NrEpcGnbS1SapUser::DataRadioBearerSetupRequestParameters params;
        params.rnti = rnti;
        params.flow = m_flow;
        params.qfi = 0;
        params.gtpTeid = 0; // don't care
        gnbRrc->GetS1SapUser()->DataRadioBearerSetupRequest(params);
        m_active = true;
    }
}

void
NrHelper::ActivateDataRadioBearer(NetDeviceContainer ueDevices, NrQosFlow flow)
{
    NS_LOG_FUNCTION(this);
    for (auto i = ueDevices.Begin(); i != ueDevices.End(); ++i)
    {
        ActivateDataRadioBearer(*i, flow);
    }
}

void
NrHelper::ActivateDataRadioBearer(Ptr<NetDevice> ueDevice, NrQosFlow flow)
{
    NS_LOG_FUNCTION(this << ueDevice);
    NS_ASSERT_MSG(!m_nrEpcHelper, "this method must not be used when the EPC is being used");

    // Normally it is the EPC that takes care of activating DRBs
    // when the UE gets connected. When the EPC is not used, we achieve
    // the same behavior by hooking a dedicated DRB activation function
    // to the Gnb RRC Connection Established trace source

    Ptr<const NrGnbNetDevice> nrGnbDevice = ueDevice->GetObject<NrUeNetDevice>()->GetTargetGnb();

    std::ostringstream path;
    path << "/NodeList/" << nrGnbDevice->GetNode()->GetId() << "/DeviceList/"
         << nrGnbDevice->GetIfIndex() << "/NrGnbRrc/ConnectionEstablished";
    Ptr<NrDrbActivator> arg = Create<NrDrbActivator>(ueDevice, flow);
    Config::Connect(path.str(), MakeBoundCallback(&NrDrbActivator::ActivateCallback, arg));
}

void
NrHelper::EnableTraces()
{
    EnableDlDataPhyTraces();
    EnableDlCtrlPhyTraces();
    EnableUlPhyTraces();
    // EnableGnbPacketCountTrace ();
    // EnableUePacketCountTrace ();
    // EnableTransportBlockTrace ();
    EnableRlcSimpleTraces();
    EnableRlcE2eTraces();
    EnablePdcpSimpleTraces();
    EnablePdcpE2eTraces();
    EnableGnbPhyCtrlMsgsTraces();
    EnableUePhyCtrlMsgsTraces();
    EnableGnbMacCtrlMsgsTraces();
    EnableUeMacCtrlMsgsTraces();
    EnableDlMacSchedTraces();
    EnableUlMacSchedTraces();
    EnablePathlossTraces();
}

Ptr<NrPhyRxTrace>
NrHelper::GetPhyRxTrace()
{
    if (!m_phyStats)
    {
        m_phyStats = CreateObject<NrPhyRxTrace>();
    }
    return m_phyStats;
}

Ptr<NrMacRxTrace>
NrHelper::GetMacRxTrace()
{
    if (!m_macStats)
    {
        m_macStats = CreateObject<NrMacRxTrace>();
    }
    return m_macStats;
}

void
NrHelper::EnableDlDataPhyTraces()
{
    NS_LOG_FUNCTION(this);
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlDataSinr",
                    MakeBoundCallback(&NrPhyRxTrace::DlDataSinrCallback, GetPhyRxTrace()));

    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/SpectrumPhy/RxPacketTraceUe",
        MakeBoundCallback(&NrPhyRxTrace::RxPacketTraceUeCallback, GetPhyRxTrace()));
}

void
NrHelper::EnableDlCtrlPhyTraces()
{
    NS_LOG_FUNCTION(this);
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlCtrlSinr",
                    MakeBoundCallback(&NrPhyRxTrace::DlCtrlSinrCallback, GetPhyRxTrace()));
}

void
NrHelper::EnableGnbPhyCtrlMsgsTraces()
{
    NS_LOG_FUNCTION(this);
    Config::Connect("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/GnbPhyRxedCtrlMsgsTrace",
                    MakeBoundCallback(&NrPhyRxTrace::RxedGnbPhyCtrlMsgsCallback, GetPhyRxTrace()));
    Config::Connect("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/GnbPhyTxedCtrlMsgsTrace",
                    MakeBoundCallback(&NrPhyRxTrace::TxedGnbPhyCtrlMsgsCallback, GetPhyRxTrace()));
}

void
NrHelper::EnableGnbMacCtrlMsgsTraces()
{
    NS_LOG_FUNCTION(this);
    Config::Connect("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbMac/GnbMacRxedCtrlMsgsTrace",
                    MakeBoundCallback(&NrMacRxTrace::RxedGnbMacCtrlMsgsCallback, GetMacRxTrace()));
    Config::Connect("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbMac/GnbMacTxedCtrlMsgsTrace",
                    MakeBoundCallback(&NrMacRxTrace::TxedGnbMacCtrlMsgsCallback, GetMacRxTrace()));
}

void
NrHelper::EnableUePhyCtrlMsgsTraces()
{
    NS_LOG_FUNCTION(this);
    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/UePhyRxedCtrlMsgsTrace",
        MakeBoundCallback(&NrPhyRxTrace::RxedUePhyCtrlMsgsCallback, GetPhyRxTrace()));
    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/UePhyTxedCtrlMsgsTrace",
        MakeBoundCallback(&NrPhyRxTrace::TxedUePhyCtrlMsgsCallback, GetPhyRxTrace()));
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/UePhyRxedDlDciTrace",
                    MakeBoundCallback(&NrPhyRxTrace::RxedUePhyDlDciCallback, GetPhyRxTrace()));
    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/UePhyTxedHarqFeedbackTrace",
        MakeBoundCallback(&NrPhyRxTrace::TxedUePhyHarqFeedbackCallback, GetPhyRxTrace()));
}

void
NrHelper::EnableUeMacCtrlMsgsTraces()
{
    NS_LOG_FUNCTION(this);
    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUeMac/UeMacRxedCtrlMsgsTrace",
        MakeBoundCallback(&NrMacRxTrace::RxedUeMacCtrlMsgsCallback, GetMacRxTrace()));
    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUeMac/UeMacTxedCtrlMsgsTrace",
        MakeBoundCallback(&NrMacRxTrace::TxedUeMacCtrlMsgsCallback, GetMacRxTrace()));
}

void
NrHelper::EnableUlPhyTraces()
{
    NS_LOG_FUNCTION(this);
    Config::Connect(
        "/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/SpectrumPhy/RxPacketTraceGnb",
        MakeBoundCallback(&NrPhyRxTrace::RxPacketTraceGnbCallback, GetPhyRxTrace()));
}

void
NrHelper::EnableGnbPacketCountTrace()
{
    NS_LOG_FUNCTION(this);
    Config::Connect(
        "/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/SpectrumPhy/ReportGnbTxRxPacketCount",
        MakeBoundCallback(&NrPhyRxTrace::ReportPacketCountGnbCallback, GetPhyRxTrace()));
}

void
NrHelper::EnableUePacketCountTrace()
{
    NS_LOG_FUNCTION(this);
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/SpectrumPhy/"
                    "ReportUeTxRxPacketCount",
                    MakeBoundCallback(&NrPhyRxTrace::ReportPacketCountUeCallback, GetPhyRxTrace()));
}

void
NrHelper::EnableTransportBlockTrace()
{
    NS_LOG_FUNCTION(this);
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/ReportDownlinkTbSize",
                    MakeBoundCallback(&NrPhyRxTrace::ReportDownLinkTBSize, GetPhyRxTrace()));
}

void
NrHelper::EnableRlcSimpleTraces()
{
    NS_LOG_FUNCTION(this);
    Ptr<NrBearerStatsSimple> rlcStats = CreateObject<NrBearerStatsSimple>("RLC");
    m_radioBearerStatsConnectorSimpleTraces.EnableRlcStats(rlcStats);
}

void
NrHelper::EnablePdcpSimpleTraces()
{
    NS_LOG_FUNCTION(this);
    Ptr<NrBearerStatsSimple> pdcpStats = CreateObject<NrBearerStatsSimple>("PDCP");
    m_radioBearerStatsConnectorSimpleTraces.EnablePdcpStats(pdcpStats);
}

void
NrHelper::EnableRlcE2eTraces()
{
    NS_LOG_FUNCTION(this);
    Ptr<NrBearerStatsCalculator> rlcStats = CreateObject<NrBearerStatsCalculator>("RLC");
    m_radioBearerStatsConnectorCalculator.EnableRlcStats(rlcStats);
}

void
NrHelper::EnablePdcpE2eTraces()
{
    NS_LOG_FUNCTION(this);
    Ptr<NrBearerStatsCalculator> pdcpStats = CreateObject<NrBearerStatsCalculator>("PDCP");
    m_radioBearerStatsConnectorCalculator.EnablePdcpStats(pdcpStats);
}

Ptr<NrBearerStatsCalculator>
NrHelper::GetRlcStatsCalculator()
{
    return DynamicCast<NrBearerStatsCalculator>(
        m_radioBearerStatsConnectorCalculator.GetRlcStats());
}

Ptr<NrBearerStatsCalculator>
NrHelper::GetPdcpStatsCalculator()
{
    return DynamicCast<NrBearerStatsCalculator>(
        m_radioBearerStatsConnectorCalculator.GetPdcpStats());
}

void
NrHelper::EnableDlMacSchedTraces()
{
    NS_LOG_FUNCTION(this);
    if (!m_macSchedStats)
    {
        m_macSchedStats = CreateObject<NrMacSchedulingStats>();
    }
    Config::Connect(
        "/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbMac/DlScheduling",
        MakeBoundCallback(&NrMacSchedulingStats::DlSchedulingCallback, m_macSchedStats));
}

void
NrHelper::EnableUlMacSchedTraces()
{
    NS_LOG_FUNCTION(this);
    if (!m_macSchedStats)
    {
        m_macSchedStats = CreateObject<NrMacSchedulingStats>();
    }
    Config::Connect(
        "/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbMac/UlScheduling",
        MakeBoundCallback(&NrMacSchedulingStats::UlSchedulingCallback, m_macSchedStats));
}

void
NrHelper::EnablePathlossTraces()
{
    NS_LOG_FUNCTION(this);
    Config::Connect("/ChannelList/*/$ns3::SpectrumChannel/PathLoss",
                    MakeBoundCallback(&NrPhyRxTrace::PathlossTraceCallback, GetPhyRxTrace()));
}

void
NrHelper::EnableDlCtrlPathlossTraces(NetDeviceContainer& ueDevs)
{
    NS_LOG_FUNCTION(this);

    for (uint32_t i = 0; i < ueDevs.GetN(); i++)
    {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(i));
        NS_ASSERT_MSG(ueDev,
                      "To EnableDlCtrlPathlossTracesfunction is passed device "
                      "container that contains non UE devices.");
        for (uint32_t j = 0; j < ueDev->GetCcMapSize(); j++)
        {
            Ptr<NrUePhy> nrUePhy = ueDev->GetPhy(j);
            Ptr<NrSpectrumPhy> nrSpectrumPhy = nrUePhy->GetSpectrumPhy();
            nrSpectrumPhy->EnableDlCtrlPathlossTrace();
        }
    }

    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/NrSpectrumPhy/"
                    "DlCtrlPathloss",
                    MakeBoundCallback(&NrPhyRxTrace::ReportDlCtrlPathloss, GetPhyRxTrace()));
}

void
NrHelper::EnableDlDataPathlossTraces(NetDeviceContainer& ueDevs)
{
    NS_LOG_FUNCTION(this);

    NS_ASSERT_MSG(ueDevs.GetN(),
                  "Passed an empty UE net device container EnableDlDataPathlossTraces function");

    for (uint32_t i = 0; i < ueDevs.GetN(); i++)
    {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(i));
        NS_ASSERT_MSG(ueDev,
                      "To EnableDlDataPathlossTracesfunction is passed device "
                      "container that contains non UE devices.");
        for (uint32_t j = 0; j < ueDev->GetCcMapSize(); j++)
        {
            Ptr<NrUePhy> nrUePhy = ueDev->GetPhy(j);
            Ptr<NrSpectrumPhy> nrSpectrumPhy = nrUePhy->GetSpectrumPhy();
            nrSpectrumPhy->EnableDlDataPathlossTrace();
        }
    }

    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/NrSpectrumPhy/"
                    "DlDataPathloss",
                    MakeBoundCallback(&NrPhyRxTrace::ReportDlDataPathloss, GetPhyRxTrace()));
}

void
NrHelper::SetPmSearchTypeId(const TypeId& typeId)
{
    m_pmSearchFactory.SetTypeId(typeId);
}

void
NrHelper::SetInitialAssocTypeId(const TypeId& typeId)
{
    m_initialAttachmentFactory.SetTypeId(typeId);
}

void
NrHelper::SetPmSearchAttribute(const std::string& name, const AttributeValue& value)
{
    NS_LOG_FUNCTION(this);
    m_pmSearchFactory.Set(name, value);
}

void
NrHelper::SetInitialAssocAttribute(const std::string& name, const AttributeValue& value)
{
    NS_LOG_FUNCTION(this);
    m_initialAttachmentFactory.Set(name, value);
}

void
NrHelper::SetupGnbAntennas(const NrHelper::AntennaParams& ap)
{
    NS_ASSERT_MSG(((ap.nAntCols % ap.nHorizPorts) == 0),
                  "The number of horizontal ports of gNB must divide number of element columns");
    NS_ASSERT_MSG(((ap.nAntRows % ap.nVertPorts) == 0),
                  "The number of vertical ports of gNB must divide number of element rows");

    auto antFactory = ObjectFactory{};
    antFactory.SetTypeId(ap.antennaElem);
    SetGnbAntennaAttribute("AntennaElement", PointerValue(antFactory.Create()));
    SetGnbAntennaAttribute("NumColumns", UintegerValue(ap.nAntCols));
    SetGnbAntennaAttribute("NumRows", UintegerValue(ap.nAntRows));
    SetGnbAntennaAttribute("IsDualPolarized", BooleanValue(ap.isDualPolarized));
    SetGnbAntennaAttribute("NumHorizontalPorts", UintegerValue(ap.nHorizPorts));
    SetGnbAntennaAttribute("NumVerticalPorts", UintegerValue(ap.nVertPorts));
    SetGnbAntennaAttribute("BearingAngle", DoubleValue(ap.bearingAngle));
    SetGnbAntennaAttribute("PolSlantAngle", DoubleValue(ap.polSlantAngle));
    SetGnbAntennaAttribute("DowntiltAngle", DoubleValue(ap.downtiltAngle));
}

void
NrHelper::SetupUeAntennas(const NrHelper::AntennaParams& ap)
{
    NS_ASSERT_MSG(((ap.nAntCols % ap.nHorizPorts) == 0),
                  "The number of horizontal ports of UE must divide number of element columns");
    NS_ASSERT_MSG(((ap.nAntRows % ap.nVertPorts) == 0),
                  "The number of vertical ports of UE must divide number of element rows");

    auto antFactory = ObjectFactory{};
    antFactory.SetTypeId(ap.antennaElem);
    SetUeAntennaAttribute("AntennaElement", PointerValue(antFactory.Create()));
    SetUeAntennaAttribute("NumColumns", UintegerValue(ap.nAntCols));
    SetUeAntennaAttribute("NumRows", UintegerValue(ap.nAntRows));
    SetUeAntennaAttribute("IsDualPolarized", BooleanValue(ap.isDualPolarized));
    SetUeAntennaAttribute("NumHorizontalPorts", UintegerValue(ap.nHorizPorts));
    SetUeAntennaAttribute("NumVerticalPorts", UintegerValue(ap.nVertPorts));
    SetUeAntennaAttribute("BearingAngle", DoubleValue(ap.bearingAngle));
    SetUeAntennaAttribute("PolSlantAngle", DoubleValue(ap.polSlantAngle));
    SetUeAntennaAttribute("DowntiltAngle", DoubleValue(ap.downtiltAngle));
}

void
NrHelper::SetupMimoPmi(const NrHelper::MimoPmiParams& mp)
{
    // If NrHelper is using default PDSCH_SISO CSI feedback flag,
    // replace it with PDSCH_MIMO to implicitly enable MIMO feedback
    if (m_csiFeedbackFlags == CQI_PDSCH_SISO)
    {
        SetAttribute("CsiFeedbackFlags", UintegerValue(CQI_PDSCH_MIMO));
    }
    // Set parameters for MIMO precoding matrix search
    auto searchTypeId = TypeId::LookupByName(mp.pmSearchMethod);
    SetPmSearchTypeId(searchTypeId);
    SetPmSearchAttribute("RankLimit", UintegerValue(mp.rankLimit));
    SetPmSearchAttribute("RankThreshold", DoubleValue(mp.rankThreshold));
    SetPmSearchAttribute("RankTechnique", StringValue(mp.rankTechnique));
    SetPmSearchAttribute("SubbandSize", UintegerValue(mp.subbandSize));
    SetPmSearchAttribute("DownsamplingTechnique", StringValue(mp.downsamplingTechnique));
    if (searchTypeId == NrPmSearchFull::GetTypeId() ||
        searchTypeId.GetParent() == NrPmSearchFull::GetTypeId())
    {
        SetPmSearchAttribute("CodebookType", TypeIdValue(TypeId::LookupByName(mp.fullSearchCb)));
    }
}

void
NrHelper::SetupInitialAssoc(const NrHelper::InitialAssocParams& params)
{
    // Set parameters for Initial Association Params
    m_initialParams = params;
    SetInitialAssocAttribute("HandoffMargin", DoubleValue(params.handoffMargin));
    SetInitialAssocAttribute("PrimaryCarrierIndex", UintegerValue(params.primaryCarrierIndex));
}

bool
NrHelper::IsMimoFeedbackEnabled() const
{
    if (m_csiFeedbackFlags == CQI_PDSCH_SISO)
    {
        return false;
    }
    if ((m_csiFeedbackFlags == CQI_PDSCH_MIMO) || (m_csiFeedbackFlags == CQI_CSI_RS) ||
        (m_csiFeedbackFlags == (CQI_PDSCH_MIMO | CQI_CSI_RS)) ||
        (m_csiFeedbackFlags == (CQI_PDSCH_MIMO | CQI_CSI_RS | CQI_CSI_IM)) ||
        (m_csiFeedbackFlags == (CQI_CSI_RS | CQI_CSI_IM)))
    {
        return true;
    }
    NS_ABORT_MSG("Unsupported NrHelper::CsiFeedbackFlags combination");
}

} // namespace ns3

// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file nr-external-core-roaming.cc
 * @ingroup examples
 *
 * Roaming between a core network of the simulation and an external 5G core network, served
 * through nr-bridge (https://github.com/Gabrielcarvfer/PacketRusher/tree/nr-bridge).
 *
 * A UE, subscriber of the home core network of the simulation (NrPointToPointEpcHelper), starts
 * attached to the home gNB. It then moves to the coverage of a gNB of the visited network, an
 * external 5G core (e.g., Open5GS) reached by nr-bridge, and comes back home:
 *
 *   home gNB --- home core (ns-3) --- remote host
 *   visited gNB --N3 (EmuFdNetDevice)-- UPF of the external core
 *                \--- nr-bridge (NGAP, NAS) --- AMF of the external core
 *                                         \--- SEPP, as the home network of the UE
 *
 * The bridge runs the NGAP and NAS procedures of the visited gNB and of the UE, and reports
 * the user-plane parameters of the PDU session, which the visited gNB uses to tunnel the
 * traffic of the UE to the UPF. As the SEPP of the home network, it tells the simulation when
 * the visited network serves the UE (NrEpcMmeApplication::ExternalUpdateLocation()), and the
 * home network asks it to release the UE when the UE attaches at home again.
 *
 * The UE pings the remote host of the home network, and the gateway of the PDU sessions of the
 * external core, an internet host, and a node of the simulation in the DN of the external core
 * (an EmuFdNetDevice that the UPF reaches through N6); each one answers only while the UE is
 * served by its network. The simulation
 * runs in real time, and must run on a host whose EmuDevice shares a link with the UPF.
 *
 * See utils/open5gs-roaming/README.md for the setup with Open5GS.
 */

#include "ns3/core-module.h"
#include "ns3/fd-net-device-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <map>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrExternalCoreRoaming");

/**
 * Get a field of a flat JSON object, as the bridge sends them.
 *
 * @param json the JSON object
 * @param key the name of the field
 * @return the value of the field (without quotes), or an empty string if absent
 */
static std::string
JsonField(const std::string& json, const std::string& key)
{
    const auto pos = json.find("\"" + key + "\":");
    if (pos == std::string::npos)
    {
        return "";
    }
    auto start = pos + key.size() + 3;
    if (json[start] == '"')
    {
        start++;
        return json.substr(start, json.find('"', start) - start);
    }
    return json.substr(start, json.find_first_of(",}", start) - start);
}

/**
 * @ingroup examples
 * @brief The RAN side of an external 5G core network, reached through nr-bridge.
 *
 * Each gNB gets an N3 interface on an EmuFdNetDevice, and the bridge establishes its NGAP
 * association. The S1-AP messages of the gNBs are relayed to the bridge, which runs the NGAP and
 * NAS procedures of the UEs and returns the parameters of their PDU sessions. The bridge is also
 * the SEPP of the home network of the UEs, a core network of the simulation.
 */
class NrExternalCoreEpcHelper : public NrRanEpcHelper
{
  public:
    /**
     * Register this type.
     * @return The object TypeId.
     */
    static TypeId GetTypeId();

    NrExternalCoreEpcHelper();
    ~NrExternalCoreEpcHelper() override;

    /**
     * Set the home network of the UEs, whose MME learns when the external network serves them.
     *
     * @param home the EPC helper of the home network
     * @param mcc the MCC of the home network
     * @param mnc the MNC of the home network
     */
    void SetHomeNetwork(Ptr<NrNoBackhaulEpcHelper> home, std::string mcc, std::string mnc);

    /**
     * Register a UE that may attach to the external network.
     *
     * @param ue the NR device of the UE
     * @param homeAddress the address of the UE in its home network
     */
    void AddRoamingUe(Ptr<NrUeNetDevice> ue, Ipv4Address homeAddress);

    // inherited from NrEpcHelper
    void AddGnb(Ptr<Node> gnbNode, Ptr<NetDevice> nrGnbNetDevice, uint16_t cellId) override;
    void AddUe(Ptr<NetDevice> ueNrDevice, uint64_t imsi) override;
    uint8_t ActivateQosFlow(Ptr<NetDevice> ueNrDevice,
                            uint64_t imsi,
                            Ptr<NrQosRule> rule,
                            NrQosFlow flow) override;
    Ptr<Node> GetSgwNode() const override;
    Ptr<Node> GetPgwNode() const override;
    Ipv4InterfaceContainer AssignUeIpv4Address(NetDeviceContainer ueDevices) override;
    Ipv6InterfaceContainer AssignUeIpv6Address(NetDeviceContainer ueDevices) override;
    Ipv4Address GetUeDefaultGatewayAddress() override;
    Ipv6Address GetUeDefaultGatewayAddress6() override;
    int64_t AssignStreams(int64_t stream) override;
    std::pair<Ptr<Node>, Ipv4Address> SetupRemoteHost(std::optional<std::string> dataRate,
                                                      std::optional<uint16_t> mtu,
                                                      std::optional<Time> delay) override;
    std::pair<Ptr<Node>, Ipv6Address> SetupRemoteHost6(std::optional<std::string> dataRate,
                                                       std::optional<uint16_t> mtu,
                                                       std::optional<Time> delay) override;

    /// allow NrMemberEpcS1apSapMme<NrExternalCoreEpcHelper> class friend access
    friend class NrMemberEpcS1apSapMme<NrExternalCoreEpcHelper>;

  protected:
    void DoDispose() override;
    void ConnectGnbToCore(Ptr<NrEpcGnbApplication> gnbApp,
                          uint16_t cellId,
                          Ipv4Address gnbAddress,
                          Ipv4Address sgwAddress) override;

  private:
    /// A UE that may attach to the external network.
    struct RoamingUe
    {
        Ptr<NrUeNetDevice> device; ///< NR device of the UE
        Ipv4Address homeAddress;   ///< address of the UE in its home network
    };

    /**
     * Open the control channel with the bridge, if not open yet.
     */
    void OpenControlChannel();

    /**
     * Send a message to the bridge.
     *
     * @param json the message
     */
    void Send(const std::string& json) const;

    /**
     * Receive the messages of the bridge, and schedule the next poll.
     */
    void Poll();

    /**
     * Handle a message of the bridge.
     *
     * @param json the message
     */
    void Handle(const std::string& json);

    /**
     * Set the IPv4 address of a UE.
     *
     * @param ue the UE
     * @param address the address
     */
    void SetUeAddress(const RoamingUe& ue, Ipv4Address address);

    /**
     * @param imsi the IMSI of a UE in the simulation
     * @return the IMSI of the UE in the core networks: the PLMN of its home network and a MSIN
     */
    std::string ImsiString(uint64_t imsi) const;

    /**
     * Callback of the home MME when the UE attaches at home again.
     *
     * @param imsi the IMSI of the UE
     */
    void ExternalCancelLocation(uint64_t imsi);

    // S1-AP SAP MME methods
    /**
     * Initial UE message.
     *
     * @param mmeUeS1Id the MME UE S1 ID (the IMSI)
     * @param gnbUeS1Id the gNB UE S1 ID (the RNTI)
     * @param imsi the IMSI
     * @param ecgi the cell ID
     */
    void DoInitialUeMessage(uint64_t mmeUeS1Id, uint16_t gnbUeS1Id, uint64_t imsi, uint16_t ecgi);
    /**
     * Initial context setup response.
     *
     * @param mmeUeS1Id the MME UE S1 ID
     * @param gnbUeS1Id the gNB UE S1 ID
     * @param erabSetupList the E-RABs set up
     */
    void DoInitialContextSetupResponse(uint64_t mmeUeS1Id,
                                       uint16_t gnbUeS1Id,
                                       std::list<NrEpcS1apSapMme::ErabSetupItem> erabSetupList);
    /**
     * Path switch request.
     *
     * @param gnbUeS1Id the gNB UE S1 ID
     * @param mmeUeS1Id the MME UE S1 ID
     * @param cgi the cell ID
     * @param erabToBeSwitchedInDownlinkList the E-RABs to switch
     */
    void DoPathSwitchRequest(
        uint64_t gnbUeS1Id,
        uint64_t mmeUeS1Id,
        uint16_t cgi,
        std::list<NrEpcS1apSapMme::ErabSwitchedInDownlinkItem> erabToBeSwitchedInDownlinkList);
    /**
     * E-RAB release indication.
     *
     * @param mmeUeS1Id the MME UE S1 ID
     * @param gnbUeS1Id the gNB UE S1 ID
     * @param erabToBeReleaseIndication the E-RABs released
     */
    void DoErabReleaseIndication(
        uint64_t mmeUeS1Id,
        uint16_t gnbUeS1Id,
        std::list<NrEpcS1apSapMme::ErabToBeReleasedIndication> erabToBeReleaseIndication);
    /**
     * UE context release complete.
     *
     * @param mmeUeS1Id the MME UE S1 ID
     * @param gnbUeS1Id the gNB UE S1 ID
     */
    void DoUeContextReleaseComplete(uint64_t mmeUeS1Id, uint16_t gnbUeS1Id);

    NrEpcS1apSapMme* m_s1apSapMme;                  ///< S1-AP SAP of the gNBs towards the core
    std::map<uint16_t, NrEpcS1apSapGnb*> m_gnbSaps; ///< S1-AP SAP of each gNB, by cell ID
    std::map<uint64_t, RoamingUe> m_ues;            ///< UEs that may attach, by IMSI
    Ptr<NrNoBackhaulEpcHelper> m_home;              ///< EPC helper of the home network
    std::string m_homeMcc;                          ///< MCC of the home network
    std::string m_homeMnc;                          ///< MNC of the home network

    std::string m_mcc;           ///< MCC of the external network
    std::string m_mnc;           ///< MNC of the external network
    uint16_t m_tac;              ///< TAC of the gNBs
    std::string m_emuDevice;     ///< host device of the N3 interfaces
    Ipv4Address m_n3Base;        ///< N3 address of the first gNB
    Ipv4Mask m_n3Mask;           ///< mask of the N3 network
    Ipv4Address m_upfAddress;    ///< N3 address of the UPF
    Ipv4Address m_bridgeAddress; ///< address of the control channel of the bridge
    uint16_t m_bridgePort;       ///< port of the control channel of the bridge
    Time m_pollInterval;         ///< interval between two polls of the control channel
    uint32_t m_n3Count{0};       ///< number of N3 interfaces created
    int m_fd{-1};                ///< socket of the control channel
    EventId m_pollEvent;         ///< next poll of the control channel
};

NS_OBJECT_ENSURE_REGISTERED(NrExternalCoreEpcHelper);

TypeId
NrExternalCoreEpcHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrExternalCoreEpcHelper")
            .SetParent<NrRanEpcHelper>()
            .SetGroupName("Nr")
            .AddConstructor<NrExternalCoreEpcHelper>()
            .AddAttribute("Mcc",
                          "MCC of the external network",
                          StringValue("001"),
                          MakeStringAccessor(&NrExternalCoreEpcHelper::m_mcc),
                          MakeStringChecker())
            .AddAttribute("Mnc",
                          "MNC of the external network",
                          StringValue("01"),
                          MakeStringAccessor(&NrExternalCoreEpcHelper::m_mnc),
                          MakeStringChecker())
            .AddAttribute("Tac",
                          "TAC of the gNBs",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NrExternalCoreEpcHelper::m_tac),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("EmuDevice",
                          "Host device of the N3 interfaces of the gNBs",
                          StringValue("n3-ns3"),
                          MakeStringAccessor(&NrExternalCoreEpcHelper::m_emuDevice),
                          MakeStringChecker())
            .AddAttribute("N3Address",
                          "N3 address of the first gNB; the next gNBs take the next addresses",
                          Ipv4AddressValue("10.210.1.2"),
                          MakeIpv4AddressAccessor(&NrExternalCoreEpcHelper::m_n3Base),
                          MakeIpv4AddressChecker())
            .AddAttribute("N3Mask",
                          "Mask of the N3 network",
                          Ipv4MaskValue("255.255.255.0"),
                          MakeIpv4MaskAccessor(&NrExternalCoreEpcHelper::m_n3Mask),
                          MakeIpv4MaskChecker())
            .AddAttribute("UpfAddress",
                          "N3 address of the UPF",
                          Ipv4AddressValue("10.210.1.1"),
                          MakeIpv4AddressAccessor(&NrExternalCoreEpcHelper::m_upfAddress),
                          MakeIpv4AddressChecker())
            .AddAttribute("BridgeAddress",
                          "Address of the control channel of nr-bridge",
                          Ipv4AddressValue("127.0.0.1"),
                          MakeIpv4AddressAccessor(&NrExternalCoreEpcHelper::m_bridgeAddress),
                          MakeIpv4AddressChecker())
            .AddAttribute("BridgePort",
                          "Port of the control channel of nr-bridge",
                          UintegerValue(9999),
                          MakeUintegerAccessor(&NrExternalCoreEpcHelper::m_bridgePort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("PollInterval",
                          "Interval between two polls of the control channel",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&NrExternalCoreEpcHelper::m_pollInterval),
                          MakeTimeChecker());
    return tid;
}

NrExternalCoreEpcHelper::NrExternalCoreEpcHelper()
    : m_s1apSapMme(new NrMemberEpcS1apSapMme<NrExternalCoreEpcHelper>(this))
{
}

NrExternalCoreEpcHelper::~NrExternalCoreEpcHelper()
{
    delete m_s1apSapMme;
}

void
NrExternalCoreEpcHelper::DoDispose()
{
    m_pollEvent.Cancel();
    if (m_fd >= 0)
    {
        close(m_fd);
        m_fd = -1;
    }
    m_ues.clear();
    m_home = nullptr;
    NrRanEpcHelper::DoDispose();
}

void
NrExternalCoreEpcHelper::SetHomeNetwork(Ptr<NrNoBackhaulEpcHelper> home,
                                        std::string mcc,
                                        std::string mnc)
{
    m_home = home;
    m_homeMcc = mcc;
    m_homeMnc = mnc;
    home->GetMmeApplication()->SetExternalCancelLocationCallback(
        MakeCallback(&NrExternalCoreEpcHelper::ExternalCancelLocation, this));
    OpenControlChannel();
    // the bridge reaches the home network at gNB 0
    Simulator::ScheduleNow(&NrExternalCoreEpcHelper::Send,
                           this,
                           R"({"msg":"home_setup","gnb":0,"mcc":")" + mcc + R"(","mnc":")" + mnc +
                               R"("})");
}

void
NrExternalCoreEpcHelper::AddRoamingUe(Ptr<NrUeNetDevice> ue, Ipv4Address homeAddress)
{
    m_ues[ue->GetImsi()] = {ue, homeAddress};
}

std::string
NrExternalCoreEpcHelper::ImsiString(uint64_t imsi) const
{
    const auto msin = std::to_string(imsi);
    return m_homeMcc + m_homeMnc + std::string(15 - 5 - msin.size(), '0') + msin;
}

void
NrExternalCoreEpcHelper::OpenControlChannel()
{
    if (m_fd >= 0)
    {
        return;
    }
    m_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    NS_ABORT_MSG_IF(m_fd < 0, "Cannot create the control socket: " << std::strerror(errno));
    sockaddr_in bridge{};
    bridge.sin_family = AF_INET;
    bridge.sin_port = htons(m_bridgePort);
    bridge.sin_addr.s_addr = htonl(m_bridgeAddress.Get());
    NS_ABORT_MSG_IF(connect(m_fd, reinterpret_cast<sockaddr*>(&bridge), sizeof(bridge)) < 0,
                    "Cannot connect to the bridge: " << std::strerror(errno));
    m_pollEvent = Simulator::ScheduleNow(&NrExternalCoreEpcHelper::Poll, this);
}

void
NrExternalCoreEpcHelper::Send(const std::string& json) const
{
    NS_LOG_INFO("To bridge: " << json);
    if (send(m_fd, json.data(), json.size(), 0) < 0)
    {
        NS_LOG_WARN("Cannot send to the bridge: " << std::strerror(errno));
    }
}

void
NrExternalCoreEpcHelper::Poll()
{
    char buffer[65536];
    ssize_t n;
    while ((n = recv(m_fd, buffer, sizeof(buffer), 0)) > 0)
    {
        Handle(std::string(buffer, n));
    }
    m_pollEvent = Simulator::Schedule(m_pollInterval, &NrExternalCoreEpcHelper::Poll, this);
}

void
NrExternalCoreEpcHelper::Handle(const std::string& json)
{
    NS_LOG_INFO("From bridge: " << json);
    const auto msg = JsonField(json, "msg");
    const auto imsiString = JsonField(json, "imsi");
    // the MSIN of the IMSIs of the core networks is the IMSI of the simulation
    const uint64_t imsi = imsiString.size() > 5 ? std::stoull(imsiString.substr(5)) : 0;
    if (msg == "gnb_ready")
    {
        std::cout << Simulator::Now().As(Time::S) << " gNB " << JsonField(json, "gnb")
                  << " connected to the external core" << std::endl;
    }
    else if (msg == "gnb_failed" || msg == "ue_failed")
    {
        std::cout << Simulator::Now().As(Time::S) << " " << msg << ": " << JsonField(json, "reason")
                  << std::endl;
    }
    else if (msg == "ue_ready")
    {
        const auto cellId = static_cast<uint16_t>(std::stoul(JsonField(json, "gnb")));
        const auto rnti = static_cast<uint16_t>(std::stoul(JsonField(json, "rnti")));
        const Ipv4Address ueAddress(JsonField(json, "ue_ip").c_str());
        std::cout << Simulator::Now().As(Time::S) << " IMSI " << imsiString
                  << " served by the external core, address " << ueAddress << std::endl;
        NS_ABORT_MSG_IF(Ipv4Address(JsonField(json, "upf").c_str()) != m_upfAddress,
                        "PDU session in an unexpected UPF " << JsonField(json, "upf"));
        auto ueIt = m_ues.find(imsi);
        NS_ABORT_MSG_IF(ueIt == m_ues.end(), "Unknown UE " << imsiString);
        SetUeAddress(ueIt->second, ueAddress);

        NrEpcS1apSapGnb::ErabToBeSetupItem erab;
        erab.erabId = std::stoul(JsonField(json, "qfi"));
        erab.erabLevelQosParameters =
            NrQosFlow(static_cast<NrQosFlow::FiveQi>(std::stoul(JsonField(json, "five_qi"))));
        erab.transportLayerAddress = m_upfAddress;
        erab.sgwTeid = std::stoul(JsonField(json, "ul_teid"));
        erab.gnbTeid = std::stoul(JsonField(json, "dl_teid"));
        m_gnbSaps.at(cellId)->InitialContextSetupRequest(imsi, rnti, {erab});
    }
    else if (msg == "home_registration")
    {
        // the external network serves a subscriber of the home network
        const bool known = m_home->GetMmeApplication()->ExternalUpdateLocation(imsi);
        std::cout << Simulator::Now().As(Time::S) << " IMSI " << imsiString
                  << " registered in the external network"
                  << (known ? "" : " (not a subscriber of the home network)") << std::endl;
    }
    else
    {
        NS_LOG_WARN("Ignoring message of the bridge: " << json);
    }
}

void
NrExternalCoreEpcHelper::SetUeAddress(const RoamingUe& ue, Ipv4Address address)
{
    auto ipv4 = ue.device->GetNode()->GetObject<Ipv4>();
    const auto interface = ipv4->GetInterfaceForDevice(ue.device);
    while (ipv4->GetNAddresses(interface) > 0)
    {
        ipv4->RemoveAddress(interface, 0);
    }
    ipv4->AddAddress(interface, Ipv4InterfaceAddress(address, Ipv4Mask("255.255.255.255")));
}

void
NrExternalCoreEpcHelper::ExternalCancelLocation(uint64_t imsi)
{
    std::cout << Simulator::Now().As(Time::S) << " IMSI " << ImsiString(imsi)
              << " attached at home: released by the external network" << std::endl;
    Send(R"({"msg":"home_cancel","gnb":0,"imsi":")" + ImsiString(imsi) + R"("})");
    auto ueIt = m_ues.find(imsi);
    if (ueIt != m_ues.end())
    {
        SetUeAddress(ueIt->second, ueIt->second.homeAddress);
    }
}

void
NrExternalCoreEpcHelper::AddGnb(Ptr<Node> gnbNode, Ptr<NetDevice> nrGnbNetDevice, uint16_t cellId)
{
    NrRanEpcHelper::AddGnb(gnbNode, nrGnbNetDevice, cellId);

    EmuFdNetDeviceHelper emu;
    emu.SetDeviceName(m_emuDevice);
    auto device = DynamicCast<FdNetDevice>(emu.Install(gnbNode).Get(0));
    device->SetAddress(Mac48Address::Allocate());
    auto ipv4 = gnbNode->GetObject<Ipv4>();
    const auto interface = ipv4->AddInterface(device);
    const Ipv4Address n3(m_n3Base.Get() + m_n3Count++);
    ipv4->AddAddress(interface, Ipv4InterfaceAddress(n3, m_n3Mask));
    ipv4->SetUp(interface);

    AddS1Interface(gnbNode, n3, m_upfAddress, cellId);
}

void
NrExternalCoreEpcHelper::ConnectGnbToCore(Ptr<NrEpcGnbApplication> gnbApp,
                                          uint16_t cellId,
                                          Ipv4Address gnbAddress,
                                          Ipv4Address sgwAddress)
{
    m_gnbSaps[cellId] = gnbApp->GetS1apSapGnb();
    gnbApp->SetS1apSapMme(m_s1apSapMme);
    OpenControlChannel();
    std::ostringstream setup;
    setup << R"({"msg":"gnb_setup","gnb":)" << cellId << R"(,"mcc":")" << m_mcc << R"(","mnc":")"
          << m_mnc << R"(","tac":)" << m_tac << R"(,"n3":")" << gnbAddress << R"("})";
    Simulator::ScheduleNow(&NrExternalCoreEpcHelper::Send, this, setup.str());
}

void
NrExternalCoreEpcHelper::DoInitialUeMessage(uint64_t mmeUeS1Id,
                                            uint16_t gnbUeS1Id,
                                            uint64_t imsi,
                                            uint16_t ecgi)
{
    std::cout << Simulator::Now().As(Time::S) << " IMSI " << ImsiString(imsi)
              << " attaches to the external network at cell " << ecgi << std::endl;
    std::ostringstream attach;
    attach << R"({"msg":"ue_attach","gnb":)" << ecgi << R"(,"rnti":)" << gnbUeS1Id << R"(,"imsi":")"
           << ImsiString(imsi) << R"("})";
    Send(attach.str());
}

void
NrExternalCoreEpcHelper::DoInitialContextSetupResponse(
    uint64_t mmeUeS1Id,
    uint16_t gnbUeS1Id,
    std::list<NrEpcS1apSapMme::ErabSetupItem> erabSetupList)
{
}

void
NrExternalCoreEpcHelper::DoPathSwitchRequest(
    uint64_t gnbUeS1Id,
    uint64_t mmeUeS1Id,
    uint16_t cgi,
    std::list<NrEpcS1apSapMme::ErabSwitchedInDownlinkItem> erabToBeSwitchedInDownlinkList)
{
    NS_FATAL_ERROR("Handovers within the external network are not supported");
}

void
NrExternalCoreEpcHelper::DoErabReleaseIndication(
    uint64_t mmeUeS1Id,
    uint16_t gnbUeS1Id,
    std::list<NrEpcS1apSapMme::ErabToBeReleasedIndication> erabToBeReleaseIndication)
{
}

void
NrExternalCoreEpcHelper::DoUeContextReleaseComplete(uint64_t mmeUeS1Id, uint16_t gnbUeS1Id)
{
}

void
NrExternalCoreEpcHelper::AddUe(Ptr<NetDevice> ueNrDevice, uint64_t imsi)
{
    NS_FATAL_ERROR("The UEs are subscribers of a core network of the simulation");
}

uint8_t
NrExternalCoreEpcHelper::ActivateQosFlow(Ptr<NetDevice> ueNrDevice,
                                         uint64_t imsi,
                                         Ptr<NrQosRule> rule,
                                         NrQosFlow flow)
{
    NS_FATAL_ERROR("The QoS flows are activated by the external core");
    return 0;
}

Ptr<Node>
NrExternalCoreEpcHelper::GetSgwNode() const
{
    return nullptr;
}

Ptr<Node>
NrExternalCoreEpcHelper::GetPgwNode() const
{
    return nullptr;
}

Ipv4InterfaceContainer
NrExternalCoreEpcHelper::AssignUeIpv4Address(NetDeviceContainer ueDevices)
{
    NS_FATAL_ERROR("The external core assigns the addresses of the UEs");
    return {};
}

Ipv6InterfaceContainer
NrExternalCoreEpcHelper::AssignUeIpv6Address(NetDeviceContainer ueDevices)
{
    NS_FATAL_ERROR("The external core assigns the addresses of the UEs");
    return {};
}

Ipv4Address
NrExternalCoreEpcHelper::GetUeDefaultGatewayAddress()
{
    return {};
}

Ipv6Address
NrExternalCoreEpcHelper::GetUeDefaultGatewayAddress6()
{
    return {};
}

int64_t
NrExternalCoreEpcHelper::AssignStreams(int64_t stream)
{
    return 0;
}

std::pair<Ptr<Node>, Ipv4Address>
NrExternalCoreEpcHelper::SetupRemoteHost(std::optional<std::string> dataRate,
                                         std::optional<uint16_t> mtu,
                                         std::optional<Time> delay)
{
    NS_FATAL_ERROR("The remote hosts of the external core are outside of the simulation");
    return {};
}

std::pair<Ptr<Node>, Ipv6Address>
NrExternalCoreEpcHelper::SetupRemoteHost6(std::optional<std::string> dataRate,
                                          std::optional<uint16_t> mtu,
                                          std::optional<Time> delay)
{
    NS_FATAL_ERROR("The remote hosts of the external core are outside of the simulation");
    return {};
}

/// Echo replies received in each phase of the scenario, by destination.
static std::map<std::string, std::map<std::string, uint32_t>> g_replies;
/// Current phase of the scenario.
static std::string g_phase = "home";

int
main(int argc, char* argv[])
{
    std::string mcc = "001";
    std::string mnc = "01";
    std::string homeMcc = "002";
    std::string homeMnc = "02";
    std::string emuDevice = "n3-ns3";
    std::string n3Address = "10.210.1.2";
    std::string upfAddress = "10.210.1.1";
    std::string bridgeAddress = "127.0.0.1";
    std::string dnGateway = "10.46.0.1";
    std::string internetHost = "8.8.8.8";
    std::string dnHostDevice = "n6-ns3";
    std::string dnHostAddress = "10.210.2.2";
    std::string dnHostGateway = "10.210.2.1";
    Time leaveTime = Seconds(5);
    Time returnTime = Seconds(20);
    Time stopTime = Seconds(32);

    CommandLine cmd(__FILE__);
    cmd.AddValue("mcc", "MCC of the external network", mcc);
    cmd.AddValue("mnc", "MNC of the external network", mnc);
    cmd.AddValue("homeMcc", "MCC of the home network of the UE", homeMcc);
    cmd.AddValue("homeMnc", "MNC of the home network of the UE", homeMnc);
    cmd.AddValue("emuDevice", "Host device of the N3 interface of the visited gNB", emuDevice);
    cmd.AddValue("n3Address", "N3 address of the visited gNB", n3Address);
    cmd.AddValue("upfAddress", "N3 address of the UPF", upfAddress);
    cmd.AddValue("bridgeAddress", "Address of the control channel of nr-bridge", bridgeAddress);
    cmd.AddValue("dnGateway", "Gateway of the PDU sessions of the external core", dnGateway);
    cmd.AddValue("internetHost",
                 "Internet host reached through the N6 interface of the external core",
                 internetHost);
    cmd.AddValue("dnHostDevice",
                 "Host device of the simulated host in the DN of the external core (none if empty)",
                 dnHostDevice);
    cmd.AddValue("dnHostAddress", "Address of the simulated host in the DN", dnHostAddress);
    cmd.AddValue("dnHostGateway", "Gateway of the simulated host in the DN", dnHostGateway);
    cmd.AddValue("leaveTime", "When the UE moves to the visited network", leaveTime);
    cmd.AddValue("returnTime", "When the UE moves back home", returnTime);
    cmd.AddValue("stopTime", "Duration of the simulation", stopTime);
    cmd.Parse(argc, argv);

    GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(true));

    auto home = CreateObject<NrPointToPointEpcHelper>();
    auto visited = CreateObjectWithAttributes<NrExternalCoreEpcHelper>(
        "Mcc",
        StringValue(mcc),
        "Mnc",
        StringValue(mnc),
        "EmuDevice",
        StringValue(emuDevice),
        "N3Address",
        Ipv4AddressValue(n3Address.c_str()),
        "UpfAddress",
        Ipv4AddressValue(upfAddress.c_str()),
        "BridgeAddress",
        Ipv4AddressValue(bridgeAddress.c_str()));
    auto [remoteHost, remoteHostAddress] = home->SetupRemoteHost("100Gb/s", 2500, Seconds(0));

    NodeContainer gnbNodes(2);
    NodeContainer ueNodes(1);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    auto positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0, 0, 10));
    positions->Add(Vector(15020, 0, 10));
    positions->Add(Vector(10, 0, 1.5));
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    auto bandwidthParts = nrHelper->CreateBandwidthParts({{1.93e9, 10e6, 1}}, "UMa", "LOS");
    nrHelper->SetEpcHelper(home);
    auto homeGnb = nrHelper->InstallGnbDevice(gnbNodes.Get(0), bandwidthParts.second);
    nrHelper->SetEpcHelper(visited);
    auto visitedGnb = nrHelper->InstallGnbDevice(gnbNodes.Get(1), bandwidthParts.second);
    nrHelper->SetEpcHelper(home);
    auto ueDevices = nrHelper->InstallUeDevice(ueNodes, bandwidthParts.second);

    InternetStackHelper internet;
    internet.Install(ueNodes);
    auto ueAddresses = home->AssignUeIpv4Address(ueDevices);
    Ipv4StaticRoutingHelper routing;
    routing.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>())
        ->SetDefaultRoute(home->GetUeDefaultGatewayAddress(), 1);
    nrHelper->AttachToGnb(ueDevices.Get(0), homeGnb.Get(0));

    auto ue = DynamicCast<NrUeNetDevice>(ueDevices.Get(0));
    visited->AddRoamingUe(ue, ueAddresses.GetAddress(0));
    visited->SetHomeNetwork(home, homeMcc, homeMnc);

    std::vector<std::pair<std::string, Ipv4Address>> destinations{
        {"home host", remoteHostAddress},
        {"external DN", Ipv4Address(dnGateway.c_str())},
        {"internet", Ipv4Address(internetHost.c_str())}};
    if (!dnHostDevice.empty())
    {
        // a node of the simulation in the DN of the external core, reached through its UPF
        auto dnHost = CreateObject<Node>();
        internet.Install(dnHost);
        EmuFdNetDeviceHelper emu;
        emu.SetDeviceName(dnHostDevice);
        auto device = DynamicCast<FdNetDevice>(emu.Install(dnHost).Get(0));
        device->SetAddress(Mac48Address::Allocate());
        auto ipv4 = dnHost->GetObject<Ipv4>();
        const auto interface = ipv4->AddInterface(device);
        ipv4->AddAddress(
            interface,
            Ipv4InterfaceAddress(Ipv4Address(dnHostAddress.c_str()), Ipv4Mask("255.255.255.0")));
        ipv4->SetUp(interface);
        routing.GetStaticRouting(ipv4)->SetDefaultRoute(Ipv4Address(dnHostGateway.c_str()),
                                                        interface);
        destinations.emplace_back("simulated DN host", Ipv4Address(dnHostAddress.c_str()));
    }
    for (const auto& [name, destination] : destinations)
    {
        PingHelper ping(destination);
        ping.SetAttribute("Interval", TimeValue(MilliSeconds(200)));
        ping.SetAttribute("Timeout", TimeValue(MilliSeconds(500)));
        ping.SetAttribute("Count", UintegerValue(stopTime.GetMilliSeconds() / 200));
        ping.SetAttribute("VerboseMode", EnumValue(Ping::SILENT));
        auto app = ping.Install(ueNodes.Get(0));
        app.Start(Seconds(1));
        app.Get(0)->TraceConnectWithoutContext(
            "Rtt",
            Callback<void, uint16_t, Time>(
                [name = name](uint16_t, Time) { g_replies[g_phase][name]++; }));
    }

    // to the visited network, and back home
    auto ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();
    Simulator::Schedule(leaveTime, [ueMobility]() {
        g_phase = "visited";
        std::cout << Simulator::Now().As(Time::S) << " UE moves to the visited network"
                  << std::endl;
        ueMobility->SetPosition({15000, 0, 1.5});
    });
    Simulator::Schedule(returnTime, [ueMobility]() {
        g_phase = "home again";
        std::cout << Simulator::Now().As(Time::S) << " UE moves back home" << std::endl;
        ueMobility->SetPosition({10, 0, 1.5});
    });

    Simulator::Stop(stopTime);
    Simulator::Run();
    Simulator::Destroy();

    // each destination answers only while its network serves the UE
    bool ok = true;
    for (const std::string phase : {"home", "visited", "home again"})
    {
        const bool atHome = phase != "visited";
        std::cout << phase << ":";
        for (const auto& [name, destination] : destinations)
        {
            const auto replies = g_replies[phase][name];
            const bool expected = (name == "home host") == atHome;
            const bool destinationOk = expected ? replies > 0 : replies == 0;
            ok = ok && destinationOk;
            std::cout << " " << name << " " << replies << (destinationOk ? "" : " (FAIL)") << ";";
        }
        std::cout << std::endl;
    }
    std::cout << (ok ? "PASS" : "FAIL") << std::endl;
    return ok ? 0 : 1;
}

/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Nicola Baldo <nbaldo@cttc.es>
 *         Budiarto Herman <budiarto.herman@magister.fi>
 */

#include "ns3/core-config.h"
#include "ns3/int64x64-128.h"
#include "ns3/csv-reader.h"
#include "ns3/event-garbage-collector.h"
#include "ns3/random-variable-stream-helper.h"
#include "ns3/abort.h"
#include "ns3/ascii-file.h"
#include "ns3/ascii-test.h"
#include "ns3/assert.h"
#include "ns3/attribute-accessor-helper.h"
#include "ns3/attribute-construction-list.h"
#include "ns3/attribute-container.h"
#include "ns3/attribute-helper.h"
#include "ns3/attribute.h"
#include "ns3/boolean.h"
#include "ns3/breakpoint.h"
#include "ns3/build-profile.h"
#include "ns3/calendar-scheduler.h"
#include "ns3/callback.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/default-deleter.h"
#include "ns3/default-simulator-impl.h"
#include "ns3/demangle.h"
#include "ns3/deprecated.h"
#include "ns3/des-metrics.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/event-id.h"
#include "ns3/event-impl.h"
#include "ns3/fatal-error.h"
#include "ns3/fatal-impl.h"
#include "ns3/fd-reader.h"
#include "ns3/environment-variable.h"
#include "ns3/global-value.h"
#include "ns3/hash-fnv.h"
#include "ns3/hash-function.h"
#include "ns3/hash-murmur3.h"
#include "ns3/hash.h"
#include "ns3/heap-scheduler.h"
#include "ns3/int64x64-double.h"
#include "ns3/int64x64.h"
#include "ns3/integer.h"
#include "ns3/length.h"
#include "ns3/list-scheduler.h"
#include "ns3/log-macros-disabled.h"
#include "ns3/log-macros-enabled.h"
#include "ns3/log.h"
#include "ns3/make-event.h"
#include "ns3/map-scheduler.h"
#include "ns3/math.h"
#include "ns3/names.h"
#include "ns3/node-printer.h"
#include "ns3/nstime.h"
#include "ns3/object-base.h"
#include "ns3/object-factory.h"
#include "ns3/object-map.h"
#include "ns3/object-ptr-container.h"
#include "ns3/object-vector.h"
#include "ns3/object.h"
#include "ns3/pair.h"
#include "ns3/pointer.h"
#include "ns3/priority-queue-scheduler.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/rng-stream.h"
#include "ns3/scheduler.h"
#include "ns3/show-progress.h"
#include "ns3/shuffle.h"
#include "ns3/simple-ref-count.h"
#include "ns3/simulation-singleton.h"
#include "ns3/simulator-impl.h"
#include "ns3/simulator.h"
#include "ns3/singleton.h"
#include "ns3/string.h"
#include "ns3/struct.h"
#include "ns3/synchronizer.h"
#include "ns3/system-path.h"
#include "ns3/system-wall-clock-ms.h"
#include "ns3/system-wall-clock-timestamp.h"
#include "ns3/test.h"
#include "ns3/time-printer.h"
#include "ns3/time-span.h"
#include "ns3/timer-impl.h"
#include "ns3/timer.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/traced-callback.h"
#include "ns3/traced-value.h"
#include "ns3/trickle-timer.h"
#include "ns3/tuple.h"
#include "ns3/type-id.h"
#include "ns3/type-name.h"
#include "ns3/uinteger.h"
#include "ns3/uniform-random-bit-generator.h"
#include "ns3/valgrind.h"
#include "ns3/vector.h"
#include "ns3/warnings.h"
#include "ns3/watchdog.h"
#include "ns3/realtime-simulator-impl.h"
#include "ns3/wall-clock-synchronizer.h"
#include "ns3/val-array.h"
#include "ns3/matrix-array.h"
#include "ns3/example-as-test.h"
#include "ns3/traced-callback-typedef-test.h"
#include "ns3/lte-common.h"
#include "ns3/lte-spectrum-phy.h"
#include "ns3/lte-spectrum-signal-parameters.h"
#include "ns3/lte-phy.h"
#include "ns3/lte-enb-phy.h"
#include "ns3/lte-ue-phy.h"
#include "ns3/lte-spectrum-value-helper.h"
#include "ns3/lte-amc.h"
#include "ns3/lte-enb-rrc.h"
#include "ns3/lte-ue-rrc.h"
#include "ns3/lte-rrc-sap.h"
#include "ns3/lte-rrc-protocol-ideal.h"
#include "ns3/lte-rrc-protocol-real.h"
#include "ns3/lte-rlc-sap.h"
#include "ns3/lte-rlc.h"
#include "ns3/lte-rlc-header.h"
#include "ns3/lte-rlc-sequence-number.h"
#include "ns3/lte-rlc-am-header.h"
#include "ns3/lte-rlc-tm.h"
#include "ns3/lte-rlc-um.h"
#include "ns3/lte-rlc-am.h"
#include "ns3/lte-rlc-tag.h"
#include "ns3/lte-rlc-sdu-status-tag.h"
#include "ns3/lte-pdcp-sap.h"
#include "ns3/lte-pdcp.h"
#include "ns3/lte-pdcp-header.h"
#include "ns3/lte-pdcp-tag.h"
#include "ns3/eps-bearer.h"
#include "ns3/lte-radio-bearer-info.h"
#include "ns3/lte-net-device.h"
#include "ns3/lte-enb-net-device.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/lte-control-messages.h"
#include "ns3/lte-helper.h"
#include "ns3/lte-stats-calculator.h"
#include "ns3/epc-helper.h"
#include "ns3/point-to-point-epc-helper.h"
#include "ns3/phy-stats-calculator.h"
#include "ns3/mac-stats-calculator.h"
#include "ns3/phy-tx-stats-calculator.h"
#include "ns3/phy-rx-stats-calculator.h"
#include "ns3/radio-bearer-stats-calculator.h"
#include "ns3/radio-bearer-stats-connector.h"
#include "ns3/radio-environment-map-helper.h"
#include "ns3/lte-hex-grid-enb-topology-helper.h"
#include "ns3/lte-global-pathloss-database.h"
#include "ns3/rem-spectrum-phy.h"
#include "ns3/ff-mac-common.h"
#include "ns3/ff-mac-csched-sap.h"
#include "ns3/ff-mac-sched-sap.h"
#include "ns3/lte-enb-cmac-sap.h"
#include "ns3/lte-ue-cmac-sap.h"
#include "ns3/lte-mac-sap.h"
#include "ns3/ff-mac-scheduler.h"
#include "ns3/rr-ff-mac-scheduler.h"
#include "ns3/lte-enb-mac.h"
#include "ns3/lte-ue-mac.h"
#include "ns3/lte-radio-bearer-tag.h"
#include "ns3/eps-bearer-tag.h"
#include "ns3/lte-phy-tag.h"
#include "ns3/lte-enb-phy-sap.h"
#include "ns3/lte-enb-cphy-sap.h"
#include "ns3/lte-ue-phy-sap.h"
#include "ns3/lte-ue-cphy-sap.h"
#include "ns3/lte-interference.h"
#include "ns3/lte-chunk-processor.h"
#include "ns3/pf-ff-mac-scheduler.h"
#include "ns3/fdmt-ff-mac-scheduler.h"
#include "ns3/tdmt-ff-mac-scheduler.h"
#include "ns3/tta-ff-mac-scheduler.h"
#include "ns3/fdbet-ff-mac-scheduler.h"
#include "ns3/tdbet-ff-mac-scheduler.h"
#include "ns3/fdtbfq-ff-mac-scheduler.h"
#include "ns3/tdtbfq-ff-mac-scheduler.h"
#include "ns3/pss-ff-mac-scheduler.h"
#include "ns3/cqa-ff-mac-scheduler.h"
#include "ns3/epc-gtpu-header.h"
#include "ns3/epc-enb-application.h"
#include "ns3/epc-sgw-pgw-application.h"
#include "ns3/lte-vendor-specific-parameters.h"
#include "ns3/epc-x2-sap.h"
#include "ns3/epc-x2-header.h"
#include "ns3/epc-x2.h"
#include "ns3/epc-x2-tag.h"
#include "ns3/epc-tft.h"
#include "ns3/epc-tft-classifier.h"
#include "ns3/lte-mi-error-model.h"
#include "ns3/epc-enb-s1-sap.h"
#include "ns3/epc-s1ap-sap.h"
#include "ns3/epc-s11-sap.h"
#include "ns3/epc-s1ap.h"
#include "ns3/epc-mme-application.h"
#include "ns3/lte-as-sap.h"
#include "ns3/epc-ue-nas.h"
#include "ns3/lte-harq-phy.h"
#include "ns3/epc-mme.h"
#include "ns3/lte-asn1-header.h"
#include "ns3/lte-rrc-header.h"
#include "ns3/lte-handover-management-sap.h"
#include "ns3/lte-handover-algorithm.h"
#include "ns3/a2-a4-rsrq-handover-algorithm.h"
#include "ns3/a3-rsrp-handover-algorithm.h"
#include "ns3/no-op-handover-algorithm.h"
#include "ns3/lte-anr-sap.h"
#include "ns3/lte-anr.h"
#include "ns3/lte-ffr-algorithm.h"
#include "ns3/lte-ffr-sap.h"
#include "ns3/lte-ffr-rrc-sap.h"
#include "ns3/lte-fr-no-op-algorithm.h"
#include "ns3/lte-fr-hard-algorithm.h"
#include "ns3/lte-fr-strict-algorithm.h"
#include "ns3/lte-fr-soft-algorithm.h"
#include "ns3/lte-ffr-soft-algorithm.h"
#include "ns3/lte-ffr-enhanced-algorithm.h"
#include "ns3/lte-ffr-distributed-algorithm.h"
#include "ns3/lte-ue-power-control.h"
#include "ns3/lte-rlc-um-lowlat.h"
#include "ns3/epc-s1ap-header.h"
#include "ns3/mc-enb-pdcp.h"
#include "ns3/mc-ue-pdcp.h"
#include "ns3/retx-stats-calculator.h"
#include "ns3/mac-tx-stats-calculator.h"
#include "ns3/lte-ccm-rrc-sap.h"
#include "ns3/lte-ue-ccm-rrc-sap.h"
#include "ns3/lte-ccm-mac-sap.h"
#include "ns3/lte-enb-component-carrier-manager.h"
#include "ns3/lte-ue-component-carrier-manager.h"
#include "ns3/no-op-component-carrier-manager.h"
#include "ns3/simple-ue-component-carrier-manager.h"
#include "ns3/cc-helper.h"
#include "ns3/component-carrier.h"
#include "ns3/component-carrier-ue.h"
#include "ns3/component-carrier-enb.h"
#include "ns3/leo-orbit-node-helper.h"
#include "ns3/group-mobility-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/box.h"
#include "ns3/constant-acceleration-mobility-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-helper.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/gauss-markov-mobility-model.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/geocentric-ecef-mobility-model.h"
#include "ns3/geographic-positions.h"
#include "ns3/hierarchical-mobility-model.h"
#include "ns3/leo-circular-orbit-mobility-model.h"
#include "ns3/leo-circular-orbit-position-allocator.h"
#include "ns3/leo-orbital-shell.h"
#include "ns3/mobility-model.h"
#include "ns3/position-allocator.h"
#include "ns3/random-direction-2d-mobility-model.h"
#include "ns3/random-walk-2d-mobility-model.h"
#include "ns3/random-waypoint-mobility-model.h"
#include "ns3/rectangle.h"
#include "ns3/steady-state-random-waypoint-mobility-model.h"
#include "ns3/waypoint-mobility-model.h"
#include "ns3/waypoint.h"
#include "ns3/application-container.h"
#include "ns3/application-helper.h"
#include "ns3/delay-jitter-estimation.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/simple-net-device-helper.h"
#include "ns3/trace-helper.h"
#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/buffer.h"
#include "ns3/byte-tag-list.h"
#include "ns3/channel-list.h"
#include "ns3/channel.h"
#include "ns3/chunk.h"
#include "ns3/header.h"
#include "ns3/net-device.h"
#include "ns3/nix-vector.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/packet-metadata.h"
#include "ns3/packet-tag-list.h"
#include "ns3/packet.h"
#include "ns3/socket-factory.h"
#include "ns3/socket.h"
#include "ns3/tag-buffer.h"
#include "ns3/tag.h"
#include "ns3/trailer.h"
#include "ns3/address-utils.h"
#include "ns3/bit-deserializer.h"
#include "ns3/bit-serializer.h"
#include "ns3/crc32.h"
#include "ns3/data-rate.h"
#include "ns3/drop-tail-queue.h"
#include "ns3/dynamic-queue-limits.h"
#include "ns3/error-channel.h"
#include "ns3/error-model.h"
#include "ns3/ethernet-header.h"
#include "ns3/ethernet-trailer.h"
#include "ns3/flow-id-tag.h"
#include "ns3/generic-phy.h"
#include "ns3/header-serialization-test.h"
#include "ns3/iana-link-type-numbers.h"
#include "ns3/iana-ieee802-numbers.h"
#include "ns3/iana-ppp-numbers.h"
#include "ns3/iana-internet-protocol-numbers.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "ns3/llc-snap-header.h"
#include "ns3/lollipop-counter.h"
#include "ns3/mac16-address.h"
#include "ns3/mac48-address.h"
#include "ns3/mac64-address.h"
#include "ns3/mac8-address.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/packet-burst.h"
#include "ns3/packet-data-calculators.h"
#include "ns3/packet-probe.h"
#include "ns3/packet-socket-address.h"
#include "ns3/packet-socket-client.h"
#include "ns3/packet-socket-factory.h"
#include "ns3/packet-socket-server.h"
#include "ns3/packet-socket.h"
#include "ns3/packetbb.h"
#include "ns3/pcap-file-wrapper.h"
#include "ns3/pcap-file.h"
#include "ns3/pcap-test.h"
#include "ns3/queue-fwd.h"
#include "ns3/queue-item.h"
#include "ns3/queue-limits.h"
#include "ns3/queue-size.h"
#include "ns3/queue.h"
#include "ns3/radiotap-header.h"
#include "ns3/sequence-number.h"
#include "ns3/simple-channel.h"
#include "ns3/simple-net-device.h"
#include "ns3/sll-header.h"
#include "ns3/timestamp-tag.h"

#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LteRrcTest");

/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief Test rrc connection establishment.
 */
class LteRrcConnectionEstablishmentTestCase : public TestCase
{
  public:
    /**
     *
     *
     * \param nUes number of UEs in the test
     * \param nBearers number of bearers to be setup in each connection
     * \param tConnBase connection time base value for all UEs in ms
     * \param tConnIncrPerUe additional connection time increment for each UE index (0...nUes-1) in
     * ms \param delayDiscStart expected duration to perform connection establishment in ms \param
     * errorExpected if true, test case will wait a bit longer to accommodate for transmission error
     * \param useIdealRrc If set to false, real RRC protocol model will be used
     * \param admitRrcConnectionRequest If set to false, eNb will not allow UE connections
     * \param description additional description of the test case
     */
    LteRrcConnectionEstablishmentTestCase(uint32_t nUes,
                                          uint32_t nBearers,
                                          uint32_t tConnBase,
                                          uint32_t tConnIncrPerUe,
                                          uint32_t delayDiscStart,
                                          bool errorExpected,
                                          bool useIdealRrc,
                                          bool admitRrcConnectionRequest,
                                          std::string description = "");

  protected:
    virtual void DoRun(void);
    uint32_t m_nUes; ///< number of UEs in the test

    /**
     * Build name string function
     *
     * \param nUes number of UEs in the test
     * \param nBearers number of bearers to be setup in each connection
     * \param tConnBase connection time base value for all UEs in ms
     * \param tConnIncrPerUe additional connection time increment for each UE index (0...nUes-1) in
     * ms \param delayDiscStart expected duration to perform connection establishment in ms \param
     * useIdealRrc If set to false, real RRC protocol model will be used \param
     * admitRrcConnectionRequest If set to false, eNb will not allow UE connections \param
     * description additional description of the test case \returns the name string
     */
    static std::string BuildNameString(uint32_t nUes,
                                       uint32_t nBearers,
                                       uint32_t tConnBase,
                                       uint32_t tConnIncrPerUe,
                                       uint32_t delayDiscStart,
                                       bool useIdealRrc,
                                       bool admitRrcConnectionRequest,
                                       std::string description = "");
    /**
     * Connect function
     * \param ueDevice the UE device
     * \param enbDevice the ENB device
     */
    void Connect(Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice);
    /**
     * Check connected function
     * \param ueDevice the UE device
     * \param enbDevice the ENB device
     */
    void CheckConnected(Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice);
    /**
     * Check not connected function
     * \param ueDevice the UE device
     * \param enbDevice the ENB device
     */
    void CheckNotConnected(Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice);
    /**
     * Connection established callback function
     * \param context the context string
     * \param imsi the IMSI
     * \param cellId the cell ID
     * \param rnti the RNTI
     */
    void ConnectionEstablishedCallback(std::string context,
                                       uint64_t imsi,
                                       uint16_t cellId,
                                       uint16_t rnti);
    /**
     * Connection timeout callback function
     * \param context the context string
     * \param imsi the IMSI
     * \param cellId the cell ID
     * \param rnti the RNTI
     */
    void ConnectionTimeoutCallback(std::string context,
                                   uint64_t imsi,
                                   uint16_t cellId,
                                   uint16_t rnti);

    uint32_t m_nBearers;       ///< number of bearers to be setup in each connection
    uint32_t m_tConnBase;      ///< connection time base value for all UEs in ms
    uint32_t m_tConnIncrPerUe; ///< additional connection time increment for each UE index
                               ///< (0...nUes-1) in ms
    uint32_t m_delayConnEnd;   ///< expected duration to perform connection establishment in ms
    uint32_t
        m_delayDiscStart;    ///< delay between connection completed and disconnection request in ms
    uint32_t m_delayDiscEnd; ///< expected duration to complete disconnection in ms
    bool m_useIdealRrc;      ///< If set to false, real RRC protocol model will be used
    bool m_admitRrcConnectionRequest; ///< If set to false, eNb will not allow UE connections
    Ptr<LteHelper> m_lteHelper;       ///< LTE helper

    /// key: IMSI
    std::map<uint64_t, bool> m_isConnectionEstablished;
};

std::string
LteRrcConnectionEstablishmentTestCase::BuildNameString(uint32_t nUes,
                                                       uint32_t nBearers,
                                                       uint32_t tConnBase,
                                                       uint32_t tConnIncrPerUe,
                                                       uint32_t delayDiscStart,
                                                       bool useIdealRrc,
                                                       bool admitRrcConnectionRequest,
                                                       std::string description)
{
    std::ostringstream oss;
    oss << "nUes=" << nUes << ", nBearers=" << nBearers << ", tConnBase=" << tConnBase
        << ", tConnIncrPerUe=" << tConnIncrPerUe << ", delayDiscStart=" << delayDiscStart;

    if (useIdealRrc)
    {
        oss << ", ideal RRC";
    }
    else
    {
        oss << ", real RRC";
    }

    if (admitRrcConnectionRequest)
    {
        oss << ", admitRrcConnectionRequest = true";
    }
    else
    {
        oss << ", admitRrcConnectionRequest = false";
    }

    if (!description.empty())
    {
        oss << ", " << description;
    }

    return oss.str();
}

LteRrcConnectionEstablishmentTestCase::LteRrcConnectionEstablishmentTestCase(
    uint32_t nUes,
    uint32_t nBearers,
    uint32_t tConnBase,
    uint32_t tConnIncrPerUe,
    uint32_t delayDiscStart,
    bool errorExpected,
    bool useIdealRrc,
    bool admitRrcConnectionRequest,
    std::string description)
    : TestCase(BuildNameString(nUes,
                               nBearers,
                               tConnBase,
                               tConnIncrPerUe,
                               delayDiscStart,
                               useIdealRrc,
                               admitRrcConnectionRequest,
                               description)),
      m_nUes(nUes),
      m_nBearers(nBearers),
      m_tConnBase(tConnBase),
      m_tConnIncrPerUe(tConnIncrPerUe),

      m_delayDiscStart(delayDiscStart),
      m_delayDiscEnd(10),
      m_useIdealRrc(useIdealRrc),
      m_admitRrcConnectionRequest(admitRrcConnectionRequest)
{
    NS_LOG_FUNCTION(this << GetName());

    // see the description of d^e in the LTE testing docs
    double dsi = 90;
    double nRaAttempts = 0;
    if (nUes <= 20)
    {
        nRaAttempts += 5;
    }
    else
    {
        NS_ASSERT(nUes <= 50);
        nRaAttempts += 10;
    }

    nRaAttempts += std::ceil(nUes / 4.0);
    double dra = nRaAttempts * 7;
    double dce = 10.0 + (2.0 * nUes) / 4.0;
    if (errorExpected)
    {
        /*
         * If transmission error happens, the UE has to repeat again from
         * acquiring system information.
         */
        dce += dsi + dce;
    }
    double nCrs;
    if (nUes <= 2)
    {
        nCrs = 0;
    }
    else if (nUes <= 5)
    {
        nCrs = 1;
    }
    else if (nUes <= 10)
    {
        nCrs = 2;
    }
    else if (nUes <= 20)
    {
        nCrs = 3;
    }
    else
    {
        nCrs = 4;
    }
    double dcr = (10.0 + (2.0 * nUes) / 4.0) * (m_nBearers + nCrs);

    m_delayConnEnd = round(dsi + dra + dce + dcr);
    NS_LOG_LOGIC(this << " " << GetName() << " dsi=" << dsi << " dra=" << dra << " dce=" << dce
                      << " dcr=" << dcr << " m_delayConnEnd=" << m_delayConnEnd);
}

void
LteRrcConnectionEstablishmentTestCase::DoRun()
{
    NS_LOG_FUNCTION(this << GetName());
    Config::Reset();

    if (m_nUes < 25)
    {
        Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(40));
    }
    else if (m_nUes < 60)
    {
        Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(80));
    }
    else if (m_nUes < 120)
    {
        Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(160));
    }
    else
    {
        Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));
    }

    // normal code
    m_lteHelper = CreateObject<LteHelper>();
    m_lteHelper->SetAttribute("UseIdealRrc", BooleanValue(m_useIdealRrc));

    NodeContainer enbNodes;
    NodeContainer ueNodes;

    enbNodes.Create(1);
    ueNodes.Create(m_nUes);

    // the following positions all nodes at (0, 0, 0)
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(enbNodes);
    mobility.Install(ueNodes);

    int64_t stream = 1;
    NetDeviceContainer enbDevs;
    enbDevs = m_lteHelper->InstallEnbDevice(enbNodes);
    stream += m_lteHelper->AssignStreams(enbDevs, stream);

    NetDeviceContainer ueDevs;
    ueDevs = m_lteHelper->InstallUeDevice(ueNodes);
    stream += m_lteHelper->AssignStreams(ueDevs, stream);

    // custom code used for testing purposes
    // instead of lteHelper->Attach () and lteHelper->ActivateXxx

    // Set AdmitConnectionRequest attribute
    for (NetDeviceContainer::Iterator it = enbDevs.Begin(); it != enbDevs.End(); ++it)
    {
        Ptr<LteEnbRrc> enbRrc = (*it)->GetObject<LteEnbNetDevice>()->GetRrc();
        enbRrc->SetAttribute("AdmitRrcConnectionRequest",
                             BooleanValue(m_admitRrcConnectionRequest));
    }

    uint32_t i = 0;
    uint32_t tmax = 0;
    for (NetDeviceContainer::Iterator it = ueDevs.Begin(); it != ueDevs.End(); ++it)
    {
        Ptr<NetDevice> ueDevice = *it;
        Ptr<NetDevice> enbDevice = enbDevs.Get(0);
        Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice>();

        uint32_t tc = m_tConnBase + m_tConnIncrPerUe * i; // time connection start
        uint32_t tcc = tc + m_delayConnEnd;               // time check connection completed;
        uint32_t td = tcc + m_delayDiscStart;             // time disconnect start
        uint32_t tcd = td + m_delayDiscEnd;               // time check disconnection completed
        tmax = std::max(tmax, tcd);

        // trick to resolve overloading
        // void (LteHelper::* overloadedAttachFunctionPointer) (Ptr<NetDevice>, Ptr<NetDevice>) =
        // &LteHelper::Attach; Simulator::Schedule (MilliSeconds (tc),
        // overloadedAttachFunctionPointer, lteHelper, *it, enbDevice);
        Simulator::Schedule(MilliSeconds(tc),
                            &LteRrcConnectionEstablishmentTestCase::Connect,
                            this,
                            ueDevice,
                            enbDevice);

        Simulator::Schedule(MilliSeconds(tcc),
                            &LteRrcConnectionEstablishmentTestCase::CheckConnected,
                            this,
                            *it,
                            enbDevice);

        // disconnection not supported yet

        uint64_t imsi = ueLteDevice->GetImsi();
        m_isConnectionEstablished[imsi] = false;

        ++i;
    }

    // Connect to trace sources in UEs
    Config::Connect(
        "/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
        MakeCallback(&LteRrcConnectionEstablishmentTestCase::ConnectionEstablishedCallback, this));
    Config::Connect(
        "/NodeList/*/DeviceList/*/LteUeRrc/ConnectionTimeout",
        MakeCallback(&LteRrcConnectionEstablishmentTestCase::ConnectionTimeoutCallback, this));

    Simulator::Stop(MilliSeconds(tmax + 1));

    Simulator::Run();

    Simulator::Destroy();
}

void
LteRrcConnectionEstablishmentTestCase::Connect(Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice)
{
    NS_LOG_FUNCTION(this);
    m_lteHelper->Attach(ueDevice, enbDevice);

    for (uint32_t b = 0; b < m_nBearers; ++b)
    {
        enum EpsBearer::Qci q = EpsBearer::NGBR_VIDEO_TCP_DEFAULT;
        EpsBearer bearer(q);
        m_lteHelper->ActivateDataRadioBearer(ueDevice, bearer);
    }
}

void
LteRrcConnectionEstablishmentTestCase::CheckConnected(Ptr<NetDevice> ueDevice,
                                                      Ptr<NetDevice> enbDevice)
{
    Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice>();
    Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc();
    const uint64_t imsi = ueLteDevice->GetImsi();
    const uint16_t rnti = ueRrc->GetRnti();
    NS_LOG_FUNCTION(this << imsi << rnti);
    NS_ASSERT_MSG(m_isConnectionEstablished.find(imsi) != m_isConnectionEstablished.end(),
                  "Invalid IMSI " << imsi);

    if (!m_admitRrcConnectionRequest)
    {
        NS_TEST_ASSERT_MSG_EQ(m_isConnectionEstablished[imsi],
                              false,
                              "Connection with RNTI " << rnti << " should have been rejected");
        return;
    }

    /*
     * Verifying UE state in UE RRC. Try to increase the test case duration if
     * the following checks.
     */
    NS_TEST_ASSERT_MSG_EQ(m_isConnectionEstablished[imsi],
                          true,
                          "RNTI " << rnti << " fails to establish connection");
    NS_TEST_ASSERT_MSG_EQ(ueRrc->GetState(),
                          LteUeRrc::CONNECTED_NORMALLY,
                          "RNTI " << rnti << " is not at CONNECTED_NORMALLY state");

    // Verifying UE context state in eNodeB RRC.

    Ptr<LteEnbNetDevice> enbLteDevice = enbDevice->GetObject<LteEnbNetDevice>();
    Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc();
    const bool hasContext = enbRrc->HasUeManager(rnti);

    if (hasContext)
    {
        Ptr<UeManager> ueManager = enbRrc->GetUeManager(rnti);
        NS_ASSERT(ueManager);
        NS_TEST_ASSERT_MSG_EQ(ueManager->GetState(),
                              UeManager::CONNECTED_NORMALLY,
                              "The context of RNTI " << rnti << " is in invalid state");
    }
    else
    {
        NS_LOG_WARN(this << " RNTI " << rnti << " thinks that it has"
                         << " established connection but the eNodeB thinks"
                         << " that the UE has failed on connection setup.");
        /*
         * The standard specifies that this case would exceed the maximum
         * retransmission limit at UE RLC (SRB1), which will then trigger an RLF.
         * However, this behaviour is not implemented yet.
         */
    }

    // Verifying other attributes on both sides.

    uint16_t ueCellId = ueRrc->GetCellId();
    uint16_t enbCellId = enbLteDevice->GetCellId();
    uint16_t ueImsi = ueLteDevice->GetImsi();

    uint8_t ueDlBandwidth = ueRrc->GetDlBandwidth();
    uint8_t enbDlBandwidth = enbLteDevice->GetDlBandwidth();
    uint8_t ueUlBandwidth = ueRrc->GetUlBandwidth();
    uint8_t enbUlBandwidth = enbLteDevice->GetUlBandwidth();
    uint8_t ueDlEarfcn = ueRrc->GetDlEarfcn();
    uint8_t enbDlEarfcn = enbLteDevice->GetDlEarfcn();
    uint8_t ueUlEarfcn = ueRrc->GetUlEarfcn();
    uint8_t enbUlEarfcn = enbLteDevice->GetUlEarfcn();

    NS_TEST_ASSERT_MSG_EQ(ueCellId, enbCellId, "inconsistent CellId");
    NS_TEST_ASSERT_MSG_EQ(ueDlBandwidth, enbDlBandwidth, "inconsistent DlBandwidth");
    NS_TEST_ASSERT_MSG_EQ(ueUlBandwidth, enbUlBandwidth, "inconsistent UlBandwidth");
    NS_TEST_ASSERT_MSG_EQ(ueDlEarfcn, enbDlEarfcn, "inconsistent DlEarfcn");
    NS_TEST_ASSERT_MSG_EQ(ueUlEarfcn, enbUlEarfcn, "inconsistent UlEarfcn");

    if (hasContext)
    {
        Ptr<UeManager> ueManager = enbRrc->GetUeManager(rnti);
        NS_ASSERT(ueManager);
        UeManager::State state = ueManager->GetState();
        uint16_t enbImsi = ueManager->GetImsi();
        NS_TEST_ASSERT_MSG_EQ(ueImsi, enbImsi, "inconsistent Imsi");

        if (state == UeManager::CONNECTED_NORMALLY)
        {
            ObjectMapValue enbDataRadioBearerMapValue;
            ueManager->GetAttribute("DataRadioBearerMap", enbDataRadioBearerMapValue);
            NS_TEST_ASSERT_MSG_EQ(enbDataRadioBearerMapValue.GetN(),
                                  m_nBearers,
                                  "wrong num bearers at eNB");
            ObjectMapValue ueDataRadioBearerMapValue;
            ueRrc->GetAttribute("DataRadioBearerMap", ueDataRadioBearerMapValue);
            NS_TEST_ASSERT_MSG_EQ(ueDataRadioBearerMapValue.GetN(),
                                  m_nBearers,
                                  "wrong num bearers at UE");

            ObjectMapValue::Iterator enbBearerIt = enbDataRadioBearerMapValue.Begin();
            ObjectMapValue::Iterator ueBearerIt = ueDataRadioBearerMapValue.Begin();
            while (enbBearerIt != enbDataRadioBearerMapValue.End() &&
                   ueBearerIt != ueDataRadioBearerMapValue.End())
            {
                Ptr<LteDataRadioBearerInfo> enbDrbInfo =
                    enbBearerIt->second->GetObject<LteDataRadioBearerInfo>();
                Ptr<LteDataRadioBearerInfo> ueDrbInfo =
                    ueBearerIt->second->GetObject<LteDataRadioBearerInfo>();
                // NS_TEST_ASSERT_MSG_EQ (enbDrbInfo->m_epsBearer, ueDrbInfo->m_epsBearer,
                // "epsBearer differs");
                NS_TEST_ASSERT_MSG_EQ((uint32_t)enbDrbInfo->m_epsBearerIdentity,
                                      (uint32_t)ueDrbInfo->m_epsBearerIdentity,
                                      "epsBearerIdentity differs");
                NS_TEST_ASSERT_MSG_EQ((uint32_t)enbDrbInfo->m_drbIdentity,
                                      (uint32_t)ueDrbInfo->m_drbIdentity,
                                      "drbIdentity differs");
                // NS_TEST_ASSERT_MSG_EQ (enbDrbInfo->m_rlcConfig, ueDrbInfo->m_rlcConfig,
                // "rlcConfig differs");
                NS_TEST_ASSERT_MSG_EQ((uint32_t)enbDrbInfo->m_logicalChannelIdentity,
                                      (uint32_t)ueDrbInfo->m_logicalChannelIdentity,
                                      "logicalChannelIdentity differs");
                // NS_TEST_ASSERT_MSG_EQ (enbDrbInfo->m_logicalChannelConfig,
                // ueDrbInfo->m_logicalChannelConfig, "logicalChannelConfig differs");

                ++enbBearerIt;
                ++ueBearerIt;
            }

            NS_ASSERT_MSG(enbBearerIt == enbDataRadioBearerMapValue.End(),
                          "too many bearers at eNB");
            NS_ASSERT_MSG(ueBearerIt == ueDataRadioBearerMapValue.End(), "too many bearers at UE");
        }
    }
}

void
LteRrcConnectionEstablishmentTestCase::CheckNotConnected(Ptr<NetDevice> ueDevice,
                                                         Ptr<NetDevice> enbDevice)
{
    Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice>();
    Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc();
    const uint64_t imsi = ueLteDevice->GetImsi();
    const uint16_t rnti = ueRrc->GetRnti();
    NS_LOG_FUNCTION(this << imsi << rnti);
    NS_ASSERT_MSG(m_isConnectionEstablished.find(imsi) != m_isConnectionEstablished.end(),
                  "Invalid IMSI " << imsi);

    bool ueStateIsConnectedNormally = (LteUeRrc::CONNECTED_NORMALLY == ueRrc->GetState());

    Ptr<LteEnbNetDevice> enbLteDevice = enbDevice->GetObject<LteEnbNetDevice>();
    Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc();
    const bool hasContext = enbRrc->HasUeManager(rnti);
    bool contextStateIsConnectedNormally = false;
    if (hasContext)
    {
        Ptr<UeManager> ueManager = enbRrc->GetUeManager(rnti);
        NS_ASSERT(ueManager);
        contextStateIsConnectedNormally = (UeManager::CONNECTED_NORMALLY == ueManager->GetState());
    }
    NS_TEST_ASSERT_MSG_EQ(
        (!m_isConnectionEstablished[imsi] || !ueStateIsConnectedNormally || !hasContext ||
         !contextStateIsConnectedNormally),
        true,
        "it should not happen that connection is completed both at the UE and at the eNB side");
}

void
LteRrcConnectionEstablishmentTestCase::ConnectionEstablishedCallback(std::string context,
                                                                     uint64_t imsi,
                                                                     uint16_t cellId,
                                                                     uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId);
    m_isConnectionEstablished[imsi] = true;
}

void
LteRrcConnectionEstablishmentTestCase::ConnectionTimeoutCallback(std::string context,
                                                                 uint64_t imsi,
                                                                 uint16_t cellId,
                                                                 uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId);
}

/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief Lte Rrc Connection Establishment Error Test Case
 */
class LteRrcConnectionEstablishmentErrorTestCase : public LteRrcConnectionEstablishmentTestCase
{
  public:
    /**
     *
     *
     * \param jumpAwayTime the time when all the UEs 'teleport' to a pre-defined
     *                     high-interference position and stay there for 100 ms
     * \param description additional description of the test case
     */
    LteRrcConnectionEstablishmentErrorTestCase(Time jumpAwayTime, std::string description = "");

  protected:
    virtual void DoRun(void);

  private:
    /// Jump away function
    void JumpAway();
    /// Jump back function
    void JumpBack();

    Time m_jumpAwayTime;             ///< jump away time
    Ptr<MobilityModel> m_ueMobility; ///< UE mobility model
};

LteRrcConnectionEstablishmentErrorTestCase::LteRrcConnectionEstablishmentErrorTestCase(
    Time jumpAwayTime,
    std::string description)
    : LteRrcConnectionEstablishmentTestCase(1, 1, 0, 0, 1, true, false, true, description),
      m_jumpAwayTime(jumpAwayTime)
{
    NS_LOG_FUNCTION(this << GetName());
}

void
LteRrcConnectionEstablishmentErrorTestCase::DoRun()
{
    NS_LOG_FUNCTION(this << GetName());
    Config::Reset();

    if (m_nUes < 25)
    {
        Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(40));
    }
    else if (m_nUes < 60)
    {
        Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(80));
    }
    else if (m_nUes < 120)
    {
        Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(160));
    }
    else
    {
        Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));
    }

    // normal code
    m_lteHelper = CreateObject<LteHelper>();
    m_lteHelper->SetAttribute("UseIdealRrc", BooleanValue(m_useIdealRrc));

    NodeContainer enbNodes;
    NodeContainer ueNodes;

    enbNodes.Create(4);
    ueNodes.Create(1);

    MobilityHelper mobility;
    mobility.Install(ueNodes); // UE position at (0, 0, 0)
    m_ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();

    Ptr<ListPositionAllocator> enbPosition = CreateObject<ListPositionAllocator>();
    enbPosition->Add(Vector(0, 0, 0));
    enbPosition->Add(Vector(100.0, 0, 0));
    enbPosition->Add(Vector(0, 100.0, 0));
    enbPosition->Add(Vector(100.0, 100.0, 0));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(enbPosition);
    mobility.Install(enbNodes);

    int64_t stream = 1;
    NetDeviceContainer enbDevs;
    enbDevs = m_lteHelper->InstallEnbDevice(enbNodes);
    stream += m_lteHelper->AssignStreams(enbDevs, stream);

    NetDeviceContainer ueDevs;
    ueDevs = m_lteHelper->InstallUeDevice(ueNodes);
    stream += m_lteHelper->AssignStreams(ueDevs, stream);

    // custom code used for testing purposes
    // instead of lteHelper->Attach () and lteHelper->ActivateXxx

    // Set AdmitConnectionRequest attribute
    for (NetDeviceContainer::Iterator it = enbDevs.Begin(); it != enbDevs.End(); ++it)
    {
        Ptr<LteEnbRrc> enbRrc = (*it)->GetObject<LteEnbNetDevice>()->GetRrc();
        enbRrc->SetAttribute("AdmitRrcConnectionRequest",
                             BooleanValue(m_admitRrcConnectionRequest));
    }

    uint32_t i = 0;
    uint32_t tmax = 0;
    for (NetDeviceContainer::Iterator it = ueDevs.Begin(); it != ueDevs.End(); ++it)
    {
        Ptr<NetDevice> ueDevice = *it;
        Ptr<NetDevice> enbDevice = enbDevs.Get(0);
        Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice>();

        uint32_t tc = m_tConnBase + m_tConnIncrPerUe * i; // time connection start
        uint32_t tcc = tc + m_delayConnEnd;               // time check connection completed;
        uint32_t td = tcc + m_delayDiscStart;             // time disconnect start
        uint32_t tcd = td + m_delayDiscEnd;               // time check disconnection completed
        tmax = std::max(tmax, tcd);

        // trick to resolve overloading
        // void (LteHelper::* overloadedAttachFunctionPointer) (Ptr<NetDevice>, Ptr<NetDevice>) =
        // &LteHelper::Attach; Simulator::Schedule (MilliSeconds (tc),
        // overloadedAttachFunctionPointer, lteHelper, *it, enbDevice);
        Simulator::Schedule(MilliSeconds(tc),
                            &LteRrcConnectionEstablishmentErrorTestCase::Connect,
                            this,
                            ueDevice,
                            enbDevice);

        Simulator::Schedule(MilliSeconds(tcc),
                            &LteRrcConnectionEstablishmentErrorTestCase::CheckConnected,
                            this,
                            *it,
                            enbDevice);

        // disconnection not supported yet

        uint64_t imsi = ueLteDevice->GetImsi();
        m_isConnectionEstablished[imsi] = false;

        ++i;
    }

    // Connect to trace sources in UEs
    Config::Connect(
        "/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
        MakeCallback(&LteRrcConnectionEstablishmentErrorTestCase::ConnectionEstablishedCallback,
                     this));
    Config::Connect(
        "/NodeList/*/DeviceList/*/LteUeRrc/ConnectionTimeout",
        MakeCallback(&LteRrcConnectionEstablishmentErrorTestCase::ConnectionTimeoutCallback, this));

    Simulator::Schedule(m_jumpAwayTime,
                        &LteRrcConnectionEstablishmentErrorTestCase::JumpAway,
                        this);
    Simulator::Schedule(m_jumpAwayTime + MilliSeconds(99),
                        &LteRrcConnectionEstablishmentErrorTestCase::CheckNotConnected,
                        this,
                        ueDevs.Get(0),
                        enbDevs.Get(0));
    Simulator::Schedule(m_jumpAwayTime + MilliSeconds(100),
                        &LteRrcConnectionEstablishmentErrorTestCase::JumpBack,
                        this);

    Simulator::Stop(MilliSeconds(tmax + 1));

    Simulator::Run();

    Simulator::Destroy();
}

void
LteRrcConnectionEstablishmentErrorTestCase::JumpAway()
{
    NS_LOG_FUNCTION(this);
    // move to a really far away location so that transmission errors occur
    m_ueMobility->SetPosition(Vector(100000.0, 100000.0, 0.0));
}

void
LteRrcConnectionEstablishmentErrorTestCase::JumpBack()
{
    NS_LOG_FUNCTION(this);
    m_ueMobility->SetPosition(Vector(0.0, 0.0, 0.0));
}

/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief Lte Rrc Test Suite
 */
class LteRrcTestSuite : public TestSuite
{
  public:
    LteRrcTestSuite();
};

LteRrcTestSuite::LteRrcTestSuite()
    : TestSuite("lte-rrc", Type::SYSTEM)
{
    //  LogComponentEnableAll (LOG_PREFIX_ALL);
    //  LogComponentEnable ("LteRrcTest", LOG_LEVEL_ALL);
    //  LogComponentEnable ("LteEnbRrc", LOG_INFO);
    //  LogComponentEnable ("LteUeRrc", LOG_INFO);

    NS_LOG_FUNCTION(this);

    for (uint32_t useIdealRrc = 0; useIdealRrc <= 1; ++useIdealRrc)
    {
        //         <----- all times in ms ----------------->

        //                                                     nUes      tConnBase delayDiscStart
        //                                                     useIdealRrc
        //                                                        nBearers       tConnIncrPerUe
        //                                                        errorExpected
        //                                                        admitRrcConnectionRequest
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(1, 0, 0, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(1, 0, 100, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(1, 1, 0, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(1, 1, 100, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(1, 2, 0, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(1, 2, 100, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(2, 0, 20, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(2, 0, 20, 10, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(2, 0, 20, 100, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(2, 1, 20, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(2, 1, 20, 10, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(2, 1, 20, 100, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(2, 2, 20, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(2, 2, 20, 10, 1, false, useIdealRrc, true),
            Duration::QUICK);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(2, 2, 20, 100, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(3, 0, 20, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(4, 0, 20, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(4, 0, 20, 300, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(20, 0, 10, 1, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(50, 0, 0, 0, 1, false, useIdealRrc, true),
            Duration::EXTENSIVE);

        // Test cases to check admitRrcConnectionRequest=false
        //                                                     nUes      tConnBase delayDiscStart
        //                                                     useIdealRrc
        //                                                        nBearers       tConnIncrPerUe
        //                                                        errorExpected
        //                                                        admitRrcConnectionRequest
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(1, 0, 0, 0, 1, false, useIdealRrc, false),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(1, 2, 100, 0, 1, false, useIdealRrc, false),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(2, 0, 20, 0, 1, false, useIdealRrc, false),
            Duration::EXTENSIVE);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(2, 1, 20, 0, 1, false, useIdealRrc, false),
            Duration::QUICK);
        AddTestCase(
            new LteRrcConnectionEstablishmentTestCase(3, 0, 20, 0, 1, false, useIdealRrc, false),
            Duration::EXTENSIVE);
    }

    // Test cases with transmission error
    AddTestCase(new LteRrcConnectionEstablishmentErrorTestCase(Seconds(0.020214),
                                                               "failure at RRC Connection Request"),
                Duration::QUICK);
    AddTestCase(new LteRrcConnectionEstablishmentErrorTestCase(Seconds(0.025),
                                                               "failure at RRC Connection Setup"),
                Duration::QUICK);
    /*
     * The following test case is related to the Idle mode, which is an
     * unsupported feature at the moment. See also Bug 1762 Comment #25.
     */
    // AddTestCase (new LteRrcConnectionEstablishmentErrorTestCase (
    //                  Seconds (0.030),
    //                  "failure at RRC Connection Setup Complete"),
    //              Duration::QUICK);
}

static LteRrcTestSuite g_lteRrcTestSuiteInstance;

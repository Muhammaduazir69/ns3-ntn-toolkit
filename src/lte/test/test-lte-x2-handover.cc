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
 */

#include "ns3/bulk-send-helper.h"
#include "ns3/flent-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/three-gpp-http-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/application-packet-probe.h"
#include "ns3/bulk-send-application.h"
#include "ns3/flent-application.h"
#include "ns3/onoff-application.h"
#include "ns3/packet-loss-counter.h"
#include "ns3/packet-sink.h"
#include "ns3/rta-tig-mobile-gaming.h"
#include "ns3/seq-ts-echo-header.h"
#include "ns3/seq-ts-header.h"
#include "ns3/seq-ts-size-header.h"
#include "ns3/sink-application.h"
#include "ns3/source-application.h"
#include "ns3/tgax-video-traffic.h"
#include "ns3/tgax-virtual-desktop.h"
#include "ns3/tgax-voip-traffic.h"
#include "ns3/three-gpp-http-client.h"
#include "ns3/three-gpp-http-header.h"
#include "ns3/three-gpp-http-server.h"
#include "ns3/three-gpp-http-variables.h"
#include "ns3/udp-client.h"
#include "ns3/udp-echo-client.h"
#include "ns3/udp-echo-server.h"
#include "ns3/udp-server.h"
#include "ns3/udp-trace-client.h"
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
#include "ns3/internet-stack-helper.h"
#include "ns3/internet-trace-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv6-global-routing-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv6-address-helper.h"
#include "ns3/ipv6-interface-container.h"
#include "ns3/ipv6-list-routing-helper.h"
#include "ns3/ipv6-routing-helper.h"
#include "ns3/ipv6-static-routing-helper.h"
#include "ns3/neighbor-cache-helper.h"
#include "ns3/rip-helper.h"
#include "ns3/ripng-helper.h"
#include "ns3/arp-cache.h"
#include "ns3/arp-header.h"
#include "ns3/arp-l3-protocol.h"
#include "ns3/arp-queue-disc-item.h"
#include "ns3/candidate-queue.h"
#include "ns3/global-route-manager-impl.h"
#include "ns3/global-route-manager.h"
#include "ns3/global-router-interface.h"
#include "ns3/icmpv4-l4-protocol.h"
#include "ns3/icmpv4.h"
#include "ns3/icmpv6-header.h"
#include "ns3/icmpv6-l4-protocol.h"
#include "ns3/ip-l4-protocol.h"
#include "ns3/ipv4-address-generator.h"
#include "ns3/ipv4-end-point-demux.h"
#include "ns3/ipv4-end-point.h"
#include "ns3/global-routing.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-interface-address.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4-packet-filter.h"
#include "ns3/ipv4-packet-info-tag.h"
#include "ns3/ipv4-packet-probe.h"
#include "ns3/ipv4-queue-disc-item.h"
#include "ns3/ipv4-raw-socket-factory.h"
#include "ns3/ipv4-raw-socket-impl.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/ipv4.h"
#include "ns3/ipv6-address-generator.h"
#include "ns3/ipv6-end-point-demux.h"
#include "ns3/ipv6-end-point.h"
#include "ns3/ipv6-extension-demux.h"
#include "ns3/ipv6-extension-header.h"
#include "ns3/ipv6-extension.h"
#include "ns3/ipv6-header.h"
#include "ns3/ipv6-interface-address.h"
#include "ns3/ipv6-interface.h"
#include "ns3/ipv6-l3-protocol.h"
#include "ns3/ipv6-list-routing.h"
#include "ns3/ipv6-option-header.h"
#include "ns3/ipv6-option.h"
#include "ns3/ipv6-packet-filter.h"
#include "ns3/ipv6-packet-info-tag.h"
#include "ns3/ipv6-packet-probe.h"
#include "ns3/ipv6-pmtu-cache.h"
#include "ns3/ipv6-queue-disc-item.h"
#include "ns3/ipv6-raw-socket-factory.h"
#include "ns3/ipv6-route.h"
#include "ns3/ipv6-routing-protocol.h"
#include "ns3/ipv6-routing-table-entry.h"
#include "ns3/ipv6-static-routing.h"
#include "ns3/ipv6.h"
#include "ns3/loopback-net-device.h"
#include "ns3/ndisc-cache.h"
#include "ns3/rip-header.h"
#include "ns3/rip.h"
#include "ns3/ripng-header.h"
#include "ns3/ripng.h"
#include "ns3/rtt-estimator.h"
#include "ns3/tcp-bbr.h"
#include "ns3/tcp-bic.h"
#include "ns3/tcp-congestion-ops.h"
#include "ns3/tcp-cubic.h"
#include "ns3/tcp-dctcp.h"
#include "ns3/tcp-header.h"
#include "ns3/tcp-highspeed.h"
#include "ns3/tcp-htcp.h"
#include "ns3/tcp-hybla.h"
#include "ns3/tcp-illinois.h"
#include "ns3/tcp-l4-protocol.h"
#include "ns3/tcp-ledbat.h"
#include "ns3/tcp-linux-reno.h"
#include "ns3/tcp-lp.h"
#include "ns3/tcp-option-rfc793.h"
#include "ns3/tcp-option-sack-permitted.h"
#include "ns3/tcp-option-sack.h"
#include "ns3/tcp-option-ts.h"
#include "ns3/tcp-option-winscale.h"
#include "ns3/tcp-option.h"
#include "ns3/tcp-prr-recovery.h"
#include "ns3/tcp-rate-ops.h"
#include "ns3/tcp-recovery-ops.h"
#include "ns3/tcp-rx-buffer.h"
#include "ns3/tcp-scalable.h"
#include "ns3/tcp-socket-base.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/tcp-socket-state.h"
#include "ns3/tcp-socket.h"
#include "ns3/tcp-tx-buffer.h"
#include "ns3/tcp-tx-item.h"
#include "ns3/tcp-vegas.h"
#include "ns3/tcp-veno.h"
#include "ns3/tcp-westwood-plus.h"
#include "ns3/tcp-yeah.h"
#include "ns3/udp-header.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/udp-socket.h"
#include "ns3/windowed-filter.h"
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
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/ppp-header.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LteX2HandoverTest");

/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief HandoverEvent structure
 */
struct HandoverEvent
{
    Time startTime;                ///< start time
    uint32_t ueDeviceIndex;        ///< UE device index
    uint32_t sourceEnbDeviceIndex; ///< source ENB device index
    uint32_t targetEnbDeviceIndex; ///< target ENB device index
};

/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief Test X2 Handover. In this test is used NoOpHandoverAlgorithm and
 * the request for handover is generated manually, and it is not based on measurements.
 */
class LteX2HandoverTestCase : public TestCase
{
  public:
    /**
     *
     *
     * \param nUes number of UEs in the test
     * \param nDedicatedBearers number of bearers to be activated per UE
     * \param handoverEventList
     * \param handoverEventListName
     * \param schedulerType the scheduler type
     * \param admitHo
     * \param useIdealRrc true if the ideal RRC should be used
     */
    LteX2HandoverTestCase(uint32_t nUes,
                          uint32_t nDedicatedBearers,
                          std::list<HandoverEvent> handoverEventList,
                          std::string handoverEventListName,
                          std::string schedulerType,
                          bool admitHo,
                          bool useIdealRrc);

  private:
    /**
     * Build name string
     * \param nUes number of UEs in the test
     * \param nDedicatedBearers number of bearers to be activated per UE
     * \param handoverEventListName
     * \param schedulerType the scheduler type
     * \param admitHo
     * \param useIdealRrc true if the ideal RRC should be used
     * \returns the name string
     */
    static std::string BuildNameString(uint32_t nUes,
                                       uint32_t nDedicatedBearers,
                                       std::string handoverEventListName,
                                       std::string schedulerType,
                                       bool admitHo,
                                       bool useIdealRrc);
    virtual void DoRun(void);
    /**
     * Check connected function
     * \param ueDevice the UE device
     * \param enbDevice the ENB device
     */
    void CheckConnected(Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice);

    /**
     * Teleport UE between both eNBs of the test
     * \param ueNode the UE node
     */
    void TeleportUeToMiddle(Ptr<Node> ueNode);

    /**
     * Teleport UE near the target eNB of the handover
     * \param ueNode the UE node
     * \param enbNode the target eNB node
     */
    void TeleportUeNearTargetEnb(Ptr<Node> ueNode, Ptr<Node> enbNode);

    uint32_t m_nUes;                              ///< number of UEs in the test
    uint32_t m_nDedicatedBearers;                 ///< number of UEs in the test
    std::list<HandoverEvent> m_handoverEventList; ///< handover event list
    std::string m_handoverEventListName;          ///< handover event list name
    bool m_epc;                                   ///< whether to use EPC
    std::string m_schedulerType;                  ///< scheduler type
    bool m_admitHo;                               ///< whether to admit the handover request
    bool m_useIdealRrc;                           ///< whether to use the ideal RRC
    Ptr<LteHelper> m_lteHelper;                   ///< LTE helper
    Ptr<PointToPointEpcHelper> m_epcHelper;       ///< EPC helper

    /**
     * \ingroup lte-test
     * \ingroup tests
     *
     * \brief BearerData structure
     */
    struct BearerData
    {
        uint32_t bid;           ///< BID
        Ptr<PacketSink> dlSink; ///< DL sink
        Ptr<PacketSink> ulSink; ///< UL sink
        uint32_t dlOldTotalRx;  ///< DL old total receive
        uint32_t ulOldTotalRx;  ///< UL old total receive
    };

    /**
     * \ingroup lte-test
     * \ingroup tests
     *
     * \brief UeData structure
     */
    struct UeData
    {
        uint32_t id;                          ///< ID
        std::list<BearerData> bearerDataList; ///< bearer ID list
    };

    /**
     * \brief Save stats after handover function
     * \param ueIndex the index of the UE
     */
    void SaveStatsAfterHandover(uint32_t ueIndex);
    /**
     * \brief Check stats a while after handover function
     * \param ueIndex the index of the UE
     */
    void CheckStatsAWhileAfterHandover(uint32_t ueIndex);

    std::vector<UeData> m_ueDataVector; ///< UE data vector

    const Time m_maxHoDuration;        ///< maximum HO duration
    const Time m_statsDuration;        ///< stats duration
    const Time m_udpClientInterval;    ///< UDP client interval
    const uint32_t m_udpClientPktSize; ///< UDP client packet size
};

std::string
LteX2HandoverTestCase::BuildNameString(uint32_t nUes,
                                       uint32_t nDedicatedBearers,
                                       std::string handoverEventListName,
                                       std::string schedulerType,
                                       bool admitHo,
                                       bool useIdealRrc)
{
    std::ostringstream oss;
    oss << " nUes=" << nUes << " nDedicatedBearers=" << nDedicatedBearers << " " << schedulerType
        << " admitHo=" << admitHo << " hoList: " << handoverEventListName;
    if (useIdealRrc)
    {
        oss << ", ideal RRC";
    }
    else
    {
        oss << ", real RRC";
    }
    return oss.str();
}

LteX2HandoverTestCase::LteX2HandoverTestCase(uint32_t nUes,
                                             uint32_t nDedicatedBearers,
                                             std::list<HandoverEvent> handoverEventList,
                                             std::string handoverEventListName,
                                             std::string schedulerType,
                                             bool admitHo,
                                             bool useIdealRrc)
    : TestCase(BuildNameString(nUes,
                               nDedicatedBearers,
                               handoverEventListName,
                               schedulerType,
                               admitHo,
                               useIdealRrc)),
      m_nUes(nUes),
      m_nDedicatedBearers(nDedicatedBearers),
      m_handoverEventList(handoverEventList),
      m_handoverEventListName(handoverEventListName),
      m_epc(true),
      m_schedulerType(schedulerType),
      m_admitHo(admitHo),
      m_useIdealRrc(useIdealRrc),
      m_maxHoDuration(Seconds(0.1)),
      m_statsDuration(Seconds(0.1)),
      m_udpClientInterval(Seconds(0.01)),
      m_udpClientPktSize(100)

{
}

void
LteX2HandoverTestCase::DoRun()
{
    NS_LOG_FUNCTION(this << BuildNameString(m_nUes,
                                            m_nDedicatedBearers,
                                            m_handoverEventListName,
                                            m_schedulerType,
                                            m_admitHo,
                                            m_useIdealRrc));

    uint32_t previousSeed = RngSeedManager::GetSeed();
    uint64_t previousRun = RngSeedManager::GetRun();
    Config::Reset();
    // This test is sensitive to random variable stream assigments
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(2);
    Config::SetDefault("ns3::UdpClient::Interval", TimeValue(m_udpClientInterval));
    Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(1000000));
    Config::SetDefault("ns3::UdpClient::PacketSize", UintegerValue(m_udpClientPktSize));

    // Disable Uplink Power Control
    Config::SetDefault("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue(false));

    Config::SetDefault("ns3::PointToPointEpcHelper::S1apLinkDelay", TimeValue(Seconds(0)));

    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(40));

    int64_t stream = 1;

    m_lteHelper = CreateObject<LteHelper>();
    m_lteHelper->SetAttribute("PathlossModel",
                              StringValue("ns3::FriisSpectrumPropagationLossModel"));
    m_lteHelper->SetSchedulerType(m_schedulerType);
    m_lteHelper->SetHandoverAlgorithmType(
        "ns3::NoOpHandoverAlgorithm"); // disable automatic handover
    m_lteHelper->SetAttribute("UseIdealRrc", BooleanValue(m_useIdealRrc));

    // set DL and UL bandwidth.
    m_lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(25));
    m_lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(25));

    NodeContainer enbNodes;
    enbNodes.Create(2);
    NodeContainer ueNodes;
    ueNodes.Create(m_nUes);

    if (m_epc)
    {
        m_epcHelper = CreateObject<PointToPointEpcHelper>();
        m_lteHelper->SetEpcHelper(m_epcHelper);
    }

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(-3000, 0, 0)); // enb0
    positionAlloc->Add(Vector(3000, 0, 0));  // enb1
    for (uint16_t i = 0; i < m_nUes; i++)
    {
        positionAlloc->Add(Vector(-3000, 100, 0));
    }
    MobilityHelper mobility;
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(enbNodes);
    mobility.Install(ueNodes);

    NetDeviceContainer enbDevices;
    enbDevices = m_lteHelper->InstallEnbDevice(enbNodes);
    stream += m_lteHelper->AssignStreams(enbDevices, stream);
    for (NetDeviceContainer::Iterator it = enbDevices.Begin(); it != enbDevices.End(); ++it)
    {
        Ptr<LteEnbRrc> enbRrc = (*it)->GetObject<LteEnbNetDevice>()->GetRrc();
        enbRrc->SetAttribute("AdmitHandoverRequest", BooleanValue(m_admitHo));
    }

    NetDeviceContainer ueDevices;
    ueDevices = m_lteHelper->InstallUeDevice(ueNodes);
    stream += m_lteHelper->AssignStreams(ueDevices, stream);

    Ipv4Address remoteHostAddr;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ipv4InterfaceContainer ueIpIfaces;
    Ptr<Node> remoteHost;
    if (m_epc)
    {
        // Create a single RemoteHost
        NodeContainer remoteHostContainer;
        remoteHostContainer.Create(1);
        remoteHost = remoteHostContainer.Get(0);
        InternetStackHelper internet;
        internet.Install(remoteHostContainer);

        // Create the Internet
        PointToPointHelper p2ph;
        p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
        p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
        p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
        Ptr<Node> pgw = m_epcHelper->GetPgwNode();
        NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
        Ipv4AddressHelper ipv4h;
        ipv4h.SetBase("1.0.0.0", "255.0.0.0");
        Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
        // in this container, interface 0 is the pgw, 1 is the remoteHost
        remoteHostAddr = internetIpIfaces.GetAddress(1);

        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
        remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                                   Ipv4Mask("255.0.0.0"),
                                                   1);

        // Install the IP stack on the UEs
        internet.Install(ueNodes);
        ueIpIfaces = m_epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevices));
    }

    // attachment (needs to be done after IP stack configuration)
    // all UEs attached to eNB 0 at the beginning
    m_lteHelper->Attach(ueDevices, enbDevices.Get(0));

    if (m_epc)
    {
        // always true: bool epcDl = true;
        // always true: bool epcUl = true;
        // the rest of this block is copied from lena-dual-stripe

        // Install and start applications on UEs and remote host
        uint16_t dlPort = 10000;
        uint16_t ulPort = 20000;

        // randomize a bit start times to avoid simulation artifacts
        // (e.g., buffer overflows due to packet transmissions happening
        // exactly at the same time)
        Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable>();
        startTimeSeconds->SetAttribute("Min", DoubleValue(0));
        startTimeSeconds->SetAttribute("Max", DoubleValue(0.010));
        startTimeSeconds->SetStream(stream++);

        for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
        {
            Ptr<Node> ue = ueNodes.Get(u);
            // Set the default gateway for the UE
            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(m_epcHelper->GetUeDefaultGatewayAddress(), 1);

            UeData ueData;

            for (uint32_t b = 0; b < m_nDedicatedBearers; ++b)
            {
                ++dlPort;
                ++ulPort;

                ApplicationContainer clientApps;
                ApplicationContainer serverApps;
                BearerData bearerData = BearerData();

                // always true: if (epcDl)
                {
                    UdpClientHelper dlClientHelper(ueIpIfaces.GetAddress(u), dlPort);
                    clientApps.Add(dlClientHelper.Install(remoteHost));
                    PacketSinkHelper dlPacketSinkHelper(
                        "ns3::UdpSocketFactory",
                        InetSocketAddress(Ipv4Address::GetAny(), dlPort));
                    ApplicationContainer sinkContainer = dlPacketSinkHelper.Install(ue);
                    bearerData.dlSink = sinkContainer.Get(0)->GetObject<PacketSink>();
                    serverApps.Add(sinkContainer);
                }
                // always true: if (epcUl)
                {
                    UdpClientHelper ulClientHelper(remoteHostAddr, ulPort);
                    clientApps.Add(ulClientHelper.Install(ue));
                    PacketSinkHelper ulPacketSinkHelper(
                        "ns3::UdpSocketFactory",
                        InetSocketAddress(Ipv4Address::GetAny(), ulPort));
                    ApplicationContainer sinkContainer = ulPacketSinkHelper.Install(remoteHost);
                    bearerData.ulSink = sinkContainer.Get(0)->GetObject<PacketSink>();
                    serverApps.Add(sinkContainer);
                }

                Ptr<EpcTft> tft = Create<EpcTft>();
                // always true: if (epcDl)
                {
                    EpcTft::PacketFilter dlpf;
                    dlpf.localPortStart = dlPort;
                    dlpf.localPortEnd = dlPort;
                    tft->Add(dlpf);
                }
                // always true: if (epcUl)
                {
                    EpcTft::PacketFilter ulpf;
                    ulpf.remotePortStart = ulPort;
                    ulpf.remotePortEnd = ulPort;
                    tft->Add(ulpf);
                }

                // always true: if (epcDl || epcUl)
                {
                    EpsBearer bearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
                    m_lteHelper->ActivateDedicatedEpsBearer(ueDevices.Get(u), bearer, tft);
                }
                double d = startTimeSeconds->GetValue();
                Time startTime = Seconds(d);
                serverApps.Start(startTime);
                clientApps.Start(startTime);

                ueData.bearerDataList.push_back(bearerData);

            } // end for b

            m_ueDataVector.push_back(ueData);
        }
    }
    else // (epc == false)
    {
        // for radio bearer activation purposes, consider together home UEs and macro UEs
        for (uint32_t u = 0; u < ueDevices.GetN(); ++u)
        {
            Ptr<NetDevice> ueDev = ueDevices.Get(u);
            for (uint32_t b = 0; b < m_nDedicatedBearers; ++b)
            {
                enum EpsBearer::Qci q = EpsBearer::NGBR_VIDEO_TCP_DEFAULT;
                EpsBearer bearer(q);
                m_lteHelper->ActivateDataRadioBearer(ueDev, bearer);
            }
        }
    }

    m_lteHelper->AddX2Interface(enbNodes);

    // check initial RRC connection
    const Time maxRrcConnectionEstablishmentDuration = Seconds(0.080);
    for (NetDeviceContainer::Iterator it = ueDevices.Begin(); it != ueDevices.End(); ++it)
    {
        Simulator::Schedule(maxRrcConnectionEstablishmentDuration,
                            &LteX2HandoverTestCase::CheckConnected,
                            this,
                            *it,
                            enbDevices.Get(0));
    }

    // schedule handover events and corresponding checks

    Time stopTime = Seconds(0);
    for (std::list<HandoverEvent>::iterator hoEventIt = m_handoverEventList.begin();
         hoEventIt != m_handoverEventList.end();
         ++hoEventIt)
    {
        // Teleport the UE between both eNBs just before the handover starts
        Simulator::Schedule(hoEventIt->startTime - MilliSeconds(10),
                            &LteX2HandoverTestCase::TeleportUeToMiddle,
                            this,
                            ueNodes.Get(hoEventIt->ueDeviceIndex));

        Simulator::Schedule(hoEventIt->startTime,
                            &LteX2HandoverTestCase::CheckConnected,
                            this,
                            ueDevices.Get(hoEventIt->ueDeviceIndex),
                            enbDevices.Get(hoEventIt->sourceEnbDeviceIndex));

        m_lteHelper->HandoverRequest(hoEventIt->startTime,
                                     ueDevices.Get(hoEventIt->ueDeviceIndex),
                                     enbDevices.Get(hoEventIt->sourceEnbDeviceIndex),
                                     enbDevices.Get(hoEventIt->targetEnbDeviceIndex));

        // Once the handover is finished, teleport the UE near the target eNB
        Simulator::Schedule(hoEventIt->startTime + MilliSeconds(40),
                            &LteX2HandoverTestCase::TeleportUeNearTargetEnb,
                            this,
                            ueNodes.Get(hoEventIt->ueDeviceIndex),
                            enbNodes.Get(m_admitHo ? hoEventIt->targetEnbDeviceIndex
                                                   : hoEventIt->sourceEnbDeviceIndex));

        Time hoEndTime = hoEventIt->startTime + m_maxHoDuration;
        Simulator::Schedule(hoEndTime,
                            &LteX2HandoverTestCase::CheckConnected,
                            this,
                            ueDevices.Get(hoEventIt->ueDeviceIndex),
                            enbDevices.Get(m_admitHo ? hoEventIt->targetEnbDeviceIndex
                                                     : hoEventIt->sourceEnbDeviceIndex));
        Simulator::Schedule(hoEndTime,
                            &LteX2HandoverTestCase::SaveStatsAfterHandover,
                            this,
                            hoEventIt->ueDeviceIndex);

        Time checkStatsAfterHoTime = hoEndTime + m_statsDuration;
        Simulator::Schedule(checkStatsAfterHoTime,
                            &LteX2HandoverTestCase::CheckStatsAWhileAfterHandover,
                            this,
                            hoEventIt->ueDeviceIndex);
        if (stopTime <= checkStatsAfterHoTime)
        {
            stopTime = checkStatsAfterHoTime + MilliSeconds(1);
        }
    }

    // m_lteHelper->EnableRlcTraces ();
    // m_lteHelper->EnablePdcpTraces();

    Simulator::Stop(stopTime);

    Simulator::Run();

    Simulator::Destroy();

    // Undo changes to default settings
    Config::Reset();
    // Restore the previous settings of RngSeed and RngRun
    RngSeedManager::SetSeed(previousSeed);
    RngSeedManager::SetRun(previousRun);
}

void
LteX2HandoverTestCase::CheckConnected(Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice)
{
    Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice>();
    Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc();
    NS_TEST_ASSERT_MSG_EQ(ueRrc->GetState(), LteUeRrc::CONNECTED_NORMALLY, "Wrong LteUeRrc state!");

    Ptr<LteEnbNetDevice> enbLteDevice = enbDevice->GetObject<LteEnbNetDevice>();
    Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc();
    uint16_t rnti = ueRrc->GetRnti();
    Ptr<UeManager> ueManager = enbRrc->GetUeManager(rnti);
    NS_TEST_ASSERT_MSG_NE(ueManager, nullptr, "RNTI " << rnti << " not found in eNB");

    UeManager::State ueManagerState = ueManager->GetState();
    NS_TEST_ASSERT_MSG_EQ(ueManagerState, UeManager::CONNECTED_NORMALLY, "Wrong UeManager state!");
    NS_ASSERT_MSG(ueManagerState == UeManager::CONNECTED_NORMALLY, "Wrong UeManager state!");

    uint16_t ueCellId = ueRrc->GetCellId();
    uint16_t enbCellId = enbLteDevice->GetCellId();
    uint8_t ueDlBandwidth = ueRrc->GetDlBandwidth();
    uint8_t enbDlBandwidth = enbLteDevice->GetDlBandwidth();
    uint8_t ueUlBandwidth = ueRrc->GetUlBandwidth();
    uint8_t enbUlBandwidth = enbLteDevice->GetUlBandwidth();
    uint8_t ueDlEarfcn = ueRrc->GetDlEarfcn();
    uint8_t enbDlEarfcn = enbLteDevice->GetDlEarfcn();
    uint8_t ueUlEarfcn = ueRrc->GetUlEarfcn();
    uint8_t enbUlEarfcn = enbLteDevice->GetUlEarfcn();
    uint64_t ueImsi = ueLteDevice->GetImsi();
    uint64_t enbImsi = ueManager->GetImsi();

    NS_TEST_ASSERT_MSG_EQ(ueImsi, enbImsi, "inconsistent IMSI");
    NS_TEST_ASSERT_MSG_EQ(ueCellId, enbCellId, "inconsistent CellId");
    NS_TEST_ASSERT_MSG_EQ(ueDlBandwidth, enbDlBandwidth, "inconsistent DlBandwidth");
    NS_TEST_ASSERT_MSG_EQ(ueUlBandwidth, enbUlBandwidth, "inconsistent UlBandwidth");
    NS_TEST_ASSERT_MSG_EQ(ueDlEarfcn, enbDlEarfcn, "inconsistent DlEarfcn");
    NS_TEST_ASSERT_MSG_EQ(ueUlEarfcn, enbUlEarfcn, "inconsistent UlEarfcn");

    ObjectMapValue enbDataRadioBearerMapValue;
    ueManager->GetAttribute("DataRadioBearerMap", enbDataRadioBearerMapValue);
    NS_TEST_ASSERT_MSG_EQ(enbDataRadioBearerMapValue.GetN(),
                          m_nDedicatedBearers + 1,
                          "wrong num bearers at eNB");

    ObjectMapValue ueDataRadioBearerMapValue;
    ueRrc->GetAttribute("DataRadioBearerMap", ueDataRadioBearerMapValue);
    NS_TEST_ASSERT_MSG_EQ(ueDataRadioBearerMapValue.GetN(),
                          m_nDedicatedBearers + 1,
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
        // NS_TEST_ASSERT_MSG_EQ (enbDrbInfo->m_epsBearer, ueDrbInfo->m_epsBearer, "epsBearer
        // differs");
        NS_TEST_ASSERT_MSG_EQ((uint32_t)enbDrbInfo->m_epsBearerIdentity,
                              (uint32_t)ueDrbInfo->m_epsBearerIdentity,
                              "epsBearerIdentity differs");
        NS_TEST_ASSERT_MSG_EQ((uint32_t)enbDrbInfo->m_drbIdentity,
                              (uint32_t)ueDrbInfo->m_drbIdentity,
                              "drbIdentity differs");
        // NS_TEST_ASSERT_MSG_EQ (enbDrbInfo->m_rlcConfig, ueDrbInfo->m_rlcConfig, "rlcConfig
        // differs");
        NS_TEST_ASSERT_MSG_EQ((uint32_t)enbDrbInfo->m_logicalChannelIdentity,
                              (uint32_t)ueDrbInfo->m_logicalChannelIdentity,
                              "logicalChannelIdentity differs");
        // NS_TEST_ASSERT_MSG_EQ (enbDrbInfo->m_logicalChannelConfig,
        // ueDrbInfo->m_logicalChannelConfig, "logicalChannelConfig differs");

        ++enbBearerIt;
        ++ueBearerIt;
    }
    NS_ASSERT_MSG(enbBearerIt == enbDataRadioBearerMapValue.End(), "too many bearers at eNB");
    NS_ASSERT_MSG(ueBearerIt == ueDataRadioBearerMapValue.End(), "too many bearers at UE");
}

void
LteX2HandoverTestCase::TeleportUeToMiddle(Ptr<Node> ueNode)
{
    Ptr<MobilityModel> ueMobility = ueNode->GetObject<MobilityModel>();
    ueMobility->SetPosition(Vector(0.0, 0.0, 0.0));
}

void
LteX2HandoverTestCase::TeleportUeNearTargetEnb(Ptr<Node> ueNode, Ptr<Node> enbNode)
{
    Ptr<MobilityModel> enbMobility = enbNode->GetObject<MobilityModel>();
    Vector pos = enbMobility->GetPosition();

    Ptr<MobilityModel> ueMobility = ueNode->GetObject<MobilityModel>();
    ueMobility->SetPosition(pos + Vector(0.0, 100.0, 0.0));
}

void
LteX2HandoverTestCase::SaveStatsAfterHandover(uint32_t ueIndex)
{
    for (std::list<BearerData>::iterator it = m_ueDataVector.at(ueIndex).bearerDataList.begin();
         it != m_ueDataVector.at(ueIndex).bearerDataList.end();
         ++it)
    {
        it->dlOldTotalRx = it->dlSink->GetTotalRx();
        it->ulOldTotalRx = it->ulSink->GetTotalRx();
    }
}

void
LteX2HandoverTestCase::CheckStatsAWhileAfterHandover(uint32_t ueIndex)
{
    uint32_t b = 1;
    for (std::list<BearerData>::iterator it = m_ueDataVector.at(ueIndex).bearerDataList.begin();
         it != m_ueDataVector.at(ueIndex).bearerDataList.end();
         ++it)
    {
        uint32_t dlRx = it->dlSink->GetTotalRx() - it->dlOldTotalRx;
        uint32_t ulRx = it->ulSink->GetTotalRx() - it->ulOldTotalRx;
        uint32_t expectedBytes =
            m_udpClientPktSize * (m_statsDuration / m_udpClientInterval).GetDouble();

        NS_TEST_ASSERT_MSG_EQ(dlRx,
                              expectedBytes,
                              "too few RX bytes in DL, ue=" << ueIndex << ", b=" << b);
        NS_TEST_ASSERT_MSG_EQ(ulRx,
                              expectedBytes,
                              "too few RX bytes in UL, ue=" << ueIndex << ", b=" << b);
        ++b;
    }
}

/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief LTE X2 Handover Test Suite.
 *
 * In this test suite, we use NoOpHandoverAlgorithm, i.e. "handover algorithm which does nothing"
 * is used and handover is triggered manually. The automatic handover algorithms (A2A4, A3Rsrp)
 * are not tested.
 *
 * The tests are designed to check that eNB-buffered data received while a handover is in progress
 * is not lost but successfully forwarded. But the test suite doesn't test for possible loss of
 * RLC-buffered data because "lossless" handover is not implemented, and there are other application
 * send patterns (outside of the range tested here) that may incur losses.
 */
class LteX2HandoverTestSuite : public TestSuite
{
  public:
    LteX2HandoverTestSuite();
};

LteX2HandoverTestSuite::LteX2HandoverTestSuite()
    : TestSuite("lte-x2-handover", Type::SYSTEM)
{
    // in the following:
    // fwd means handover from enb 0 to enb 1
    // bwd means handover from enb 1 to enb 0

    HandoverEvent ue1fwd;
    ue1fwd.startTime = MilliSeconds(100);
    ue1fwd.ueDeviceIndex = 0;
    ue1fwd.sourceEnbDeviceIndex = 0;
    ue1fwd.targetEnbDeviceIndex = 1;

    HandoverEvent ue1bwd;
    ue1bwd.startTime = MilliSeconds(400);
    ue1bwd.ueDeviceIndex = 0;
    ue1bwd.sourceEnbDeviceIndex = 1;
    ue1bwd.targetEnbDeviceIndex = 0;

    HandoverEvent ue1fwdagain;
    ue1fwdagain.startTime = MilliSeconds(700);
    ue1fwdagain.ueDeviceIndex = 0;
    ue1fwdagain.sourceEnbDeviceIndex = 0;
    ue1fwdagain.targetEnbDeviceIndex = 1;

    HandoverEvent ue2fwd;
    ue2fwd.startTime = MilliSeconds(110);
    ue2fwd.ueDeviceIndex = 1;
    ue2fwd.sourceEnbDeviceIndex = 0;
    ue2fwd.targetEnbDeviceIndex = 1;

    HandoverEvent ue2bwd;
    ue2bwd.startTime = MilliSeconds(350);
    ue2bwd.ueDeviceIndex = 1;
    ue2bwd.sourceEnbDeviceIndex = 1;
    ue2bwd.targetEnbDeviceIndex = 0;

    std::string hel0name("none");
    std::list<HandoverEvent> hel0;

    std::string hel1name("1 fwd");
    std::list<HandoverEvent> hel1;
    hel1.push_back(ue1fwd);

    std::string hel2name("1 fwd & bwd");
    std::list<HandoverEvent> hel2;
    hel2.push_back(ue1fwd);
    hel2.push_back(ue1bwd);

    std::string hel3name("1 fwd & bwd & fwd");
    std::list<HandoverEvent> hel3;
    hel3.push_back(ue1fwd);
    hel3.push_back(ue1bwd);
    hel3.push_back(ue1fwdagain);

    std::string hel4name("1+2 fwd");
    std::list<HandoverEvent> hel4;
    hel4.push_back(ue1fwd);
    hel4.push_back(ue2fwd);

    std::string hel5name("1+2 fwd & bwd");
    std::list<HandoverEvent> hel5;
    hel5.push_back(ue1fwd);
    hel5.push_back(ue1bwd);
    hel5.push_back(ue2fwd);
    hel5.push_back(ue2bwd);

    std::string hel6name("2 fwd");
    std::list<HandoverEvent> hel6;
    hel6.push_back(ue2fwd);

    std::string hel7name("2 fwd & bwd");
    std::list<HandoverEvent> hel7;
    hel7.push_back(ue2fwd);
    hel7.push_back(ue2bwd);

    std::vector<std::string> schedulers;
    schedulers.push_back("ns3::RrFfMacScheduler");
    schedulers.push_back("ns3::PfFfMacScheduler");
    for (std::vector<std::string>::iterator schedIt = schedulers.begin();
         schedIt != schedulers.end();
         ++schedIt)
    {
        for (int32_t useIdealRrc = 1; useIdealRrc >= 0; --useIdealRrc)
        {
            //                                     nUes, nDBearers, helist, name, sched, admitHo,
            //                                     idealRrc
            AddTestCase(
                new LteX2HandoverTestCase(1, 0, hel0, hel0name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 0, hel0, hel0name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 5, hel0, hel0name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 5, hel0, hel0name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 0, hel1, hel1name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 1, hel1, hel1name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 2, hel1, hel1name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 0, hel1, hel1name, *schedIt, false, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 1, hel1, hel1name, *schedIt, false, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 2, hel1, hel1name, *schedIt, false, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 0, hel1, hel1name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 1, hel1, hel1name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 2, hel1, hel1name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 0, hel1, hel1name, *schedIt, false, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 1, hel1, hel1name, *schedIt, false, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 2, hel1, hel1name, *schedIt, false, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 0, hel2, hel2name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 1, hel2, hel2name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 2, hel2, hel2name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 0, hel3, hel3name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 1, hel3, hel3name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(1, 2, hel3, hel3name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 0, hel3, hel3name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 1, hel3, hel3name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 2, hel3, hel3name, *schedIt, true, useIdealRrc),
                Duration::QUICK);
            AddTestCase(
                new LteX2HandoverTestCase(2, 0, hel4, hel4name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 1, hel4, hel4name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 2, hel4, hel4name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 0, hel5, hel5name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 1, hel5, hel5name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(2, 2, hel5, hel5name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(3, 0, hel3, hel3name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(3, 1, hel3, hel3name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(3, 2, hel3, hel3name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(3, 0, hel4, hel4name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(3, 1, hel4, hel4name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(3, 2, hel4, hel4name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(3, 0, hel5, hel5name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(3, 1, hel5, hel5name, *schedIt, true, useIdealRrc),
                Duration::EXTENSIVE);
            AddTestCase(
                new LteX2HandoverTestCase(3, 2, hel5, hel5name, *schedIt, true, useIdealRrc),
                Duration::QUICK);
        }
    }
}

static LteX2HandoverTestSuite g_lteX2HandoverTestSuiteInstance;

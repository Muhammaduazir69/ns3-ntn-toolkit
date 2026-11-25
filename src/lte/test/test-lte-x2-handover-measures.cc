/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Authors: Nicola Baldo <nbaldo@cttc.es>
 *          Manuel Requena <manuel.requena@cttc.es>
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

NS_LOG_COMPONENT_DEFINE("LteX2HandoverMeasuresTest");

/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief CheckPointEvent structure
 */
struct CheckPointEvent
{
    Time checkStartTime;     ///< check start time
    Time checkStopTime;      ///< check stop time
    Time checkInterval;      ///< check interval
    uint32_t ueDeviceIndex;  ///< UE device index
    uint32_t enbDeviceIndex; ///< ENB device index

    /**
     *  Constructor
     *
     * \param start the start time
     * \param stop the stop time
     * \param interval the interval time
     * \param ueIndex the UE index
     * \param enbIndex the ENB index
     */
    CheckPointEvent(Time start, Time stop, Time interval, uint32_t ueIndex, uint32_t enbIndex)
        : checkStartTime(start),
          checkStopTime(stop),
          checkInterval(interval),
          ueDeviceIndex(ueIndex),
          enbDeviceIndex(enbIndex)
    {
    }
};

/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief Test different X2 handover measures and algorithms, e.g. A2A4RsrqHandoverAlgorithm and
 * A3RsrpHandoverAlgorithm. Test defines different handover parameters and scenario configurations.
 */
class LteX2HandoverMeasuresTestCase : public TestCase
{
  public:
    /**
     *
     *
     * \param nEnbs number of eNBs in the test
     * \param nUes number of UEs in the test
     * \param nDedicatedBearers number of bearers to be activated per UE
     * \param checkPointEventList
     * \param checkPointEventListName
     * \param useUdp true if UDP is to be used, false if TCP is to be used
     * \param schedulerType type of scheduler to be used (e.g. "ns3::PfFfMacScheduler")
     * \param handoverAlgorithmType type of handover algorithm to be used (e.g.
     * "ns3::A3RsrpHandoverAlgorithm") \param admitHo \param useIdealRrc true if ideal RRC is to be
     * used, false if real RRC is to be used
     *
     * \return
     */
    LteX2HandoverMeasuresTestCase(uint32_t nEnbs,
                                  uint32_t nUes,
                                  uint32_t nDedicatedBearers,
                                  std::list<CheckPointEvent> checkPointEventList,
                                  std::string checkPointEventListName,
                                  bool useUdp,
                                  std::string schedulerType,
                                  std::string handoverAlgorithmType,
                                  bool admitHo,
                                  bool useIdealRrc);

  private:
    /**
     * Build name string
     * \param nEnbs number of eNBs in the test
     * \param nUes number of UEs in the test
     * \param nDedicatedBearers number of bearers to be activated per UE
     * \param checkPointEventListName
     * \param useUdp true if UDP is to be used, false if TCP is to be used
     * \param schedulerType the scheduler type
     * \param handoverAlgorithmType type of handover algorithm to be used (e.g.
     * "ns3::A3RsrpHandoverAlgorithm") \param admitHo \param useIdealRrc true if the ideal RRC
     * should be used \returns the name string
     */
    static std::string BuildNameString(uint32_t nEnbs,
                                       uint32_t nUes,
                                       uint32_t nDedicatedBearers,
                                       std::string checkPointEventListName,
                                       bool useUdp,
                                       std::string schedulerType,
                                       std::string handoverAlgorithmType,
                                       bool admitHo,
                                       bool useIdealRrc);
    virtual void DoRun(void);
    /**
     * Check connected function
     * \param ueDevice the UE device
     * \param enbDevice the ENB device
     */
    void CheckConnected(Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice);

    uint32_t m_nEnbs;                                 ///< number of eNBs in the test
    uint32_t m_nUes;                                  ///< number of UEs in the test
    uint32_t m_nDedicatedBearers;                     ///< number of UEs in the test
    std::list<CheckPointEvent> m_checkPointEventList; ///< check point event list
    std::string m_checkPointEventListName;            ///< check point event list name
    bool m_epc;                                       ///< whether to use EPC
    bool m_useUdp;                                    ///<  whether to use UDP traffic
    std::string m_schedulerType;                      ///< scheduler type
    std::string m_handoverAlgorithmType;              ///< handover algorithm type
    bool m_admitHo;                                   ///< whether to configure to admit handover
    bool m_useIdealRrc;                               ///< whether to use ideal RRC
    Ptr<LteHelper> m_lteHelper;                       ///< LTE helper
    Ptr<PointToPointEpcHelper> m_epcHelper;           ///< EPC helper

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
     * \brief Save stats  function
     * \param ueIndex the index of the UE
     */
    void SaveStats(uint32_t ueIndex);
    /**
     * \brief Check stats  function
     * \param ueIndex the index of the UE
     */
    void CheckStats(uint32_t ueIndex);

    std::vector<UeData> m_ueDataVector; ///< UE data vector

    const Time m_maxHoDuration;        ///< maximum HO duration
    const Time m_statsDuration;        ///< stats duration
    const Time m_udpClientInterval;    ///< UDP client interval
    const uint32_t m_udpClientPktSize; ///< UDP client packet size
};

std::string
LteX2HandoverMeasuresTestCase::BuildNameString(uint32_t nEnbs,
                                               uint32_t nUes,
                                               uint32_t nDedicatedBearers,
                                               std::string checkPointEventListName,
                                               bool useUdp,
                                               std::string schedulerType,
                                               std::string handoverAlgorithmType,
                                               bool admitHo,
                                               bool useIdealRrc)
{
    std::ostringstream oss;
    oss << "nEnbs=" << nEnbs << " nUes=" << nUes << " nDedicatedBearers=" << nDedicatedBearers
        << " udp=" << useUdp << " " << schedulerType << " " << handoverAlgorithmType
        << " admitHo=" << admitHo << " hoList: " << checkPointEventListName;
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

LteX2HandoverMeasuresTestCase::LteX2HandoverMeasuresTestCase(
    uint32_t nEnbs,
    uint32_t nUes,
    uint32_t nDedicatedBearers,
    std::list<CheckPointEvent> checkPointEventList,
    std::string checkPointEventListName,
    bool useUdp,
    std::string schedulerType,
    std::string handoverAlgorithmType,
    bool admitHo,
    bool useIdealRrc)
    : TestCase(BuildNameString(nEnbs,
                               nUes,
                               nDedicatedBearers,
                               checkPointEventListName,
                               useUdp,
                               schedulerType,
                               handoverAlgorithmType,
                               admitHo,
                               useIdealRrc)),
      m_nEnbs(nEnbs),
      m_nUes(nUes),
      m_nDedicatedBearers(nDedicatedBearers),
      m_checkPointEventList(checkPointEventList),
      m_checkPointEventListName(checkPointEventListName),
      m_epc(true),
      m_useUdp(useUdp),
      m_schedulerType(schedulerType),
      m_handoverAlgorithmType(handoverAlgorithmType),
      m_admitHo(admitHo),
      m_useIdealRrc(useIdealRrc),
      m_maxHoDuration(Seconds(0.1)),
      m_statsDuration(Seconds(0.5)),
      m_udpClientInterval(Seconds(0.01)),
      m_udpClientPktSize(100)
{
}

void
LteX2HandoverMeasuresTestCase::DoRun()
{
    NS_LOG_FUNCTION(this << BuildNameString(m_nEnbs,
                                            m_nUes,
                                            m_nDedicatedBearers,
                                            m_checkPointEventListName,
                                            m_useUdp,
                                            m_schedulerType,
                                            m_handoverAlgorithmType,
                                            m_admitHo,
                                            m_useIdealRrc));

    Config::Reset();
    Config::SetDefault("ns3::UdpClient::Interval", TimeValue(m_udpClientInterval));
    Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(1000000));
    Config::SetDefault("ns3::UdpClient::PacketSize", UintegerValue(m_udpClientPktSize));
    Config::SetDefault("ns3::LteEnbRrc::HandoverJoiningTimeoutDuration",
                       TimeValue(MilliSeconds(200)));
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(20));
    Config::SetDefault("ns3::PointToPointEpcHelper::S1apLinkDelay", TimeValue(Seconds(0)));

    // Disable Uplink Power Control
    Config::SetDefault("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue(false));

    int64_t stream = 1;

    m_lteHelper = CreateObject<LteHelper>();
    m_lteHelper->SetAttribute("PathlossModel",
                              StringValue("ns3::FriisSpectrumPropagationLossModel"));
    m_lteHelper->SetAttribute("UseIdealRrc", BooleanValue(m_useIdealRrc));
    m_lteHelper->SetSchedulerType(m_schedulerType);

    if (m_handoverAlgorithmType == "ns3::A2A4RsrqHandoverAlgorithm")
    {
        m_lteHelper->SetHandoverAlgorithmType("ns3::A2A4RsrqHandoverAlgorithm");
        m_lteHelper->SetHandoverAlgorithmAttribute("ServingCellThreshold", UintegerValue(30));
        m_lteHelper->SetHandoverAlgorithmAttribute("NeighbourCellOffset", UintegerValue(1));
    }
    else if (m_handoverAlgorithmType == "ns3::A3RsrpHandoverAlgorithm")
    {
        m_lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
        m_lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(1.5));
        m_lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(128)));
    }
    else
    {
        NS_FATAL_ERROR("Unknown handover algorithm " << m_handoverAlgorithmType);
    }

    double distance = 1000.0; // m
    double speed = 150;       // m/s

    NodeContainer enbNodes;
    enbNodes.Create(m_nEnbs);
    NodeContainer ueNodes;
    ueNodes.Create(m_nUes);

    if (m_epc)
    {
        m_epcHelper = CreateObject<PointToPointEpcHelper>();
        m_lteHelper->SetEpcHelper(m_epcHelper);
    }

    // Install Mobility Model in eNBs
    // eNBs are located along a line in the X axis
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < m_nEnbs; i++)
    {
        Vector enbPosition(distance * (i + 1), 0, 0);
        enbPositionAlloc->Add(enbPosition);
    }
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPositionAlloc);
    enbMobility.Install(enbNodes);

    // Install Mobility Model in UE
    // UE moves with a constant speed along the X axis
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);
    for (uint16_t i = 0; i < m_nUes; i++)
    {
        ueNodes.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 0));
        ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
            Vector(speed, 0, 0));
    }

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
        bool epcDl = true;
        bool epcUl = false;
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

                if (m_useUdp)
                {
                    if (epcDl)
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
                    if (epcUl)
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
                }
                else // use TCP
                {
                    if (epcDl)
                    {
                        BulkSendHelper dlClientHelper(
                            "ns3::TcpSocketFactory",
                            InetSocketAddress(ueIpIfaces.GetAddress(u), dlPort));
                        dlClientHelper.SetAttribute("MaxBytes", UintegerValue(0));
                        clientApps.Add(dlClientHelper.Install(remoteHost));
                        PacketSinkHelper dlPacketSinkHelper(
                            "ns3::TcpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
                        ApplicationContainer sinkContainer = dlPacketSinkHelper.Install(ue);
                        bearerData.dlSink = sinkContainer.Get(0)->GetObject<PacketSink>();
                        serverApps.Add(sinkContainer);
                    }
                    if (epcUl)
                    {
                        BulkSendHelper ulClientHelper("ns3::TcpSocketFactory",
                                                      InetSocketAddress(remoteHostAddr, ulPort));
                        ulClientHelper.SetAttribute("MaxBytes", UintegerValue(0));
                        clientApps.Add(ulClientHelper.Install(ue));
                        PacketSinkHelper ulPacketSinkHelper(
                            "ns3::TcpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), ulPort));
                        ApplicationContainer sinkContainer = ulPacketSinkHelper.Install(remoteHost);
                        bearerData.ulSink = sinkContainer.Get(0)->GetObject<PacketSink>();
                        serverApps.Add(sinkContainer);
                    }
                } // end if (useUdp)

                Ptr<EpcTft> tft = Create<EpcTft>();
                if (epcDl)
                {
                    EpcTft::PacketFilter dlpf;
                    dlpf.localPortStart = dlPort;
                    dlpf.localPortEnd = dlPort;
                    tft->Add(dlpf);
                }
                if (epcUl)
                {
                    EpcTft::PacketFilter ulpf;
                    ulpf.remotePortStart = ulPort;
                    ulpf.remotePortEnd = ulPort;
                    tft->Add(ulpf);
                }

                if (epcDl || epcUl)
                {
                    EpsBearer bearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
                    m_lteHelper->ActivateDedicatedEpsBearer(ueDevices.Get(u), bearer, tft);
                }
                Time startTime = Seconds(startTimeSeconds->GetValue());
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
        NS_LOG_FUNCTION(maxRrcConnectionEstablishmentDuration);
        Simulator::Schedule(maxRrcConnectionEstablishmentDuration,
                            &LteX2HandoverMeasuresTestCase::CheckConnected,
                            this,
                            *it,
                            enbDevices.Get(0));
    }

    // schedule the checkpoint events

    Time stopTime = Seconds(0);
    for (std::list<CheckPointEvent>::iterator checkPointEventIt = m_checkPointEventList.begin();
         checkPointEventIt != m_checkPointEventList.end();
         ++checkPointEventIt)
    {
        for (Time checkPointTime = checkPointEventIt->checkStartTime;
             checkPointTime < checkPointEventIt->checkStopTime;
             checkPointTime += checkPointEventIt->checkInterval)
        {
            Simulator::Schedule(checkPointTime,
                                &LteX2HandoverMeasuresTestCase::CheckConnected,
                                this,
                                ueDevices.Get(checkPointEventIt->ueDeviceIndex),
                                enbDevices.Get(checkPointEventIt->enbDeviceIndex));

            Time saveStatsTime = checkPointTime;
            Simulator::Schedule(saveStatsTime,
                                &LteX2HandoverMeasuresTestCase::SaveStats,
                                this,
                                checkPointEventIt->ueDeviceIndex);

            Time checkStats = saveStatsTime + m_statsDuration;
            Simulator::Schedule(checkStats,
                                &LteX2HandoverMeasuresTestCase::CheckStats,
                                this,
                                checkPointEventIt->ueDeviceIndex);

            if (stopTime <= checkStats)
            {
                stopTime = checkStats + Seconds(1);
            }
        }
    }

    Simulator::Stop(stopTime);
    Simulator::Run();
    Simulator::Destroy();
}

void
LteX2HandoverMeasuresTestCase::CheckConnected(Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice)
{
    NS_LOG_FUNCTION(ueDevice << enbDevice);

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
LteX2HandoverMeasuresTestCase::SaveStats(uint32_t ueIndex)
{
    NS_LOG_FUNCTION(ueIndex);
    for (std::list<BearerData>::iterator it = m_ueDataVector.at(ueIndex).bearerDataList.begin();
         it != m_ueDataVector.at(ueIndex).bearerDataList.end();
         ++it)
    {
        if (it->dlSink)
        {
            it->dlOldTotalRx = it->dlSink->GetTotalRx();
        }
        if (it->ulSink)
        {
            it->ulOldTotalRx = it->ulSink->GetTotalRx();
        }
    }
}

void
LteX2HandoverMeasuresTestCase::CheckStats(uint32_t ueIndex)
{
    NS_LOG_FUNCTION(ueIndex);
    uint32_t b = 1;
    for (std::list<BearerData>::iterator it = m_ueDataVector.at(ueIndex).bearerDataList.begin();
         it != m_ueDataVector.at(ueIndex).bearerDataList.end();
         ++it)
    {
        uint32_t dlRx = 0;
        uint32_t ulRx = 0;

        if (it->dlSink)
        {
            dlRx = it->dlSink->GetTotalRx() - it->dlOldTotalRx;
        }

        if (it->ulSink)
        {
            ulRx = it->ulSink->GetTotalRx() - it->ulOldTotalRx;
        }
        double expectedBytes =
            m_udpClientPktSize * (m_statsDuration.GetSeconds() / m_udpClientInterval.GetSeconds());

        NS_LOG_LOGIC("expBytes " << expectedBytes << " dlRx " << dlRx << " ulRx " << ulRx);

        //                                tolerance
        if (it->dlSink)
        {
            NS_TEST_ASSERT_MSG_GT(dlRx,
                                  0.500 * expectedBytes,
                                  "too few RX bytes in DL, ue=" << ueIndex << ", b=" << b);
        }
        if (it->ulSink)
        {
            NS_TEST_ASSERT_MSG_GT(ulRx,
                                  0.500 * expectedBytes,
                                  "too few RX bytes in UL, ue=" << ueIndex << ", b=" << b);
        }
        ++b;
    }
}

/**
 * \ingroup lte-test
 * \ingroup tests
 *
 * \brief Lte X2 Handover Measures Test Suite
 */
class LteX2HandoverMeasuresTestSuite : public TestSuite
{
  public:
    LteX2HandoverMeasuresTestSuite();
};

LteX2HandoverMeasuresTestSuite::LteX2HandoverMeasuresTestSuite()
    : TestSuite("lte-x2-handover-measures", Type::SYSTEM)
{
    Time checkInterval = Seconds(1);

    std::string cel1name("ho: 0 -> 1");
    std::list<CheckPointEvent> cel1;
    cel1.push_back(CheckPointEvent(Seconds(1), Seconds(10.1), checkInterval, 0, 0));
    cel1.push_back(CheckPointEvent(Seconds(11), Seconds(17), checkInterval, 0, 1));

    std::string cel2name("ho: 0 -> 1 -> 2");
    std::list<CheckPointEvent> cel2;
    cel2.push_back(CheckPointEvent(Seconds(1), Seconds(10.1), checkInterval, 0, 0));
    cel2.push_back(CheckPointEvent(Seconds(11), Seconds(17.1), checkInterval, 0, 1));
    cel2.push_back(CheckPointEvent(Seconds(18), Seconds(24), checkInterval, 0, 2));

    std::string cel3name("ho: 0 -> 1 -> 2 -> 3");
    std::list<CheckPointEvent> cel3;
    cel3.push_back(CheckPointEvent(Seconds(1), Seconds(10.1), checkInterval, 0, 0));
    cel3.push_back(CheckPointEvent(Seconds(11), Seconds(17.1), checkInterval, 0, 1));
    cel3.push_back(CheckPointEvent(Seconds(18), Seconds(24.1), checkInterval, 0, 2));
    cel3.push_back(CheckPointEvent(Seconds(25), Seconds(37), checkInterval, 0, 3));

    int32_t useIdealRrc;
    std::string sched = "ns3::PfFfMacScheduler";
    std::string ho = "ns3::A2A4RsrqHandoverAlgorithm";
    for (useIdealRrc = 1; useIdealRrc >= 0; --useIdealRrc)
    {
        //                                          nEnbs, nUes, nDBearers, celist, name, useUdp,
        //                                          sched, ho, admitHo, idealRrc
        AddTestCase(new LteX2HandoverMeasuresTestCase(2,
                                                      1,
                                                      0,
                                                      cel1,
                                                      cel1name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    TestCase::Duration::TAKES_FOREVER);
        AddTestCase(new LteX2HandoverMeasuresTestCase(2,
                                                      1,
                                                      1,
                                                      cel1,
                                                      cel1name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    Duration::QUICK);
        AddTestCase(new LteX2HandoverMeasuresTestCase(2,
                                                      1,
                                                      2,
                                                      cel1,
                                                      cel1name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    TestCase::Duration::TAKES_FOREVER);
        AddTestCase(new LteX2HandoverMeasuresTestCase(3,
                                                      1,
                                                      0,
                                                      cel2,
                                                      cel2name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    TestCase::Duration::TAKES_FOREVER);
        AddTestCase(new LteX2HandoverMeasuresTestCase(3,
                                                      1,
                                                      1,
                                                      cel2,
                                                      cel2name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    TestCase::Duration::TAKES_FOREVER);
        AddTestCase(new LteX2HandoverMeasuresTestCase(3,
                                                      1,
                                                      2,
                                                      cel2,
                                                      cel2name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    Duration::EXTENSIVE);
        AddTestCase(new LteX2HandoverMeasuresTestCase(4,
                                                      1,
                                                      0,
                                                      cel3,
                                                      cel3name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    Duration::EXTENSIVE);
        AddTestCase(new LteX2HandoverMeasuresTestCase(4,
                                                      1,
                                                      1,
                                                      cel3,
                                                      cel3name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    TestCase::Duration::TAKES_FOREVER);
        AddTestCase(new LteX2HandoverMeasuresTestCase(4,
                                                      1,
                                                      2,
                                                      cel3,
                                                      cel3name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    TestCase::Duration::TAKES_FOREVER);
    }

    sched = "ns3::RrFfMacScheduler";
    for (useIdealRrc = 1; useIdealRrc >= 0; --useIdealRrc)
    {
        //                                          nEnbs, nUes, nDBearers, celist, name, useUdp,
        //                                          sched, admitHo, idealRrc
        AddTestCase(new LteX2HandoverMeasuresTestCase(2,
                                                      1,
                                                      0,
                                                      cel1,
                                                      cel1name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    Duration::EXTENSIVE);
        AddTestCase(new LteX2HandoverMeasuresTestCase(3,
                                                      1,
                                                      0,
                                                      cel2,
                                                      cel2name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    TestCase::Duration::TAKES_FOREVER);
        AddTestCase(new LteX2HandoverMeasuresTestCase(4,
                                                      1,
                                                      0,
                                                      cel3,
                                                      cel3name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    TestCase::Duration::TAKES_FOREVER);
    }

    ho = "ns3::A3RsrpHandoverAlgorithm";
    sched = "ns3::PfFfMacScheduler";
    for (useIdealRrc = 1; useIdealRrc >= 0; --useIdealRrc)
    {
        //                                          nEnbs, nUes, nDBearers, celist, name, useUdp,
        //                                          sched, admitHo, idealRrc
        AddTestCase(new LteX2HandoverMeasuresTestCase(2,
                                                      1,
                                                      0,
                                                      cel1,
                                                      cel1name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    Duration::EXTENSIVE);
        AddTestCase(new LteX2HandoverMeasuresTestCase(3,
                                                      1,
                                                      0,
                                                      cel2,
                                                      cel2name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    TestCase::Duration::TAKES_FOREVER);
        AddTestCase(new LteX2HandoverMeasuresTestCase(4,
                                                      1,
                                                      0,
                                                      cel3,
                                                      cel3name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    TestCase::Duration::TAKES_FOREVER);
    }

    sched = "ns3::RrFfMacScheduler";
    for (useIdealRrc = 1; useIdealRrc >= 0; --useIdealRrc)
    {
        //                                          nEnbs, nUes, nDBearers, celist, name, useUdp,
        //                                          sched, admitHo, idealRrc
        AddTestCase(new LteX2HandoverMeasuresTestCase(2,
                                                      1,
                                                      0,
                                                      cel1,
                                                      cel1name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    Duration::QUICK);
        AddTestCase(new LteX2HandoverMeasuresTestCase(3,
                                                      1,
                                                      0,
                                                      cel2,
                                                      cel2name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    TestCase::Duration::TAKES_FOREVER);
        AddTestCase(new LteX2HandoverMeasuresTestCase(4,
                                                      1,
                                                      0,
                                                      cel3,
                                                      cel3name,
                                                      true,
                                                      sched,
                                                      ho,
                                                      true,
                                                      useIdealRrc),
                    Duration::EXTENSIVE);
    }

} // end of LteX2HandoverMeasuresTestSuite ()

static LteX2HandoverMeasuresTestSuite g_lteX2HandoverMeasuresTestSuiteInstance;

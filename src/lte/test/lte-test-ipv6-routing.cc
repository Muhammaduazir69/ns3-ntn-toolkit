/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Jadavpur University, India
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
 * Author: Manoj Kumar Rana <manoj24.rana@gmail.com>
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
#include "ns3/config-store.h"
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
#include "ns3/epc-helper.h"
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
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv6-static-routing.h"
#include "ns3/lte-helper.h"
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

#include <algorithm>

/* *
   * Scenario:  3 UEs, 2 ENBs, 1 Remote Host, UE0<-->ENB0, UE1<-->ENB0, UE2<-->ENB1
                Servers: UE1, UE2, Remote Host
                Client: UE0 (3 clients)
                UDP Echo Packets transmitted between client and server

   * Pass criteria: 1) Every UDP Echo Request and Reply messages sent and received respectively
                       at UE0 must be matched by their UID, source address, destination address,
                       source port and destination port
                    2) Every request reply must follow proper route (e.g. In case of UE0->UE1,
                       packet must travel this route:
UE0->ENB0->PGW->ENB1->UE1->ENB1->PGW->ENB0->UE0) 3) The above check also ensures no redundancy of
the followed route for a packet
* */

using namespace ns3;

class LteIpv6RoutingTestCase : public TestCase
{
  public:
    LteIpv6RoutingTestCase();
    virtual ~LteIpv6RoutingTestCase();

    /**
     * \brief Initialize testing parameters.
     */
    void Checker();

    /**
     * \brief sent Packets from client's IPv6 interface.
     * \param: p packet
     * \param: ipv6 Ipv6 object
     * \param: interface Ipv6interface from which the packet is transmitted
     */
    void SentAtClient(Ptr<const Packet> p, Ptr<Ipv6> ipv6, uint32_t interface);

    /**
     * \brief Received Packets at client's IPv6 interface.
     * \param: p packet
     * \param: ipv6 Ipv6 object
     * \param: interface Ipv6interface at which the packet is received
     */
    void ReceivedAtClient(Ptr<const Packet> p, Ptr<Ipv6> ipv6, uint32_t interface);

    /**
     * \brief Received Packet at pgw from enb.
     * \param: p packet
     */
    void EnbToPgw(Ptr<Packet> p);

    /**
     * \brief Received Packet at pgw from enb.
     * \param: p packet
     */
    void TunToPgw(Ptr<Packet> p);

  private:
    virtual void DoRun(void);
    Ipv6InterfaceContainer m_ueIpIface;   //!< IPv6 interface container for ue
    Ipv6Address m_remoteHostAddr;         //!< remote host address
    std::list<uint64_t> m_pgwUidRxFrmEnb; //!< list of uids of packets received at pgw from enb
    std::list<uint64_t>
        m_pgwUidRxFrmTun; //!< list of uids of packets received at pgw from tunnel net device

    std::list<Ptr<Packet>> m_clientTxPkts; //!< list of sent packets from client
    std::list<Ptr<Packet>> m_clientRxPkts; //!< list of received packets at client
};

LteIpv6RoutingTestCase::LteIpv6RoutingTestCase()
    : TestCase("Test IPv6 Routing at LTE")
{
}

LteIpv6RoutingTestCase::~LteIpv6RoutingTestCase()
{
}

void
LteIpv6RoutingTestCase::SentAtClient(Ptr<const Packet> p, Ptr<Ipv6> ipv6, uint32_t interface)
{
    Ipv6Header ipv6Header;
    p->PeekHeader(ipv6Header);
    if (ipv6Header.GetNextHeader() == UdpL4Protocol::PROT_NUMBER)
    {
        m_clientTxPkts.push_back(p->Copy());
    }
}

void
LteIpv6RoutingTestCase::ReceivedAtClient(Ptr<const Packet> p, Ptr<Ipv6> ipv6, uint32_t interface)
{
    Ipv6Header ipv6Header;
    p->PeekHeader(ipv6Header);
    if (ipv6Header.GetNextHeader() == UdpL4Protocol::PROT_NUMBER)
    {
        m_clientRxPkts.push_back(p->Copy());
    }
}

void
LteIpv6RoutingTestCase::EnbToPgw(Ptr<Packet> p)
{
    Ipv6Header ipv6Header;
    p->PeekHeader(ipv6Header);
    if (ipv6Header.GetNextHeader() == UdpL4Protocol::PROT_NUMBER)
    {
        m_pgwUidRxFrmEnb.push_back(p->GetUid());
    }
}

void
LteIpv6RoutingTestCase::TunToPgw(Ptr<Packet> p)
{
    Ipv6Header ipv6Header;
    p->PeekHeader(ipv6Header);
    if (ipv6Header.GetNextHeader() == UdpL4Protocol::PROT_NUMBER)
    {
        m_pgwUidRxFrmTun.push_back(p->GetUid());
    }
}

void
LteIpv6RoutingTestCase::Checker()
{
    bool b = false;
    bool check = true;
    // Extract each received reply packet of the client
    for (std::list<Ptr<Packet>>::iterator it1 = m_clientRxPkts.begin(); it1 != m_clientRxPkts.end();
         it1++)
    {
        Ipv6Header ipv6header1;
        UdpHeader udpHeader1;
        Ptr<Packet> p1 = (*it1)->Copy();
        p1->RemoveHeader(ipv6header1);
        uint64_t uid = p1->GetUid();
        p1->RemoveHeader(udpHeader1);
        // Search each packet in list of sent request packet of the client
        for (std::list<Ptr<Packet>>::iterator it2 = m_clientTxPkts.begin();
             it2 != m_clientTxPkts.end();
             it2++)
        {
            Ptr<Packet> p2 = (*it2)->Copy();
            Ipv6Header ipv6header2;
            p2->RemoveHeader(ipv6header2);
            Ipv6Address sorceAddress = ipv6header2.GetSource();
            Ipv6Address destinationAddress = ipv6header2.GetDestination();
            UdpHeader udpHeader2;
            p2->RemoveHeader(udpHeader2);
            uint16_t sourcePort;
            uint16_t destinationPort;
            sourcePort = udpHeader2.GetSourcePort();
            destinationPort = udpHeader2.GetDestinationPort();
            // Check whether the uids, addresses and ports match
            if ((p2->GetUid() == p1->GetUid()) && sorceAddress == ipv6header1.GetDestination() &&
                destinationAddress == ipv6header1.GetSource() &&
                sourcePort == udpHeader1.GetDestinationPort() &&
                destinationPort == udpHeader1.GetSourcePort())
            {
                b = true;
                break;
            }
        }
        check &= b;
        if (std::find(m_pgwUidRxFrmEnb.begin(), m_pgwUidRxFrmEnb.end(), uid) !=
            m_pgwUidRxFrmEnb.end())
        {
            check &= true;
            m_pgwUidRxFrmEnb.remove(uid);
        }
        if (std::find(m_pgwUidRxFrmTun.begin(), m_pgwUidRxFrmTun.end(), uid) !=
            m_pgwUidRxFrmTun.end())
        {
            check &= true;
            m_pgwUidRxFrmTun.remove(uid);
        }
        b = false;
    }

    NS_TEST_ASSERT_MSG_EQ(check, true, "Failure Happens IPv6 routing of LENA");
    NS_TEST_ASSERT_MSG_EQ(m_clientTxPkts.size(),
                          m_clientRxPkts.size(),
                          "No. of Request and Reply messages mismatch");
    NS_TEST_ASSERT_MSG_EQ(m_pgwUidRxFrmEnb.size(), 0, "Route is not Redundant in Lte IPv6 test");
    NS_TEST_ASSERT_MSG_EQ(m_pgwUidRxFrmTun.size(), 0, "Route is not Redundant in Lte IPv6 test");
}

void
LteIpv6RoutingTestCase::DoRun(void)
{
    double distance = 60.0;

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    epcHelper->SetAttribute("S1apLinkDelay", TimeValue(Seconds(0)));

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(2);
    ueNodes.Create(3);

    // Install Mobility Model
    Ptr<ListPositionAllocator> positionAlloc1 = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> positionAlloc2 = CreateObject<ListPositionAllocator>();

    positionAlloc1->Add(Vector(distance * 0, 0, 0));
    positionAlloc1->Add(Vector(distance * 0 + 5, 0, 0));
    positionAlloc1->Add(Vector(distance * 1, 0, 0));

    positionAlloc2->Add(Vector(distance * 0, 0, 0));
    positionAlloc2->Add(Vector(distance * 1, 0, 0));

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc1);
    mobility.Install(ueNodes);

    mobility.SetPositionAllocator(positionAlloc2);
    mobility.Install(enbNodes);

    // Install the IP stack on the UEs
    internet.Install(ueNodes);

    // Install LTE Devices to the nodes
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    // Assign IP address to UEs, and install applications
    m_ueIpIface = epcHelper->AssignUeIpv6Address(NetDeviceContainer(ueLteDevs));

    Ipv6StaticRoutingHelper ipv6RoutingHelper;

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv6StaticRouting> ueStaticRouting =
            ipv6RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv6>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress6(), 1);
    }

    // Attach two UEs at first eNodeB and one UE at second eNodeB
    lteHelper->Attach(ueLteDevs.Get(0), enbLteDevs.Get(0));
    lteHelper->Attach(ueLteDevs.Get(1), enbLteDevs.Get(0));
    lteHelper->Attach(ueLteDevs.Get(2), enbLteDevs.Get(1));

    Ipv6AddressHelper ipv6h;
    ipv6h.SetBase(Ipv6Address("6001:db80::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer internetIpIfaces = ipv6h.Assign(internetDevices);

    internetIpIfaces.SetForwarding(0, true);
    internetIpIfaces.SetDefaultRouteInAllNodes(0);

    Ptr<Ipv6StaticRouting> remoteHostStaticRouting =
        ipv6RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv6>());
    remoteHostStaticRouting
        ->AddNetworkRouteTo("7777:f00d::", Ipv6Prefix(64), internetIpIfaces.GetAddress(0, 1), 1, 0);

    // interface 0 is localhost, 1 is the p2p device
    m_remoteHostAddr = internetIpIfaces.GetAddress(1, 1);

    // Install and start applications on UEs and remote host
    UdpEchoServerHelper echoServer1(10);
    UdpEchoServerHelper echoServer2(11);
    UdpEchoServerHelper echoServer3(12);

    ApplicationContainer serverApps = echoServer1.Install(remoteHost);
    serverApps.Add(echoServer2.Install(ueNodes.Get(1)));
    serverApps.Add(echoServer3.Install(ueNodes.Get(2)));

    serverApps.Start(Seconds(4.0));
    serverApps.Stop(Seconds(12.0));

    UdpEchoClientHelper echoClient1(m_remoteHostAddr, 10);
    UdpEchoClientHelper echoClient2(m_ueIpIface.GetAddress(1, 1), 11);
    UdpEchoClientHelper echoClient3(m_ueIpIface.GetAddress(2, 1), 12);

    echoClient1.SetAttribute("MaxPackets", UintegerValue(1000));
    echoClient1.SetAttribute("Interval", TimeValue(Seconds(0.2)));
    echoClient1.SetAttribute("PacketSize", UintegerValue(1024));

    echoClient2.SetAttribute("MaxPackets", UintegerValue(1000));
    echoClient2.SetAttribute("Interval", TimeValue(Seconds(0.2)));
    echoClient2.SetAttribute("PacketSize", UintegerValue(1024));

    echoClient3.SetAttribute("MaxPackets", UintegerValue(1000));
    echoClient3.SetAttribute("Interval", TimeValue(Seconds(0.2)));
    echoClient3.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps1 = echoClient1.Install(ueNodes.Get(0));
    ApplicationContainer clientApps2 = echoClient2.Install(ueNodes.Get(0));
    ApplicationContainer clientApps3 = echoClient3.Install(ueNodes.Get(0));

    clientApps1.Start(Seconds(4.0));
    clientApps1.Stop(Seconds(6.0));

    clientApps2.Start(Seconds(6.1));
    clientApps2.Stop(Seconds(8.0));

    clientApps3.Start(Seconds(8.1));
    clientApps3.Stop(Seconds(10.0));

    // Set Cllback for Client Sent and Received packets
    Ptr<Ipv6L3Protocol> ipL3 = (ueNodes.Get(0))->GetObject<Ipv6L3Protocol>();
    ipL3->TraceConnectWithoutContext("Tx",
                                     MakeCallback(&LteIpv6RoutingTestCase::SentAtClient, this));
    ipL3->TraceConnectWithoutContext("Rx",
                                     MakeCallback(&LteIpv6RoutingTestCase::ReceivedAtClient, this));

    // Set Callback at SgwPgWApplication of epc to get the packets from enb and from tunnel net
    // device
    Ptr<Application> appPgw = pgw->GetApplication(0);
    appPgw->TraceConnectWithoutContext("RxFromS1u",
                                       MakeCallback(&LteIpv6RoutingTestCase::EnbToPgw, this));
    appPgw->TraceConnectWithoutContext("RxFromTun",
                                       MakeCallback(&LteIpv6RoutingTestCase::TunToPgw, this));

    Simulator::Schedule(Time(Seconds(12.0)), &LteIpv6RoutingTestCase::Checker, this);

    Simulator::Stop(Seconds(14));
    Simulator::Run();

    Simulator::Destroy();
}

/**
 * \brief test suite 1
 */
class LteIpv6RoutingTestSuite : public TestSuite
{
  public:
    LteIpv6RoutingTestSuite();
};

LteIpv6RoutingTestSuite::LteIpv6RoutingTestSuite()
    : TestSuite("lte-ipv6-routing-test", Type::UNIT)
{
    AddTestCase(new LteIpv6RoutingTestCase, Duration::QUICK);
}

static LteIpv6RoutingTestSuite lteipv6testsuite;

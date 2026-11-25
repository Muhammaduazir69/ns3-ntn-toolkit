.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

Scope and Limitations
*********************
This module implements a partial set of the features defined in the 3GPP NR standard. This section
collects the main scope decisions and the functional limitations that are noted throughout the rest
of the documentation, so that they can be seen at a glance.

At the physical and MAC layers, key Release-15 aspects that are still missing are spatial
multiplexing (beyond the modelled MIMO gains), configured-grant scheduling and puncturing, and an
error model for the control channels.

Above the radio interface, the higher layers are largely a port of the |ns3| 'LTE' higher layers
adapted to NR, and as such they model a deliberately reduced feature set. The main functional
limitations, organised by layer, are:

* **RLC.** The Acknowledged Mode entity does not model a successful-delivery indication to the upper
  layers, SDU discard requested by PDCP, or RLC re-establishment, and it does not re-segment the
  retransmission buffer (exactly one PDU is built per transmission opportunity). See the
  :doc:`RLC layer <rlc-layer>` section.
* **PDCP.** There is no ROHC header compression, no ciphering/deciphering, no integrity protection,
  no in-sequence delivery or duplicate elimination at re-establishment, and no duplicate discarding;
  the entity has no hyperframe number and no PDCP control PDUs, and uses only the 12-bit sequence
  number (the 15-/18-bit NR formats are not present). See the :doc:`PDCP layer <pdcp-layer>` section.
* **SDAP.** SDAP is not implemented as a layer: there is no SDAP header, no QoS-flow-to-DRB
  multiplexing (the mapping is a fixed one-to-one relation), no reflective QoS and no end-marker
  handling. See the :doc:`SDAP layer <sdap-layer>` section.
* **NAS.** Only the NAS *Active* state is exercised; there is no PLMN/CSG selection, no idle-mode
  location-update or paging procedure, and no authentication or security. See the
  :doc:`NAS layer <nas-layer>` section.
* **Core network and interfaces.** The core is a 4G EPC, not a 5G standalone (SA) core -- there is
  no NGAP, UPF, N4/PFCP, Xn or Service-Based Interface, and almost the entire EPC would need to be
  replaced to model a 5G SA core. EPS-bearer QoS is not enforced on the S1-U, S5 and X2-U GTP-U links
  (link over-provisioning is assumed). X2 supports only *seamless* (not lossless) handover with no
  dedicated handover-failure recovery, and X2-AP is carried over UDP because SCTP is not modelled.
  See the :doc:`EPC <epc>` and :doc:`Interfaces <interfaces>` sections.

The two companion topics below are useful when configuring and interpreting simulations within this
scope: the simulation-wide settings that have been lifted out of the individual layers, and how to
identify the component (BWP/component-carrier) a trace or message belongs to.

.. toctree::
   :maxdepth: 2

   Simulation-wide settings <simulation-wide-settings>
   Identifying components <identifying-components>

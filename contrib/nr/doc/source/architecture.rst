.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

Architecture
************
The 'NR' module has been designed to perform end-to-end simulations of 3GPP-oriented cellular networks. The end-to-end overview of a typical
simulation with the 'NR' module is drawn in Figure :ref:`fig-e2e`. In dark gray, we represent the existing, and unmodified, ns-3 and LENA components.
In light gray, we describe the NR components. On one side, we have a remote host (depicted as a single node in the Figure, for simplicity,
but there can be multiple nodes) that connects to an PGW/SGW (Packet Gateway and Service Gateway), through a link. Such a connection can be
defined with any technology that is currently available in ns-3. The diagram illustrates a single link, but there are no limits on the topology,
including any number of remote hosts. Inside the PGW and SGW, the ``NrEpcPgwApplication`` and ``NrEpcSgwApplication`` encapsulate the packet using the GTP protocol. Through an IP
connection, which represents the backhaul of the NR network (again, described with a single link in the Figure, but the topology can vary),
the GTP packet is received by the gNB. There, after decapsulating the payload, the packet is transmitted inside the NR stack through the entry
point represented by the class ``NrGnbNetDevice``. The packet, if received correctly at the UE, is passed to higher layers by the class
``NrUeNetDevice``. The path crossed by packets in the UL case is the same as the one described above but in the opposite direction.

.. _fig-e2e:

.. figure:: figures/drawing2.*
   :align: center
   :scale: 80 %

   End-to-end class overview

Concerning the RAN, we detail what is happening between ``NrGnbNetDevice`` and ``NrUeNetDevice`` in Figure :ref:`fig-ran`. The ``NrGnbMac``
and ``NrUeMac`` MAC classes implement the module Service Access Point (SAP) provider and user interfaces, enabling the communication with
the RLC layer. The module supports RLC TM, SM, UM, and AM modes. The MAC layer contains the scheduler (``NrMacScheduler`` and derived classes).
Every scheduler also implements an SAP for RRC layer configuration (``NrGnbRrc``). The ``NrPhy`` classes are used to perform the directional
communication for both downlink (DL) and uplink (UL), to transmit/receive the data and control channels. Each ``NrPhy`` class writes into an
instance of the ``NrSpectrumPhy`` class, which is shared between the UL and DL parts.

.. _fig-ran:

.. figure:: figures/drawing1.png
   :align: center
   :scale: 80 %

   RAN class overview

Two interesting blocks in Figure :ref:`fig-ran` are the ``BwpManagerGnb`` and ``BwpManagerUe`` layers. 3GPP does not explicitly define them, and as such,
they are virtual layers. Still, they help construct a fundamental feature of our simulator: the multiplexing of different BWPs. NR has included
the definition of 3GPP BWPs for energy-saving purposes, as well as to multiplex a variety of services with different QoS requirements.
The component carrier concept was already introduced in LTE, and persists in NR through our general BWP concept, as a way to aggregate carriers
and thereby improve the system capacity. In the 'NR' simulator, it is possible to divide the entire bandwidth into different BWPs.
Each BWP can have its own PHY and MAC configuration (e.g., specific numerology, scheduler rationale, and so on). We added the possibility
for any node to transmit and receive flows in different BWPs by assigning each bearer to a specific BWP. Distributing the data flow
among different BWPs could be done by implementing another variant of the BWP manager. The introduction of a proxy layer to multiplex and demultiplex
the data was necessary to glue everything together, and this is the purpose of these two new classes (``BwpManagerGnb`` and ``BwpManagerUe``).

Note: The 3GPP definition for "Bandwidth Part" (BWP) is made for energy-saving purposes at the UE nodes. As per the 3GPP standard,
the active 3GPP BWP at a UE can vary semi-statically, and multiple 3GPP BWPs can span over the same frequency spectrum region.
In this text, and through the code, we use the word BWP to refer to various things that are not always in line with the 3GPP definition.

First of all, we use it to indicate the minimum piece of spectrum that can be modeled. In this regard, a BWP has a center frequency and a bandwidth,
plus some characteristics for the 3GPP channel model (e.g., the scenario). Each device can handle multiple BWPs, but such BWPs must be orthogonal in
frequency (i.e., they must span over different frequency spectrum regions, that can be contiguous or not, depending on the user-defined configuration).

Secondly, the 'NR' module is communicating through each BWP with a PHY and a MAC entity, as well as with one spectrum channel and one antenna instance.
In other words, for every spectrum bandwidth part, the module will create a PHY, a MAC, a Spectrum channel, and an antenna. We consider, in the code,
that this set of components form a BWP. Moreover, we have a router between the RLC queues and the different MAC entities, which is called the BWP manager.

Summarizing, our BWP terminology can refer to orthogonal 3GPP BWPs, as well as to orthogonal 3GPP Component Carriers, and it is up to the BWP manager
to route the flows accordingly based on the behavior the user wants to implement. Our primary use case for BWPs is to avoid interference, as well as to
send different flow types through different BWPs, to achieve a dedicated-resource RAN slicing.

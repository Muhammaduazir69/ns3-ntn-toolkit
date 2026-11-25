.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

Examples
********

Several example programs are provided to highlight the operation.


cttc-3gpp-channel-simple-ran.cc
===============================
The program ``nr/examples/cttc-3gpp-channel-simple-ran.cc``
allows users to select the numerology and test the performance considering
only the RAN. The scenario topology is simple, and it
consists of a single gNB and single UE. The scenario is illustrated in
::`fig-scenario-simple`.

.. _fig-scenario-simple:

.. figure:: figures/scenario-simple.*
   :align: center
   :scale: 50 %

   NR scenario for simple performance evaluation (RAN part only)

The output of the example is printed on the screen and it shows the PDCP and RLC delays.
The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-3gpp-channel-simple-ran_8cc.html


cttc-3gpp-channel-nums.cc
=========================
The program ``examples/cttc-3gpp-channel-nums.cc``
allows users to select the numerology and test the end-to-end performance.
:numref:`fig-end-to-end` shows the simulation setup.
The user can run this example with UDP full buffer traffic and can specify the
UDP packet interval.

.. _fig-end-to-end:

.. figure:: figures/end-to-end.*
   :align: center

   NR end-to-end system performance evaluation

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-3gpp-channel-nums_8cc.html


cttc-3gpp-channel-simple-fdm.cc
===============================

The program ``examples/cttc-3gpp-channel-simple-fdm.cc`` can be used to
simulate FDM  of numerologies in scenario with a single UE and gNB.
In this program the packet is directly injected to the gNB, so this program
can be used only for simulation of the RAN part.
This program allows the user to configure 2 BWPs.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-3gpp-channel-simple-fdm_8cc.html

cttc-3gpp-channel-nums-fdm.cc
=============================
The program ``examples/cttc-3gpp-channel-nums-fdm.cc`` allows the user to configure
2 UEs and 1 or 2 BWPs and test the end-to-end performance.
This example is designed to expect the full configuration of each BWP.
The configuration of BWP is composed of the following parameters:
central carrier frequency, bandwidth and numerology. There are 2 UEs, and each UE has one flow.
One flow is of URLLC traffic type, while the another is eMBB.
URLLC is configured to be transmitted over the first BWP, and the eMBB over the second BWP.
:numref:`fig-end-to-end` shows the simulation setup.
Note that this simulation topology is as the one used in ``scratch/cttc-3gpp-channel-nums.cc``
The user can run this example with UDP full buffer traffic or can specify the
UDP packet interval and UDP packet size per type of traffic.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-3gpp-channel-nums-fdm_8cc.html


cttc-3gpp-indoor-calibration.cc
===============================
The program ``examples/cttc-3gpp-indoor-calibration`` is the simulation
script created for the NR-MIMO Phase 1 system-level calibration.
The scenario implemented in this simulation script is according to
the topology described in 3GPP TR 38.901 V17.0.0 (2022-03) 7.2-1:
"Layout of indoor office scenarios".
The simulation assumptions and the configuration parameters follow
the evaluation assumptions agreed at 3GPP TSG RAN WG1 meeting #88,
and which are summarized in R1-1703534 Table 1.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-3gpp-indoor-calibration_8cc.html


cttc-error-model.cc
===================
The program ``examples/cttc-error-model`` allows the user to test the end-to-end
performance with the new NR PHY abstraction model for error modeling by using a fixed MCS.
It allows the user to set the MCS, the MCS table, the error model type, the gNB-UE distance, and the HARQ method.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-error-model_8cc.html

cttc-error-model-comparison.cc
==============================
The program ``examples/cttc-error-model-comparison`` allows the user to compare the Transport
Block Size that is obtained for each MCS index under different error models (NR and LTE)
and different MCS Tables. It allows the user to configure the MCS Table and the
error model type.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-error-model-comparison_8cc.html


cttc-error-model-amc.cc
=======================
The program ``examples/cttc-error-model-amc`` allows the user to test the end-to-end
performance with the new NR PHY abstraction model for error modeling by using
adaptive modulation and coding (AMC).
It allows the user to set the AMC approach (error model-based or Shannon-based),
the MCS table, the error model type, the gNB-UE distance, and the HARQ method.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-error-model-amc_8cc.html


cttc-3gpp-channel-example.cc
============================
The program ``examples/cttc-3gpp-channel-example`` allows the user to setup a
simulation using the implementation of the 3GPP channel model described in TR 38.900.
The network topology consists, by default, of 2 UEs and 2 gNbs. The user can
select any of the typical scenarios in TR 38.900 such as Urban Macro (UMa),
Urban Micro Street-Canyon (UMi-Street-Canyon), Rural Macro (RMa) or Indoor
Hotspot (InH) in two variants: 'InH-OfficeMixed' and 'InH-OfficeOpen'. The
example also supports either mobile or static UEs.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-3gpp-channel-example_8cc.html

cttc-lte-ca-demo.cc
===================
The program ``examples/cttc-lte-ca-demo`` allows the user to setup a simulation
to test the configuration of inter-band Carrier Aggregation in an LTE deployment.
One Component Carrier (CC) is created in LTE Band 40 and two CCs are created in
LTE Band 38. The second CC in Band 38 can be configured to operate in TDD or FDD
mode; the other two CCs are fixed to TDD.  The user can provide the TDD pattern
to use in every TDD CC as input.

In this example, the deployment consists of one gNB and one UE. The UE can be
configure to transmit different traffic flows simultaneously. Each flow is mapped
to a unique CC, so the total UE traffic can be aggregated.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-lte-ca-demo_8cc.html

cttc-nr-cc-bwp-demo.cc
======================
The program ``examples/cttc-nr-cc-bwp-demo`` allows the user to setup a simulation
to test the configuration of intra-band Carrier Aggregation (CA) in an NR deployment.
The example shows how to configure the operation spectrum by defining all the
operation bands, CCs and BWPs that will
be used in the simulation. The user can select whether the creation of the spectrum
structures is automated with the CcBwpCreator helper; or if the user wants to
manually provide a more complex spectrum configuration.

In this example, the NR deployment consists of one gNB and one UE. The operation
mode is set to TDD. The user can provide a TDD pattern as input to the simulation;
otherwise the simulation will assume by default that downlink and uplink
transmissions can occur in the same slot.

The UE can be configured to transmit three different traffic flows simultaneously.
Each flow is mapped to a unique CC, so the total UE traffic can be aggregated.
The generated traffic can be only DL, only UL, or both at the
same time. UE data transmissions will occur in the right DL or UL slot according
to the configured TDD pattern.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-nr-cc-bwp-demo_8cc.html

cttc-nr-demo.cc
===============
The program ``examples/cttc-nr-demo`` is recommended as a tutorial to the use of
the ns-3 NR module. In this example, the user can understand the basics to
successfully configure a full NR simulation with end-to-end data transmission.

Firstly, the example creates the network deployment using the GridScenario helper.
By default, the deployment consists of a single gNB and two UEs, but the user
can provide a different number of gNBs and UEs per gNB.

The operation mode is set to TDD. The user can provide a TDD pattern as input to
the simulation; otherwise the simulation will assume by default that downlink and
uplink transmissions can occur in the same slot.

The example performs inter-band Carrier Aggregation of two CC, and each CC has one BWP occupying the whole CC bandwidth.

It is possible to set different configurations for each CC such as the numerology,
the transmission power or the TDD pattern. In addition, each gNB can also have a
different configuration of its CCs.

The UE can be configure to transmit two traffic flows simultaneously. Each flow
is mapped to a single CC, so the total UE traffic can be aggregated.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-nr-demo_8cc.html

cttc-nr-demo-sionna-rt.cc
=========================
The program ``examples/cttc-nr-demo-sionna-rt`` extends the basic
``cttc-nr-demo`` scenario with the Sionna RT channel model. It is intended as
an entry point for users who want to run an end-to-end NR simulation while
obtaining the radio channel from Sionna RT ray tracing instead of the default
3GPP stochastic channel model.

The example uses the ``GridScenarioHelper`` to create a configurable grid
deployment with one or more gNBs and UEs. By default, it creates one gNB and
two UEs, configures one operational band at 28 GHz, and installs two UDP
traffic flows mapped to NR bearers. The ``doubleOperationalBand`` command-line
option enables a second operational band, following the same bandwidth-part
mapping style as ``cttc-nr-demo``.

The channel is selected through ``NrChannelHelper`` by setting
``ChannelModel`` to ``SionnaRT`` and configuring
``SionnaRtSpectrumPropagationLossModel`` as the phased-array spectrum
propagation loss model. The example exposes Sionna RT scene and path-solver
parameters on the command line, including ``Scenario``, ``maxDepth``, ``los``,
``specularReflection``, ``diffuseReflection``, ``diffraction``,
``edgeDiffraction``, ``refraction``, and ``syntheticArray``.  Scene rendering
output can be configured with ``outputFileName``, ``outputFileDirectory``,
``cameraPosition``, and ``cameraLookAt``.

The example is built only when Sionna RT dependencies are available and the
platform is not Windows. See the Sionna-RT installation chapter in the ns-3
manual for installation instructions.  It can be run with:

.. sourcecode:: bash

   ./ns3 run cttc-nr-demo-sionna-rt

lena-lte-comparison (initially s3-scenario.cc)
==============================================

``lena-lte-comparison`` directory contains the program that can be run through
``lena-lte-comparison-user.cc`` or ``lena-lte-comparison-campaign.cc``.
Users can use any of these two scripts. The two scripts run exactly the same program.

The original idea was to use ``lena-lte-comparison-user.cc`` as any other
NR example, i.e., by running it from the command line, and to use
``lena-lte-comparison-campaign.cc`` for simulation
campaigns, i.e., to run it from the simulation campaign tool (e.g., SEM tool).
Because of this, ``lena-lte-comparison-campaign.cc`` script should have a minimal set of
parameters that are relevant for the simulation campaign,
and its input parameter list should not be changed often to not
loose compatibility with the simulation campaign script (e.g., SEM script).
Hence, when it is needed to add some new input parameters, these can be added to
``lena-lte-comparison-user.cc``.

``lena-lte-comparison`` provides a complex multi-cell hexagonal network
deployment with site sectorization. The program provides two options for
the topology configuration: hexagonal grid topology and user-defined topology.
Hexagonal grid deployment is the default deployment, which follows the typical
hexagonal grid topology where the structure of the network is
organized into rings, and each ring is composed of sectorized sites.
Each site has 3 sectors. The antenna of each sector has its antenna oriented toward its sector area.
Sector areas are equally sized, meaning that each sector covers 120º in azimuth.

.. _fig-hex-grid:

.. figure:: figures/hex-grid.*
   :align: center
   :scale: 80 %

   Hexagonal grid deployment with different rings (when 0 rings
   configured, only one site is created and this site is shown in blue color,
   the 1st ring is shown in red color, the third ring in orange and
   the fourth in green. Each ring is composed of 6 sites.)

User-defined topology can be provided through the ``.csv`` file which should
contain the tower coordinates (instead of hexagonal grid).
``lena-lte-comparison`` folder already contains 4 examples of ``.csv`` files for different number
of sites, e.g., see ``examples-sites.2.csv``.

The deployment supports two frequency configurations. It can be set to a full frequency reuse of 1
(in example referred to as overlapping frequency scenario),
or a frequency reuse of 3 (in example referred to as non overlapping frequency scenario),
which is typical in cellular networks.
In non-overlapping scenario each sector of a site transmits in a separate frequency band.
These separate frequency bands are typically called sub-bands.
In this deployment, the sectors of the same site do not interfere.
Sub-bands are centered in different frequencies having equal bandwidths.
Sub-band utilization is repeated for all sites.

Scenario supports various propagation scenarios:
Urban Macro (UMa), Urban Micro (UMi) and Rural Macro (RMa).
The choice of the scenario determines the values of scenario-specific parameters,
such as the height of the gNB, the transmit power, and the propagation model,
the Inter-Site Distance (ISD), etc. These scenario-specific parameters and
their values for a specific scenario configurations
are listed in Table :ref:`tab-lena-lte-comparison-scenarios`.

.. _tab-lena-lte-comparison-scenarios:

.. table:: lena-lte-comparison scenarios and configurations

   +--------------+------------+---------------+---------------+--------------------+-----------------+
   |  scenario    |  ISD (km)  | BS height (m) | UE height (m) | UE-BS min distance |  Tx power (dBm) |
   +--------------+------------+---------------+---------------+--------------------+-----------------+
   |   UMa        |    1.7     |       30      |      1.5      |        30.2        |       43        |
   +--------------+------------+---------------+---------------+--------------------+-----------------+
   |   UMi        |    0.5     |       10      |      1.5      |         10         |       30        |
   +--------------+------------+---------------+---------------+--------------------+-----------------+
   |   RMa        |     7      |       45      |      1.5      |        44.6        |       43        |
   +--------------+------------+---------------+---------------+--------------------+-----------------+


``lena-lte-comparison`` can be used to perform LENA vs 5G-LENA comparison,
validation and calibration campaigns.
E.g., LTE can be simulated either using the LENA module or the 5G-LENA module.
To simulate LTE using LENA module it is needed to set parameter ``technology`` to
LTE and ``simulator`` to LENA.
To simulate LTE scenario using 5G-LENA, it is needed to set parameter ``technology`` to LTE and
``simulator`` to 5GLENA.
When configured to simulated LTE using 5G-LENA,
then NR devices and protocol stack will be configured to use LTE settings,
e.g., the numerology will be set to match the LTE slot duration and subcarrier spacing,
MAC-PHY processing delays will be set to typical LTE values, etc.
To simulate NR it is needed to set ``technology`` to
NR and ``simulator`` to 5GLENA.
With this configuration, default NR settings will be used to create the scenario.

Several traffic types are supported by the script: saturation, single packet, low-load and medium-load.
Saturation traffic mode generates traffic of 80 Mbps for bandwidth of 20 MHz,
and it scales depending on the selected bandwidth. Single packet traffic sends only a single packet of
12 bytes, and is normally used just to measure the latencies.
Low-load traffic mode generates traffic of 1 Mbps for bandwidth of 20 MHz,
and medium-load generates traffic of 20 Mbps for bandwidth of 20 MHz.
These traffic types also scale with bandwidth.

In addition to the parameters discussed above, this example also allows configuring parameters, such as:
the number of UEs per sector, the duration of the applications,
the bandwidth (typical values used in this scenario are 20, 10, or 5 MHz per carrier),
the numerology, the traffic direction (DL or UL), operation mode (TDD or FDD),
the TDD pattern,  the error model type, calibration, traffic scenario, scheduler
(proportional fair, round robin, etc,), enable/disable uplink power control,
configure power allocation mode for NR LTE ( uniform per RB used or uniform per bandwidth),
base station antenna downtilt angle (deg).
Parameter ``calibration`` can be used to configure the simulation
in such a way that it is possible to compare LENA and 5GLENA simulators.

The simulation saves the results in the database, which will be generated in root ns-3
directory by default (if not configured differently). Database contains several tables:
``e2e``, ``gnbRxPower``, ``rbStats``, ``sinr``, ``slotStats`` and ``ueTxPower``.
e2e table contains end-to-end metrics, such as the number of transmitted and received
packets, offered and achieved throughput, delay and jitter. ``gnbRxPower`` contains,
among others, traces related to the power corresponding to each reception.
``rbStats`` table contains traces related to RB usage. ``sinr`` contains SINR traces.
``slotStats`` contains traces per slot, e.g., the number of scheduled UEs, symbols used,
RBs used, etc. ``ueTxPower`` contains the traces related to UE transmissions,
i.e., the power and the RB used.

This example script also generates a gnuplot script that can be used to plot the
topology.
The gnuplot script is generated by default in the root ns-3 folder,
if not configured differently, and can be used to plot the topology
e.g., by running in the command line ``gnuplot hexagonal-topology.gnuplot``.

The complete details of the simulation script are provided in:
https://cttc-lena.gitlab.io/nr/html/lena-lte-comparison-campaign_8cc_source.html,
https://cttc-lena.gitlab.io/nr/html/lena-lte-comparison-user_8cc.html,
https://cttc-lena.gitlab.io/nr/html/lena-lte-comparison_8cc_source.html,
https://cttc-lena.gitlab.io/nr/html/lena-v1-utils_8cc_source.html,
https://cttc-lena.gitlab.io/nr/html/lena-v2-utils_8cc_source.html.

.. _notchingExample:

cttc-nr-notching.cc
===================
The program ``examples/cttc-nr-notching`` allows the user to setup a simulation
to test the notching functionality described in :ref:`Notching`. The example
allows the configuration of:

* Variable number of gNBs
* Definition of a (continuous) set of notched RBGs (through: notchedRbStart and numOfNotchedRbs)
* Operation Mode: TDD/FDD + pattern for TDD case
* Frequency, Bandwidth (5, 10 and 20 MHz), Numerology and transmit power per gNB
* Scheduler Type: TDMA RR, PF, MR / OFDMA RR, PF, MR per gNB
* Possibility to have only DL flows, only UL or both
* Logging and traces

Moreover, the user can study the variations in the throughput (e.g. when increasing
the number of notched RBGs, the throughput gets decreased), as well in the SINR.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-nr-notching_8cc.html


.. _realisticBeamforming:

cttc-realistic-beamforming.cc
=============================
The example ``cttc-realistic-beamforming.cc`` included in the ``nr``
module demonstrates the usage of the proposed framework.
It is a simulation script for the realistic BF evaluation.
The topology is very simple: it consists of a single gNB and single UE,
placed at a certain distance from each other and communicating over a wireless channel.
Simulation allows to configure various parameters out of which the most important are:
the distance (by configuring deltaX and deltaY parameters, which
basically determine the position of the UE), the type of the BF method (ideal or real),
the random run number (which will allow us to run many simulations and to average the results),
the UE power, 3GPP scenario (Urban Macro, Urban Micro, Indoor Hotspot, etc).
The output is saved in database (simulation configuration and average SINR).
The database is created in the root project directory if not configured differently.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-realistic-beamforming_8cc.html.

cttc-nr-3gpp-calibration
========================

``3gpp-outdoor-calibration`` directory contains the program that can be run through
``cttc-nr-3gpp-calibration-user.cc``.

This example has been created to calibrate the 5G-LENA simulator under 3GPP
reference scenarios defined in ITU IMT-2020 report [IMT-2020]_ for NR-based outdoor
deployments. The example includes 4 test environment scenarios as defined in the
report. For each scenario, we have added a set of pre-defined parameters, meaning
parameters that will not be changed during the simulations, such as the frequency
or the bandwidth, while we have included some other parameters to be defined through
the command line, in case we want to study variations in the KPIs based on different
configurations. Examples include the gNB and UE polarization and slant angles,
the beamforming method used, and the activation/deactivation of fading and shadowing.

The 4 pre-defined scenarios used for the calibration of the 5G-LENA are the Rural-eMBB
Configuration A and Configuration B, and the Dense-eMBB Configuration A and Configuration B.
These different evaluation configurations basically vary some parameters (like the
carrier frequency, the total transmit power, the simulation bandwidth, the number
of antenna elements per gNB/UE, and the gNB/UE noise figure) for a given test environment.
Moreover, depending on the scenario under evaluation, UEs might be indoor/outdoor,
while the antenna configuration and height, among other parameters, of both gNBs
and UEs, are also varied. Finally, let us notice that all scenarios have been
evaluated under full buffer traffic, as indicated by 3GPP reference results.

The network layout consists in a hexagonal topology with 37 sites of 3 sectors each,
thus leading to 111 Base Stations (BS), as shown in :numref:`fig-calibration-hex-grid`.
However, in the measurements we consider only the 21 inner BSs, while the 111 BSs
are simulated to account for the wrap-around effect. Notice that each sector has
its antenna arrays oriented towards its sector area, and each sector area is equally
sized, meaning that each sector covers 120 degrees in azimuth. Moreover, notice that the
antennas of each sector are pointing to 30, 150 and 270 degrees w.r.t. the horizontal axis.

.. _fig-calibration-hex-grid:

.. figure:: figures/calibrationHex.*
   :align: center
   :scale: 80 %

   Wrap-around hexagonal deployment.

The calculation of the Pathloss follows the TR 38.901 [TR38901]_ model and includes the
building penetration losses as defined in 7.4.3. It is calculated based on the
following equation:

:math:`PL = PL_{b} + PL_{tw} + PL_{in} + N(0,\sigma_P^2)`


where :math:`PL_{b}` is the basic outdoor Pathloss given in Table 7.4.1-1 of TR 38.901
according to the LOS and NLOS conditions, :math:`PL_{tw}` is the O2I building penetration
loss, :math:`PL_{in}` is the inside loss dependent on the depth into the building,
and :math:`\sigma_P` is the standard deviation for the penetration loss.
:math:`PL_{tw}`, :math:`PL_{in}` and :math:`\sigma_P` are given in Table 7.4.3-2
of TR 38.901.

The evaluation KPIs are the Downlink Geometry and the Coupling Gain. More information
can be found in [SIMPAT-calibration]_. The curves of Downlink Geometry and Coupling
Gain are compared against to that of various 3GPP industrial simulators provided
in [RP180524]_. Let us notice that due to the fact that the simulator in the current
version does not support Handover, and therefore due to the user mobility, there
is the chance that users can be located in out-of-coverage areas, we omit from the
SINR results (Downlink Geometry) calculations associated to a CQI (Channel Quality
Indicator)=0.

For the calibration purpose, the example provides the possibility to extract the
REM map and the SINR as measured at each REM point (each point of the map), as well
as the end-to-end results of the above presented KPIs, i.e. the Coupling Gain and the
Downlink Geometry and compare it against the 3GPP reference simulators as presented
in [RP180524]_. All the REM and end-to-end results can be found in [SIMPAT-calibration]_.

Finally, let us highlight that with this study we aimed to provide to the research
community an open source tool that allows the testing, evaluation, validation, and
experimentation of existing and/or new features, guaranteeing the resemblance of
the results to that of an industrial private product or of a real network.

traffic-generator-example.cc
============================
The program ``traffic-generator-example`` included in the ``nr`` module consists of a simple topology with two nodes, a TX and an RX node. We install on each of these nodes a SimpleNetDevice that assumes an infinite bandwidth and we connect them through s SimpleChannel which does not introduce neither error nor delay to the packet transmission/reception. On the TX node we install the traffic generator by specifying the type of the NGMN traffic generator (FTP, VoIP, video, gaming) and on the RX node we install the PacketSink application. The example supports several command line parameters. The parameter trafficType can be used to configure the traffic type (NGMN FTP, video, gaming and VoIP). The example gathers the measurements (the bytes transmitted) per the measurement window interval, and writes them to the output file in the root ns-3-dev folder, so one can plot this to see the pattern of each traffic type.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/traffic-generator-example_8cc.html.

cttc-nr-traffic-ngmn-mixed.cc
=============================
The program ``cttc-nr-traffic-ngmn-mixed`` included in the ``nr`` is an hegagonal topology example used to show how to configure different NGMN types of traffics or NGMN mixed scenario. The example consists of an hexagonal grid deployment consisting on a central site and a number of outer rings of sites around this central site. Each site is sectorized with three cells, pointing to 30º, 150º and 270º w.r.t. the horizontal axis, and we allocate a different band to each sector of the site. We provide a number of simulation parameters that can be configured through the command line. The parameter trafficTypeConf defines the NGMN traffic to be simulated (UDP CBR, FTP Model 1, NGMN FTP, NGMN VIDEO, HTTP, NGMN GAMING, NGMN VOIP, NGMN MIXED). In the case of NGMN MIXED, among the multiple UEs in a cell, we can define (through command line parameters) the percentage of UEs with each traffic type, to simulate mixed traffic scenarios, e.g., 10% FTP, 20% HTTP, 20% VIDEO STREAMING, 30% VoIP, 20% GAMING.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-nr-traffic-ngmn-mixed_8cc.html.

cttc-nr-traffic-3gpp-xr.cc
==========================
The program ``cttc-nr-traffic-3gpp-xr`` included in the ``nr`` consists on a simple topology of 1 gNB and various UEs, and is used to show how to configure different 3GPP XR types of traffics or mixed scenario. It can be configured with different 3GPP XR traffic generators (by using XR traffic mixer helper). We provide a number of simulation parameters that can be configured through the command line. For example, we can define the number of VR UEs, the number of AR UEs and the number of CG UEs to be simulated, resulting in a mixed XR traffic scenario.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-nr-traffic-3gpp-xr_8cc.html.

cttc-nr-simple-qos-sched.cc
===========================
The program ``examples/cttc-nr-simple-qos-sched`` is a simple example for the
QoS schedulers (see :ref:`QosSchedulers`) that is composed of 1 gNB and
various UEs (even UEs get voice 5QI=1 and odd UEs get AR 5QI=80), to test and
validate the correct functionality of the new QoS MAC schedulers. In this example,
we can configure the load (full buffer or medium load) of the voice UEs, and see
the proper behaviour of the scheduler. More precisely, when full buffer of voice
UEs is configured, we can see that the resulting throughputs follow the proportion
of scheduling priorities ratio.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-nr-simple-qos-sched_8cc.html.

cttc-nr-multi-flow-qos-sched
============================

The program ``examples/cttc-nr-multi-flow-qos-sched`` is an example that allows
testing the performance of the QoS schedulers (see :ref:`QosSchedulers`)
in conjunction with the LC QoS Assignment (see :ref:`LcAssignment`) versus other
schedulers, such as the RR and PF in conjunction with the LC RR scheduler.
The example has been designed to test the E2E delay and throughput in a single-cell
scenario with 2 UEs, where 1 UE has a NON-GBR flow and the other UE has 2 flows,
the QCI of which can be set as desired.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-nr-multi-flow-qos-sched_8cc.html.

gsoc-nr-channel-models.cc
===========================

The program ``examples/gsoc-nr-channel-models`` demonstrates how to automatically
or manually configure spectrum channels for end-to-end simulation. Users
can choose between Friis or phased-array channel models configured by the ``NrChannelHelper``.
The example features a topology with a single remote host generating UDP traffic.
Additionally, this example collects traffic and path loss traces.

cttc-nr-fh-xr
=============

The program ``examples/cttc-nr-fh-xr`` is an example that allows to perform
evaluations with respect to the impact that fronthaul capacity limitations
can have on scenarios with delay-critical XR traffic. The main purpose is to
give to the user a tool that will allow him to test various fronthaul control
methods and various fronthaul link capacities and study the impact on the
end-to-end throughput and delay.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/cttc-nr-fh-xr_8cc.html.

gsoc-nr-rl-based-sched
=========================

The program ``examples/gsoc-nr-rl-based-sched`` provides a example for evaluating
the performance of RL-based schedulers (see :ref:`RLScheduler`). It is designed to assess
E2E delay and throughput in a single-cell scenario with two UEs.
One UE has a single flow with NON-GBR traffic (5QI=80), while the second UE has multiple flows
with NON-GBR traffic (5QI=80) and delay-critical (DC)-GBR traffic (5QI=87).

Using this program, users can compare the performance of RL-based schedulers with other schedulers,
such as the QoS schedulers (see :ref:`QosSchedulers`) set up with either LC QoS Assignment (see :ref:`LcAssignment`)
or LC RR assignment, by configuring the ``schedulerType`` parameter.

To enable the RL-based scheduler, the ns3-gym module must be installed and the enableAi
parameter must be set to true.

The complete details of the simulation script are provided in
https://cttc-lena.gitlab.io/nr/html/gsoc-nr-rl-based-sched_8cc.html.

rl-sched-gym-env-intro.py
#########################

Simple script for testing ``gsoc-nr-rl-based-sched`` example. The script runs the example
with the default ``Ns3Env`` environment without any specific RL model. In the script, the
action is sampled using the ``sample`` method of the action space, and the selected action
is sent to the simulator through the ``Ns3Env`` environment.

rl-sched-gym-env-ppo.py
########################

Proximal Policy Optimization (PPO) script for testing ``gsoc-nr-rl-based-sched`` example.
The script runs the example with the PPO model under the ``Ns3Env`` environment. For each
iteration, the model is trained with the collected data from the simulation and send the
selected actions to the simulator through the ``Ns3Env`` environment.

cttc-nr-mimo-demo.cc
=====================
The program ``examples/cttc-nr-mimo-demo`` is an example that shows how to setup and
use SU-MIMO. The scenario consists of a simple topology, in which there is one gNB and one UE.
An additional pair of gNB and UE can be enabled to simulate the interference (see enableInterfNode).
Example creates one DL flow that goes through only BWP. The example prints on-screen and
into the database the end-to-end result of the flow of interest.

Configuring SU-MIMO
###################

The example shows how to configure some of the main parameters to enable SU-MIMO in the
simulation, such as: ``NrHelper::CsiFeedbackFlags``, and how to setup most of the MIMO related parameters
through a structure ``NrHelper::MimoPmiParams`` that is being passed to ``NrHelper``.
Some of the SU-MIMO related parameters that are being configured through this structure are: the type of the
precoding matrix search, i.e., the type of ``NrPmSearch`` algorithm; ``RankLimit`` which is the parameter of ``PmSearch``
algorithm; ``SubbandSize`` and ``DownsamplingTechnique``. If the search algorithm is of type ``NrPmSearchFull`` then
the additional parameter that can be set is ``CodebookType``. The codebook to be used for the full search can be:
a) ``ns3::NrCbTwoPort``, the two-port codebook defined in 3GPP TS 38.214 Table 5.2.2.2.1-1, or
b) ``ns3::NrCbTypeOneSp``, Type-I Single-Panel Codebook 3GPP TS 38.214 Rel. 15, Sec. 5.2.2.2.1 supporting codebook mode 1 only,
and limited to rank 4. The other parameters to play with in this example are: ``NrUePhy::WbPmiUpdateInterval``,
the wide-band PMI update interval in ms, and ``NrUePhy::SbPmiUpdateInterval``, the sub-band PMI update interval in ms.

Configuring CSI feedback type
#############################
Finally, ``NrHelper::CsiFeedbackFlags`` parameter defines the type of the CSI feedback. For example, the CSI feedback can
be based only on DATA, and thus is aperiodic and might not contain the information of all the RBGs. Also, the CSI
feedback can be based on CSI-RS and CSI-IM, and hence is periodic and provides the information over all the bandwidth.
This parameter can take the following values: ``CQI_PDSCH_MIMO = 1``, ``CQI_CSI_RS = 2``, ``CQI_PDSCH_MIMO|CQI_CSI_RS = 3``,
``CQI_CSI_RS|CQI_CSI_IM = 6``, ``CQI_PDSCH_MIMO|CQI_CSI_RS|CQI_CSI_IM = 7``, and ``CQI_PDSCH_SISO = 8``.

gsoc-leo-demo-example.cc
========================
This example demonstrates how to integrate LEO satellite mobility with the 3GPP NTN channel
and propagation models, using ``NrChannelHelper`` to configure the gNBs through ``NrHelper``.
It builds a small NTN (Non-Terrestrial Network) scenario where satellites move along circular
orbits and exchange traffic with a ground node in both the downlink (satellite to ground) and
the uplink (ground to satellite).

The example instantiates satellite nodes following circular LEO orbits using the
``LeoCircularOrbitMobilityModel``. To simplify node creation it uses the ``LeoOrbitNodeHelper``,
which is parameterized by the constellation altitude, inclination, number of orbital planes and
number of satellites per plane. These values are grouped in a ``LeoOrbitalShell`` data class.
Both classes now live in the ns-3 ``mobility`` module.

On the ground, a node with fixed geographic position connects to the satellites within a given
3GPP NTN scenario, selectable via ``--scenario`` (default ``NTN-Rural``; the available choices
are listed in ``helper/nr-channel-helper.h``). A bidirectional UDP traffic pattern is installed:
a downlink flow from a remote host to the ground node and an uplink flow from the ground node
back to the remote host, each carrying 15000 bytes. The example prints the bytes received in
each direction at the end of the run.

**Deployment presets.** The ``--application`` option selects a representative NTN deployment,
setting the carrier frequency, bandwidth, satellite EIRP density, terminal transmit power, antenna
gains, satellite receiver noise figure and orbit altitude. Any of those values can still be
overridden individually on the command line (e.g. ``--satNoiseFigure``, ``--altitudeKm``). Default
values are taken from 3GPP TR 38.821 and public system parameters.

.. list-table::
   :header-rows: 1

   * - ``--application``
     - Band
     - Bandwidth
     - Sat EIRP
     - Terminal Tx
     - Sat gain
     - Terminal gain
     - Sat NF
     - Altitude
   * - ``dtm`` (direct-to-mobile handheld)
     - 0.7 GHz
     - 5 MHz
     - 50 dBW/MHz
     - 23 dBm
     - 60 dBi
     - 0 dBi
     - 1.5 dB
     - 550 km
   * - ``vsat`` (broadband terminal, default)
     - Ka, 20 GHz
     - 100 MHz
     - 24 dBW/MHz
     - 33 dBm
     - 38.5 dBi
     - 40 dBi
     - 5 dB
     - 1200 km
   * - ``backhaul`` (ground gateway dish)
     - Ka, 20 GHz
     - 400 MHz
     - 20 dBW/MHz
     - 40 dBm
     - 38.5 dBi
     - 50 dBi
     - 5 dB
     - 1200 km

The ``dtm`` case is the most challenging: a 0 dBi, 23 dBm handheld return link is severely
power-limited. To close it, the preset mirrors how real direct-to-cell systems are dimensioned --
a low (~550 km) orbit, a low cellular band, a low-noise satellite receiver (1.5 dB) and a very
large satellite antenna. The 60 dBi satellite gain is an *effective* value: beyond the physical
array, it stands in for the narrowband uplink processing gain that real direct-to-cell systems
(NB-IoT-like) use to concentrate the handheld's limited power, which the 5 MHz NR waveform here
cannot represent directly. With these values both directions deliver in full in the smoke test and
partially under ``--realisticPower`` (the uplink remains the marginal direction).

**Power configuration.** By default the example intentionally
over-drives the link as a connectivity smoke test: the configured ``--satEIRP`` is applied as the
conducted transmit power and the antenna gains are set on every element of the antenna array, so
the radiated EIRP and the effective gains end up well above the configured values. This
guarantees connectivity but is not a physically faithful link budget. Passing ``--realisticPower``
compensates for this: the conducted power is reduced by the satellite antenna gain so the radiated
EIRP matches ``--satEIRP``, and the per-element gains are reduced by the array factor
(``10*log10(numElements)``) so the array boresight gain matches the configured
``--satAntennaGainDb`` / ``--vsatAntennaGainDb``. The resulting SINRs are then representative of an
operational NTN link.

A mobility/antenna trace can be written with ``--traceFile``, and a custom constellation can be
loaded from a CSV file with ``--orbitFile`` (see ``LeoOrbitNodeHelper`` for the file format).
This example is loosely based on the ns-3 ``leo-satellite-example`` (orbital mobility and antenna
pointing) and ``cttc-3gpp-channel-example``.

To execute it:

.. sourcecode:: bash

    $ ./ns3 run gsoc-leo-demo-example -- --application=vsat --realisticPower --duration=4

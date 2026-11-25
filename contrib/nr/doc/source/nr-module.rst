.. Copyright (c) 2022 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

Module Documentation
----------------------------

.. highlight:: cpp

.. heading hierarchy:
   ------------- Chapter
   ************* Section (#.#)
   ============= Subsection (#.#.#)
   ############# Paragraph (no number)

..
   The document is written following RST formatting. Please, check your writing on grammarly; to ease it, use the auto-wrapping feature of your text editor, without entering new lines unless you would like to start a new paragraph.

Introduction
------------
The 3rd Generation Partnership Project (3GPP) has devoted significant efforts to standardize the fifth-generation (5G) New Radio (NR) access technology [TS38300]_, which is designed to be extremely flexible from its physical layer definition and up to the architecture. The objective is to be able to work in a wide range of frequency bands and address many different use cases and deployment options.

As the NR specification is developed and evolves, a network simulator that is capable of simulating emerging NR features is of great interest for both scientific and industrial communities. In recent years a lot of effort has been made by New York University (NYU) Wireless and the University of Padova to develop a simulator that will allow simulations of communications in millimeter-wave (mmWave) bands, the heart of future 5G cellular wireless systems. Hence, a new mmWave simulation tool has been developed as a new module of ns-3. A complete description of the mmWave module is provided in [end-to-end-mezz]_. The mmWave module source code is still not part of the standard ns-3 distribution and is available at a different repository [mmwave-module]_. In the mmWave module, the physical (PHY) layer and medium access control (MAC) are a modified version of the ns-3 'LTE' PHY and MAC layers, supporting a mmWave channel, propagation, beamforming, and antenna models. The MAC layer supports Time Division Duplexing (TDD), and a Time Division Multiple Access (TDMA) MAC scheduling, with enhanced Hybrid Automatic Repeat and reQuest (HARQ) for low latency applications. The higher layers are mostly based on ns-3 'LTE' module functionalities but are extended to support features such as dual connectivity and low latency radio link control (RLC) layer.

In this document, we describe the implementation that we have initiated to generate a 3GPP-compliant NR module able to provide ns-3 simulation capabilities in the bands above and below 6 GHz, aligned with 3GPP NR Release-15, following the description in [TS38300]_. The work has been initially funded by InterDigital Communications Inc, and continues with funding from the Lawrence Livermore National Lab (LLNL) and a grant from the National Institute of Standards and Technologies (NIST).

The 'NR' module is focused on targeting the 3GPP Release-15 NR specification. As such, it incorporates fundamental PHY-MAC NR features like
a flexible frame structure by means of multiple numerologies support, bandwidth parts (BWPs), Frequency Division Multiplexing (FDM) of numerologies,
Orthogonal Frequency-Division Multiple Access (OFDMA), flexible time- and frequency- resource allocation and scheduling, Low-Density Parity Check (LDPC)
coding for data channels, modulation and coding schemes (MCSs) with up to 256-QAM, and dynamic TDD, among others.
The NR higher layers and core network (RLC, PDCP, RRC, NAS, EPC) are mainly based on the higher layers implementation
from |ns3| 'LTE' module. The NR module provides an NR non-standalone (NSA) implementation.

The source code for the 'NR' module lives currently in the directory ``src/nr``.

Over time, extensions found in this module may migrate to the existing ns-3 main development tree.

The rest of this document is organized into five major chapters:

2. **Design:**  Describes the models developed for ns-3 extension to support NR features and procedures.
3. **Usage:**  Documents how users may run and extend the NR test scenarios.
4. **Validation:**  Documents how the models and scenarios have been verified and validated by test programs.
5. **Open Issues and Future Work:**  Points to the issue tracker where known open issues are reported, and to the list of pending and ongoing features maintained in ``FEATURES.md``.
6. **References:**  Lists the bibliographic references used throughout this document.


Design
------
In this section, we present the design of the different features and procedures that we have developed following 3GPP Release-15 NR activity. For those features/mechanisms/layers that still have not been upgraded to NR, the current design following LTE specifications is also indicated.



.. toctree::
   :maxdepth: 2

   architecture
   scope
   phy-layer
   mac-layer
   rlc-layer
   pdcp-layer
   sdap-layer
   rrc-layer
   nas-layer
   epc
   interfaces
   helpers
   application-layer
   extensions

Usage
-----

This section is principally concerned with the usage of the model, using
the public API. We discuss on examples available to the user.


.. toctree::
   :maxdepth: 2

   examples

Validation
----------


.. toctree::
   :maxdepth: 2

   tests

Open issues and future work
---------------------------

Known open issues of the 'NR' module are tracked in the issue tracker of the 'NR' GitLab repository: https://gitlab.com/cttc-lena/nr/-/work_items. Please refer to it for the list of currently open issues, and use it to report any new issue you may find.

Regarding future work, the list of supported, partially supported, and planned features of the 'NR' module is maintained in the ``FEATURES.md`` file, located at the root directory of the 'NR' repository and available online at the 5G-LENA website: https://5g-lena.cttc.es/features/. That file provides an at-a-glance view of the status of each feature, organized per layer, together with the calibration and testing framework, and the extensions built on top of the 'NR' module (Sidelink and NR-V2X, O-RAN, and NR-U). It also contains the planned roadmap with the pending and ongoing enhancements.

References
----------

.. [mmwave-module] NYU WIRELESS, University of Padova. "ns-3 module for simulating mmwave-based cellular systems". Available at https://github.com/nyuwireless/ns3-mmwave.

.. [TR38900] 3GPP TR 38.900. "Study on channel model for frequency above 6GHz, (Release 14) TR 38.912v14.0.0 (2016-12), 3rd Generation Partnership Project". 2016.

.. [TR38901] 3GPP TR 38.901, "Study on channel model for frequencies from 0.5 to 100 GHz (Release 17)", v17.0.0, March 2022.

.. [end-to-end-mezz] Marco Mezzavilla, Menglei Zhang, Michele Polese, Russell Ford, Sourjya Dutta, Sundeep Rangan, Michele Zorzi. "End-to-End Simulation of 5G mmWave Networks". In IEEE Communication Surveys and Tutorials, vol. 13, No 20,  pp. 2237-2263, April 2018.

.. [WNS32018-NR]  Biljana Bojovic, Sandra Lagen, L. Giupponi. "Implementation and Evaluation of Frequency Division Multiplexing of Numerologies for 5G New Radio in ns-3". In Workshop on ns-3, June 2018, Mangalore, India.

.. [CAMAD2018-NR] Natale Patriciello, Sandra Lagen, L. Giupponi, Biljana Bojovic. "5G New Radio Numerologies and their Impact on the End-To-End Latency". In Proceedings of IEEE International Workshop on Computer-Aided Modeling Analysis and Design of Communication Links and Networks (IEEE CAMAD), 17-19 September 2018, Barcelona (Spain).

.. [CA-WNS32017] Biljana Bojovic, D. Abrignani Melchiorre, M. Miozzo, L. Giupponi, N. Baldo. "Towards LTE-Advanced and LTE-A Pro Network Simulations: Implementing Carrier Aggregation in LTE Module of ns-3". In Proceedings of the Workshop on ns-3, Porto, Portugal, June 2017.

.. [ff-api] FemtoForum , "LTE MAC Scheduler Interface v1.11". Document number: FF_Tech_001_v1.11 , Date issued: 12-10-2010.

.. [TS38133] 3GPP TS 38.133, 5G; NR; Requirements for support of radio resource management (Release 19), v19.3.0, Jan. 2026.

.. [TS38211] 3GPP  TS  38.211, TSG  RAN;  NR;  Physical channels and modulation (Release 18), v18.4.0, Sep. 2024.

.. [TS38212] 3GPP  TS  38.212, TSG  RAN;  NR;  Multiplexing  and  channel  coding (Release 16), v16.0.0, Dec. 2019.

.. [TS38213] 3GPP  TS  38.213, TSG  RAN;  NR;  Physical  layer  procedures  for  control (Release 16), v16.0.0, Dec. 2019.

.. [TS38214] 3GPP  TS  38.214, TSG  RAN;  NR;  Physical  layer  procedures  for  data (Release 16), v16.0.0, Dec. 2019.

.. [TS38300] 3GPP TS 38.300, TSG RAN; NR; Overall description; Stage 2 (Release 16), v16.0.0, Dec. 2019

.. [calibration-l2sm] A.-M. Cipriano,  R.  Visoz,  and  T.  Salzer.  "Calibration  issues  of  PHY layer  abstractions  for  wireless  broadband  systems". IEEE  Vehicular Technology Conference, Sept. 2008.

.. [nr-l2sm] Sandra Lagen, K. Wanuga, H. Elkotby, S. Goyal, N. Patriciello, L. Giupponi. "New Radio Physical Layer Abstraction for System-Level Simulations of 5G Networks". In Proceedings of IEEE International Conference on Communications (IEEE ICC), 7-11 June 2020, Dublin (Ireland).

.. [baldo2009] Nicola Baldo and M. Miozzo. "Spectrum-aware Channel and PHY layer modeling for ns3". Proceedings of ICST NSTools 2009, Pisa, Italy.

.. [notching1] Howard McDonald, D. Shyy, M. Steele and C. Patterson. "LTE Uplink Interference Mitigation Features," MILCOM 2018 - 2018 IEEE Military Communications Conference (MILCOM), Los Angeles, CA, 2018, pp. 505-511, doi: 10.1109/MILCOM.2018.8599850

.. [notching2] Howard McDonald et al. "AWS-3 Interference Mitigation: Improving Spectrum Sharing with LTE & 5G Uplink Spectrum Control," MILCOM 2019 - 2019 IEEE Military Communications Conference (MILCOM), Norfolk, VA, USA, 2019, pp. 102-107, doi: 10.1109/MILCOM47813.2019.9020877

.. [lte-ulpc] LTE ns-3 implementation of uplink power control: https://www.nsnam.org/docs/models/html/lte-design.html#power-control

.. [SigProc5G] F.-L. Luo and C. J. Zhang. "Signal Processing for 5G: Algorithms and Implementations". John Wiley & Sons., Aug. 2016.

.. [TS38331]  3GPP. "TS 38.331, Radio Resource Control (RRC), (Rel. 15)". 2018.

.. [IMT-2020] ITU-R. "Submission, evaluation process and consensus building for IMT-2020, ITU-R IMT-2020/2-E". 2019.

.. [SIMPAT-calibration] Katerina Koutlia, Biljana Bojovic, Z. Ali, S. Laǵen. "Calibration of the 5G-LENA system level simulator in 3GPP reference scenarios". Simulation Modelling Practice and Theory 119, 2022.

.. [RP180524] Huawei. "RP-180524 Summary of Calibration Results for IMT-2020 Self Evaluation". 3GPP TSG RAN Meeting #79, 2018.

.. [NGMN-traffics] NGMN Alliance. "NGMN Radio Access Performance Evaluation Methodology". 2008.

.. [TR38838] 3GPP. "TR 38.838 Study on XR (Extended Reality) Evaluations for NR". V17.0.0, 2022.

.. [WNS32022-ngmn] Biljana Bojovic, Sandra Lagen. "Enabling NGMN mixed traffic models for ns-3". In Workshop on ns-3, June 2022.

.. [WNS3-QosSchedulers] Katerina Koutlia, Sandra Lagen, and Biljana Bojovic. "Enabling QoS Provisioning Support for Delay-Critical Traffic and Multi-Flow Handling in ns-3 5G-LENA". In Proceedings of the 2023 Workshop on ns-3 (WNS3 '23). Association for Computing Machinery, New York, NY, USA, 45–51. https://doi.org/10.1145/3592149.3592159.

.. [Palomar2006] Daniel P. Palomar and Yi Jiang. "MIMO Transceiver Design via Majorization Theory"

.. [interf-whitening] "Whitening transformation": https://en.wikipedia.org/wiki/Whitening_transformation

.. [eigen3] Eigen library: https://libeigen.gitlab.io

.. [ComNetFhControl] Katerina Koutlia, Sandra Lagén. "On the impact of Open RAN Fronthaul Control in scenarios with XR Traffic". Computer Networks, Volume 253, August 2024.

.. [Sasaoka2019] Naoto Sasaoka, Takumi Sasaki, Yoshio Itoh. "PMI/RI Selection Based on Channel Capacity Increment Ratio". 2019 International Symposium on Multimedia and Communication Technology (ISMAC). doi: 10.1109/ISMAC.2019.8836179.

.. [Maleki2023] Marjan Maleki, Juening Jin and Martin Haardt. "Low Complexity PMI Selection for BICM-MIMO Rate Maximization in 5G New Radio Systems". 2023 31st European Signal Processing Conference (EUSIPCO). doi: 10.23919/EUSIPCO58844.2023.10290121.

.. [TS23501] 3GPP. "TS 23.501, System architecture for the 5G System (5GS)", V20.2.0, 2026.

.. [TS24501] 3GPP. "TS 24.501, Non-Access-Stratum (NAS) protocol for 5G System (5GS)", V20.0.0, 2026.

.. [TS37324] 3GPP TS 37.324, "Evolved Universal Terrestrial Radio Access (E-UTRA) and NR; Service Data Adaptation Protocol (SDAP) specification".

.. [TS38322] 3GPP TS 38.322, "NR; Radio Link Control (RLC) protocol specification".

.. [TS38323] 3GPP TS 38.323, "NR; Packet Data Convergence Protocol (PDCP) specification".

..

   References inherited from LTE

.. [TS23401] 3GPP TS 23.401, "General Packet Radio Service (GPRS) enhancements for Evolved Universal Terrestrial Radio Access Network (E-UTRAN) access".

.. [TS24301] 3GPP TS 24.301, "Non-Access-Stratum (NAS) protocol for Evolved Packet System (EPS); Stage 3".

.. [TS29274] 3GPP TS 29.274, "3GPP Evolved Packet System (EPS); Evolved General Packet Radio Service (GPRS) Tunnelling Protocol for Control plane (GTPv2-C); Stage 3".

.. [TS29281] 3GPP TS 29.281, "General Packet Radio System (GPRS); GPRS Tunnelling Protocol User Plane (GTPv1-U)".

.. [TS36133] 3GPP TS 36.133 "E-UTRA Requirements for support of radio resource management"

.. [TS36300] 3GPP TS 36.300, "Evolved Universal Terrestrial Radio Access (E-UTRA) and Evolved Universal Terrestrial Radio Access Network (E-UTRAN); Overall description; Stage 2".

.. [TS36321] 3GPP TS 36.321, E-UTRA; Medium Access Control (MAC) protocol specification.

.. [TS36322] 3GPP TS 36.322, "Evolved Universal Terrestrial Radio Access (E-UTRA); Radio Link Control (RLC) protocol specification".

.. [TS36323] 3GPP TS 36.323, "Evolved Universal Terrestrial Radio Access (E-UTRA); Packet Data Convergence Protocol (PDCP) specification".

.. [TS36331] 3GPP TS 36.331 "E-UTRA Radio Resource Control (RRC) protocol specification"

.. [TS36413] 3GPP TS 36.413, "Evolved Universal Terrestrial Radio Access Network (E-UTRAN); S1 Application Protocol (S1AP)".

.. [TS36420] 3GPP TS 36.420, "Evolved Universal Terrestrial Radio Access Network (E-UTRAN); X2 general aspects and principles".

.. [TS36423] 3GPP TS 36.423, "Evolved Universal Terrestrial Radio Access Network (E-UTRAN); X2 Application Protocol (X2AP)".

.. [R4-081920] 3GPP TSGR4_48 R4-081920 "LTE PDCCH/PCFICH Demodulation Performance Results with Implementation Margin"

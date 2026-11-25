.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

Identifying components
**********************

Often, a simulation user will need to identify the object from which some messages come from, or to be able to read the output traces correctly.
Each message will be associated with one tuple -- ccId and bwpId. The meaning of these names does not reflect the natural sense that we could give to
these words. In particular, the definition is the following:

* the bwpId is the index of an imaginary vector that holds all the instances of BWP (as a paired MAC/PHY) in the node. It is assigned at the creation time by the helper, and the BWP with ID 0 will be the primary carrier;
* the ccId is a number that identifies the MAC/PHY pair uniquely in the entire simulation.

All the nodes in the simulation will have the same number of BWPs. Each one will be numbered from 0 to n-1, where n is the total number of spectrum parts.
For example:

.. _tab-example-spectrum:

.. table:: An example spectrum division

   +------------+------------+---------------+---------------+---------------+
   |                                Band 1                                   |
   +------------+------------+---------------+---------------+---------------+
   |           CC 0          |              CC 1             |     CC 2      |
   +------------+------------+---------------+---------------+---------------+
   |          BWP0           |              BWP1             |      BWP2     |
   +------------+------------+---------------+---------------+---------------+


The ccId numbering is, for some untrained eyes, weird. But that is because some numbers in the sequence are used to identify the Cell ID. For example, let's consider a scenario in which we split the spectrum into three parts. We have four GNBs, and the numbering will be the following:

.. _tab-example-bwp:

.. table:: CcId numbering example

   +------------+------------+---------------+---------------+---------------+
   |    GNB     |  Cell ID   | CcId for BWP0 | CcId for BWP1 | CcId for BWP2 |
   +============+============+===============+===============+===============+
   |   GNB 0    |      1     |       0       |       1       |      2        |
   +------------+------------+---------------+---------------+---------------+
   |   GNB 1    |      2     |       0       |       1       |      2        |
   +------------+------------+---------------+---------------+---------------+
   |   GNB 2    |      3     |       0       |       1       |      2        |
   +------------+------------+---------------+---------------+---------------+
   |   GNB 3    |      4     |       0       |       1       |      2        |
   +------------+------------+---------------+---------------+---------------+

If we would use this as a simulation scenario, the messages that come from the same CcId number (e.g. 0), would refer to the same portion of the spectrum (if the same BWPs are configured to all devices).
These IDs, internally at the GNB, would be translated into the BWP 0 in all the cases. The BWP 1 will be associated with the CcId 1 (respectively), and everything else follows.

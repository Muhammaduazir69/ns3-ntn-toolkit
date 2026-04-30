..
   SPDX-License-Identifier: GPL-2.0-only
   Copyright (c) 2026 Muhammad Uzair and contributors

ns3-ai-ntn Module
=================

.. include:: replace.txt
.. highlight:: cpp

Overview
--------

``ns3-ai-ntn`` is a fork of the upstream ``ns3-ai`` module
(Yin et al., WNS3 2020).  It preserves the shared-memory interface,
ProtoBuf binding, and Gym-style environment wrapper of upstream, and
adds:

* NTN-specific Gym environments that wrap ``ntn-cho`` state.
* A Flower AI adaptor for federated learning across ns-3 instances.
* FedAvg / FedProx / FedNova / SCAFFOLD benchmarks for the NTN CHO
  decision task.
* ns-3.43 build fixes.

Upstream attribution
~~~~~~~~~~~~~~~~~~~~

Upstream project:
https://github.com/hust-diangroup/ns3-ai

Please cite:

.. sourcecode:: bibtex

   @inproceedings{yin2020ns3ai,
     title = {{ns3-ai}: Fostering Artificial Intelligence Algorithms
              for Networking Research},
     author = {Yin, Hao and Liu, Pengyu and Liu, Keshu and Cao, Liu
               and Zhang, Lijun and Gao, Yayu and Hei, Xiaojun},
     booktitle = {Proc. ACM Workshop on NS-3 (WNS3)},
     year = {2020}
   }

Usage
-----

See ``examples/ntn-handover-gym.cc`` for the NTN CHO RL environment,
and ``python_utils/flower_adaptor.py`` for the federated-learning
adaptor.

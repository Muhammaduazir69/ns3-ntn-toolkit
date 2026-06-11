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

* ns-3.43 / Python 3.13 / NumPy 2.0 / Gymnasium 1.0 compatibility
  fixes (per-target LTO disable, shared-memory RAII, no ``std::exit``
  in library code).
* The ``ns3_ai_ntn`` Python package (``python_utils/``): four NTN
  Gymnasium environments (``HandoverEnv``, ``BeamMgmtEnv``,
  ``SliceEnv``, ``PowerCtrlEnv``), Stable-Baselines3 PPO training,
  PyTorch Geometric GAT constellation-graph models, MAPPO / MASAC
  multi-agent baselines, and an ``ns3gym`` compatibility shim.
* An AI-RAN inference contract (``grpc/``):
  ``AiranInferenceClient`` / ``AiranInferenceServer`` exchanging
  length-prefixed protobuf over pluggable in-process or TCP
  ``InferenceChannel`` transports, a Triton ``config.pbtxt`` parser,
  two shipped Triton model-repository skeletons, and deterministic
  mock runtimes for testing.

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

Build as part of the toolkit tree::

   ./ns3 configure --enable-examples --enable-tests
   ./ns3 build

The bundled examples (``a-plus-b``, ``rl-tcp``, ``rate-control``,
``lte-cqi``, ``multi-bss``) each carry a README with run instructions;
the Python script spawns the matching ns-3 binary itself. The NTN RL
environments and trainers live in ``python_utils/`` (see its README).

Testing
-------

Run the C++ AI-RAN inference test suite with::

   ./test.py -s oran-ntn-airan-inference

and the Python tests with ``pytest python_utils/tests/``.

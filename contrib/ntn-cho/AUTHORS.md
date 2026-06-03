# Authors

## Maintainer

- **Muhammad Uzair** — `muhammaduzairr69@gmail.com`
  ORCID: 0009-0002-4104-2680
  Design, implementation, TTE estimator, CHO algorithms, tests, examples,
  documentation.

## Acknowledgements

- SGP4 orbit propagation and the Walker constellation geometry are
  provided by the SNS3 [`satellite`](https://github.com/sns3/sns3-satellite)
  module (`SatSGP4MobilityModel` + antenna-gain patterns); this module
  consumes them rather than re-implementing orbit propagation.
- The optional `NtnAiInterface` uses the
  [`ns3-ai`](https://github.com/hust-diangroup/ns3-ai) shared-memory
  bridge.
- 3GPP TR 38.821 and TS 38.331 (Release 17, conditional handover) are
  the normative source for the CHO state machine; TR 38.811 §6.1.1.1
  defines the seven UE mobility classes.

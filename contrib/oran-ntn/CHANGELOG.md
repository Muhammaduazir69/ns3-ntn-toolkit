# Changelog

All notable changes to this module are documented here.

## [1.0.0] — 2026-04-23

### Added
- Initial public release of `oran-ntn`.
- Near-RT RIC (`OranRic`) with E2AP session management.
- 13 xApps, each with its own decision interval and KPM subscription:
  `TrafficSteeringXapp`, `HandoverControlXapp`, `QosOptimiserXapp`,
  `BeamManagementXapp`, `LoadBalancerXapp`,
  `AnomalyDetectorXapp`, `EnergySaverXapp`, `SlicingXapp`,
  `RlmXapp`, `CellOnOffXapp`, `AdmissionControlXapp`,
  `InterferenceCoordinatorXapp`, `SpaceRicXapp`.
- **A1 policy engine** with 11 policy types covering QoS, handover,
  and slicing.
- **E2SM-RC action runtime** with 28 actions (22 in Table V, 6
  internal).
- **Conflict-resolution matrix** for 4 named co-located xApps.
- **Space RIC autonomous-mode extensions**: feeder outage, orbital
  autonomy, stressed-feeder KPI collection.
- 60-second full-xApp scenario
  (`examples/oran-ntn-scenario-b-full-xapps.cc`) producing
  `action_log.csv` (7369 actions, 0 conflicts).
- KPM dataset schema with 30+ measurement IDs.
- 5 unit-test suites.

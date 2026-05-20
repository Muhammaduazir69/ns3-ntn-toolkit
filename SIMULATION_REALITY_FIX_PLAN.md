# Simulation Reality Fix Plan
*Date:* 2026-05-19
*Owner:* Muhammad Uzair (Independent Researcher)
*Symptom reported:* every example finishes in 1–2 wall-clock seconds regardless of `--simTime`.

---

## 1. Root cause (confirmed by audit)

Most `contrib/<module>/examples/*.cc` are **calculator scripts**, not real
event-driven simulations. A user-space `for(double t=0; t<simTime; t+=dt)`
loop iterates orbital positions and writes CSVs, but the ns-3 event queue
stays empty.  When `Simulator::Run()` is called at all, no traffic, no
Internet stack, and no periodic timers are scheduled — so the simulator
returns immediately.

Per-example diagnostic (key columns from the audit):

| Example                                  | Run | Sched | Inet | TrafApp | Verdict       |
|------------------------------------------|----:|------:|-----:|--------:|---------------|
| ntn-cho-full-constellation               |  0  |   0   |   0  |    0    | **calc-only** |
| ntn-cho-leo-basic                        |  1  |   0   |   0  |    0    | **empty run** |
| ntn-realistic-mobility-demo              |  0  |   0   |   0  |    0    | **calc-only** |
| thz-ntn-demo                             |  0  |   0   |   0  |    0    | **calc-only** |
| thz-ntn-{leo-ground, isl, isac, um-mimo, ris-assisted, beam-tracking, dband-constellation, full-stack} | 1 | 0 | 0 | 0 | **empty run** |
| oran-ntn-full-scenario                   |  1  |   4   |   0  |    0    | **partial**   |
| ntn-rrc-{leo-pass, full-stack, from-tle} |  1  |   2-4 |   0  |    0    | **partial**   |
| sagin-{haps-leo-relay, uav-swarm, aero}  |  1  |   1   |   0  |    0    | **partial**   |
| ntn-three-slice-leo-geo                  |  1  |   1   |   0  |    0    | **partial**   |
| ntn-observability-demo                   |  1  |   2   |   0  |    0    | **partial**   |
| ntn-v2x-rural-highway                    |  1  |   1   |   0  |    0    | **partial**   |
| leo-pass-sionna-vs-tr38811               |  0  |   0   |   0  |    0    | **calc-only** |
| nrtv-p2p-example, three-gpp-http-example |  1  |   0   |   1  |    0    | **app via custom helper** |
| (upstream) mmwave-tcp-building-example   |  1  |   3   |   1  |    2    | **real**      |
| (upstream) ns3-ai rate-control, lte-cqi  |  1  |   2   |   1  |    2-3  | **real**      |

Across the 11 NTN-specific modules we own, **22 of 25 examples are not running
real packets through the ns-3 stack.** This is why a 600 s sim finishes in
228 ms.

---

## 2. What "realistic" means here

For each example to actually exercise the model it advertises, three pieces
must exist:

1. **Internet protocol stack on every node**
   `InternetStackHelper().Install(allNodes)`
   plus IP address assignment via `Ipv4AddressHelper`.

2. **A NetDevice on each link that uses the module's channel/PHY**
   *  CHO/RRC/V2X/SAGIN → mmWave (or NR-LENA) attached to LEO gNB/UE pair.
   *  THz examples → `MultiModelSpectrumChannel` carrying a
      `ThzNtnPropagationLossModel` + `ThzNtnPhy` (already exists in
      `contrib/thz-ntn/model`).
   *  Slice/observability → use the same mmWave stack with
      slice-tagged bearers.

3. **Real traffic that flows over that NetDevice**
   *  `OnOffHelper` with `DataRate("X Mbps")` and `PacketSize=1400` for
      bulk eMBB / video.
   *  `UdpClientHelper` with `Interval=10ms`, `PacketSize=128` for URLLC
      pings.
   *  `BulkSendHelper` over TCP for elastic flows.
   *  `PacketSinkHelper` on the other endpoint.

If any of these three is missing, `Simulator::Run()` returns instantly
because there is nothing to advance the clock through.

---

## 3. Fix architecture

### 3.1 New shared scaffolding helper

`contrib/ntn-traffic/helper/ntn-realistic-scenario-helper.{h,cc}`

This helper wires the common parts every example needs, so each example
becomes a thin ~80-line wrapper.

```cpp
class NtnRealisticScenarioHelper {
public:
  enum class StackKind { Mmwave, Thz, Mixed };
  enum class TrafficProfile { NbIotPeriodic, EmbbStreaming, UrllcPings,
                              MixedBouquet, DigitalTwinTelemetry };

  NodeContainer InstallConstellation(uint32_t numSats,
                                     uint32_t numPlanes,
                                     double altitudeKm,
                                     double inclinationDeg);
  NodeContainer InstallUes(uint32_t numUes, NtnMobilityClass cls);

  void InstallProtocolStack(NodeContainer all, StackKind stack);
  void InstallTraffic(NodeContainer ues,
                      NodeContainer gateways,
                      TrafficProfile profile,
                      Time start, Time stop);
  void InstallPeriodicTimers(Time choEval = MilliSeconds(40),
                             Time kpm     = MilliSeconds(100),
                             Time xapp    = MilliSeconds(20));
  void EnableTracing(std::string outDir, std::string runTag);
};
```

The helper is published from `ntn-traffic` because every other module
already lists it as a build dependency.

### 3.2 Each example rewritten as a thin scenario

Example after the rewrite (~80 lines):

```cpp
int main(int argc, char* argv[]) {
  double simTime = 600.0;
  uint32_t numUes = 30;
  CommandLine cmd; ...; cmd.Parse(argc, argv);

  NtnRealisticScenarioHelper h;
  auto sats = h.InstallConstellation(66, 6, 780.0, 86.4);
  auto ues  = h.InstallUes(numUes, NtnMobilityClass::Urban);
  h.InstallProtocolStack(NodeContainer(sats, ues),
                         NtnRealisticScenarioHelper::StackKind::Mmwave);
  h.InstallTraffic(ues, sats,
                   NtnRealisticScenarioHelper::TrafficProfile::MixedBouquet,
                   Seconds(1.0), Seconds(simTime - 1.0));
  h.InstallPeriodicTimers();
  h.EnableTracing(outputDir, runTag);

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
```

### 3.3 Replacing the user-space time loop in `ntn-cho-full-constellation`

The existing `for (double t = 0; t < simTime; t += dt)` loop becomes a
single recursive scheduled event:

```cpp
void ChoEvaluateStep(double dt, double simTime) {
  if (Simulator::Now().GetSeconds() >= simTime) return;
  /* per-step CHO logic that used to live in the body of the for-loop */
  Simulator::Schedule(Seconds(dt), &ChoEvaluateStep, dt, simTime);
}
Simulator::Schedule(Seconds(0), &ChoEvaluateStep, 0.1, simTime);
```

Now the events actually live on the queue and the wall-clock advances in
step with the work being done.

### 3.4 THz examples need a spectrum channel

`thz-ntn-*` currently iterates frequency/elevation in C++ user-space. Wire
them through a real `MultiModelSpectrumChannel` so each transmission burst
runs through `ThzNtnPropagationLossModel::DoCalcRxPower()` and lands on a
real receiver:

```cpp
auto channel = CreateObject<MultiModelSpectrumChannel>();
channel->AddPropagationLossModel(CreateObject<ThzNtnPropagationLossModel>());
channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
auto txPhy = CreateObject<ThzNtnPhy>(); txPhy->SetChannel(channel);
auto rxPhy = CreateObject<ThzNtnPhy>(); rxPhy->SetChannel(channel);
auto wf    = CreateObject<WaveformGenerator>(); wf->SetSamplingPeriod(...);
Simulator::Schedule(Seconds(0.1), &WaveformGenerator::Start, wf);
Simulator::Schedule(Seconds(0.1+burst), &WaveformGenerator::Stop,  wf);
```

### 3.5 oran-ntn already has the skeleton, just add traffic

`oran-ntn-full-scenario` already schedules KPM + xApp ticks. It just lacks
the Internet stack and UE↔gateway flows. Adding them is a 30-line patch:
install `PointToPointEpcHelper`, set up a remote host, run `OnOffHelper`
between remote host and each UE.

---

## 4. Validation gates (regression protection)

To stop this from happening again, the helper exposes a `--validate` flag
that asserts at end of run:

| Metric                              | Floor (per 600 s sim) |
|-------------------------------------|----------------------:|
| Wall-clock seconds                  | ≥ 20                 |
| ns-3 events processed               | ≥ 50 000              |
| Packets transmitted on data plane   | ≥ 1 000               |
| Packets received (PacketSink count) | ≥ 0.9 × tx            |
| CSV rows in scenario output         | ≥ simTime / cadence   |

If any floor is missed, the example aborts with `NS_FATAL_ERROR` so the
CI catches stub regressions.

The helper writes a `sim_health.csv` next to every output containing the
five floors and the actual measured values, so reviewers can verify
realism directly.

---

## 5. Phased delivery

### Phase 1 — Scaffolding (1 day)
- [ ] Implement `NtnRealisticScenarioHelper` in `ntn-traffic/helper/`.
- [ ] Add `sim_health.csv` writer and `--validate` flag.
- [ ] Unit test: 60 s sim should produce ≥ 100 tx packets and ≥ 5 s wall clock.

### Phase 2 — High-impact examples (2 days)
Order by paper dependency:
1. `ntn-cho-full-constellation` (drives Paper 2 results).
2. `oran-ntn-full-scenario` (drives Paper 3 results).
3. `thz-ntn-leo-ground`, `thz-ntn-isl`, `thz-ntn-beam-tracking` (drive Paper 4).
4. `ntn-cho-leo-basic` (drives Paper 1's lifecycle figure).

Each example: rewrite around the helper, run for 600 s, capture
`sim_health.csv`, confirm gates pass.

### Phase 3 — Remaining examples (2 days)
- ntn-rrc (3 examples)
- sagin (3 examples)
- ntn-slice, ntn-v2x, ntn-observability, ntn-sionna
- ntn-traffic NRTV / HTTP examples (already partial — finish wiring).

### Phase 4 — Paper data refresh (1 day)
- Re-run all 4 papers' simulation scenarios using the rewritten examples.
- Regenerate every CSV under `papers/sim_runs/`.
- Re-run `papers/templates/regen_all_figures.py`.
- Rebuild the 4 PDFs and verify all numerical tables (Table I–VII per paper)
  still hold within statistical noise; flag any that don't.

### Phase 5 — CI gate (0.5 day)
- Add a GitHub Actions job: build, then run each example for 60 s with
  `--validate`, parse `sim_health.csv`, fail if any floor missed.
- Commit a `tools/check_simulation_health.py` that aggregates per-example
  health into a single Markdown report.

### Phase 6 — Documentation (0.5 day)
- Update each paper's "Reproducibility" section with:
  * the new `--validate` invocation,
  * a one-line citation to the `sim_health.csv` proving each result.
- Note in cover letters that v2 results come from event-driven runs (not
  the previous user-space loops).

---

## 6. Risk register

| Risk                                          | Mitigation                                   |
|-----------------------------------------------|----------------------------------------------|
| 600 s × 30 UE event-driven sims become slow   | Use `RealtimeSimulatorImpl` only optionally; default to discrete-event; throttle xApp cadence to 20 ms / KPM 100 ms; profile and parallelise via MPI if needed. |
| Numerical results in current papers shift     | Re-run with the same RNG seeds; report a delta table in each cover letter. |
| `MultiModelSpectrumChannel` on THz module triggers latent bugs | Add a `thz-ntn-spectrum-channel-test` in `contrib/thz-ntn/test/` first. |
| Validation gates flake on slow CI runners      | Health gates take `runtime_multiplier` arg; CI uses 0.5×, local uses 1.0×. |

---

## 7. Decision needed from you

Two answers before I start writing code:

1. **Scope:** do you want me to start with **Phase 1 + Phase 2** (the four
   paper-driving examples), validate them, and only then continue to the rest?
   Or rewrite all 25 examples in one sweep?

2. **Paper data:** if Phase 4 surfaces meaningful deltas in the numeric
   tables (e.g. handover-success rate moves from 83.14% → 86 %), do you
   want me to (a) update the papers in-place, or (b) report the deltas to
   you first and let you choose?

My recommendation: **start Phase 1 + the four paper-driving examples**,
report deltas back to you, and we triage the remaining 21 examples after.

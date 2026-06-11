# thz-ntn examples

The examples live in `examples/`. There are 16 `.cc` files. Two are not built:
`examples/thz-ntn-isac.cc` is wrapped in an `if(FALSE)` block in
`examples/CMakeLists.txt` (it uses the legacy ISAC API and is left out pending
the ISAC scheduler redesign), and `examples/thz-ntn-demo.cc` is not referenced
by `examples/CMakeLists.txt` at all. The remaining 14 examples are built.

The examples fall into two groups:

- **Analytic examples** — link budgets and parameter sweeps computed from the
  thz-ntn sub-models, written to CSV.
- **Real-radio examples** — a full mmwave NR NTN cell (`NtnRealStackHelper`
  from `contrib/ntn-traffic`: SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC)
  with SGP4/Walker satellite mobility and TR 38.811 ground terminals. The THz
  physics enter the packet path as live channel plug-ins
  (`ThzNtnPropagationLossModel`, `NtnStaticExtraLossModel`); traffic is carried
  by `NtnOranApplication` QoS flows measured at `NtnOranSink`, and each example
  writes `<outputDir>/sim_health.csv`. The carrier is capped at 100 GHz
  (sub-THz / W-band) by the 3GPP spectrum model.

## Build and run

```bash
# from the ns-3-dev root
./ns3 configure --enable-examples --enable-tests
./ns3 build thz-ntn

# run via the ns3 wrapper
./ns3 run thz-ntn-leo-ground
./ns3 run "thz-ntn-leo-ground --freq=300e9"
```

Built example binaries are placed under
`build/contrib/thz-ntn/examples/ns3.43-<name>-<profile>`, where `<profile>` is
the active build profile (e.g. `default`, `debug`, or `optimized`). For
example:

```bash
./build/contrib/thz-ntn/examples/ns3.43-thz-ntn-leo-ground-default
```

## Built examples

### Analytic

| Example | Description |
|---|---|
| `thz-ntn-leo-ground` | LEO-to-ground sub-THz downlink link budget over a pass, gated by molecular absorption. |
| `thz-ntn-isl` | Inter-satellite link SNR / capacity / Doppler vs separation distance (300 GHz, vacuum). |
| `thz-ntn-um-mimo` | Antenna array and beam-squint analysis: configures frequency/bandwidth/element count, builds a DFT codebook, and reads worst-case wideband squint loss from `ComputeBeamSquintLoss_dB()`. |
| `thz-ntn-ris-assisted` | Direct vs RIS-assisted SNR, N^2 scaling law, and quantisation loss using the current `ThzNtnRis` API with cascaded path loss from Sat/RIS/GT mobility positions. |
| `thz-ntn-dband-constellation` | D-band constellation UT-to-satellite association and link budgets. |
| `thz-ntn-full-stack` | Integration demo: downlink link budget + RIS assist, ISL SNR/capacity from satellite mobility, ISAC debris sensing, EKF beam tracking, and achievable spectral efficiency. |

### Real-radio (mmwave NR NTN stack, measured KPIs)

| Example | Description |
|---|---|
| `thz-ntn-real-stack` | Flagship channel-plugin demo: gaseous absorption + rain (`ThzNtnPropagationLossModel`) chained onto the real mmwave channel; toggling rain mid-run degrades the measured link. |
| `thz-ntn-leo-ground-downlink-traffic` | End-to-end downlink over a receding SGP4 satellite; molecular absorption attenuates real packets, FSPL comes from the stack's Friis model over the live geometry. |
| `thz-ntn-isl-traffic` | Real mmwave NR ISL between two cross-plane SGP4 satellites of a Starlink-class Walker shell; the `ThzNtnIslChannel` analytic budget is printed beside the measured SINR. |
| `thz-ntn-beam-tracking` | EKF beam tracking of a real SGP4 pass; the tracker's pointing loss is applied as a live channel reconfiguration, so mispointing degrades the measured KPIs. |
| `thz-ntn-ris-relay-traffic` | RIS recovers a blocked THz link mid-simulation; blockage and RIS engagement are live channel events. |
| `thz-ntn-isac-coexist-traffic` | ISAC comm/sense coexistence: the real `ThzNtnIsacScheduler`'s comm share gates the live downlink TDM-style. |
| `thz-ntn-weather-traffic` | Weather front (fog / rain / snow phases) as a live channel plug-in attenuating real packets. |
| `thz-ntn-ric-controlled-traffic` | Closed-loop thz-ntn x oran-ntn demo: measured DL SINR feeds an `OranNtnE2Node` KPM report; an xApp engages the RIS when intrinsic SINR crosses a threshold; data-plane goodput tracks the loop. |

Note: `thz-ntn-isac.cc` (legacy ISAC API) and `thz-ntn-demo.cc` are present in
`examples/` but are not built.

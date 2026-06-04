# thz-ntn examples

The examples live in `examples/`. There are 15 `.cc` files. Two are not built:
`examples/thz-ntn-isac.cc` is wrapped in an `if(FALSE)` block in
`examples/CMakeLists.txt` (it uses the legacy ISAC API and is left out pending
the ISAC scheduler redesign), and `examples/thz-ntn-demo.cc` is not referenced
by `examples/CMakeLists.txt` at all. The remaining 13 examples are built.

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

| Example | Description |
|---|---|
| `thz-ntn-leo-ground` | LEO-to-ground sub-THz downlink link budget over a pass, gated by molecular absorption. |
| `thz-ntn-leo-ground-downlink-traffic` | Real UDP downlink (NetDevice + IP + apps + FlowMonitor) over a LEO sub-THz link gated by molecular absorption. |
| `thz-ntn-isl` | Inter-satellite link SNR / capacity / Doppler vs separation distance (300 GHz, vacuum). |
| `thz-ntn-isl-traffic` | Real packet transmission over a 300 GHz ISL gated by `ThzNtnIslChannel` SNR. |
| `thz-ntn-beam-tracking` | EKF / position-based beam tracking through a LEO pass, with a traffic-helper health report. |
| `thz-ntn-um-mimo` | Antenna array and beam-squint analysis: configures frequency/bandwidth/element count, builds a DFT codebook, and reads worst-case wideband squint loss from `ComputeBeamSquintLoss_dB()`. |
| `thz-ntn-ris-relay-traffic` | RIS recovers a blocked THz link mid-simulation over a real data plane. |
| `thz-ntn-isac-coexist-traffic` | ISAC comm/sense coexistence via `ThzNtnIsacScheduler` over a real data plane. |
| `thz-ntn-weather-traffic` | Weather front (fog / rain / snow) over a THz downlink with a real data plane. |
| `thz-ntn-ris-assisted` | Direct vs RIS-assisted SNR, N^2 scaling law, and quantisation loss using the current `ThzNtnRis` API with cascaded path loss from Sat/RIS/GT mobility positions. |
| `thz-ntn-dband-constellation` | D-band constellation UT-to-satellite association and link budgets. |
| `thz-ntn-full-stack` | Integration demo: downlink link budget + RIS assist, ISL SNR/capacity from satellite mobility, ISAC debris sensing, EKF beam tracking, and achievable spectral efficiency. |
| `thz-ntn-ric-controlled-traffic` | Closed-loop thz-ntn x oran-ntn demo: THz KPIs feed an `OranNtnE2Node` KPM report; an xApp toggles the RIS when SINR crosses a threshold; data-plane goodput tracks the loop. |

Note: `thz-ntn-isac.cc` (legacy ISAC API) and `thz-ntn-demo.cc` are present in
`examples/` but are not built.

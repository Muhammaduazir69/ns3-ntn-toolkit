# ns-3.43 source patches carried by the toolkit

Apply from the ns-3 root with `patch -p1 < contrib/ntn-traffic/patches/ns3/<file>`.
The tree already contains them; the files exist so the divergence from pristine
ns-3.43 is explicit and reproducible.

| # | File | Why |
|---|------|-----|
| 0001 | `src/mobility/model/geocentric-constant-position-mobility-model.cc` | Opt-in elevation convention for `GetElevationAngle`, the input of the mainline TR 38.811 NTN channel (`ThreeGppChannelConditionModel::GetQuantizedElevationAngle`, `ThreeGppChannelModel` NTN scenarios). ns-3.43 chooses the "ground" end by the ECEF z coordinate, so the ends are swapped whenever the satellite is equatorward of the terminal, and returns `abs(asin)`. GlobalValue `GeocentricElevationModel` = `Legacy` (default, bit-identical) or `Wgs84LowerEnd`: lower end by geographic altitude, signed elevation about the local normal of that end, floored at 0.01 deg so the 1/sin(elevation) gas term stays finite (below 0 deg the link is Earth-blocked by `NtnEarthBlockageLossModel`). `NtnRealStackHelper::SetChannelModel(Ns3Tr38811)` selects it for the run and restores the previous value when the helper is destroyed. |

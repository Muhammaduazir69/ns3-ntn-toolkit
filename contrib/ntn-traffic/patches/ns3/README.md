# ns-3.49 source patches carried by the toolkit

The toolkit tree is ns-3.49 with the four changes below applied to `src/`. Each file is
`git diff ns-3.49` of the files it touches, so it applies to a pristine ns-3.49 checkout
from the ns-3 root with `patch -p1 < contrib/ntn-traffic/patches/ns3/<file>`. The toolkit
tree already contains them; the files make the divergence from upstream explicit and
reproducible. They were first written on ns-3.43 and carried through ns-3.48 to ns-3.49.

| # | Files | Why |
|---|------|-----|
| [0001](0001-mobility-geocentric-elevation-convention.patch) | `src/mobility/model/geocentric-constant-position-mobility-model.cc` | Opt-in elevation convention for `GetElevationAngle`, the input of the mainline TR 38.811 NTN channel (`ThreeGppChannelConditionModel::GetQuantizedElevationAngle`, `ThreeGppChannelModel` NTN scenarios). Upstream ns-3 chooses the "ground" end by the ECEF z coordinate, so the ends are swapped whenever the satellite is equatorward of the terminal, and returns `abs(asin)`. GlobalValue `GeocentricElevationModel` = `Legacy` (default, bit-identical) or `Wgs84LowerEnd`: lower end by geographic altitude, signed elevation about the local normal of that end, floored at 0.01 deg so the 1/sin(elevation) gas term stays finite (below 0 deg the link is Earth-blocked by `NtnEarthBlockageLossModel`). `NtnRealStackHelper::SetChannelModel(Ns3Tr38811)` selects it for the run and restores the previous value when the helper is destroyed. |
| [0002](0002-internet-tcp-timestamps-and-synsent-reset.patch) | `src/internet/model/tcp-socket-base.{cc,h}`, `src/internet/test/tcp-syn-connection-failed-test.cc` | RFC 7323 section 4.3: Last.ACK.sent follows the ACK field of every segment sent, data segments included. Upstream advanced it only in `SendEmptyPacket()`, so a peer that piggybacks every ACK on data (an HTTP server) kept echoing the first timestamp and retransmission timeouts grew on long links. Also an opt-in `Rfc9293SynSentReset`: in SYN-SENT, an RST that acknowledges the SYN closes the connection at once (RFC 9293 3.10.7.3) instead of retrying the SYN; the default keeps upstream behavior. With a regression test. |
| [0003](0003-applications-3gpp-http-page-model.patch) | `src/applications/model/three-gpp-http-variables.{cc,h}` | HTTP traffic model aligned with 3GPP TR 36.814 Table A.2.1.3-1 (page and object size distributions). |
| [0004](0004-spectrum-3gpp-channel-update-period-and-orientation.patch) | `src/spectrum/model/three-gpp-channel-model.cc` | The 3GPP channel displacement rule is evaluated once per `UpdatePeriod`, not on every query: at LEO speed (about 3.75 m per 0.5 ms slot) checking it on every call regenerated the small-scale draw about once per link per slot and destroyed the channel's time correlation. A regenerated matrix also keeps the s/u orientation of the matrix it replaces, because the long-term component conjugates the u-side weights (ns-3.48, #1314). |

Not listed here: `src/lte` is the LTE fork that the mmWave module (`contrib/mmwave`) requires,
not upstream LTE (514 files differ from ns-3.49). 5G-LENA's own changes are in
`contrib/nr/ntn-patches` (see its README).

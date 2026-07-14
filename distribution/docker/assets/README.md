# Docker Hub branding assets

Files in this directory ship the brand for the `uzairdocker69/ns3-ntn-toolkit`
Docker Hub repository. **The Hub UI does not pull these automatically** — you
upload them manually once, then they persist.

## One-time upload steps

1. Open <https://hub.docker.com/repository/docker/uzairdocker69/ns3-ntn-toolkit/general>
2. **Repository logo** → *Choose file* → `dockerhub_avatar_512.png` (Hub re-encodes to 256×256).
3. **Short description** → paste:

   > Pre-integrated ns-3.43 distribution for 6G non-terrestrial network research.
   > 13 modules: LEO/SGP4, 3GPP NR-NTN, O-RAN+FlexRIC, sub-THz, Sionna RT, SAGIN, V2X, slicing, RL, digital twin.

4. **Full description** → import the contents of
   `/home/uzair/6g_ntn_ns3/ns-3-dev/README.md`
   (the Hub Markdown renderer supports the same syntax as GitHub).
5. **Categories** → `Internet of Things`, `Operating Systems` (closest fits Hub offers).

## Asset reference

| File | Use |
|---|---|
| `dockerhub_avatar.png`     | 256 × 256 — what the Hub displays in search/listings |
| `dockerhub_avatar_512.png` | 512 × 512 — upload this; Hub down-samples internally with better resampling than the 256 source |

The PNGs above are the rendered Docker Hub assets committed here. The source SVG
and its `cairosvg` regeneration notes live in the separate (unpublished) branding
workspace, not in this repository.

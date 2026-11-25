# NTN patches on 5G-LENA nr

The toolkit's `contrib/nr` is 5G-LENA **v5.2** (20 November 2026, on ns-3.49) with the
toolkit's NTN changes applied.

- `ntn-toolkit-on-5g-lena-v5.2.patch` is the complete, current delta: `git diff v5.2` of
  this directory against an upstream `cttc-lena/nr` checkout of tag `v5.2`. Applying it to
  a clean v5.2 checkout reproduces the shipped `contrib/nr`.
- The numbered files (`05-...` to `22-...`) are the historical series, one patch per
  change, as each was first written against the earlier bases (v3.3.1 on ns-3.43, then
  v5.1 on ns-3.48). They document what each change does and why; they do not apply
  directly to v5.2.

Changes in the move to v5.2:

- Patch 11 (control messages ordered by type) is superseded upstream: v5.2 keeps the
  order in which the messages were produced, which removes the same run-to-run
  nondeterminism.
- Patch 23 (new): the direct-path UE combining weights are conjugated. Since ns-3.48
  (#1314) the receive side of the 3GPP channel combines with the conjugate of its
  weights, so the transmit-style direct-path weights are right for the gNB and must be
  conjugated for the UE. Without it a multi-element UE array lost array gain as the
  satellite moved (measured on a LEO pass with a 1x2 UE: 21 to 9 dB over 19 s).
- Patches 16 and 17 (PHR and BSR MAC CE reservation) now sit on v5.2's transport-block
  layout, including its STATUS-PDU admission.
- The UL HARQ fix (every NACK is visited; a process changes state only when its
  retransmission is placed) now sits on v5.2's retransmission spans and reshaping.

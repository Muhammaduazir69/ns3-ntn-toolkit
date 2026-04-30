# ntn-traffic — NTN-oriented traffic generators for ns-3

`ntn-traffic` provides traffic generators with NTN-appropriate
default parameters: CBR, NRTV over TCP/UDP, 3GPP HTTP over a
satellite bent-pipe / regenerative link, and a `TrafficTimeTag` for
per-packet latency tracking.

- ns-3 version: `release ns-3.43`
- Version: `1.0.0`
- License: GPL-2.0-only
- Maintainer: Muhammad Uzair (ORCID 0009-0002-4104-2680)

> **Name change:** the internal name of this module used to be
> `traffic`. It was renamed to `ntn-traffic` before the App Store
> submission because the unqualified name `traffic` is ambiguous in
> a public catalogue.

## Quick start

```bash
cd ns-3-dev
# ntn-traffic ships only inside the umbrella toolkit:
git clone --depth=1 https://github.com/Muhammaduazir69/ns3-ntn-toolkit /tmp/ntn-toolkit
cp -r /tmp/ntn-toolkit/contrib/ntn-traffic contrib/ntn-traffic
./ns3 configure --enable-examples --enable-tests \
    --enable-modules=ntn-traffic
./ns3 build
./ns3 run "nrtv-p2p-example"
./ns3 test --suite=ntn-traffic
```

## What's in the module

```
model/    — cbr-application, nrtv-{header,tcp-client,tcp-server,
            udp-server,variables,video-worker},
            three-gpp-http-satellite-{client,variables},
            traffic-time-tag
helper/   — traffic helper
examples/ — nrtv-p2p-example, nrtv-variables-plot,
            three-gpp-http-example
test/     — 2 suites
```

## License

GPL-2.0-only. See `LICENSE`.

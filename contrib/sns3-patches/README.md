# SNS3 satellite module: pinned base and toolkit patch

The toolkit builds against the SNS3 `satellite` module (https://github.com/sns3/sns3-satellite), which this
repository does not vendor because its data corpus is large. This directory makes the build reproducible:
it pins the exact upstream base and carries the toolkit's changes as a patch.

```bash
# from the ns-3 root
contrib/sns3-patches/fetch-sns3.sh            # code + data (about 2 GB)
contrib/sns3-patches/fetch-sns3.sh --no-data  # code only, enough to build
```

| Item | Value |
|---|---|
| Base | sns3-satellite `165b49152a25195d3ee04d77699a7bc184fb9ddf` (21 Jul 2025) |
| Data | sns3-data at the commit the base pins as its `data` submodule (`4200675f`) |
| Patch | `01-sns3-ntn-toolkit.patch`, 65 files |

## What the patch changes

- Module dependency: SNS3 links and includes the companion `traffic` module; the toolkit's `ntn-traffic`
  module provides it here. `CMakeLists.txt` (`${libtraffic}` to `${libntn-traffic}`), 58 examples and one
  test (`traffic-module.h` to `ntn-traffic-module.h`).
- `model/satellite-frame-allocator.cc`, `model/satellite-bbframe-container.cc`: shuffles use
  `std::random_shuffle` where the base uses `std::shuffle` with a default-seeded `std::default_random_engine`.
  This is the state every published toolkit result was produced with; it is kept so those results reproduce.
- `stats/satellite-stats-resources-granted-helper.cc/.h`: the disconnect/reconnect of a terminal's collector
  on handover, written out once per collector type in the base, is one path that selects the collector's
  trace sink through a member pointer.
- `helper/satellite-helper.cc`: one statement that silences an unused-value warning.

## Verification

`fetch-sns3.sh --no-data` followed by `diff -rq --exclude=data` against the tree the toolkit is built and
tested with: 0 mismatches (17 Nov 2026). The data files match sns3-data at the pinned commit (same file
set; upstream stores some text files with CRLF line endings).

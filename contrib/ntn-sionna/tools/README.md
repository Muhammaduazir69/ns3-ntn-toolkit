# ntn-sionna tools (Roadmap §4.2.9 + §4.2.10)

Two standalone Python 3 CLIs that prepare external geospatial data for
the `ntn-sionna` channel pipeline. Both run on minimal stdlib installs
and degrade gracefully when optional deps (`rasterio`, `laspy`) are
missing.

## `osm_to_sionna_scene.py` (§4.2.9)

Converts an OpenStreetMap XML dump into a Sionna RT scene description.

```
python3 osm_to_sionna_scene.py \
    --osm city.osm.xml --ref-lat 52.52 --ref-lon 13.405 \
    --out berlin
```

Produces `berlin.xml` (Sionna RT scene with buildings + roads in ENU
metres) and `berlin.mtl` (Mitsuba material library). Buildings come
from OSM `building=*` ways; roads from `highway=*` ways with
3GPP-default lane widths.

## `lidar_dem_ingest.py` (§4.2.10)

Merges JAXA AW3D30 global DEM with USGS 3DEP LiDAR ground points into
a unified row-major elevation grid (`.gridbin`).

```
python3 lidar_dem_ingest.py \
    --aw3d30 N52E013.tif --lidar berlin_2023.las \
    --bounds 52.50,13.39,52.55,13.45 --res 30 \
    --out berlin_grid.bin
```

Coverage rules: AW3D30 fills the grid first; LiDAR ground points
(class 2) override individual cells with the per-cell median.

## Tests

```
python3 -m pytest tools/tests/ -q
```

10 unit tests cover OSM parsing, ENU projection, scene XML/MTL writes,
grid sizing, binary round-trip, and LiDAR override behaviour.

# Offline maps for ESP32-S3

## Supported formats

The firmware supports two offline raster tile layouts:

1. **MBTiles** (recommended): one SQLite database per map, stored at `/maps/<name>.mbtiles`.
2. **Legacy XYZ PNG folders**: `/maps/<name>/<z>/<x>/<y>.png`.

The existing Leaflet UI continues to request `/tiles?map=<name>&z={z}&x={x}&y={y}`. The firmware detects an MBTiles archive first and falls back to the legacy PNG directory when no matching archive exists.

MBTiles lookup uses the standard `tiles` table and converts Leaflet XYZ Y to MBTiles TMS row:
`tile_row = (2^z - 1) - y`.

Raster PNG and JPEG tiles are supported. Vector MBTiles (`format=pbf`) are **not** rendered: they require a vector-tile renderer and style, which the current Leaflet raster layer does not provide. PMTiles is not implemented.

## microSD layout

Format the card as FAT32 and create:

```text
/maps/world.mbtiles
/maps/europe.mbtiles
/maps/europe_satellite.mbtiles
```

Each file is a separate selectable map in the existing map selector. Keep individual files below the FAT32 4 GiB file-size limit. The SD card is mounted read-only for map access; the firmware opens each archive with SQLite read-only mode and keeps one database/statement open for repeated tile requests.

## Recommended 32 GB map set

A practical balance is:
- `world.mbtiles`: OSM-style raster overview, zoom 0–6 (or 0–7 if the generated archive remains small).
- `europe.mbtiles`: detailed OSM raster basemap, zoom 7–12 for travel/navigation across Europe.
- `europe_satellite.mbtiles`: optional satellite imagery plus labels, only for a smaller chosen area and only from a provider/license that explicitly permits offline export. Satellite imagery for the entire world at zoom 10–16 will not fit in 32 GB.

For a true hybrid, combine satellite imagery with a transparent labels/roads overlay on the desktop during tile generation, producing one raster MBTiles file. This avoids requiring a second Leaflet layer and extra rendering on the ESP32. If keeping separate layers, firmware/UI work is needed to expose an overlay layer.

## Data source and licensing

- OpenStreetMap data can be used to create a self-hosted raster map. Include visible attribution: © OpenStreetMap contributors. Do not bulk-download the public `tile.openstreetmap.org` tile service for offline use; use a self-hosted renderer or a provider that explicitly permits offline downloads.
- Satellite imagery must come from a provider that grants offline export for the intended area and use. Do not scrape Esri/Google/Yandex public tile endpoints. ArcGIS export availability and terms depend on the account/service.

## Build notes

PlatformIO dependency: `siara-cc/Sqlite3Esp32@^2.5`.
SQLite accesses SD_MMC via the ESP32 VFS mount point `/sdcard`. Tile lookup is indexed by the standard MBTiles key (zoom, column, row). Cache size is deliberately small to limit RAM use on ESP32-S3.

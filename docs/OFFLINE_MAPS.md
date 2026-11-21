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
/maps/central_russia.mbtiles
/maps/local_detail.mbtiles
```

Each file is a separate selectable map in the existing map selector. Keep individual files below the FAT32 4 GiB file-size limit. The SD card is mounted read-only for map access; the firmware opens each archive with SQLite read-only mode and keeps one database/statement open for repeated tile requests.

## Recommended 32 GB map set

A practical balance is:
- `world.mbtiles`: OpenAndroMaps world raster overview, zoom 1–9 (about 1.5 GB).
- `central_russia.mbtiles`: custom raster OSM map covering European Russia from the western border through Volga and to the Ural Mountains, target zoom 7–13.
- `local_detail.mbtiles`: optional zoom 14–16 for a smaller frequently used area (for example, one region/city/field site).
- A satellite/hybrid layer should be limited to a smaller selected area and sourced only from a provider/license that explicitly permits offline export. A satellite map of the entire central part of Russia at zoom 10–16 is not a realistic 32 GB target.

For a true hybrid, combine satellite imagery with a transparent labels/roads overlay on the desktop during tile generation, producing one raster MBTiles file. This avoids requiring a second Leaflet layer and extra rendering on the ESP32. If keeping separate layers, firmware/UI work is needed to expose an overlay layer.

## Concrete map recommendations for central Russia

- Global overview: `OAM-World-1-9-J80.mbtiles`, raster JPG, zoom 1–9, about 1.5 GB. Official download index: https://www.openandromaps.org/en/downloads/general-maps (manual download link is listed on the page).
- OAM also publishes a Europe-large raster MBTiles at zoom 9–12 (`OAM-W1-8-EPlarge9-12-J80.mbtiles`, about 7.9 GB). Its coverage reaches toward the western Urals, but the archive is larger than the FAT32 4 GiB per-file limit and its coverage boundary should be checked before treating it as the complete east-to-Urals regional map.
- For a complete regional coverage up to the Ural range, Geofabrik provides current OSM PBF extracts for Central, Northwestern, Volga and Ural federal districts. These are source vector datasets, **not ready-to-display raster MBTiles**. They must be rendered on a PC (for example with QGIS' XYZ/MBTiles tile export) into raster MBTiles, zoom 7–13. Sources: https://download.geofabrik.de/russia.html and https://download.geofabrik.de/russia/ural-fed-district.html.
- Keep each resulting MBTiles file below 4 GiB if the SD card is FAT32. If a single regional export would exceed that, split it into adjacent western/eastern or northern/southern MBTiles files. The existing map selector can expose each file independently.
- Zoom 14–16 should be generated only for a much smaller local area. Rendering the full Central Russia footprint at zoom 16 would waste storage and take a very long time.

## Data source and licensing

- OpenStreetMap data can be used to create a self-hosted raster map. Include visible attribution: © OpenStreetMap contributors. Do not bulk-download the public `tile.openstreetmap.org` tile service for offline use; use a self-hosted renderer or a provider that explicitly permits offline downloads.
- Satellite imagery must come from a provider that grants offline export for the intended area and use. Do not scrape Esri/Google/Yandex public tile endpoints. ArcGIS export availability and terms depend on the account/service.

## Build notes

PlatformIO dependency: `siara-cc/Sqlite3Esp32@^2.5`.
SQLite accesses SD_MMC via the ESP32 VFS mount point `/sdcard`. Tile lookup is indexed by the standard MBTiles key (zoom, column, row). Cache size is deliberately small to limit RAM use on ESP32-S3.

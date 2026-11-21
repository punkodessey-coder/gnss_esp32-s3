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

The optional Windows flasher copies the selected databases into:

```text
/maps/world.mbtiles
/maps/european_russia.mbtiles
```

Each file appears as a separate selectable map in the existing map selector. The card must be visible to Windows through a PC card reader during copying. The flasher does not format the card or remove other files.

Use FAT32. Both selected files are individually below the 4 GiB FAT32 file-size limit. The SD card is mounted read-only for map access; the firmware opens each archive with SQLite read-only mode and keeps one database/statement open for repeated tile requests.

## Selected map package for European Russia

- `world.mbtiles`: OpenAndroMaps World, raster JPG, zoom 1–9, about 1.5 GB.
- `european_russia.mbtiles`: OpenAndroMaps Europe-MENA regional raster MBTiles, zoom 1–11 (regional detail zoom 8–11), about 3.57 GB.

The regional file is the best ready-made single-file raster package selected for European Russia and remains below FAT32's 4 GiB per-file limit. Its coverage is the provider-defined Europe-MENA region, not a custom boundary ending exactly at the Ural Mountains; check the provider's coverage overview for the far eastern edge. These are OSM-style road/topographic maps, not satellite imagery or a satellite hybrid.

The repository contains a PC-side staging folder `maps/` with `download_maps.ps1`. Run it manually to download the two databases, or choose option 2 in `flash_windows.bat` to download missing files and copy them to microSD during the flashing workflow. Option 1 flashes firmware only.

## Data source and licensing

- OpenStreetMap data can be used to create a self-hosted raster map. Include visible attribution: © OpenStreetMap contributors. Do not bulk-download the public `tile.openstreetmap.org` tile service for offline use; use a self-hosted renderer or a provider that explicitly permits offline downloads.
- Satellite imagery must come from a provider that grants offline export for the intended area and use. Do not scrape Esri/Google/Yandex public tile endpoints. ArcGIS export availability and terms depend on the account/service.

## Build notes

PlatformIO dependency: `siara-cc/Sqlite3Esp32@^2.5`.
SQLite accesses SD_MMC via the ESP32 VFS mount point `/sdcard`. Tile lookup is indexed by the standard MBTiles key (zoom, column, row). Cache size is deliberately small to limit RAM use on ESP32-S3.

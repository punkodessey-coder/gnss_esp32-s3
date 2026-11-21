# Offline map files for ESP32-S3

This folder is the **PC-side staging folder**. The firmware flasher can optionally copy its MBTiles files to the microSD card.

## Included map package

Run `download_maps.ps1` to fetch the two raster MBTiles databases:

| File on microSD | Source package | Zoom | Approx. size |
|---|---|---:|---:|
| `world.mbtiles` | OpenAndroMaps World | 1–9 | 1.5 GB |
| `european_russia.mbtiles` | OpenAndroMaps Europe-MENA regional map | 1–11 (regional detail 8–11) | 3.57 GB |

The regional Europe-MENA map is the ready-made raster package selected as the best available one-file regional coverage for European Russia. Its source package is 3,568,889,856 bytes (below the FAT32 4 GiB per-file limit). Coverage is a provider-defined Europe-MENA region, not a custom polygon ending exactly at the Urals; check the coverage overview for the far eastern edge before relying on it for a specific route.

Official map catalogue and coverage: https://www.openandromaps.org/en/downloads/general-maps
Direct file index: https://osm.mathematik.tu-darmstadt.de/openstreetmap/openandromaps/world/

The maps are JPG raster tiles inside SQLite MBTiles. They are compatible with the firmware's current raster tile handler. These are **not satellite/hybrid imagery**; they are OSM-style road/topographic overview maps.

## Download

From the repository root in PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File .\\maps\\download_maps.ps1
```

The script saves the files into this folder and validates the SQLite file header. The large binary databases are intentionally not committed to GitHub; download them locally instead.

## microSD

The flasher copies the two files into:

```text
X:\maps\world.mbtiles
X:\maps\european_russia.mbtiles
```

Replace `X:` with the actual microSD drive letter. The microSD must be inserted into a PC card reader during the copy step. The flasher does not format the card or delete existing files.

Use FAT32. Both selected databases are individually below the FAT32 4 GiB file-size limit. Keep free space for GNSS logs and other files.

## Attribution and license

OpenAndroMaps map content is based on OpenStreetMap and other cited sources; retain the attribution and licensing notices supplied by the provider. OSM attribution: © OpenStreetMap contributors. See the provider's terms and source information before redistribution.

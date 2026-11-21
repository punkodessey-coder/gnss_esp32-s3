# Downloads Leaflet 1.9.4 into maps\lib so it can be copied to the microSD card (X:\lib).
# The ESP32 Wi-Fi access point has no internet, so the web UI loads Leaflet from the card.
$ErrorActionPreference = 'Stop'
$base = 'https://unpkg.com/leaflet@1.9.4/dist'
$root = Join-Path $PSScriptRoot 'lib'
$files = @(
    'leaflet.js', 'leaflet.css',
    'images/layers.png', 'images/layers-2x.png',
    'images/marker-icon.png', 'images/marker-icon-2x.png', 'images/marker-shadow.png'
)
New-Item -ItemType Directory -Force -Path (Join-Path $root 'images') | Out-Null
foreach ($file in $files) {
    $target = Join-Path $root ($file -replace '/', '\')
    Write-Host "Downloading $file"
    Invoke-WebRequest -Uri "$base/$file" -OutFile $target -UseBasicParsing
    if ((Get-Item -LiteralPath $target).Length -lt 100) { throw "Downloaded file is too small: $file" }
}
Write-Host "[OK] Leaflet saved to $root" -ForegroundColor Green

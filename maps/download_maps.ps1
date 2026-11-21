$ErrorActionPreference = 'Stop'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

$MapDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Maps = @(
    @{
        Name = 'world.mbtiles'
        Url = 'https://ftp2.gwdg.de/pub/misc/openstreetmap/openandromaps/world/OAM-World-1-9-J80.mbtiles'
        MinBytes = 1000000000
        Description = 'World OSM-style overview, zoom 1-9'
    },
    @{
        Name = 'european_russia.mbtiles'
        Url = 'https://ftp2.gwdg.de/pub/misc/openstreetmap/openandromaps/world/OAM-W1-7-EPMENA8-11-J80.mbtiles'
        MinBytes = 2500000000
        Description = 'Europe + Middle East + North Africa regional map, zoom 8-11'
    }
)

function Test-SqliteFile([string]$Path) {
    if (!(Test-Path -LiteralPath $Path)) { return $false }
    $stream = [System.IO.File]::OpenRead($Path)
    try {
        if ($stream.Length -lt 1000000) { return $false }
        $header = New-Object byte[] 16
        $read = $stream.Read($header, 0, 16)
        $magic = [System.Text.Encoding]::ASCII.GetString($header, 0, $read)
        return $magic.StartsWith('SQLite format 3')
    } finally {
        $stream.Dispose()
    }
}

Write-Host ''
Write-Host 'Offline map package for ESP32-S3' -ForegroundColor Cyan
Write-Host "Destination: $MapDir"
Write-Host 'The files are raster MBTiles and are read directly by the firmware.'
Write-Host ''

foreach ($map in $Maps) {
    $target = Join-Path $MapDir $map.Name
    if (Test-SqliteFile $target) {
        Write-Host "[OK] $($map.Name) already exists; keeping it." -ForegroundColor Green
        continue
    }

    if (Test-Path -LiteralPath $target) {
        Remove-Item -LiteralPath $target -Force
    }

    $partial = "$target.download"
    if (Test-Path -LiteralPath $partial) { Remove-Item -LiteralPath $partial -Force }

    Write-Host "Downloading $($map.Description)" -ForegroundColor Yellow
    Write-Host "URL: $($map.Url)"
    try {
        Start-BitsTransfer -Source $map.Url -Destination $partial -DisplayName "ESP32 map: $($map.Name)" -Priority Foreground
    } catch {
        Write-Warning "BITS transfer failed: $($_.Exception.Message)"
        if (Test-Path -LiteralPath $partial) { Remove-Item -LiteralPath $partial -Force }
        Invoke-WebRequest -Uri $map.Url -OutFile $partial -UseBasicParsing
    }

    if (!(Test-Path -LiteralPath $partial)) {
        throw "Download did not create file: $partial"
    }
    $length = (Get-Item -LiteralPath $partial).Length
    if ($length -lt $map.MinBytes -or !(Test-SqliteFile $partial)) {
        Remove-Item -LiteralPath $partial -Force
        throw "Downloaded file failed validation: $($map.Name), size=$length bytes"
    }

    Move-Item -LiteralPath $partial -Destination $target -Force
    Write-Host "[OK] Saved $($map.Name) ($([math]::Round($length / 1GB, 2)) GB)" -ForegroundColor Green
}

Write-Host ''
Write-Host 'Both map databases are ready in the maps folder.' -ForegroundColor Green
Write-Host 'They will be copied to microSD by flash_windows.bat if you select that option.'

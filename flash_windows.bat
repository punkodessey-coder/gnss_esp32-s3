@echo off
setlocal EnableExtensions EnableDelayedExpansion
title ESP32-S3 firmware flasher

echo ==========================================
echo   ESP32-S3 GNSS firmware flasher
echo ==========================================
echo.
echo 1. Flash firmware only
echo 2. Flash firmware and copy offline maps to microSD
echo 0. Exit
echo.
choice /C 120 /N /M "Select mode [1/2/0]: "
if errorlevel 3 exit /b 0
if errorlevel 2 (
  set "COPY_MAPS=1"
) else (
  set "COPY_MAPS=0"
)

if not exist "%~dp0bootloader.bin" (
  echo [ERROR] bootloader.bin not found next to this script.
  pause
  exit /b 1
)
if not exist "%~dp0partitions.bin" (
  echo [ERROR] partitions.bin not found next to this script.
  pause
  exit /b 1
)
if not exist "%~dp0firmware.bin" (
  echo [ERROR] firmware.bin not found next to this script.
  pause
  exit /b 1
)

if "%COPY_MAPS%"=="1" (
  set "NEED_MAPS=0"
  if not exist "%~dp0maps\world.mbtiles" set "NEED_MAPS=1"
  if not exist "%~dp0maps\european_russia.mbtiles" set "NEED_MAPS=1"
  if "!NEED_MAPS!"=="1" (
    echo.
    echo One or both offline map files are missing. Downloading them now...
    powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0maps\download_maps.ps1"
    if errorlevel 1 (
      echo [ERROR] Map download failed. Firmware has not been flashed.
      pause
      exit /b 1
    )
  )
  if not exist "%~dp0maps\world.mbtiles" (
    echo [ERROR] world.mbtiles is missing.
    pause
    exit /b 1
  )
  if not exist "%~dp0maps\european_russia.mbtiles" (
    echo [ERROR] european_russia.mbtiles is missing.
    pause
    exit /b 1
  )
  echo.
  echo Insert the microSD card into a PC card reader.
  echo Do NOT leave it only inside the ESP32 board: Windows must see it as a drive.
  set /p SDLETTER=Enter microSD drive letter only, for example E: 
  set "SDLETTER=!SDLETTER::=!"
  set "SDLETTER=!SDLETTER:\=!"
  if "!SDLETTER!"=="" (
    echo [ERROR] microSD drive letter is required.
    pause
    exit /b 1
  )
  if not exist "!SDLETTER!:\NUL" (
    echo [ERROR] Drive !SDLETTER!: was not found.
    pause
    exit /b 1
  )
  echo.
  echo Destination: !SDLETTER!:\maps
  echo The script will copy/overwrite only world.mbtiles and european_russia.mbtiles.
  echo It will NOT format the card or delete other files.
  choice /C YN /N /M "Continue with this microSD drive? [Y/N]: "
  if errorlevel 2 exit /b 0
)

where py >nul 2>nul
if errorlevel 1 (
  echo [ERROR] Python is not installed or 'py' is unavailable.
  echo Install Python from https://www.python.org/downloads/
  pause
  exit /b 1
)
py -m esptool --help >nul 2>nul
if errorlevel 1 (
  echo esptool is not installed. Installing...
  py -m pip install --upgrade esptool
  if errorlevel 1 (
    echo [ERROR] Failed to install esptool.
    pause
    exit /b 1
  )
)

echo.
echo Connect ESP32-S3 by USB cable.
echo If the board is not detected: hold BOOT, briefly press RESET, then release BOOT.
echo.
set /p PORT=Enter COM port, for example COM5: 
if "%PORT%"=="" (
  echo [ERROR] COM port is required.
  pause
  exit /b 1
)

echo.
echo Flashing ESP32-S3 on %PORT%...
echo.
py -m esptool --chip esp32s3 --port %PORT% --baud 460800 --before default-reset --after hard-reset write-flash 0x0 "%~dp0bootloader.bin" 0x8000 "%~dp0partitions.bin" 0x10000 "%~dp0firmware.bin"
if errorlevel 1 (
  echo.
  echo [ERROR] Flashing failed.
  echo Try holding BOOT, press RESET, release BOOT, then run this script again.
  pause
  exit /b 1
)

echo.
echo Firmware flashed successfully.

if "%COPY_MAPS%"=="1" (
  echo.
  echo Copying offline maps to microSD...
  if not exist "!SDLETTER!:\maps" mkdir "!SDLETTER!:\maps"
  robocopy "%~dp0maps" "!SDLETTER!:\maps" world.mbtiles european_russia.mbtiles /Z /R:2 /W:3
  set "ROBOCODE=!ERRORLEVEL!"
  if !ROBOCODE! GEQ 8 (
    echo [ERROR] Map copy failed. Robocopy code: !ROBOCODE!
    echo Firmware is flashed, but maps were not fully copied.
    pause
    exit /b 1
  )
  echo [OK] Offline maps copied to !SDLETTER!:\maps
)

echo.
echo Reset the board, connect to Wi-Fi ESP32-GPS and open http://192.168.4.1
echo If the microSD was copied successfully, the map selector will list both maps.
echo.
pause

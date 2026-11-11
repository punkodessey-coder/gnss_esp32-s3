@echo off
setlocal enabledelayedexpansion

title ESP32-S3 firmware flasher

echo ==========================================
echo   ESP32-S3 GNSS firmware flasher
echo ==========================================
echo.

if not exist "bootloader.bin" (
  echo [ERROR] bootloader.bin not found next to this script.
  pause
  exit /b 1
)

if not exist "partitions.bin" (
  echo [ERROR] partitions.bin not found next to this script.
  pause
  exit /b 1
)

if not exist "firmware.bin" (
  echo [ERROR] firmware.bin not found next to this script.
  pause
  exit /b 1
)

echo Found required files:
echo   bootloader.bin
echo   partitions.bin
echo   firmware.bin
echo.

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

py -m esptool --chip esp32s3 --port %PORT% --baud 460800 --before default-reset --after hard-reset write-flash 0x0 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin

if errorlevel 1 (
  echo.
  echo [ERROR] Flashing failed.
  echo Try holding BOOT, press RESET, release BOOT, then run this script again.
) else (
  echo.
  echo Firmware flashed successfully.
  echo Reset the board, connect to Wi-Fi ESP32-GPS and open http://192.168.4.1
)

echo.
pause

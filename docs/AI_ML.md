# AI / ML subsystem

This branch adds a local GNSS/IMU monitoring baseline to the ESP32-S3 firmware. It runs without LoRa, cloud services, or an external computer.

## What is implemented

- **Online anomaly baseline:** a small Welford running-statistics model compares incoming feature values with the device's recent baseline. It uses HDOP, satellites used, average NMEA GSV SNR (when present), accelerometer-magnitude deviation from gravity, and gyro magnitude.
- **GNSS degradation warning:** a simple trend rule flags a simultaneous HDOP increase and GSV SNR decrease. It is an early-warning heuristic, not a probability forecast.
- **GNSS consistency check:** compares the displacement-derived speed between successive fixes with the receiver-reported speed. A mismatch is an integrity warning; it is not proof of spoofing.
- **IMU activity labels:** conservative rules classify GNSS-indicated movement, vibration-like acceleration, rotation, low dynamics, or unavailable/uncertain IMU. QMI8658 has no magnetometer, so this does not estimate absolute heading.
- **Web UI:** the AI / ML tab exposes model state, sample count, profile conformity score, anomaly code, GNSS consistency warning, motion label, GSV SNR, and short-term trends.
- **Optional CSV dataset logging:** start/stop logging from the AI / ML tab. Rows are written approximately once per second to `/ai_dataset.csv` on microSD and can be downloaded from `/ai/dataset`.

## Important limitations

The current on-device model is a lightweight online statistical anomaly detector, **not a trained neural network** and not an ESP-DL/TensorFlow Lite model. The score is a heuristic profile-conformity indicator, not a calibrated probability or certified navigation-integrity metric. The baseline is held in RAM and is reset at reboot; the CSV log remains on the microSD card.

NMEA GSV SNR is a coarse signal-strength field and is not the same as calibrated C/N0 measurements from a receiver's raw telemetry. Some receivers omit SNR, in which case that feature is skipped.

With the current NEO-6M + QMI8658 data path, the firmware cannot inspect raw RF/IQ samples, identify the physical source of interference, or reliably distinguish jamming from masking/multipath. The GNSS position-speed consistency check is only a simple anomaly cue. It does not provide reliable spoofing detection. The IMU activity label is not a full motion classifier and cannot provide long-term inertial navigation.

## How to use

1. Flash the firmware from this branch and connect to the device Wi-Fi AP.
2. Open `http://192.168.4.1`, then select **AI / ML**.
3. Keep a valid GNSS fix for at least 12 one-second samples to initialize the GNSS baseline.
4. If a microSD card is mounted, press **Начать запись CSV** to collect a dataset. Stop logging before removing the card.
5. Download `ai_dataset.csv` from the same tab. The file can be used on a PC to inspect operating conditions and prepare a future supervised/unsupervised model.

No LoRa code or radio configuration is added by this branch.

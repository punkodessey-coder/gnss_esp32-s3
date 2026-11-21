# GNSS ESP32-S3 — BLE, IMU и карты

Прошивка для Waveshare ESP32-S3-LCD-1.47B(-M): GNSS NMEA, ST7789, Wi-Fi AP, веб-интерфейс, BLE GATT, QMI8658 и карты.

## Подключение

| Узел | Сигнал | GPIO |
|---|---|---:|
| GNSS | TX модуля → RX ESP32 | 44 |
| GNSS | RX модуля ← TX ESP32 (опционально) | 43 |
| GNSS | UART | 57600 бод, 8N1 |
| ST7789 | MOSI / SCLK | 45 / 40 |
| ST7789 | CS / DC / RST / BL | 42 / 41 / 39 / 46 |
| QMI8658 | SDA / SCL | 48 / 47 |
| microSD | CLK / CMD / D0 | 14 / 15 / 16 |
| microSD | D1 / D2 / D3 | 18 / 17 / 21 |

Дисплей настроен на поворот 90°.

## Карты

При открытии страницы автоматически выбирается онлайн-карта OpenStreetMap. В меню доступны Esri Satellite, OpenTopoMap и Яндекс Гибрид (для последнего требуется API-ключ JavaScript API Яндекса, который задаётся в настройках карты). Для онлайн-тайлов интернет должен быть доступен на телефоне/планшете: точка доступа ESP32 сама не раздаёт интернет.

При наличии microSD карты из каталога `/maps` автоматически добавляются в тот же список. Их можно выбрать в любой момент; для них интернет не требуется.

Структура тайлов: `/maps/<имя_карты>/<z>/<x>/<y>.png`.

## Использование

1. Подключитесь к Wi-Fi `ESP32-GPS`, пароль `12345678`.
2. Откройте `http://192.168.4.1`.
3. Вкладка GPS показывает координаты; вкладка IMU — акселерометр и гироскоп, калибровку и сброс.
4. BLE-устройство называется `ESP32-GNSS`. Работа Web Bluetooth зависит от поддержки браузера.

## Сборка

PlatformIO: основная конфигурация `esp32s3`, тест экрана — `display_test`.

## AI / ML (feature/ai-ml-gnss)

- Added the **AI / ML** web tab with an online GNSS/IMU anomaly baseline, signal/HDOP trends, movement labels, and GNSS position-speed consistency warnings.
- Optional one-second CSV dataset logging to microSD: start/stop from the AI tab and download `/ai_dataset.csv`.
- The current model is a lightweight online statistical detector, not yet a trained neural network. See [docs/AI_ML.md](docs/AI_ML.md) for behavior, usage, and limitations.
- This branch does not include LoRa changes.

## Последние изменения в feature/gnss-config-manager

- Спутники, используемые в решении, берутся из GGA с проверкой качества фикса; видимые спутники отдельно считаются из GSV. Устаревшие значения сбрасываются.
- В u-blox Configuration View используются длины и поля UBX-CFG для поколения u-blox 6: CFG-MSG, CFG-PRT, CFG-RATE, CFG-NAV5, CFG-SBAS, CFG-NMEA, CFG-TP5 и CFG-CFG. CFG-GNSS отсутствует у NEO-6M; на странице GNSS указано ограничение GPS/SBAS.
- Маркер текущего положения отображается круговой точкой, а не стандартной меткой.
- Тайм-аут потери фикса настраивается в веб-интерфейсе от 1 до 60 секунд.
- Яндекс Гибрид загружается через официальный JavaScript API и требует API-ключ. Ключ можно сохранить в настройках карты.
- NMEA-имитация движется по замкнутому круговому маршруту радиусом около 20 м в центре Сарова за 60 секунд на круг и выдаёт обновляемые RMC/GGA/GSA/GSV/VTG.


## Offline maps (feature/offline-mbtiles)

- Existing Leaflet `/tiles` endpoint supports raster MBTiles (SQLite, JPG/PNG) and legacy XYZ PNG folders. See [docs/OFFLINE_MAPS.md](docs/OFFLINE_MAPS.md) for microSD layout and recommended map packages.

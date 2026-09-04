# Tab5 DXFT8 RF-card mock

This repository contains bench firmware for a YD-ESP32-23
(ESP32-S3-WROOM-1 N16R8) used in place of the planned Tab5 DXFT8 RF daughter
board.

The first milestone emulates the daughter board's Si5351 on I2C:

- seven-bit address `0x60`;
- address acknowledgement for an I2C scanner;
- a 256-byte shadow register file for writes;
- register-pointer and auto-increment behavior for writes;
- one-byte register reads; and
- transaction logging through the YD board's CH343 USB-to-UART port.

The project targets the locally installed ESP-IDF 5.5.1 slave-driver v2 API.
I2S/PCM1808 emulation will be added as a separate milestone.

## Immediate test: Tab5 factory demo Port A

The Tab5 factory demo initializes its external Grove/Port A I2C controller on
G53/G54 and enables its internal pull-ups. Use three wires only:

| Tab5 HY2.0-4P Port A | Signal | YD-ESP32-23 |
| --- | --- | --- |
| Black / GND | Ground | GND |
| Yellow / G53 | SDA | GPIO8 |
| White / G54 | SCL | GPIO9 |
| Red / 5 V | Power | **Do not connect** |

Power the Tab5 normally. Power the YD board separately through its CH343
USB-to-UART connector. The grounds must be connected, but the power rails must
not be tied together.

Open the factory demo's external I2C tester/scan. Address `0x60` should appear.
An address-only probe does not deliver a data byte, so it may not generate a
transaction line in the YD serial log; seeing `0x60` on the Tab5 is the test.

Factory-demo source showing external SDA=G53 and SCL=G54:
<https://github.com/m5stack/M5Tab5-UserDemo/blob/main/platforms/tab5/components/m5stack_tab5/m5stack_tab5.c>

## Final RF-card I2C wiring

The same YD firmware can later be moved to the RF-card M5-Bus signals:

| Tab5 M5-Bus | Signal | YD-ESP32-23 |
| --- | --- | --- |
| Pin 1 or pin 3 | Ground | GND |
| Pin 17 / P4 G31 | SDA | GPIO8 |
| Pin 18 / P4 G32 | SCL | GPIO9 |

The Tab5 system bus has 2.2 kOhm pull-ups to 3.3 V. Do not add another strong
pair for the final wiring. If a different standalone master has no pull-ups,
add removable 4.7 kOhm pull-ups from SDA and SCL to the YD board's 3.3 V rail,
or temporarily enable the weak ESP32-S3 pull-ups in `menuconfig`.

Official Tab5 pin map:
<https://docs.m5stack.com/en/core/Tab5>

## Build

```sh
source ~/esp/esp-idf/export.sh
cd ~/tab5_dxft8/rf_board_mock
idf.py set-target esp32s3
idf.py build
```

The pin and address defaults can be changed under:

```text
Component config
  -> Tab5 DXFT8 mock configuration
```

## Flash and monitor

Connect the YD board's CH343 USB-to-UART connector, identify its serial port,
then flash it:

```sh
ls /dev/cu.*
idf.py -p /dev/cu.YOUR_PORT flash monitor
```

Exit the monitor with `Ctrl-]`.

Expected startup output includes:

```text
Tab5 DXFT8 RF-card mock - I2C milestone
ready: address=0x60 (7-bit), SDA=GPIO8, SCL=GPIO9
Ready for the Tab5 external I2C scan
```

## Current readback limitation

ESP-IDF's I2C slave API reports that a master wants to read, but does not tell
the application the requested byte count. This first milestone deliberately
queues one byte per read request so unread response bytes cannot contaminate a
later transaction. It is suitable for address scans, Si5351 register writes,
and single-byte status reads. General multi-byte read emulation will be added
after observing the exact transaction pattern used by the Tab5 Si5351 driver.

The real Si5351 primarily receives configuration writes, so this limitation
does not block the factory-demo scan or the next firmware bring-up step.

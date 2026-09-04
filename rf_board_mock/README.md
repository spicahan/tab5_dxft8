# Tab5 DXFT8 RF-card mock

This repository contains bench firmware for a YD-ESP32-23
(ESP32-S3-WROOM-1 N16R8) used in place of the planned Tab5 DXFT8 RF daughter
board.

The firmware emulates the daughter board's Si5351 on I2C:

- seven-bit address `0x60`;
- address acknowledgement for an I2C scanner;
- a 256-byte shadow register file for writes;
- register-pointer and auto-increment behavior for writes;
- one-byte register reads;
- read-only, write-zero-to-clear, and self-clearing register behavior;
- validation and decoded-frequency logging for PLLA/B and CLK0/1; and
- transaction logging through the YD board's CH343 USB-to-UART port.

It also emulates the PCM1808 digital interface:

- I2S slave transmission driven by the Tab5's BCLK and LRCK;
- 48 kHz stereo Philips I2S, with 24 meaningful bits in 32-bit slots;
- deterministic blocks with complementary channel tags and one continuous
  modulo-65536 frame sequence; and
- independent pulse-counter measurement of the Tab5's 12.288 MHz MCLK. With
  the default configuration, valid pattern data is withheld until MCLK passes
  and revoked if MCLK is later lost.

The project targets the locally installed ESP-IDF 5.5.1 slave-driver v2 API.
The decoder defaults to the RF-board v1.3 schematic's active 26 MHz source,
AC-coupled into XA with XB unused. It expects register 183 = `0x12`, models PLL
lock status after reset strobes, and validates the actual board clock plan:

- CLK0: 7.074 MHz TX/PA clock from PLLA;
- CLK1: 28.296 MHz RX/QSD clock from PLLB;
- register 3 = `0xFF` for all off, `0xFD` for RX, and `0xFC` for TX-ready.

CLK1 deliberately remains enabled in the TX-ready state because the board's
QSD is hard-enabled; G47/G48 perform the actual RF-path switching. The mock
warns on unsafe initialization order, a TX-only `0xFE` clock state, enabled
unused outputs, missing PLL reset, wrong source/control registers, or invalid
PLL/MultiSynth parameters.

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

## PCM1808 I2S mock wiring

Leave the I2C wires on Port A. Add the following M5-Bus connections, with a
common ground and no connection between the boards' power rails:

| Tab5 M5-Bus | Direction | YD-ESP32-23 |
| --- | --- | --- |
| GND | — | GND |
| Pin 2 / P4 GPIO16 / MCLK | -> | GPIO4 / pulse-counter monitor |
| Pin 8 / P4 GPIO45 / BCLK | -> | GPIO5 / I2S BCLK input |
| Pin 19 / P4 GPIO3 / LRCK | -> | GPIO6 / I2S WS input |
| Pin 20 / P4 GPIO4 / DIN | <- | GPIO7 / I2S DOUT |

The production-compatible format is:

```text
Fs          48,000 Hz
sample      signed 24-bit, left-aligned in a 32-bit slot
channels    stereo (left then right)
frame       64 BCLK
MCLK        256fs = 12.288 MHz
BCLK        64fs  = 3.072 MHz
protocol    Philips I2S (one-bit delay after LRCK changes)
```

The S3 I2S peripheral does not consume MCLK in slave mode, so GPIO4 feeds a
pulse counter solely to verify that signal. Before the Tab5 starts its clocks,
the mock normally logs that it is waiting for MCLK and its I2S writer quietly
remains stopped. Those are idle states, not failures. The monitor samples
frequently, only enables the pattern after an in-tolerance MCLK measurement,
and changes to silence if MCLK is subsequently lost; therefore the Tab5 cannot
pass with the MCLK wire omitted, including after an earlier successful run.

With jumper wires, place optional 22–47 ohm series resistors at the driving
end: Tab5 for MCLK/BCLK/LRCK and YD GPIO7 for DOUT. Keep wires short and pair
the clock bundle with nearby ground conductors.

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
Tab5 DXFT8 RF-card mock - Si5351 + PCM1808 integration
ready: address=0x60 (7-bit), SDA=GPIO8, SCL=GPIO9
frequency decoder reference=26000000 Hz (active external reference on XA), register 183=0x12
PCM1808 I2S slave armed: BCLK=GPIO5, LRCK=GPIO6, DOUT=GPIO7
format: 48000 Hz, Philips I2S, stereo, 24 valid bits in 32-bit slots, 64 BCLK/frame
MCLK monitor: GPIO4, expected=12288000 Hz (+/-5%)
Ready for the Tab5 I2C and I2S host tests
```

## Current readback limitation

ESP-IDF's I2C slave API reports that a master wants to read, but does not tell
the application the requested byte count. This first milestone deliberately
queues one byte per read request so unread response bytes cannot contaminate a
later transaction. It is suitable for address scans, Si5351 register writes,
and single-byte status reads. General multi-byte read emulation can be added
when another host needs it.

The Tab5 host deliberately uses one-byte status/readback transactions, a
pattern also supported by the real Si5351, so this limitation does not reduce
the current integration test's coverage.

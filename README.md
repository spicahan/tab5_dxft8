# Tab5 DXFT8 firmware bring-up

This repository keeps the two sides of the bench setup separate:

- `rf_board_mock/` runs on the YD-ESP32-23 and emulates both the RF daughter
  board's Si5351A at I2C address `0x60` and its PCM1808 I2S output.
- `tab5/` runs on the M5Stack Tab5 and contains reusable Si5351 and PCM1808
  host drivers plus UART-only integration tests.

For the current Port A milestone, power both boards independently over USB and
connect only ground, SDA, and SCL:

| Signal | Tab5 Port A | YD-ESP32-23 |
| --- | --- | --- |
| GND | black | GND |
| SDA | yellow / GPIO53 | GPIO8 |
| SCL | white / GPIO54 | GPIO9 |
| 5 V | red | **not connected** |

The Tab5 test uses the RF-board v1.3 active 26 MHz reference coupled into the
Si5351 XA input. It validates ordinary register access, safe output
initialization, fractional synthesis, the 7.074 MHz transceiver clock plan,
readback, PLL reset behavior, and the board's clock-enable states:

- CLK0: 7.074 MHz TX/PA clock from PLLA.
- CLK1: 28.296 MHz RX/QSD clock from PLLB.
- Si5351 register 3: `0xFF` for all clocks off, `0xFD` for RX (CLK1 only),
  and `0xFC` for the TX-ready clock state (CLK0 and CLK1).

CLK1 remains running in the TX-ready clock state; the separate G47/G48 signals
perform the actual RF-path RX/TX control and are outside this I2C-only
milestone. The optional TX clock-path exercise is disabled by default. See each
subdirectory's README for build, flash, configuration, and safety details.

The active-XA default writes register 183 as `0x12` (0 pF load setting). Both
the reference frequency and input/load mode are configurable for development
modules that instead use a passive crystal. Do not carry the `0x12` setting
over blindly when changing the reference hardware.

## I2S bench wiring

Keep the existing I2C connection on Port A and add these five connections to
the Tab5 M5-Bus. Power both boards separately over USB; connect ground, but do
not connect their 5 V or 3.3 V rails.

| Signal | Tab5 M5-Bus / ESP32-P4 | Direction | YD-ESP32-23 mock |
| --- | --- | --- | --- |
| GND | pin 1, 3, or another GND | — | GND |
| MCLK / PCM1808 SCKI | pin 2 / GPIO16 | Tab5 -> mock | GPIO4 |
| BCLK | pin 8 / GPIO45 | Tab5 -> mock | GPIO5 |
| LRCK / WS | pin 19 / GPIO3 | Tab5 -> mock | GPIO6 |
| PCM1808 DOUT | pin 20 / GPIO4 | mock -> Tab5 | GPIO7 |

The shared wire format is 48 kHz stereo Philips I2S with 24 significant bits
left-aligned in each 32-bit slot: MCLK = 12.288 MHz (256fs), BCLK = 3.072 MHz
(64fs), and LRCK = 48 kHz. The mock emits a continuous self-checking pattern.
The Tab5 verifies channel order, the Philips one-bit delay, 24-bit alignment,
zero padding, and uninterrupted frame sequence. The mock separately counts
MCLK on GPIO4 and reports whether it is near 12.288 MHz; by default it withholds
valid pattern data until that clock passes and replaces the pattern with silence
if MCLK is later lost, so the Tab5 cannot pass without the MCLK connection.

On the real daughter board, that format assumes the PCM1808 hardware straps
are `MD1=0`, `MD0=0` (slave) and `FMT=0` (Philips I2S).

For flying-wire tests, keep all wires short and run ground next to the clock
signals. Optional 22–47 ohm series-damping resistors belong at each signal's
source: at the Tab5 end of MCLK/BCLK/LRCK and at the YD GPIO7 end of DOUT.
They are not pull-downs and must not be wired from a signal to ground.

## Bench validation

The complete Port A flow was built, flashed, and exercised on 2026-09-04 with
the YD-ESP32-23 mock and a Tab5. The Tab5 passed address probing, status reads,
all programmed-register readbacks, PLL-reset handling, and stable-lock checks.
The mock independently decoded PLLA/B at 792.288 MHz, CLK0 at 7.074 MHz, and
CLK1 at 28.296 MHz; it accepted the `0xFD -> 0xFC -> 0xFD` state sequence with
no validation errors or dropped transaction logs. The test ended in RX
(`0xFD`: CLK1 on, CLK0 off).

Register programming follows the manufacturer documentation:

- [Skyworks Si5351 data sheet](https://www.skyworksinc.com/-/media/SkyWorks/SL/documents/public/data-sheets/Si5351-B.pdf)
- [Skyworks AN619 register-calculation guide](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf)

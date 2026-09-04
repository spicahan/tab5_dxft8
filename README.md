# Tab5 DXFT8 firmware bring-up

This repository keeps the two sides of the bench setup separate:

- `rf_board_mock/` runs on the YD-ESP32-23 and emulates the RF daughter
  board's Si5351A at I2C address `0x60`.
- `tab5/` runs on the M5Stack Tab5 and contains the reusable Si5351 host
  driver plus a UART-only integration test.

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

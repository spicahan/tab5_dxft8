# Tab5 Si5351 host test

Native ESP-IDF 5.5.1 firmware for exercising the RF-board mock—and later a
real Si5351A—from the M5Stack Tab5. It is a UART-only bring-up program with no
GUI, M5GFX, M5Unified, Wi-Fi, or other managed-component dependency.

## Wiring for this milestone

Power the Tab5 and YD-ESP32-23 separately over USB. Connect only:

| Tab5 Port A | RF-board mock |
| --- | --- |
| Black / GND | GND |
| Yellow / GPIO53 / SDA | GPIO8 / SDA |
| White / GPIO54 / SCL | GPIO9 / SCL |
| Red / 5 V | **Not connected** |

The Tab5 is the I2C master on Port A: SDA is GPIO53 and SCL is GPIO54. The
default bus speed is 100 kHz and its weak internal pull-ups are enabled for the
short bring-up jumpers. Use proper external pull-ups on the production
daughter board.

## Build and run

```sh
source ~/esp/esp-idf/export.sh
cd ~/tab5_dxft8/tab5
idf.py set-target esp32p4
idf.py build
idf.py -p /dev/cu.usbmodem101 flash monitor
```

The deterministic startup test:

1. Probes address `0x60` and reads status registers 0 and 1.
2. Disables every output with Si5351 register 3 = `0xFF`, powers down the
   output drivers, and applies the configured XA reference mode.
3. Programs the actual 40 m plan for RF 7.074 MHz: PLLA/B at 792.288 MHz,
   TX CLK0 at 7.074 MHz from PLLA, and RX/QSD CLK1 at 28.296 MHz from PLLB.
4. Reads every persistent register back in separate one-byte transactions and
   checks PLL reset behavior, then requires three consecutive clean lock-status
   reads before enabling a clock output.
5. Enters RX with register 3 = `0xFD`: CLK1 on and CLK0 off.
6. When `CONFIG_DXFT8_RUN_TX_PATH_SELF_TEST` is enabled, briefly exercises
   `RX (0xFD) -> TX-ready clocks (0xFC) -> RX (0xFD)`. The QSD CLK1 remains
   running while CLK0 is added for the PA clock.
7. Finishes in RX with CLK1 enabled and CLK0 disabled.

Any failure requests all Si5351 outputs off and leaves the error visible in the
UART log. Register 3 output enables are active-low: `0xFF` disables all
outputs, `0xFD` enables CLK1 only, and `0xFC` enables CLK0 and CLK1. These are
clock states, not complete RF-path states. The daughter board's separate G47
and G48 controls perform the actual complementary RX/TX switching; exercising
those GPIOs is outside this I2C-only milestone.

For a mock-board acceptance run, enable
`CONFIG_DXFT8_RUN_TX_PATH_SELF_TEST` with `idf.py menuconfig`. Its default is
off: never enable this option with a powered PA or antenna connected. The test
only changes Si5351 clock enables; it does not assert G47/G48. The normal final
state is RX, never TX-ready.

## Reference input configuration

The RF-board v1.3 uses a 26 MHz active oscillator, AC-coupled into the Si5351
XA pin with XB left unused. Its default configuration is:

- `CONFIG_DXFT8_SI5351_REFERENCE_HZ=26000000`
- `CONFIG_DXFT8_SI5351_EXTERNAL_REFERENCE=y`
- register 183 = `0x12` (0 pF load field plus the required low reserved bits)

The `0x12` value is specific to the active-XA arrangement; it replaces the
common `0xD2` 10 pF passive-crystal setting. The reference frequency and
6/8/10 pF passive-crystal modes remain selectable with `idf.py menuconfig` for
other Si5351 modules. Match those settings to the hardware rather than merely
copying the v1.3 default. Because the active-XA case is less common than a
passive crystal, verify the oscillator amplitude, startup, and frequency on
the first assembled RF board.

## Manufacturer references

- [Skyworks Si5351 data sheet](https://www.skyworksinc.com/-/media/SkyWorks/SL/documents/public/data-sheets/Si5351-B.pdf)
- [Skyworks AN619 register-calculation guide](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf)

# AltiVario Hardware & Wiring Guide

## ESP32-S3 Version

### Wiring Diagram

```
                          ┌──────────────┐
                     3V3 ─┤ 3V3      VIN ├─ VBUS (USB 5V)
                     GND ─┤ GND      GND ├─ GND
    Battery ADC ◄─── R ──┤ GPIO 1    43 ├─
                          │              │
        I2S BCLK ────────┤ GPIO 4    44 ├─
       I2S LRCLK ────────┤ GPIO 5       │
        I2S DOUT ────────┤ GPIO 6       │
                          │              │
        I2C SDA ──┬──────┤ GPIO 8       │
        I2C SCL ──┬──────┤ GPIO 9       │
                  │  │    │              │
         SD CS ──────────┤ GPIO 10      │
       SD MOSI ──────────┤ GPIO 11      │
        SD SCK ──────────┤ GPIO 12      │
       SD MISO ──────────┤ GPIO 13      │
                          │              │
       GPS TX ───────────┤ GPIO 17 (RX) │
       GPS RX ◄──────────┤ GPIO 18 (TX) │
                          │              │
         Piezo ◄─────────┤ GPIO 38      │
                          │              │
  Boot Button ───────────┤ GPIO 0       │
                          └──────────────┘

  I2C Bus (GPIO 8/9, 400 kHz)
  ┌──────┐  ┌────────┐
  │MS5611│  │SSD1306 │
  │ 0x77 │  │ 0x3C   │
  │      │  │128x64  │
  └──┬─┬─┘  └──┬──┬──┘
     │ │        │  │
 SDA─┘ └─SCL   │  │
     │ │        │  │
     └─┼────────┘  │     4.7k        4.7k
       └───────────┘      │            │
       │           │      │            │
       ├───────────┼──────┘            │
       │           ├───────────────────┘
       │           │
      SDA         SCL ─── each pulled to 3V3

  Battery Voltage Divider (GPIO 1)
                 ┌─── VBAT (3.0–4.2 V)
                 │
                ┌┤┐
                │ │ R1 = 100k
                └┤┘
                 ├─── GPIO 1 (ADC)
                 │
                ┌┤┐  ┌───┐
                │ │ R2 = 100k
                └┤┘  │100n│ filter cap
                 │   └─┬─┘
                 ├─────┘
                GND

  Vadc = VBAT × R2 / (R1 + R2) = VBAT / 2
  At full charge: 4.2 V → 2.1 V (within ESP32 ADC range)

  MAX98357A I2S DAC
  ┌──────────┐
  │ MAX98357A│
  │          │
  │    BCLK ─┤◄── GPIO 4
  │   LRCLK ─┤◄── GPIO 5
  │     DIN ─┤◄── GPIO 6
  │     VIN ─┤◄── 3V3
  │     GND ─┤─── GND
  │  GAIN/SD ─┤─── leave floating (15 dB) or tie to GND (9 dB)
  └──────────┘
  44100 Hz, 16-bit mono, left channel
```

### Pin Table

| GPIO | Function        | Peripheral            | Notes                        |
|------|----------------|-----------------------|------------------------------|
| 0    | Button         | Boot button           | Internal pull-up             |
| 1    | ADC1_CH0       | Battery voltage       | 2:1 voltage divider          |
| 4    | I2S BCLK       | MAX98357A DAC         | Bit clock                    |
| 5    | I2S LRCLK      | MAX98357A DAC         | Word select                  |
| 6    | I2S DOUT       | MAX98357A DAC         | Data out                     |
| 8    | I2C SDA        | MS5611 + SSD1306      | 4.7k pull-up to 3V3         |
| 9    | I2C SCL        | MS5611 + SSD1306      | 4.7k pull-up to 3V3         |
| 10   | SPI CS         | MicroSD card          |                              |
| 11   | SPI MOSI       | MicroSD card          |                              |
| 12   | SPI SCK        | MicroSD card          |                              |
| 13   | SPI MISO       | MicroSD card          |                              |
| 17   | UART1 RX       | GPS NEO-6M TX         | 9600 baud, 3.3V logic       |
| 18   | UART1 TX       | GPS NEO-6M RX         | 9600 baud, 3.3V logic       |
| 38   | LEDC PWM       | Piezo buzzer          | Software tone generation     |

### Peripheral Notes

**I2C Bus (400 kHz)**
- MS5611 barometric sensor at address `0x77`
- SSD1306 128x64 OLED display at address `0x3C`
- Use 4.7k ohm pull-up resistors on SDA and SCL to 3V3
- Keep traces short; add 100nF decoupling caps at each device VCC

**SPI Bus (MicroSD)**
- Standard SPI wiring: CS, MOSI, SCK, MISO
- Use a MicroSD breakout with built-in level shifting and 3.3V regulator, or ensure the card module runs at 3.3V logic
- 100nF decoupling cap at card module VCC

**GPS (NEO-6M)**
- UART at 9600 baud, 8N1
- The NEO-6M runs at 3.3V logic natively; no level shifting needed when powered at 3.3V
- If using a 5V GPS module, add a voltage divider or level shifter on the TX→ESP32 RX line

**Audio**
- *Piezo mode:* Single piezo buzzer driven by LEDC PWM on GPIO 38. Volume controlled via duty cycle.
- *DAC mode:* MAX98357A I2S amplifier. 44.1 kHz sample rate, 16-bit mono. Sine wave generated in software. Connect a small 4-8 ohm speaker to the amplifier output.

**Battery Monitoring**
- 2:1 resistive voltage divider (2 x 100k ohm) from LiPo VBAT to GPIO 1
- 100nF ceramic capacitor from GPIO 1 to GND for ADC noise filtering
- Oversampled: 16 ADC readings averaged per measurement
- Thresholds: warn at 3.4V, stop logging at 3.2V, deep sleep at 3.0V

**Power Supply**
- Single-cell LiPo (3.7V nominal, 4.2V full, 3.0V cutoff)
- 3.3V LDO regulator (e.g., AMS1117-3.3 or MCP1700-3302)
- Schottky diode (e.g., SS14 / 1N5817) in series with battery positive for reverse polarity protection
- Total quiescent budget should stay under 5 mA to preserve battery in idle

### Component BOM

| Component             | Part / Spec                        | Qty |
|-----------------------|------------------------------------|-----|
| MCU                   | ESP32-S3 DevKit (N8R2 or similar)  | 1   |
| Barometric sensor     | MS5611 breakout (I2C, 3.3V)        | 1   |
| OLED display          | SSD1306 128x64 I2C (0.96")        | 1   |
| GPS module            | u-blox NEO-6M (3.3V)              | 1   |
| MicroSD breakout      | SPI, 3.3V logic                    | 1   |
| I2S amplifier         | MAX98357A breakout                 | 1   |
| Speaker               | 4-8 ohm, 1-3W                     | 1   |
| Piezo buzzer          | Passive, 3.3V compatible           | 1   |
| LiPo battery          | 3.7V, 1000-2000 mAh               | 1   |
| Voltage regulator     | AMS1117-3.3 or MCP1700-3302        | 1   |
| Schottky diode        | SS14 / 1N5817                      | 1   |
| Pull-up resistors     | 4.7k ohm                          | 2   |
| Voltage divider       | 100k ohm                          | 2   |
| Decoupling caps       | 100nF ceramic                     | 4-5 |
| ADC filter cap        | 100nF ceramic                     | 1   |

---

## Legacy Version (ATmega328P)

### Wiring Diagram

```
                          ┌──────────────┐
                    RESET─┤ 1   ATmega 28├─ VCC (3.3–5V)
                          │      328P    │
      Speaker 2 ◄────────┤ D2        A5 ├──── I2C SCL ──┐
      LED Red   ◄── R ───┤ D3        A4 ├──── I2C SDA ──┤
      LED Green ◄── R ───┤ D4           │               │
                          │              │    ┌──────┐   │
      NMEA LED  ◄── R ───┤ D6           │    │BMP085│   │
                          │              │    │ 0x77 │   │
      Speaker 1 ◄────────┤ D8           │    └──┬─┬─┘   │
                          │              │   SDA─┘ └SCL  │
                     GND ─┤ GND      GND ├       │   │   │
                          └──────────────┘       │   │   │
                                                 └───┼───┘
                                                     │
                                                    GND

  R = 220–330 ohm current-limiting resistor for each LED

  Dual Piezo Wiring (Volume Levels):
  - Volume 0: muted
  - Volume 1: Speaker 1 (D8) active, Speaker 2 (D2) held LOW
  - Volume 2: Both speakers active (+6 Hz offset for louder sound)

  Battery:
  - No external divider needed
  - Uses AVR internal 1.1V bandgap reference to measure VCC directly
```

### Pin Table

| Pin  | Function      | Peripheral          | Notes                         |
|------|--------------|---------------------|-------------------------------|
| D2   | Timer tone   | Piezo speaker 2     | Dual-piezo for louder volume  |
| D3   | Digital out  | Bicolor LED (red)   | 220-330 ohm series resistor   |
| D4   | Digital out  | Bicolor LED (green) | 220-330 ohm series resistor   |
| D6   | Digital out  | NMEA activity LED   | 220-330 ohm series resistor   |
| D8   | Timer tone   | Piezo speaker 1     | Primary speaker               |
| A4   | I2C SDA      | BMP085              | Internal or external pull-up  |
| A5   | I2C SCL      | BMP085              | Also used for I2C bus recovery|

### Peripheral Notes

**I2C Bus (100 kHz)**
- BMP085 barometric pressure sensor at address `0x77`
- Standard-mode I2C (100 kHz), use 4.7k ohm pull-ups if not on breakout board
- Bus recovery: SCL toggled 9 times on A5 if sensor becomes unresponsive

**Audio**
- Two passive piezo buzzers driven by AVR timer tone library
- Volume 1: single piezo on D8, D2 held LOW as ground reference
- Volume 2: both piezos active with +6 Hz frequency offset for a louder, richer sound

**LEDs**
- Bicolor LED (common cathode) on D3 (red) and D4 (green)
  - Green: battery good (>3.6V)
  - Orange: both on (battery warning, 3.4-3.6V)
  - Red: battery low (<3.4V)
- NMEA activity LED on D6: blinks when sending NMEA sentences
- All LEDs need 220-330 ohm current-limiting resistors

**Serial Output**
- Hardware UART at 9600 baud
- Outputs LK8EX1 and/or Flymaster F1 NMEA sentences (configurable)
- 3 sentences per second

**Battery Monitoring**
- No external voltage divider required
- Uses AVR internal 1.1V bandgap reference measured against VCC
- Formula: `VCC (mV) = 1126400 / ADC_reading`
- Thresholds: good >3.6V, warning <3.6V, low <3.4V

### Component BOM

| Component          | Part / Spec                     | Qty |
|--------------------|---------------------------------|-----|
| MCU                | ATmega328P (Arduino Pro Mini)   | 1   |
| Barometric sensor  | BMP085 breakout (I2C, 3.3V)    | 1   |
| Piezo buzzer       | Passive piezo                   | 2   |
| Bicolor LED        | Common cathode red/green        | 1   |
| NMEA LED           | Standard green or yellow LED    | 1   |
| LED resistors      | 220-330 ohm                    | 3   |
| Pull-up resistors  | 4.7k ohm (if no breakout)      | 2   |
| Decoupling caps    | 100nF ceramic                  | 2   |
| LiPo battery       | 3.7V single cell               | 1   |

---

## Recommended Passives & Protection

These components are not strictly required for basic operation but are strongly recommended for reliability:

| Component            | Value / Part   | Purpose                                          |
|----------------------|----------------|--------------------------------------------------|
| I2C pull-ups         | 4.7k ohm       | Required if not provided by breakout boards      |
| Decoupling caps      | 100nF ceramic  | Place at VCC pin of each IC and sensor module    |
| LED resistors        | 220-330 ohm    | Current limiting for all LEDs (Legacy)           |
| Schottky diode       | SS14 / 1N5817  | Reverse polarity protection on battery input     |
| ADC filter cap       | 100nF ceramic  | Reduces ADC noise on battery voltage reading     |
| AVR BOD fuse         | BODLEVEL 2.7V  | Prevents erratic operation at low battery (Legacy). Set fuses: `EXTENDED = 0x05` |

---

## Power Budget

### ESP32-S3 Version

| Component          | Active Current | Sleep/Idle     |
|--------------------|---------------|----------------|
| ESP32-S3           | ~80 mA        | ~5 mA (light)  |
| MS5611             | ~1.4 mA       | ~0.01 mA       |
| SSD1306 OLED       | ~20 mA        | ~0.01 mA (off) |
| NEO-6M GPS         | ~45 mA        | ~10 mA (PSM)   |
| MicroSD (writing)  | ~40 mA        | ~0.1 mA        |
| MAX98357A + speaker| ~10 mA        | ~0.01 mA (SD)  |
| Piezo              | ~5 mA         | 0 mA           |
| AMS1117-3.3 LDO    | ~5 mA (Iq)    | ~5 mA          |
| **Total (active)** | **~200 mA**   |                |

Estimated battery life (active flying, GPS + logging + display + audio):
- 1000 mAh LiPo: ~5 hours
- 2000 mAh LiPo: ~10 hours

### Legacy Version (ATmega328P)

| Component          | Active Current |
|--------------------|---------------|
| ATmega328P @ 8 MHz | ~5 mA         |
| BMP085             | ~1 mA         |
| Dual piezo         | ~10 mA        |
| LEDs               | ~15 mA        |
| **Total (active)** | **~30 mA**    |

Estimated battery life:
- 500 mAh LiPo: ~16 hours
- 1000 mAh LiPo: ~33 hours

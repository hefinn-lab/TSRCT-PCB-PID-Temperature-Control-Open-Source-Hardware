# Dependencies and running the V1.1 tools

This document describes the dependencies and execution instructions
for the V1.1 firmware and companion Python tools distributed with
repository release v1.1.1.

## Firmware

- Board: **Arduino Nano Every (ATmega4809)**, using **Arduino megaAVR Boards** in Boards Manager. A classic ATmega328P Nano is not compatible with the direct TCB2 timer implementation.
- LCD library: **LiquidCrystal_I2C**, using the Frank de Brabander **1.1.2 API** (`init()` and `backlight()`). Select the matching implementation; libraries sharing this header name can expose different APIs.
- `Arduino.h`, `Wire.h`, `SPI.h`, AVR interrupt/watchdog headers, and standard C headers come with the selected board toolchain.
- The modified MAX31865 driver is defined locally in both sketches. **No separate Adafruit_MAX31865, Adafruit BusIO, or PID library is required by these files.** Adafruit attribution remains required for the incorporated code.

## Matched files

| Task | Arduino sketch | Python companion |
| --- | --- | --- |
| Nominal control | `V1.1_TSRCT_PCB_Nominal_Dual_Channel.ino` | `V1.1_TSRCT_PCB_Nominal_Logger.py` |
| Open-loop identification and analytical tuning | `Analytical-Tuning-uC-Firmware-V1.1.ino` | `Analytical-Tuning-Logger.py` |

Use the filenames above when selecting the firmware and companion logger. Open each `.ino` in Arduino IDE and accept its prompt to create a matching sketch folder if needed. Upload only the sketch for the desired task. Before enabling heater power, check sensor configuration, channel pairing, and heater-off behaviour on the assembled system.

## Python

Use **Python 3.9 or newer**. From the repository root, create a virtual environment and install the dependencies:

```bash
python -m venv .venv
```

Activate it on Windows PowerShell:

```powershell
.venv\Scripts\Activate.ps1
```

Or on macOS/Linux:

```bash
source .venv/bin/activate
```

Then install:

```bash
python -m pip install -r requirements.txt
```

| Tool | External packages |
| --- | --- |
| Nominal logger | `pyserial`, `openpyxl` |
| Analytical logger | `pyserial`, `numpy`, `pandas`, `matplotlib`, `openpyxl`, `Pillow` |
| Optional analytical nonlinear fit | `scipy` |

Pillow is used by workbook image embedding. SciPy is optional; without it the primary crossing-based FOPDT identification and analytical tuning remain available.

```bash
python -m pip install -r requirements-optional.txt
```

To record the package versions used for your experiments:

```bash
python --version
python -m pip freeze > requirements-tested.txt
```

## Run

Close Arduino Serial Monitor and replace the example port with the board's port. Run from the directory containing the chosen logger, or supply its path.

```bash
python V1.1_TSRCT_PCB_Nominal_Logger.py --port COM4
python Analytical-Tuning-Logger.py --port COM5
```

On Linux, a port may be `/dev/ttyACM0`; on macOS, use the actual `/dev/cu.*` device.

Both pairs use **115200 baud**. The loggers are passive: they record and analyse telemetry rather than command heater shutdown. Closing a logger does not stop heating; use the board's local control/abort function.

For offline reconstruction:

```bash
python V1.1_TSRCT_PCB_Nominal_Logger.py --rebuild capture.csv
python Analytical-Tuning-Logger.py --rebuild capture.csv --controller isimc-pid
```

The analytical logger exports a selected channel's gains; apply the exported settings to the corresponding nominal-firmware channel. Default hardware settings in the V1.1 sketches are three-wire Pt100, 430 ohm reference conversion, and 50 Hz rejection.

Hardware design editing uses KiCad 10 as identified in the manuscript; KiCad is not needed to run the firmware or loggers.

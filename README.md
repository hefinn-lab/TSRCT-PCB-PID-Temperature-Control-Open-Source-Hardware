# An Open-Source Dual-Channel RTD Thermal-Control Platform for Research-Scale PEM Water Electrolysis Balance-of-Plant Integration

**TSRCT-PCB-01** is an Arduino Nano Every–based temperature-control board for experimental thermal control. It connects two Pt100 or Pt1000 resistance temperature detectors (RTDs) to independent **5 V digital solid-state-relay (SSR) control outputs**, with controller power and host communication through a single USB connection. External SSRs switch a separately powered heater circuit.

<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/a1a3f5d8-ef99-4d6e-8ecc-c1d42591dc18" />

The accompanying manuscript describes the circuit, analytical tuning methods, experimental results, and limitations. Reported validation uses isolated, dry electrolysis-cell end plates; integration with flowing water and an operating PEM water electrolyser is a subsequent development stage.

## Basic functions

- Two MAX31865 RTD acquisition channels with independent temperature set points.
- Pt100 or Pt1000 operation, with matching component population and firmware settings; configurable 2-, 3-, or 4-wire connections.
- PI/PID control, manual operation, set-point ramping, and anti-windup.
- Windowed SSR actuation, with nominal 1 s control and actuation windows.
- Open-loop step-response acquisition and FOPDT-based analytical SIMC PI / iSIMC PID tuning tools.
- USB serial telemetry and companion Python logging and analysis.
- RTD fault handling, stale-measurement detection, and over-temperature alarms that disable both heater commands.
- Connections for an I²C display, push button, and audible alarm.

## Adafruit design attribution and changes

**The RTD front ends are based on the Adafruit MAX31865 breakout design by Limor Fried/Ladyada for Adafruit Industries.** The MAX31865 interface software is derived from Adafruit's Arduino library. The original projects are:

- [Adafruit MAX31865 hardware: schematics and PCB files](https://github.com/adafruit/Adafruit-MAX31865-PCB)
- [Adafruit MAX31865 Arduino library](https://github.com/adafruit/Adafruit_MAX31865)

**Hardware changes:** TSRCT-PCB-01 integrates two RTD front ends into one control PCB with the Nano Every, SSR command connections, and display, button, and alarm interfaces. The layout consolidates the measurement and control wiring while retaining configurable RTD connections. Reference resistors and input-filter capacitors are populated for the selected sensor type.

**Firmware changes:** RTD acquisition has been changed to a **non-blocking state machine**. Bias settling, conversion waiting, and readout are scheduled as separate stages so acquisition does not hold up the main loop during the sensor waiting periods. The application adds independent control loops, timer-scheduled SSR windows, analytical tuning support, fault handling, and host telemetry. The V1.1 timer implementation uses the Nano Every's ATmega4809 TCB2 peripheral; another microcontroller requires timer adaptation.

## RTD wiring and soldering

Use the Adafruit learning guide for the underlying RTD connection principles and solder-jumper configuration:

- [MAX31865 guide: overview](https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier/)
- [Assembly and soldering guide](https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier/assembly)
- [RTD wiring and configuration: 2-, 3-, and 4-wire sensors](https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier/rtd-wiring-config)

These instructions illustrate the Adafruit breakout. Use the **TSRCT-PCB-01 schematic and PCB labels** to identify the corresponding terminals and jumpers before soldering or cutting a trace.

| Sensor | Nominal resistance at 0 °C | Reference resistor | RTD input-filter capacitor |
| --- | ---: | ---: | ---: |
| Pt100 | 100 Ω | 430 Ω | 100 nF |
| Pt1000 | 1000 Ω | 4.3 kΩ | 10 nF |

Set the firmware's nominal RTD resistance, reference-resistor value, and wiring mode to match each populated channel. Changing from Pt100 to Pt1000 requires matching hardware and software configuration. The V1.1 default configuration uses two three-wire Pt100 sensors with 430 Ω reference resistors.

## Getting started

1. Assemble the board using its schematic and bill of materials, then configure the RTD connections as above.
2. Use the Arduino Nano Every target and Arduino megaAVR Boards core. Follow the selected sketch's dependency and configuration notes.
3. Connect the RTDs and compatible SSR control inputs, checking input-current requirements against the board's output capability. Heater power must pass through the external switching circuit, with independent thermal protection.
4. Upload the nominal-control or analytical-tuning sketch and run its companion Python logger. Verify sensor readings and heater-off behaviour before enabling heating.

## Attribution and licensing

The PCB design follows the manuscript's **CC BY-SA 3.0** hardware licence. Retain the original Adafruit attribution and licence notices with redistributed or modified design files. Adafruit-derived software retains its upstream licence and copyright notices; consult the licence notices accompanying each software component. Hardware licensing does not replace software licensing.

Please acknowledge the original Adafruit projects when reusing these RTD front ends or their interface code, and refer to the accompanying manuscript for the TSRCT-PCB-01 design and experimental methods.

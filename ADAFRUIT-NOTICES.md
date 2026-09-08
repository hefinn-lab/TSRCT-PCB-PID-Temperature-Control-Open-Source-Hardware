# Adafruit attribution and upstream notices

## Hardware

The TSRCT-PCB-01 RTD front ends are adapted from the **Adafruit MAX31865 PCB**, designed by **Limor Fried/Ladyada for Adafruit Industries**.

- [Original hardware repository](https://github.com/adafruit/Adafruit-MAX31865-PCB)
- [Upstream CC BY-SA 3.0 licence](https://github.com/adafruit/Adafruit-MAX31865-PCB/blob/master/license.txt)
- [Assembly and wiring guide](https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier/)

Changes in TSRCT-PCB-01 include integrating two RTD front ends with an Arduino Nano Every, independent SSR command connections, and user-interface circuitry on one PCB. Hardware adaptations are distributed under CC BY-SA 3.0. See LICENSE-CC-BY-SA-3.0.txt for the legal text.

The following upstream README text is retained for attribution (its descriptions concern the original Adafruit breakout):

> Adafruit MAX31865 PCB
>
> Click here to purchase one from the Adafruit shop
>
> PCB files for the Adafruit MAX31865 RTD Sensor Breakout. Format is EagleCAD schematic and board layout
>
> https://www.adafruit.com/product/3328
>
> Description
>
> For precision temperature sensing, nothing beats a Platinum RTD. Resistance temperature detectors (RTDs) are temperature sensors that contain a resistor that changes resistance value as its temperature changes, basically a kind of thermistor. In this sensor, the resistor is actually a small strip of Platinum with a resistance of 100 ohms at 0°C, thus the name PT100.
>
> Compared to most NTC/PTC thermistors, the PT type of RTD is much most stable and precise (but also more expensive) PT100's have been used for many years to measure temperature in laboratory and industrial processes, and have developed a reputation for accuracy (better than thermocouples), repeatability, and stability.
>
> However, to get that precision and accuracy out of your PT100 RTD you must use an amplifier that is designed to read the low resistance. Better yet, have an amplifier that can automatically adjust and compensate for the resistance of the connecting wires. If you're looking for a great RTD sensor, today is your lucky day because we have a lovely Adafruit RTD Sensor Amplifier with the MAX31865 breakout for use with any 2, 3 or 4 wire PT100 RTD!
>
> License
>
> Adafruit invests time and resources providing this open source design, please support Adafruit and open-source hardware by purchasing products from Adafruit!
>
> Designed by Limor Fried/Ladyada for Adafruit Industries.
>
> Creative Commons Attribution/Share-Alike, all text above must be included in any redistribution. See license.txt for additional details.

## Software

The local MAX31865 interface and temperature-conversion routines in the V1.1 sketches are derived from the **Adafruit MAX31865 Arduino library**, written by **Limor Fried/Ladyada for Adafruit Industries**.

- [Original software repository](https://github.com/adafruit/Adafruit_MAX31865)
- [Source file carrying the upstream notice](https://github.com/adafruit/Adafruit_MAX31865/blob/master/Adafruit_MAX31865.cpp)

The upstream source banner is preserved below. Retain this banner in redistributed source files containing the derived driver:

```cpp
/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
```

The upstream README also states:

> Adafruit MAX31865
>
> This is the Adafruit MAX31865 Arduino Library
>
> Tested and works great with the Adafruit Thermocouple Breakout w/MAX31865
>
> http://www.adafruit.com/products/3328
>
> These sensors use SPI to communicate, 4 pins are required to interface
>
> Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!
>
> Written by Limor Fried/Ladyada for Adafruit Industries.
>
> BSD license, check license.txt for more information All text above must be included in any redistribution

TSRCT modifications separate bias settling, conversion waiting, and readout into a non-blocking acquisition sequence, and integrate acquisition with dual-channel control, timer-based actuation, fault supervision, and telemetry. Original TSRCT contributions are MIT-licensed; the upstream portions retain the notices above.

The inspected upstream repository describes its software licence as **BSD**, without an available standalone `license.txt` identifying the precise variant. This file preserves the retrieved notice; it does not invent a BSD-2-Clause/BSD-3-Clause designation or replace any full licence supplied with the actual incorporated source version. Keep that version's complete notices wherever present.

Adafruit is credited for its original contributions; this attribution does not imply endorsement of TSRCT-PCB-01.

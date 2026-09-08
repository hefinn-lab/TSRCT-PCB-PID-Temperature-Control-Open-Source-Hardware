# TSRCT-PCB-01 v1.1.1 — Initial public release

Initial public release of TSRCT-PCB-01, an open-source dual-channel Pt100/Pt1000 PID temperature-control platform for Arduino Nano Every, developed for research-scale PEM water electrolysis thermal management.

This release brings together:

- Hardware design and fabrication files, bill of materials, and assembly documentation.
- Dual-channel nominal-control firmware with non-blocking MAX31865 acquisition, 5 V SSR commands, PI/PID control, and fault handling.
- Open-loop step-identification firmware and Python tools for FOPDT identification and SIMC PI / iSIMC PID tuning.
- Python serial logging, CSV records, and XLSX reporting.
- Dependency instructions, citation metadata, and component-specific licensing and Adafruit attribution.

The RTD front ends are adapted from Adafruit's MAX31865 hardware design. The incorporated Adafruit-derived interface code has been restructured for non-blocking acquisition.

Hardware: CC BY-SA 3.0. Original software: MIT. Adafruit-derived software retains its upstream BSD notice; see LICENSE.md and ADAFRUIT-NOTICES.md.

See DEPENDENCIES.md for the supported board, matched firmware/logger pairs, and setup. Experimental performance and validation scope are described in the accompanying manuscript; flowing-water and full-cell integration are future validation stages.

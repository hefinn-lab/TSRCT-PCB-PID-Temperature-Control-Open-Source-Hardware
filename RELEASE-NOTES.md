# TSRCT-PCB-01 v1.1.1

Public release of TSRCT-PCB-01, an open-source dual-channel Pt100/Pt1000 PID temperature-control platform for Arduino Nano Every, developed for research-scale PEM water electrolysis thermal management.

Repository release **v1.1.1** packages firmware and companion Python tools **V1.1**.

This release brings together:

- Hardware design and fabrication files, bill of materials, and assembly documentation.
- Dual-channel nominal-control firmware with non-blocking MAX31865 acquisition, 5 V SSR commands, PI/PID control, and fault handling.
- Open-loop step-identification firmware and Python tools for FOPDT identification and SIMC PI / iSIMC PID tuning.
- Python serial logging with CSV and XLSX output.
- Dependency instructions, citation metadata, and component-specific licensing and Adafruit attribution.

The RTD front ends are adapted from Adafruit's MAX31865 hardware design. The incorporated Adafruit-derived interface code has been restructured for non-blocking acquisition.

Hardware: CC BY-SA 3.0. Original software: MIT. Adafruit-derived software retains its upstream BSD notice; see [LICENSE.md](LICENSE.md) and [ADAFRUIT-NOTICES.md](ADAFRUIT-NOTICES.md).

See [DEPENDENCIES.md](DEPENDENCIES.md) for the supported board, matched firmware/logger pairs, and setup. The accompanying manuscript describes the experimental performance on isolated, dry end plates. Before enabling heating, adopters should confirm operation on their assembled hardware using the checks in the [README](README.md).

# Licensing

TSRCT-PCB-01 uses separate licences for hardware and software. These are component-specific terms, not a choice of licence for the entire repository.

| Material | Licence / notice |
| --- | --- |
| TSRCT-PCB-01 schematics, PCB layouts, and fabrication outputs | Creative Commons Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0); see LICENSE-CC-BY-SA-3.0.txt |
| Original TSRCT firmware, Python tools, and original software modifications | MIT; see LICENSE-MIT.txt |
| Adafruit-derived MAX31865 interface code incorporated into the firmware | Retains Adafruit's upstream BSD notice; see ADAFRUIT-NOTICES.md |
| Separately installed Arduino core, LCD library, and Python packages | Their respective upstream licences |

Original TSRCT software copyright (c) 2026 Hamish Finn and contributors.

The RTD front-end hardware is adapted from the Adafruit MAX31865 PCB by Limor Fried/Ladyada for Adafruit Industries. TSRCT-PCB-01 integrates two front ends with the microcontroller, SSR command outputs, and user-interface connections. Hardware adaptations are distributed under CC BY-SA 3.0.

The software incorporates Adafruit-derived MAX31865 interface and temperature-conversion code, restructured for non-blocking acquisition. The MIT grant covers original TSRCT contributions; it does not remove or replace Adafruit's notices or relicense upstream material.

Preserve existing file-level notices when copying or redistributing files. The hardware licence does not automatically apply to independent software, and the software licence does not replace the hardware licence. This file does not assign new licences to externally authored publications, figures, datasets, or other third-party material.

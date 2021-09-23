# StratoDIB

This repository contains the code to run the Data Interface Board (DIB) on the FLOATS instrument flown by LASP on the CNES Strateole 2 super-pressure balloon campaign. StratoDIB inherits functionality from StratoCore, version-controlled in a separate repository.

## Hardware

All of the instruments use Teensy 3.6 Arduino-compatible MCU boards as the primary computer. Thus, this and all other Strateole 2 code is implemented for Arduino, meaning that all of this C++ code uses the Arduino drivers for the Teensy 3.6 and is compiled using the Arduino IDE with the Teensy plug-in.

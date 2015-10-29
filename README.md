The WiPy project
========================
<p align="center">
  <img src="https://cloud.githubusercontent.com/assets/7749335/6927441/bdb84f70-d7ed-11e4-96c8-9dd8dda12857.png" alt="The WiPy Logo"/>
</p>

This is the WiPy project which aims to offer a powerful, simple,
yet flexible IoT hardware and software solution. This is achieved by
combining MicroPython (https://github.com/micropython/micropython),
a lean and mean implementation of Python 3 targeted to microcontrollers,
with the CC3200 from Texas Instruments, a state of the art SoC that
combines a Cortex-M4 MCU with an ultra low power WiFi network processor.

Components in this repository:
- docs/ -- API documentation.
- hardware/ -- Schematics, and PCB files of the WiPy.
- micropython/ -- Contains the MicroPython sources (as a git submodul) used to built the latest WiPy SW release.
- misc/ -- Miscellaneous stuff like the WiPy logo.

The micropython folder is a git submodule pointing to the micropython repository, check it out with:

    $ git submodule update --init

"make" is used to build the software, or "gmake" on BSD-based systems.
You will also need bash and Python (at least 2.7 or 3.3).

To build:

    $ cd cc3200
    $ make BTARGET=application BTYPE=release BOARD=WIPY

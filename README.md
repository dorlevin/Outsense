# Outsense
code for Intel Edison module using camera, leds, proximity sensor, and MCU programming for Power management.

software prerequisites:
* MCU-SDK quickstart - https://software.intel.com/en-us/creating-applications-with-mcu-sdk-for-intel-edison-board
* install latest libmraa on Edison
* install dropbox deamon on Edison - https://www.dropbox.com/install?os=lnx
* install imageMagick on Edison - http://forum.arduino.cc/index.php?topic=318259.0
* install openCV3 on Edison - https://software.intel.com/en-us/articles/opencv-300-ipp-tbb-enabled-on-yocto-with-intel-edison
use the Intel MCU-SDK, to program the MCU with mcu_main.c file, for creating the registered wake-up timer, and run the outsense project, after compiling it.

making the object:
* make outsense

usage:
./simulator <--debug> <--dropbox>
* debug flag for more verbose printing
* dropbox flag for sending every picture to dropbox, instead of daily



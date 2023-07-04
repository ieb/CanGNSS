# NMEA2000 GNSS Receiver

A NMEA2000 device to provide rapid position and COG SOG update as well as other GNSS information including satelite data.

# PGN messages and rates.


* 129025L Position Rapid update 5Hz
* 129026L COG/SOG Rapid Update 2.5Hz
* 129029L Position data 1Hz
* 126992L System Time, 1Hz
* 129539L GNSS DOPs 1Hz
* 129540L GNSS Satellites in View 0.5Hz
* 127258L Magnetic Variation 1Hz

Magnetic variation is calculated every 60s from WMN2020 World model coefficients.

# Hardware

* Attiny3224 MCU
* UBlox M8N module pre configured at 19200 Baud with UBX messages.
* MCP 2515 CAN driver with TJ1050 Transceiver

# Code

Starts up, establishes CAN connectivity then reads UBX messages as they are produced and transfers them directly onto the NMEA200 bus.  There is a monitoring UART that runs at 115200 which can be used to see log messages and reboot. Press h to get a menu. The WNM2020 model takes 63ms to calculate which is long enough for a standard 64 byte UART buffer to overflow. This code requires a patched megaTinyCode code base with a 256 byte UART buffer.


![](images/board.jpg)


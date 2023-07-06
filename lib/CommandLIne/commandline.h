#pragma once

#include "SmallNMEA2000.h"
#include <avr/wdt.h>



// symbols from main.cpp so we dont have to get complicated.
extern void changeDiagnostics();
extern void dumpStatus();
extern void factoryReset();
extern void saveConfig();
extern void disconnectUBX();



class CommandLine {
    public:
        CommandLine(Stream * io) : io{io}  {};


        void begin() {
        };


        void checkCommand() {
            if (io->available()) {
                char chr = io->read();
                switch ( chr ) {
                    case 'h': showHelp(); break;
                    case 's': showStatus(); break;
                    case 'R': doReset(); break;
                    case 'd': changeDiagnostics(); break;
                    case 'u': disconnectUBX(); break;
                    case 'F': factoryReset(); break;
                    case 'S': saveConfig(); break;
                }
            }
        };
    private:
        Stream * io;

        void doReset() {
#ifdef MEGATINYCORE_MCU            
            _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
#else
            wdt_enable(WDTO_60MS);
            while(1) {}
#endif
        }




        void showHelp() {
            io->println(F("Pressure Monitor - key presses"));
            io->println(F("  - 'h' or '?' to show this message"));
            io->println(F("  - 's' show status"));
            io->println(F("  - 'd' change diagnostics"));
            io->println(F("  - 'u' Disconnect UBX"));
            io->println(F("  - 'R' reset"));
            io->println(F("  - 'F' factory reset"));
            io->println(F("  - 'S' save config"));
        };

        
        void showStatus() {

            io->println(F("Status"));

            dumpStatus();
#ifdef MEGATINYCORE_MCU            
            io->print((F("MCU V=")));
            io->print(readMCUVoltage());
            io->print((F("mV T=")));
            int32_t t = readMCUTemperature();
            t -= 273;
            io->print(t);
            io->println((F("C")));
#else
            io->println("");
#endif


        };

#ifdef MEGATINYCORE_MCU            

        uint16_t readMCUVoltage() {
            analogReference(INTERNAL1V024);
            delayMicroseconds(100);
            int32_t vddmeasure = analogReadEnh(ADC_VDDDIV10, 12); // Take it at 12 bits
            vddmeasure *= 10; // since we measured 1/10th VDD
            int16_t returnval = vddmeasure >> 2; // divide by 4 to get into millivolts.
            if (vddmeasure & 0x02) {
                // if last two digits were 0b11 or 0b10 we should round up
                returnval++;
            }
            return returnval;
        }

        uint16_t readMCUTemperature() {
            int8_t sigrowOffset = SIGROW.TEMPSENSE1;
            uint8_t sigrowGain = SIGROW.TEMPSENSE0;
            analogSampleDuration(128); // must be >= 32µs * f_CLK_ADC per datasheet 30.3.3.7
            analogReference(INTERNAL1V024);
            uint32_t reading = analogRead(ADC_TEMPERATURE);
            reading -= sigrowOffset;
            reading *= sigrowGain;
            reading += 0x80; // Add 1/2 to get correct rounding on division below
            reading >>= 8; // Divide result to get Kelvin
            return reading;

        }
#endif
};



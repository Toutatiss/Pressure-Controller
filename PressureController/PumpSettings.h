#ifndef PUMPSETTINGS_H
#define PUMPSETTINGS_H

#include "PressureUnit.h"
#include "FlowUnit.h"
#include "PumpLetter.h"
#include "PumpMode.h"

// Define the pump structure
struct PumpSettings
{
    PumpLetter pumpLetter;  // The letter of the pump (A, B, C, D, etc.)
    PumpMode mode;          // The pump mode (CONSTANT_PRESSURE or CONSTANT_FLOW)
    float pressureSetpoint; // The current pressure setpoint (in the specified pressure unit)
    float flowRateSetpoint; // The current flowrate setpoint (in the specified flow unit)
    float pressure;         // The current pressure (in the specified pressure unit)
    float flowRate;         // The current flowrate (in the specified flow unit)
    float volumeRemaining;  // Volume of fluid still in pump reservoir
    bool isOn;              // Tracks whether the pump is on or not

    // Constructor
    PumpSettings(PumpLetter letter, PumpMode mode)
        : pumpLetter(letter), mode(mode), pressureSetpoint(0.0), flowRateSetpoint(0.0), pressure(0.0), flowRate(0.0), volumeRemaining(0.0), isOn(true) {}
};

#endif

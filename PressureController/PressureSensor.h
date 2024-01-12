#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H

#include "PressureUnit.h"

// Define the pump structure
struct PressureSensor
{
    PressureUnit unit;          // Output unit for pressure sensor
    float differentialPressure; // differential pressure as a float
    float temperature;          // temperature as a float
    float diffPressSetpoint;
    // Constructor
    PressureSensor()
        : unit(BAR), differentialPressure(0.0), temperature(0.0), diffPressSetpoint(5.0) {}
};

#endif

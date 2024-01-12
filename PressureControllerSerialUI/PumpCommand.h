// Enum definition in a separate header file (Pump.h)
#ifndef PUMPCOMMAND_H
#define PUMPCOMMAND_H

enum PumpCommand
{
    SET_PRESSURE,
    SET_MAX_PRESSURE,
    SET_MIN_PRESSURE,
    SET_FLOW_RATE,
    SET_MAX_FLOW_RATE,
    SET_MIN_FLOW_RATE,
    SWITCH_PUMP_STATE,
    SWITCH_ALL_PUMPS,
    PUMP_MODE,
};

#endif

/*
    PressureController.h - Library for managing communication with the EAE Deformation Rig
    Written by Tat Kalintsev, 02/10/2023
    Made for the School of Earth, Atmosphere and Environment, Monash University.
    Software Version 1.0
*/

#ifndef PressureController_h
#define PressureController_h

#include "Arduino.h"
#include "ModbusMaster.h"
#include "string.h"

#include "PumpLetter.h"
#include "PumpMode.h"
#include "PressureUnit.h"
#include "FlowUnit.h"
#include "PumpSettings.h"
#include "PressureSensor.h"
#include "Valve.h"

// Debug Stream
#define DEBUG_STREAM Serial // Comment this out to remove debugging lines

#ifdef DEBUG_STREAM
#define debugPrint(...) DEBUG_STREAM.print(__VA_ARGS__)
#define debugPrintln(...) DEBUG_STREAM.println(__VA_ARGS__)
#else
#define debugPrint(...)
#define debugPrintln(...)
#endif

// Define Addresses:
// TELEDYNE PUMPS //
// COILS:
#define TOGGLE_PUMP_A 0x0000
#define TOGGLE_PUMP_B 0x0001
#define TOGGLE_PUMP_C 0x0002
#define TOGGLE_PUMP_D 0x0003 // NOTE: Check back later to convert these to incremented values instead of raw definitions?

#define CONST_PRESS_A 0x0009
#define CONST_PRESS_B 0x000A
#define CONST_PRESS_C 0x000B
#define CONST_PRESS_D 0x000C

#define CONST_FLOW_A 0x000D
#define CONST_FLOW_B 0x000E
#define CONST_FLOW_C 0x000F
#define CONST_FLOW_D 0x0010

#define SELECT_ATM 0x0054
#define SELECT_BAR 0x0055
#define SELECT_KPA 0x0056
#define SELECT_PSI 0x0057

#define SELECT_ML_MIN 0x0058
#define SELECT_ML_HR 0x0059
#define SELECT_UL_MIN 0x005A
#define SELECT_UL_HR 0x005B

// INPUT REGISTERS
#define SET_PRESS_A 0x0000
#define SET_PRESS_B 0x0002
#define SET_FLOW_A 0x000C
#define SET_FLOW_B 0x000E

// READ-ONLY REGISTERS
#define READ_PRESS_A 0x0048
#define READ_PRESS_B 0x004E

#define READ_FLOW_A 0x004A
#define READ_FLOW_B 0x0050

#define READ_VOL_A 0x004C
#define READ_VOL_B 0x0052

// KELLER DIFF PRESSURE TRANSDUCER //
// HOLDING REGISTERS:
#define DIFF_PRESS 0x0002
#define TEMPERATURE 0x0008

class PressureController
{
public:
    // Constructor function
    PressureController(ModbusMaster &pumpNode, ModbusMaster &pressureNode);

    // Parameter initialiser
    void begin();

    // Verify Communication
    bool pumpNetworkActive = false;
    bool sensorNetworkActive = false;

    //*** User-side functions for sketch programming ***//
    // Pump Functions //
    void updatePumpData(PumpSettings &pumpASettings, PumpSettings &pumpBSettings);

    // Access Current Values
    float readCurrentPressure(PumpLetter pump);
    float readCurrentFlow(PumpLetter pump);

    // Access Setpoints
    float readPressureSetpoint(PumpLetter pump);
    float readFlowSetpoint(PumpLetter pump);
    void setPumpPressure(PumpLetter pump, float pressure);
    void setPumpFlowrate(PumpLetter pump, float flowrate);

    // Enable/Disable Pumps
    void switchPump(PumpLetter pump, bool state);

    // Read and Update Modes
    PumpMode readPumpMode(PumpLetter pump);
    void constantPressure(PumpLetter pump);
    void constantFlow(PumpLetter pump);

    // Read and Update Units
    PressureUnit readPressureUnit();
    FlowUnit readFlowUnit();
    void changePressureUnit(PressureUnit unit);
    void changeFlowUnit(FlowUnit unit);

    // Sensor Functions //
    void updateSensorData(PressureSensor &diffSensor);
    float readDiffPressure();
    float readTemperature();

    // Valve Functions //
    void switchAirValve(Valve valve, bool state);

private:
    //*** Pin Assignments ***//
    // Network pins
    uint8_t _MAX485_PUMP_PIN = 48;
    uint8_t _MAX485_SENSOR_PIN = 49;

    // Valve pins
    uint8_t AIR_OUT = 46;
    uint8_t AIR_IN = 47;

    //*** Modbus ***//
    // Nodes
    ModbusMaster *_pumpNode;
    ModbusMaster *_pressureNode;

    // Network Variables
    uint8_t _commandAttempts = 10;

    // Pressure Controller Instance
    static PressureController *instance; // This is used to work around ModbusMaster static functions

    // ModbusMaster Setup Functions
    void _preTransmission();
    void _postTransmission();
    static void _preTransmissionProxy();
    static void _postTransmissionProxy();

    //*** Helper Functions ***// - These functions are used to call ModbusMaster commands
    // Coil Functions
    void _writeDeviceCoil(ModbusMaster *node, uint8_t startAddress, uint8_t addressIndex, bool state, bool *success);
    void _readDeviceCoils(ModbusMaster *node, uint8_t startAddress, uint16_t *outputArray, bool *success, uint8_t addressIndex = 0, uint8_t numCoils = 1);

    // Register Functions
    void _writeDeviceRegisters(ModbusMaster *node, uint8_t startAddress, uint8_t addressIndex, int numRegs, float value, bool *success);
    void _readDeviceRegisters(ModbusMaster *node, uint8_t startAddress, float *outputArray, bool *success, uint8_t addressIndex = 0, uint8_t addressInc = 2, int numRegs = 2);

    // Address Indexing Function
    int _selectAddress(uint8_t startingAddress, int index, int increment = 1);

    // Error handling
    void _modbusError(ModbusMaster *node, uint8_t result, const char *functionOrigin);

    // Integer Indexing of Enums
    int _enumToInt(const PumpLetter &pump);
    int _enumToInt(const Valve &valve);
    int _enumToInt(const PressureUnit &unit);
    int _enumToInt(const FlowUnit &unit);
};

#endif
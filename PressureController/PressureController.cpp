#include "PressureController.h"

/* This is used in begin() as part of pre/post transmission setup.
    It helps to get around problems wiith calling static functions from ModbusMaster */
PressureController *PressureController::instance = nullptr;

// *** SECTION: PUBLIC FUNCTIONS *** //
/*
    -- Constructor Function --
    Sets up the library with the two modbus nodes we're using
    It also assigns the DE/RE pin we are using for each of the networks.
*/
PressureController::PressureController(ModbusMaster &pumpNode, ModbusMaster &pressureNode)
{
    _pumpNode = &pumpNode;
    _pressureNode = &pressureNode;
}

/*
    -- Begin --
    Call at the setup of the Arduino sketch to initialise the network
*/
void PressureController::begin()
{
    // Valve Configuration
    pinMode(AIR_IN, OUTPUT);
    pinMode(AIR_OUT, OUTPUT);

    // Modbus Configuration
    instance = this;

    pinMode(_MAX485_PUMP_PIN, OUTPUT);
    pinMode(_MAX485_SENSOR_PIN, OUTPUT);

    digitalWrite(_MAX485_PUMP_PIN, 0);
    digitalWrite(_MAX485_SENSOR_PIN, 0);

    // Pressure Sensor Modbus
    Serial1.begin(9600, SERIAL_8N1);
    _pressureNode->begin(1, Serial1);
    _pressureNode->preTransmission(_preTransmissionProxy);
    _pressureNode->postTransmission(_postTransmissionProxy);

    // Pump Modbus
    Serial2.begin(115200, SERIAL_8E1);
    _pumpNode->begin(1, Serial2);
    _pumpNode->preTransmission(_preTransmissionProxy);
    _pumpNode->postTransmission(_postTransmissionProxy);

    debugPrintln(F("Network Setup Done"));
    debugPrintln(F("Network Error Detection is Active!"));
}

// Network Availability Functions
/* -- Act On Failure Conditions --
    Function to set the pump controller to act on all detectable failure conditions
*/
// void PressureController::actOnFailureConditions()
// {
// }

// ~~~ Subsection: User-Side Functions ~~~ //
// These functions are for programming the network easily from within the sketch
// Pumps:
/* -- Update Pump Data --
    Function to request all pump data in one go

    INPUTS:
        pumpASettings - address of pumpASettings struct
        pumpBSettings - address of pumpBSettings struct
*/
void PressureController::updatePumpData(PumpSettings &pumpASettings, PumpSettings &pumpBSettings)
{
    // Read Current Values //
    float pumpCurrValuesArr[6];
    _readDeviceRegisters(_pumpNode, READ_PRESS_A, pumpCurrValuesArr, &pumpNetworkActive, 0, 2, 12);

    if (!pumpNetworkActive)
    {
        debugPrintln(F("*** Error: Failed to get current pump values!"));
        return;
    }

    // Write pump A values
    pumpASettings.pressure = pumpCurrValuesArr[0];
    pumpASettings.flowRate = pumpCurrValuesArr[1];
    pumpASettings.volumeRemaining = pumpCurrValuesArr[2];
    pumpASettings.mode = readPumpMode(A);

    // Write pump B values
    pumpBSettings.pressure = pumpCurrValuesArr[3];
    pumpBSettings.flowRate = pumpCurrValuesArr[4];
    pumpBSettings.volumeRemaining = pumpCurrValuesArr[5];
    pumpASettings.mode = readPumpMode(B);

    // Read Pressure Setpoints //
    float pressureSetValuesArr[2];
    _readDeviceRegisters(_pumpNode, SET_PRESS_A, pressureSetValuesArr, &pumpNetworkActive, 0, 2, 4);

    if (!pumpNetworkActive)
    {
        debugPrintln(F("*** Error: Failed to get pump pressure setpoints!"));
        return;
    }

    pumpASettings.pressureSetpoint = pressureSetValuesArr[0];
    pumpBSettings.pressureSetpoint = pressureSetValuesArr[1];

    // Read Flow Setpoints //
    float flowSetValuesArr[2];
    _readDeviceRegisters(_pumpNode, SET_FLOW_A, flowSetValuesArr, &pumpNetworkActive, 0, 2, 4);

    if (!pumpNetworkActive)
    {
        debugPrintln(F("*** Error: Failed to get pump flowrate setpoints!"));
        return;
    }

    pumpASettings.flowRateSetpoint = flowSetValuesArr[0];
    pumpBSettings.flowRateSetpoint = flowSetValuesArr[1];
}

// Access Current Values
/* -- Read Current Pressure --
    Function to read a pump's current pressure

    INPUTS:
        pump - a single letter (A, B, C, or D) denoting pump to control
    OUTPUTS:
        returns the current pressure, in the current units, as float
*/
float PressureController::readCurrentPressure(PumpLetter pump)
{
    float pressArr[1];
    _readDeviceRegisters(_pumpNode, READ_PRESS_A, pressArr, &pumpNetworkActive, _enumToInt(pump), 6);
    return pressArr[0];
}

/* -- Read Current Flow --
    Function to read a pump's current flow

    INPUTS:
        pump - a single letter (A, B, C, or D) denoting pump to control
    OUTPUTS:
        returns the current flow, in the current units, as float
*/
float PressureController::readCurrentFlow(PumpLetter pump)
{
    float flowArr[1];
    _readDeviceRegisters(_pumpNode, READ_FLOW_A, flowArr, &pumpNetworkActive, _enumToInt(pump), 6);
    return flowArr[0];
}

// Access Setpoints
/* -- Read Pressure Setpoint --
    Function to read a pump's pressure setpoint

    INPUTS:
        pump - a single letter (A, B, C, or D) denoting pump to control
    OUTPUTS:
        returns the pressure setpoint, in the current units, as float
*/
float PressureController::readPressureSetpoint(PumpLetter pump)
{
    float pressSetArr[1];
    _readDeviceRegisters(_pumpNode, SET_PRESS_A, pressSetArr, &pumpNetworkActive, _enumToInt(pump));
    return pressSetArr[0];
}

/* -- Read Flow Setpoint --
    Function to read a pump's flow setpoint

    INPUTS:
        pump - a single letter (A, B, C, or D) denoting pump to control
    OUTPUTS:
        returns the flow setpoint, in the current units, as float
*/
float PressureController::readFlowSetpoint(PumpLetter pump)
{
    float flowSetArr[1];
    _readDeviceRegisters(_pumpNode, SET_FLOW_A, flowSetArr, &pumpNetworkActive, _enumToInt(pump));
    return flowSetArr[0];
}

/* -- Set Pump Pressure --
    Function to adjust a pump's pressure setpoint

    INPUTS:
        pump - a single letter (A, B, C, or D) denoting pump to control
        pressure - a floating point value of the desired pressure value
*/
void PressureController::setPumpPressure(PumpLetter pump, float pressure)
{
    _writeDeviceRegisters(_pumpNode, SET_PRESS_A, _enumToInt(pump), 2, pressure, &pumpNetworkActive);
}

/* -- Set Pump Flowrate --
    Function to adjust a pump's flowrate setpoint

    INPUTS:
        pump - a single letter (A, B, C, or D) denoting pump to control
        pressure - a floating point value of the desired flowrate value
*/
void PressureController::setPumpFlowrate(PumpLetter pump, float flowrate)
{
    _writeDeviceRegisters(_pumpNode, SET_FLOW_A, _enumToInt(pump), 2, flowrate, &pumpNetworkActive);
}

// Enable/Disable Pumps
/* -- Switch Pump State --
    Function to turn on/off a given pump
    Designed to work with up to 4 pumps connected to a single pump controller
    Pump of interest is addressed by its letter (A,B,C or D)
    Find the letter of your pump by checking which port it is connected to on the back of the controller

    INPUTS:
        pump - a single letter (A, B, C, or D) denoting pump to control
        state - desired state of the pump (true == ON, false == OFF)
*/
void PressureController::switchPump(PumpLetter pump, bool state)
{
    _writeDeviceCoil(_pumpNode, TOGGLE_PUMP_A, _enumToInt(pump), state, &pumpNetworkActive);
}

// Read and Update Modes
/* -- Pump Mode --
    Function to determine current pump mode.
    Pump of interest is addressed by its letter (A,B,C or D)
    Find the letter of your pump by checking which port it is connected to on the back of the controller

    INPUTS:
        pump - a single letter (A, B, C, or D) denoting pump to control
    OUTPUTS:
        pump - a single letter (A, B, C, or D) denoting pump to control
*/
PumpMode PressureController::readPumpMode(PumpLetter pump)
{
    uint16_t pressureMode[1];
    uint16_t flowMode[1];
    _readDeviceCoils(_pumpNode, CONST_PRESS_A, pressureMode, &pumpNetworkActive, _enumToInt(pump));
    _readDeviceCoils(_pumpNode, CONST_FLOW_A, flowMode, &pumpNetworkActive, _enumToInt(pump));

    if (pressureMode[0] == 1)
    {
        return CONSTANT_PRESSURE;
    }
    else if (flowMode[0] == 1)
    {
        return CONSTANT_FLOW;
    }
    else
    {
        return CONSTANT_PRESSURE;
    }
}

/* -- Constant Pressure Mode --
    Function to enable constant pressure mode on a given pump
    Designed to work with up to 4 pumps connected to a single pump controller
    Pump of interest is addressed by its letter (A,B,C or D)
    Find the letter of your pump by checking which port it is connected to on the back of the controller

    INPUTS:
        pump - a single letter (A, B, C, or D) denoting pump to control
*/
void PressureController::constantPressure(PumpLetter pump)
{
    _writeDeviceCoil(_pumpNode, CONST_PRESS_A, _enumToInt(pump), true, &pumpNetworkActive);
}

/* -- Constant Flow Mode --
    Function to enable constant flow mode on a given pump
    Designed to work with up to 4 pumps connected to a single pump controller
    Pump of interest is addressed by its letter (A,B,C or D)
    Find the letter of your pump by checking which port it is connected to on the back of the controller

    INPUTS:
        pump - a single letter (A, B, C, or D) denoting pump to control
*/
void PressureController::constantFlow(PumpLetter pump)
{
    _writeDeviceCoil(_pumpNode, CONST_FLOW_A, _enumToInt(pump), true, &pumpNetworkActive);
}

// Read and Update Units
/* -- Find Pressure Unit --
    Function to determine current pressure units.

    OUTPUTS:
        PressureUnit - a pressure unit of type PressureUnit
*/
PressureUnit PressureController::readPressureUnit()
{
    uint16_t coilArray[1];
    _readDeviceCoils(_pumpNode, SELECT_ATM, coilArray, &pumpNetworkActive, 0, 4);

    switch (coilArray[0])
    {
    case 1:
        return ATM;
    case 2:
        return BAR;
    case 4:
        return KPA;
    case 8:
        return PSI;
    default:
        changePressureUnit(ATM); // Change to a default pressure unit in the event units are undefined
        return ATM;
    }
}

/* -- Find Flow Unit --
    Function to determine current flow units.

    OUTPUTS:
        PressureUnit - a pressure unit of type PressureUnit
*/
FlowUnit PressureController::readFlowUnit()
{
    uint16_t coilArray[1];
    _readDeviceCoils(_pumpNode, SELECT_ML_MIN, coilArray, &pumpNetworkActive, 0, 4);

    switch (coilArray[0])
    {
    case 1:
        return ML_MIN;
    case 2:
        return ML_HR;
    case 4:
        return UL_MIN;
    case 8:
        return UL_HR;
    default:
        changeFlowUnit(ML_MIN); // Change to a default flow unit in the event units are undefined
        return ML_MIN;
    }
}

/* -- Change Pressure Units --
    Function to change pressure units across all connected pumps

    INPUTS:
        unit - Word (ATM, BAR, kPa, or PSI) denoting the required unit.
*/
void PressureController::changePressureUnit(PressureUnit unit)
{
    _writeDeviceCoil(_pumpNode, SELECT_ATM, _enumToInt(unit), true, &pumpNetworkActive);
}

/* -- Change Flow Units --
    Function to change flow rate units across all connected pumps

    INPUTS:
        unit - Word (ML_MIN, ML_HR, UL_MIN, UL_HR) denoting the required unit.
*/
void PressureController::changeFlowUnit(FlowUnit unit)
{
    _writeDeviceCoil(_pumpNode, SELECT_ML_MIN, 0, true, &pumpNetworkActive);
}

// Differential Pressure Sensor:
/* -- Update Sensor Data --
    Function to request all pump data in one go

    INPUTS:
        diffSensor - address of differenential sensor struct
*/
void PressureController::updateSensorData(PressureSensor &diffSensor)
{
    float sensorDataArr[4]; // Array of 2 floats

    _readDeviceRegisters(_pressureNode, DIFF_PRESS, sensorDataArr, &sensorNetworkActive, 0, 2, 8); // Read 8 registers into floatArr

    diffSensor.differentialPressure = sensorDataArr[0];
    diffSensor.temperature = sensorDataArr[3];
}

/* -- Read Differential Pressure --
    Function to obtain the value of the differential pressure

    INPUTS:
        N/A

    OUTPUTS:
        diffPress - Differential pressure, in units of bar, as a floating point value
*/
float PressureController::readDiffPressure()
{
    float diffPressArr[1];
    _readDeviceRegisters(_pressureNode, DIFF_PRESS, diffPressArr, &sensorNetworkActive);
    return diffPressArr[0];
}

/* -- Read Temperature --
    Function to obtain the value of the fluid temperature as measured by the pressure sensor
    Note that accuracy is around +/- 2 degrees C

    INPUTS:
        N/A

    OUTPUTS:
        temp - Temperature, in units of Celcius, as a floating point value
*/
float PressureController::readTemperature()
{
    float tempArr[1];
    _readDeviceRegisters(_pressureNode, TEMPERATURE, tempArr, &sensorNetworkActive);
    return tempArr[0];
}

// Air Valves:
/* -- Switch Air Valve State --
    Function to turn on/off a valve
    Valve of interest is addressed as intake or outtake (IN or OUT)
    The intake valve supplies air to the fluid valve to close it.
    The outtake valve vents air to reopen the fluid valve.

    INPUTS:
        valve - the word IN or OUT, denoting which valve to actuate
        state - desired state of the valve (true == ON, false == OFF)
*/
void PressureController::switchAirValve(Valve valve, bool state)
{
    digitalWrite(_enumToInt(valve), (state ? HIGH : LOW));
}

// *** SECTION: PRIVATE FUNCTIONS *** //
// ~~~ Subsection: Transmission Functions ~~~ // *NOTE: Need to test behaviour to make sure no erroneous behaviours is caused by reusing functions for both networks!
void PressureController::_preTransmissionProxy()
{
    instance->_preTransmission();
}

void PressureController::_postTransmissionProxy()
{
    instance->_postTransmission();
}

void PressureController::_preTransmission()
{
    digitalWrite(_MAX485_SENSOR_PIN, 1);
    digitalWrite(_MAX485_PUMP_PIN, 1);
}

void PressureController::_postTransmission()
{
    digitalWrite(_MAX485_SENSOR_PIN, 0);
    digitalWrite(_MAX485_PUMP_PIN, 0);
}

// ~~~ Subsection: Modbus Commands ~~~ //
// These functions are called by the user-side functions to do the work in the background
/* -- Write To Device Coil --
    Function to write a state to a device coil
    These are typically only only one bit on/off commands
    This function writes a state to one coil

    INPUTS:
        node        - Pointer to a network node. (device) (_pumpNode for pumps, _pressureNode for pressure sensor)
        address     - A modbus coil address (e.g. 0x0001)
        state       - Desired state of the coil (true == ON, false == OFF)

    OUTPUTS:
        Currently returns nothing except debug messages
*/
void PressureController::_writeDeviceCoil(ModbusMaster *node, uint8_t startAddress, uint8_t addressIndex, bool state, bool *success)
{
    uint8_t result = 0;           // variable stores success outcome of request
    int on_off = (state ? 1 : 0); // change boolean to integer
    int address = _selectAddress(startAddress, addressIndex);

    // Send command to specified address, attempt it a few times before timing out
    for (uint8_t i = 0; i < _commandAttempts; i++)
    {
        result = node->writeSingleCoil(address, on_off);
        if (result == node->ku8MBSuccess)
        {
            *success = true;
            return;
        }
    }

    // If message failed, send debug error
    *success = false;
    _modbusError(node, result, "_writeDeviceCoil");
}

/* -- Read Device Coils --
    Function to read a number of sequential coils.
    Defaults to reading only one coil

    INPUTS:
        node            - Pointer to a network node. (device) (_pumpNode for pumps, _pressureNode for pressure sensor)
        startAddress    - The first modbus coil address (e.g. 0x0001)
        outputArray     - Array of 16-bit integers to output to
        addressIndex    - For example, if dealing with different pumps, what number pump (pump A = 0, pump B = 1 etc.)
        addressInc      - Number of registers to increment by. E.g 2 for if the next address is directly after the starting one.
        numCoils        - Number of coils to sequentially read


*/
void PressureController::_readDeviceCoils(ModbusMaster *node, uint8_t startAddress, uint16_t *outputArray, bool *success, uint8_t addressIndex = 0, uint8_t numCoils = 1)
{
    uint8_t result = 0; // variable stores success outcome of request
    int address = _selectAddress(startAddress, addressIndex);

    // Attempt to send command a few times
    for (uint8_t i = 0; i < _commandAttempts; i++)
    {
        result = node->readCoils(address, numCoils);
        if (result == node->ku8MBSuccess)
        {
            *success = true;
            break; // Leave loop if command successful
        }
    }

    // Warn if command failed and leave function
    if (result != node->ku8MBSuccess)
    {
        _modbusError(node, result, "_readDeviceCoils");
        debugPrintln(F("* Default values used"));

        // Populate the outputArray with default values
        for (int j = 0; j < numCoils; j++)
        {
            outputArray[j] = 0;
        }

        *success = false;
        return;
    }

    // Otherwise populate array with found values in chunks of 16
    for (int i = 0; i < numCoils; i += 16)
    {
        outputArray[i / 16] = node->getResponseBuffer(i / 16);
    }
}

/* -- Write Device Registers --
    Function to write to a floating point value to a number of sequential registers. (Two normally)

    INPUTS:
        node            - Pointer to a network node. (device) (_pumpNode for pumps, _pressureNode for pressure sensor)
        startAddress    - The first modbus register address (e.g. 0x0001)
        numRegs         - Number of registers to write to
        value           - The floating point value to be written
*/
void PressureController::_writeDeviceRegisters(ModbusMaster *node, uint8_t startAddress, uint8_t addressIndex, int numRegs, float value, bool *success)
{
    uint8_t result = 0; // variable stores success outcome of request
    int address = _selectAddress(startAddress, addressIndex, 2);

    // First break the floating point value into two chunks
    uint16_t firstChunk = *((uint16_t *)&value);      // extract the first 16 bits
    uint16_t secondChunk = *((uint16_t *)&value + 1); // extract the second 16 bits

    // Load the two chunks into the transmit buffer
    node->setTransmitBuffer(0, secondChunk);
    node->setTransmitBuffer(1, firstChunk);

    // Send command to specified address, attempt it a few times before timing out
    for (uint8_t i = 0; i < _commandAttempts; i++)
    {
        result = node->writeMultipleRegisters(address, numRegs);
        if (result == node->ku8MBSuccess)
        {
            *success = true;
            return;
        }
    }

    // If message failed, send debug error
    *success = false;
    _modbusError(node, result, "_writeDeviceRegisters");
}

/* -- Read Device Registers --
    Function to read a number of sequential registers.

    INPUTS:
        node            - Pointer to a network node (_pumpNode for pumps, _pressureNode for pressure sensor)
        startAddress    - The first modbus register address (e.g. 0x0001)
        outputArray     - Array of floats to output to
        addressIndex    - For example, if dealing with different pumps, what number pump (pump A = 0, pump B = 1 etc.)
        addressInc      - Number of registers to increment by. E.g 2 for if the next address is directly after the starting one.
        numRegs         - Number of registers to sequentially read

*/
void PressureController::_readDeviceRegisters(ModbusMaster *node, uint8_t startAddress, float *outputArray, bool *success, uint8_t addressIndex = 0, uint8_t addressInc = 2, int numRegs = 2)
{
    uint8_t result = 0; // variable stores success outcome of request
    uint8_t address = _selectAddress(startAddress, addressIndex, addressInc);

    // Attempt to send command a few times
    for (uint8_t i = 0; i < _commandAttempts; i++)
    {
        result = node->readHoldingRegisters(address, numRegs);
        if (result == node->ku8MBSuccess)
        {
            *success = true;
            break; // Leave loop if command successful
        }
    }

    // Warn if command failed and leave function
    if (result != node->ku8MBSuccess)
    {
        _modbusError(node, result, "_readDeviceRegisters");
        debugPrintln(F("* Default values used"));

        // Populate the outputArray with default values
        for (int j = 0; j < numRegs; j++)
        {
            outputArray[j] = 0.0;
        }

        *success = false;
        return;
    }

    // Otherwise process received data
    for (int i = 0; i < numRegs; i += 2) // Reconstruct float for each pair of registers
    {
        uint32_t b1 = node->getResponseBuffer(i);
        uint32_t b2 = node->getResponseBuffer(i + 1);
        uint32_t combined = ((uint32_t)b1 << 16) | b2;
        memcpy(&outputArray[i / 2], &combined, sizeof(float));
    }
}

// ~~~ Subsection: Utilities ~~~ //
// Convert enums to integer indices
int PressureController::_enumToInt(const PumpLetter &pump)
{
    return static_cast<int>(pump);
}

int PressureController::_enumToInt(const Valve &valve)
{
    return static_cast<int>(valve) + AIR_OUT;
}

int PressureController::_enumToInt(const PressureUnit &unit)
{
    return static_cast<int>(unit);
}

int PressureController::_enumToInt(const FlowUnit &unit)
{
    return static_cast<int>(unit);
}

/* -- Select Coil Address --
    Function to choose correct coil address based on pump letter

    INPUTS:
        startingAddress    - Pump A's coil address (e.g. 0x0000)
        pump               - Desired pump letter (e.g. A, B, C, D, a, b, c or d)
        increment          - Address increment (usually 1 for coils, usually 2 for registers)

    OUTPUTS:
        returns calculated address.
        If pump specified is not an acceptable letter, returns -1.
*/
int PressureController::_selectAddress(uint8_t startingAddress, int index, int increment = 1)
{
    return startingAddress + (index * increment); // Calculate the new address
}

/* -- Modbus Error --
    Function to output a message when a modbus error is encountered

    INPUTS:
        node               - Current Modbus node
        result             - The result code returned by the Modbus command
        functionOrigin     - Name of the packaged function that called the Modbus command
*/
void PressureController::_modbusError(ModbusMaster *node, uint8_t result, const char *functionOrigin)
{
    if (result != node->ku8MBSuccess)
    {
        debugPrint(F("*** Error in "));
        debugPrint(functionOrigin);
        debugPrint(F("(): "));
        debugPrintln(result, HEX);
    }
}

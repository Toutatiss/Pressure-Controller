/*
    PressureControllerSerial.UI.h - Library for managing the serial interface for pressure controller
    Written by Tat Kalintsev
    02/10/2023
    Created for the School of Earth, Atmosphere and Environment, Monash University.
    Software Version 1.0
*/

#ifndef PressureControlleSerialUI_h
#define PressureControllerSerialUI_h

#include "Arduino.h"
#include "Wire.h"
#include "DS3231.h"
#include "SD.h"

#include "PressureController.h"
#include "State.h"
#include "PumpCommand.h"

#include "BlockNot.h"
#include "SafeString.h"

// Pins:
#define RTC_BATT A15
#define ALARM_INTERRUPT_PIN 2

class PressureControllerSerialUI
{
public:
    // Constructor function
    PressureControllerSerialUI();

    // Parameter initialiser
    void begin(int baud);

    // Interrupt variable
    volatile bool alarmDone = false;

    // User-side functions for sketch programming
    void run();

    // Pump functions
    void setPumpPressure(PumpLetter pump, float value);
    void setPumpFlowrate(PumpLetter pump, float value);
    void setPumpMode(PumpLetter pump, bool pumpMode);
    void turnAllPumpsToState(bool state);
    void turnPumpToState(PumpLetter pump, bool state);

private:
    bool inDebuggingMode = false;
    //*** Modbus Network Variables ***//
    // Nodes //
    ModbusMaster _pumpNode;
    ModbusMaster _pressureNode;

    // Network //
    PressureController _network;
    void _checkNetworkStates();
    void _testNetworks();

    // Network State Trackers //
    bool _pumpsResponding = false;
    bool _diffSensorResponding = false;
    bool _prevPumpsResponding = false;
    bool _prevDiffSensorResponding = false;

    //*** Control Timers ***//
    BlockNot _dataStreamTimer; // Controls when new data should be requested during experiment
    BlockNot _SDBufferTimer;   // Controls when to update the SD buffer
    BlockNot _SDWriteTimer;    // Controls when to offload the SD buffer data onto the SD
    BlockNot _feedbackTimer;   // Controls how often to run the controller function

    const float _updateTime = 1; // Time in seconds until data-based updates should occur

    //*** Serial Input ***//
    // Input collectors //
    uint8_t _userInput = 0; // Used for navigating between states

    // State trackers //
    State _currentState = STATE_SET_PARAMETERS;
    State _previousState;

    // Serial Handlers //
    void _handleSerialInput(char *input);
    void _executeFunction(uint8_t *functionNumber);
    void _clearSerialBuffer();
    void _readSerialData(SafeString &safeString);

    // Specific Data Requesters //
    void _requestSerialData(char *pump, float *value); // Read a pump and float
    void _requestSerialData(char *pump, bool *state);  // Read a pump and bool
    void _requestSerialData(int *value);               // Read an int
    void _requestSerialData(bool *state);              // Read a bool
    void _requestSerialData(float *value);             // Read a float

    // *** Devices *** //
    // ~ Real-Time Clock ~ //
    DS3231 _rtc;

    // Variables For Getting Time
    bool _century = false;    // Needed for reading month
    bool _twelveHour = false; // Needed for reading hour
    bool _ampm = false;       // Needed for reading hour

    // Alarm Variables
    int _alarmDays = 1;
    bool _alarmDayIsDay = false;

    // Functions
    void _setupRTCAlarms();
    void _changeExperimentDur();
    void _setSecondsUntilNextAlarm(unsigned long *seconds);

    // ~ SD Card ~ //
    const char *_filename;
    File _txtFile;
    bool SDIsAvailable = false;
    void _setupSD();
    void _printHeadingsToSD();
    void _runSD();

    // ~ Pumps ~ //
    // Settings Structs and Variables
    PumpSettings _pumpASettings;
    PumpSettings _pumpBSettings;
    PressureUnit _currentPressureUnit = BAR;
    FlowUnit _currentFlowUnit = ML_MIN;

    // Instantiation tracker
    bool pumpsInstantiated = false;

    // Functions
    void _instantiatePumps();
    void _changePumpSetting(PumpCommand command, bool isPressure = true);
    void _setAllPumpsState(PumpCommand command); // Set boolean to all pumps

    // ~ Pressure Sensor ~ //
    PressureSensor _diffPressSensor;

    // ~ Solenoid Valves ~ //
    bool _airPurged = false; // Tracks whether there is air in the valve system
    void _closeDiffValve();
    void _openDiffValve();

    // *** PID Controller *** //
    bool _newControlCycle = false; // Tracks when to refresh static variables

    void _changeDiffPressSetpoint();
    void _runDifferentialController();

    // *** Experiment Management *** //
    bool _experimentStarted = false;

    void _initialiseExperiment();
    void _runExperiment();

    // *** Data Struct Updaters *** //
    void _updateAllDataStructs();
    void _updatePumpSetpoints();
    void _updateCurrPumpValues();
    void _updateCurrSensorValues();

    // *** Data Printing *** //
    void _printTimeStamp(DateTime dateAndTime);
    void _printCurrentState();
    void _updateAllInfo();
    void _printHeadings();
    void _printPumpInfo();
    void _printSensorInfo();

    // *** String and Character Formatting Utilities *** //
    void _enumToSafeString(PressureUnit &unit, SafeString &safeUnit);
    void _enumToSafeString(FlowUnit &unit, SafeString &safeUnit);
    void _enumToSafeString(PumpMode &mode, SafeString &safeUnit);
    char _enumToChar(PumpLetter &letter);
    PumpLetter _pumpCharToEnum(char pump);
    void _charToBool(bool *state, char boolChar);

    // *** Interrupt Functions *** //
    void _isrAlarmTriggered();
};

#endif
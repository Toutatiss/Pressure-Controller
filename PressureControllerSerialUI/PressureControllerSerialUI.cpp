#include "Arduino.h"
#include "PressureControllerSerialUI.h"

// Global Variables
// Buffer to hold SD data before it is written
createSafeString(_buffer, 1024);

// *** SECTION: PUBLIC FUNCTIONS *** //
// Constructor function
PressureControllerSerialUI::PressureControllerSerialUI()
    : _dataStreamTimer(1, SECONDS),
      _SDWriteTimer(10, SECONDS),
      _feedbackTimer(1, SECONDS),
      _network(_pumpNode, _pressureNode),
      _pumpASettings(A, CONSTANT_PRESSURE),
      _pumpBSettings(B, CONSTANT_FLOW)
{
    _filename = "livedata.txt";
}

/* -- Begin --
    Function to intialise the serial UI from the user-side
*/
void PressureControllerSerialUI::begin(int baud)
{
    // Start up serial output with the computer
    Serial.begin(baud);
    Serial.println(F("\n--- Starting up Serial Interface... ---\n"));
    Serial.setTimeout(500); // Adds a time-out against excessively long string reads

    // Initialise SPI
    Serial.println(F("Starting peripherals..."));
    Wire.begin();

    // Start up SD card
    _setupSD();

    // Check on RTC Time
    Serial.print(F("RTC Time: "));
    _printTimeStamp(RTClib::now());

    // Start up the Modbus network
    Serial.println(F("Starting networks..."));
    _network.begin();

    // Check network availability normally

    // Query Networks
    Serial.println(F("Querying networks..."));
    _testNetworks();

    // Update UI state trackers
    _checkNetworkStates();

    Serial.println(F("Setup Complete!"));
    Serial.println(F("\n\n\n-- Welcome to the Deformation System Serial Interface!--"));
    _printCurrentState();
}

// ~~~ Subsection: User-Side Functions ~~~ //
// These functions are for programming the interface easily from within the sketch

/* -- Run --
    Function to facilitate the serial UI from the user-side
*/
void PressureControllerSerialUI::run()
{
    // Check for incoming serial commands
    if (Serial.available() > 0)
    {
        char command = Serial.read();
        _handleSerialInput(&command);
    }

    // Monitor alarm interrupt to know when to end experiment
    if (alarmDone && _experimentStarted)
    {
        Serial.println(F("ALERT: Experiment finished! System automatically resetting..."));
        _experimentStarted = false;
        _currentState = STATE_RESET;
    }

    switch (_currentState)
    {
    case STATE_FUNCTION:
        break;
    case STATE_SET_PARAMETERS:
        // Stop experiment if neutral state is entered
        if (_experimentStarted)
        {
            // Disable alarms
            _setupRTCAlarms();
            _experimentStarted = false;
        }
        break;
    case STATE_EXPERIMENT:
        _initialiseExperiment();
        _runExperiment();
        break;
    case STATE_DIFFERENTIAL:
        _initialiseExperiment();
        _runExperiment();
        _runDifferentialController();
        break;
    case STATE_RESET:
        if (_pumpsResponding)
        {
            Serial.println(F("Disabling Pumps..."));
            turnAllPumpsToState(false);
        }
        else
        {
            Serial.print(F("\n### Failed to disable pumps: Pumps not responding."));
        }

        if (!_airPurged)
        {
            Serial.println(F("Purging Air..."));
            _openDiffValve();
        }

        Serial.println(F("Reset Complete!"));
        Serial.println(F("Returning to Set Parameters...\n"));

        // Return back to the set parameters state
        _currentState = STATE_SET_PARAMETERS;
        _printCurrentState();
        break;
    case STATE_HELP:
    default:
        break;
    }
}

/* -- Set Pump Mode --
    Function to set a particular pump to constant pressure or constant flow mode

    INPUTS:
        pump          - The pump to change the mode of
        state         - The state to change the pump to (true == CP, false == CF)
*/
void PressureControllerSerialUI::setPumpMode(PumpLetter pump, bool pumpMode)
{
    if (!_pumpsResponding)
    {
        Serial.println(F("Skipped function: 'Set Pump Mode' (Network not responding.)"));
        return;
    }

    PumpMode mode = pumpMode ? CONSTANT_PRESSURE : CONSTANT_FLOW; // Determine what the mode should be

    if (mode == CONSTANT_PRESSURE)
    {
        switch (pump)
        {
        case A:
            _network.constantPressure(A);
            _pumpASettings.mode = mode;
            break;
        case B:
            _network.constantPressure(B);
            _pumpBSettings.mode = mode;
            break;
        }
    }
    else
    {
        switch (pump)
        {
        case A:
            _network.constantFlow(A);
            _pumpASettings.mode = mode;
            break;
        case B:
            _network.constantFlow(B);
            _pumpBSettings.mode = mode;
            break;
        }
    }
}

/* -- Set Pump Pressure --
    Function to set a pump's pressure

    INPUTS:
        pump          - Letter of the desired pump as a PumpLetter enum
        value         - The desired pressure setpoint
*/
void PressureControllerSerialUI::setPumpPressure(PumpLetter pump, float value)
{
    if (!_pumpsResponding)
    {
        Serial.println(F("Skipped function: 'Set Pump Pressure' (Network not responding.)"));
        return;
    }

    _network.setPumpPressure(pump, value);

    switch (pump)
    {
    case A:
        _pumpASettings.pressureSetpoint = value;
        break;
    case B:
        _pumpBSettings.pressureSetpoint = value;
        break;
    }
}

/* -- Set Pump Flowrate --
    Function to set a pump's flowrate

    INPUTS:
        pump          - Letter of the desired pump as a PumpLetter enum
        value         - The desired flowrate setpoint
*/
void PressureControllerSerialUI::setPumpFlowrate(PumpLetter pump, float value)
{
    if (!_pumpsResponding)
    {
        Serial.println(F("Skipped function: 'Set Pump Flowrate' (Network not responding.)"));
        return;
    }

    _network.setPumpFlowrate(pump, value);

    switch (pump)
    {
    case A:
        _pumpASettings.flowRateSetpoint = value;
        break;
    case B:
        _pumpBSettings.flowRateSetpoint = value;
        break;
    }
}

// Pump Helper Functions
/* -- Turn All Pumps To State --
    Function to turn all pumps on/off

    INPUTS:
        state          - The state to change the pump to (true == ON, false == OFF)
*/
void PressureControllerSerialUI::turnAllPumpsToState(bool state)
{
    if (!_pumpsResponding)
    {
        Serial.println(F("Skipped function: 'Turn All Pumps To State' (Network not responding.)"));
        return;
    }

    _network.switchPump(A, state);
    _network.switchPump(B, state);
    _pumpASettings.isOn = state;
    _pumpBSettings.isOn = state;
}

/* -- Turn Pump To State --
    Function to turn a particular pump on/off

    INPUTS:
        pump          - The pump to change the state of
        state         - The state to change the pump to (true == ON, false == OFF)
*/
void PressureControllerSerialUI::turnPumpToState(PumpLetter pump, bool state)
{
    if (!_pumpsResponding)
    {
        Serial.println(F("Skipped function: 'Turn Pump To State' (Network not responding.)"));
        return;
    }

    switch (pump)
    {
    case A:
        _network.switchPump(A, state);
        _pumpASettings.isOn = state;
        break;
    case B:
        _network.switchPump(B, state);
        _pumpBSettings.isOn = state;
        break;
    }
}

// *** SECTION: PRIVATE FUNCTIONS *** //
// ~~~ Subsection: Serial Handler Functions ~~~ //
/* -- Handle Serial Input --
    Function to handle state-related user inputs and state navigation

    INPUTS:
        input         - A single character denoting the desired user input
*/
void PressureControllerSerialUI::_handleSerialInput(char *input)
{
    if (_currentState == STATE_FUNCTION)
    {
        if (isdigit(*input)) // Handle numerical inputs for functions
        {
            // Process input
            _userInput = *input - '0'; // Convert ASCII digit to integer
            _executeFunction(&_userInput);

            // Return to previous state
            _currentState = _previousState;
            _printCurrentState();
        }
        else if (*input == 'c' || *input == 'C') // Allow function operation to be cancelled
        {
            // Return to previous state
            Serial.println(F("Cancelled function operation. Reverting to previous state..."));
            _currentState = _previousState;
            _printCurrentState();
        }
        else // Warn of invalid input
        {
            Serial.println(F("### Invalid function number entered. Enter a number from the list, or 'c' to cancel."));
        }
    }

    else
    {
        switch (*input)
        {
        case 's':
        case 'S':
            _previousState = _currentState;
            _currentState = STATE_SET_PARAMETERS;
            break;
        case 'e':
        case 'E':
            _previousState = _currentState;
            _currentState = STATE_EXPERIMENT;
            break;
        case 'd':
        case 'D':
            _previousState = _currentState;
            _currentState = STATE_DIFFERENTIAL;
            break;
        case 'r':
        case 'R':
            _previousState = _currentState;
            _currentState = STATE_RESET;
            break;
        case 'f':
        case 'F':
            _previousState = _currentState;
            _currentState = STATE_FUNCTION;
            break;
        case 'h':
        case 'H':
            _previousState = _currentState;
            _currentState = STATE_HELP;
            break;
        default:
            Serial.println(F("### Invalid state command!"));
            break;
        }

        // Display the current state after changing
        _printCurrentState();
    }

    // Purge extra characters from read buffer
    _clearSerialBuffer();
}

/* -- Execute Function --
    Function to call a specific pressure controller function

    INPUTS:
        functionNumber         - The index of the function to be called
*/
void PressureControllerSerialUI::_executeFunction(uint8_t *functionNumber)
{
    switch (*functionNumber)
    {
    case 1:
        // Function 1: Set a pump's pressure
        Serial.println(F("--- Set Pressure ---"));
        _changePumpSetting(SET_PRESSURE);
        break;
    case 2:
        // Function 2: Set a pump's flow rate
        Serial.println(F("--- Set Flowrate ---"));
        _changePumpSetting(SET_FLOW_RATE, false); // false refers to it being a flow operation
        break;
    case 3:
        // Function 3: Switch a pump's state
        Serial.println(F("--- Switch a single pump ON/OFF ---"));
        _changePumpSetting(SWITCH_PUMP_STATE);
        break;
    case 4:
        // Function 4: Switch all pumps to a state
        Serial.println(F("--- Switch all pumps ON/OFF ---"));
        _setAllPumpsState(SWITCH_ALL_PUMPS);
        break;
    case 5:
        // Function 5: Set a pump's mode to constant pressure or constant flow
        Serial.println(F("--- Switch a pump to Constant Pressure Mode ---"));
        Serial.println(F("P = Constant Pressure\nF = Constant Flow"));
        _changePumpSetting(PUMP_MODE);
        break;
    case 6:
        // Function 6: Close Differential Mode Valve
        Serial.println(F("--- Closing Differential Valve... ---"));
        _closeDiffValve();
        Serial.println(F("--- Valve Closed! ---"));
        break;
    case 7:
        // Function 7: Open Differential Mode Valve
        Serial.println(F("--- Opening Differential Valve... ---"));
        _openDiffValve();
        Serial.println(F("--- Valve Opened! ---"));
        break;
    case 8:
        // Function 8: Change Experiment Duration
        Serial.println(F("--- Change Experiment Duration ---"));
        _changeExperimentDur();
        break;
    case 9:
        // Function 9: Refresh network connections
        Serial.println(F("--- Refreshing Network Connections... ---"));

        _testNetworks();

        // Check for changes
        _checkNetworkStates();
        Serial.println(F("Refresh Done!"));
        break;
    case 0:
        // Function 0: Change Differential Pressure Setpoint
        Serial.println(F("--- Change Differential Pressure Setpoint ---"));
        _changeDiffPressSetpoint();
        break;
    default:
        Serial.println(F("### Unrecognised function number!"));
    }
}

/* -- Clear Serial Buffer --
    Function to remove remaining characters from the serial buffer
*/
void PressureControllerSerialUI::_clearSerialBuffer()
{
    while (Serial.available() > 0)
    {
        Serial.read();
        delay(3);
    }
}

/* -- Read Serial Data --
    Reads a fixed amount of characters from serial

    INPUTS:
        &safeString         - A safestring location where the collected data should be saved to
*/
void PressureControllerSerialUI::_readSerialData(SafeString &safeString)
{
    uint8_t maxNumChars = 12; // Reads this many characters
    uint8_t currCharIndex = 0;

    // Read pump character
    while (!Serial.available())
    {
        // Wait for input
    }

    while (Serial.available())
    {
        delay(2);
        if (currCharIndex >= maxNumChars) // Clear buffer and stop reading after certain number of characters
        {
            _clearSerialBuffer();
            break;
        }
        char c = Serial.read();
        safeString += c;
        currCharIndex++;
    }
}

/* -- Read Pump Command Input --
    Function to collect input from the user for controlling a pump

    INPUTS:
        *pump       - The address of a variable to store the pump letter
        *value      - The address of a variable to store the parameter value (float or boolean)
*/
// Pump Float
void PressureControllerSerialUI::_requestSerialData(char *pump, float *value)
{
    if (!_pumpsResponding)
    {
        Serial.println(F("Skipped pump float read command. (Network not responding.)"));
        return;
    }

    createSafeString(serialInput, 11);
    createSafeString(pumpString, 1);
    createSafeString(valueString, 10);

    // Read from serial
    _readSerialData(serialInput);

    // Extract strings
    serialInput.substring(pumpString, 0, 1);
    serialInput.substring(valueString, 1);

    // Post-process strings
    pumpString.toUpperCase();
    valueString.trim();

    // Return as required type
    *pump = pumpString.charAt(0);
    valueString.toFloat(*value);
}

// Pump State
void PressureControllerSerialUI::_requestSerialData(char *pump, bool *state)
{
    if (!_pumpsResponding)
    {
        Serial.println(F("Skipped pump state read command. (Network not responding.)"));
        return;
    }

    createSafeString(serialInput, 3);
    createSafeString(pumpString, 1);
    createSafeString(stateString, 3);

    // Read from serial
    _readSerialData(serialInput);

    // Extract strings
    serialInput.substring(pumpString, 0, 1);
    serialInput.substring(stateString, 1);

    // Post-process strings
    pumpString.toUpperCase();
    stateString.trim();

    stateString.toUpperCase();

    // Return as required type
    *pump = pumpString.charAt(0);
    char boolChar = stateString.charAt(0);
    _charToBool(state, boolChar);
}

// Generic Integer
void PressureControllerSerialUI::_requestSerialData(int *value)
{
    createSafeString(serialInput, 10);

    // Read from serial
    _readSerialData(serialInput);

    // Convert value to int
    serialInput.toInt(*value); // Use *value to assign to the memory location pointed by value
}

// Generic State
void PressureControllerSerialUI::_requestSerialData(bool *state)
{
    createSafeString(serialInput, 1);

    // Read from serial
    _readSerialData(serialInput);

    // Post-process strings
    serialInput.toUpperCase();

    // Return as required type
    char boolChar = serialInput.charAt(0);
    _charToBool(state, boolChar);
}

// Generic Float
void PressureControllerSerialUI::_requestSerialData(float *value)
{
    createSafeString(serialInput, 10);

    // Read from serial
    _readSerialData(serialInput);

    serialInput.toFloat(*value); // Use *value to assign to the memory location pointed by value
}

// ~~~ Subsection: RTC Functions ~~~ //
/* -- Setup RTC Alarms --
    First-time setup for the RTC alarms
*/
void PressureControllerSerialUI::_setupRTCAlarms()
{
    byte alarm2Bits = 0b01100000; // Alarm 2 when minutes match
    // This sets alarm 2 to an impossible value, so that it will never accidentally be triggered
    _rtc.setA2Time(
        _rtc.getDate(), _rtc.getHour(_twelveHour, _ampm), 0xFF,
        alarm2Bits, _alarmDayIsDay, _twelveHour, _ampm);
    _rtc.turnOffAlarm(2);
    _rtc.checkIfAlarm(2);

    // Disable Alarm 1
    _rtc.turnOffAlarm(1);
    _rtc.checkIfAlarm(1);
}

/* -- Change Experiment Duration --
    User interface function to change experiement duration
*/
void PressureControllerSerialUI::_changeExperimentDur()
{
    int days = 0;
    Serial.print(F("Current experiment duration: "));
    Serial.println(_alarmDays);
    Serial.println(F("Enter new experiment duration (whole days):"));
    _requestSerialData(&_alarmDays);

    Serial.print(F("Experiment duration set to: "));
    Serial.print(_alarmDays);
    Serial.println(F(" day(s)."));
}

/* -- Set Seconds Until Next Alarm --
    Function to set when the next alarm will be, in seconds
*/
void PressureControllerSerialUI::_setSecondsUntilNextAlarm(unsigned long *seconds)
{
    // Disable Alarm 1
    _rtc.turnOffAlarm(1);
    _rtc.checkIfAlarm(1);

    byte alarm1Bits = 0b00000000; // Alarm 1 when date, hours, minutes and seconds match

    // Get the current time as a DateTime type
    DateTime alarmDT = RTClib::now();

    // Create time of next alarm, starting from the current time
    uint32_t nextAlarm = alarmDT.unixtime();

    // Add on however many more seconds
    nextAlarm += *seconds;

    // Convert back to DateTime
    alarmDT = DateTime(nextAlarm);

    // Upload the new time to Alarm 1
    _rtc.setA1Time(
        alarmDT.day(), alarmDT.hour(), alarmDT.minute(), alarmDT.second(),
        alarm1Bits, _alarmDayIsDay, _twelveHour, _ampm);

    // Enable Alarm 1
    _rtc.turnOnAlarm(1);
    _rtc.checkIfAlarm(1);

    // Recalculate days
    unsigned long secondsInADay = 86400;

    Serial.print(F("Experiment set to last "));
    Serial.print(*seconds / secondsInADay);
    Serial.println(F(" day(s)."));
    Serial.print(F("Experiment will finish: "));
    _printTimeStamp(alarmDT);
}

// ~~~ Subsection: SD Card Functions ~~~ //
/* -- Setup SD --
    Function to initialise the SD card
*/
void PressureControllerSerialUI::_setupSD()
{
    // Initialise SD
    if (!SD.begin())
    {
        Serial.println(F("### Error initialising SD Card."));
        SDIsAvailable = false;
        return;
    }

    // Try to open the file for writing
    _txtFile = SD.open(_filename, FILE_WRITE);
    if (!_txtFile)
    {
        Serial.print(F("### Error opening SD card file: "));
        Serial.println(_filename);
        SDIsAvailable = false;
        return;
    }

    Serial.println(F("SD card detected!"));
    SDIsAvailable = true;
}

/* -- Print Headings to SD --
    Prints the column headings to the SD card
*/
void PressureControllerSerialUI::_printHeadingsToSD()
{
    // Start printing to a fresh line
    _txtFile.println();

    // Add Timestamp heading
    _txtFile.print(F("Time"));
    _txtFile.print("\t");

    // Add pump data headings
    if (_pumpsResponding)
    {
        _txtFile.print(F("Pressure A"));
        _txtFile.print("\t");
        _txtFile.print(F("Pressure A Setpoint"));
        _txtFile.print("\t");
        _txtFile.print(F("Flow A"));
        _txtFile.print("\t");
        _txtFile.print(F("Flow A Setpoint"));
        _txtFile.print("\t");
        _txtFile.print(F("Volume A"));
        _txtFile.print("\t");
        _txtFile.print(F("Pressure B"));
        _txtFile.print("\t");
        _txtFile.print(F("Pressure B Setpoint"));
        _txtFile.print("\t");
        _txtFile.print(F("Flow B"));
        _txtFile.print("\t");
        _txtFile.print(F("Flow B Setpoint"));
        _txtFile.print("\t");
        _txtFile.print(F("Volume B"));
    }

    // Add sensor data headings
    if (_diffSensorResponding)
    {
        _txtFile.print("\t");
        _txtFile.print(F("Diff Pressure"));
        _txtFile.print("\t");
        _txtFile.print(F("Temperature"));
    }

    if (_pumpsResponding || _diffSensorResponding)
    {
        _txtFile.println(); // Move to the next line
    }
}

/* -- Run SD --
    Function to write data to the SD card
*/
void PressureControllerSerialUI::_runSD()
{
    // Sync data as when its updated
    if (_SDWriteTimer.TRIGGERED)
    {
        _txtFile.write(_buffer.c_str()); // Write the buffered data to the file
        _txtFile.flush();                // Flush buffered data to the SD card
        _buffer = "";                    // Clear the buffer
    }

    // Add time to buffer
    _buffer += _rtc.getDate();
    _buffer += F("/");
    _buffer += _rtc.getMonth(_century);
    _buffer += F("/");
    _buffer += _rtc.getYear();
    _buffer += F(" ");
    _buffer += _rtc.getHour(_twelveHour, _ampm);
    _buffer += F(":");
    _buffer += _rtc.getMinute();
    _buffer += F(":");
    _buffer += _rtc.getSecond();
    _buffer += F("\t");

    // Add pump data to buffer
    // Pump A
    _buffer += _pumpASettings.pressure;
    _buffer += F("\t");
    _buffer += _pumpASettings.pressureSetpoint;
    _buffer += F("\t");
    _buffer += _pumpASettings.flowRate;
    _buffer += F("\t");
    _buffer += _pumpASettings.flowRateSetpoint;
    _buffer += F("\t");
    _buffer += _pumpASettings.volumeRemaining;
    _buffer += F("\t");

    // Pump B
    _buffer += _pumpBSettings.pressure;
    _buffer += F("\t");
    _buffer += _pumpBSettings.pressureSetpoint;
    _buffer += F("\t");
    _buffer += _pumpBSettings.flowRate;
    _buffer += F("\t");
    _buffer += _pumpBSettings.flowRateSetpoint;
    _buffer += F("\t");
    _buffer += _pumpBSettings.volumeRemaining;
    _buffer += F("\t");

    // Add sensor data to buffer
    _buffer += _diffPressSensor.differentialPressure;
    _buffer += F("\t");
    _buffer += _diffPressSensor.temperature;
    _buffer += F("\t");

    // Move to next line
    _buffer += F("\r\n");

    // check if the SD card is available to write data without blocking
    // and if the buffered data is enough for the full chunk size
    unsigned int chunkSize = _txtFile.availableForWrite();
    if (chunkSize && _buffer.length() >= chunkSize)
    {
        // write to file and blink LED
        digitalWrite(LED_BUILTIN, HIGH);
        _txtFile.write(_buffer.c_str(), chunkSize);
        digitalWrite(LED_BUILTIN, LOW);

        // remove written data from buffer
        _buffer.remove(0, chunkSize);
    }
}

// ~~~ Subsection: Printing Functions ~~~ //
/* -- Print Time Stamp --
    Function to print out current time and date as a timestamp
*/
void PressureControllerSerialUI::_printTimeStamp(DateTime dateAndTime)
{
    Serial.print(dateAndTime.day());
    Serial.print(F("/"));
    Serial.print(dateAndTime.month());
    Serial.print(F("/"));
    Serial.print(dateAndTime.year());
    Serial.print(F(" "));
    Serial.print(dateAndTime.hour());
    Serial.print(F(":"));
    Serial.print(dateAndTime.minute());
    Serial.print(F(":"));
    Serial.println(dateAndTime.second());
}

/* -- Print Current State --
    Function to print out the current state of the UI
*/
void PressureControllerSerialUI::_printCurrentState()
{
    switch (_currentState)
    {
    case STATE_SET_PARAMETERS:
        Serial.println(F("\n--- Current State: Set Parameters ---"));
        Serial.println(F("Review and revise the current Pressure Controller settings."));
        Serial.println(F("Enter 'h' for help"));
        Serial.println(F("Enter 'f' for the function list\n"));
        break;
    case STATE_EXPERIMENT:
        Serial.println(F("\n--- Current State: Experiment ---"));
        break;
    case STATE_DIFFERENTIAL:
        Serial.println(F("\n--- Current State: Auto Differential ---"));
        Serial.println(F("Notice: This mode is only compatible with BAR. Ensure the pump units are set to BAR."));
        break;
    case STATE_RESET:
        Serial.println(F("\n--- Resetting System... ---"));
        break;
    case STATE_FUNCTION:
        Serial.println(F("\n--- Current State: Excecute Function ---"));
        Serial.println(F("Enter a function number from the list, or 'c' to cancel."));
        Serial.println(F("\n- Pump Functions -"));
        Serial.println(F("1: Set a pump's pressure"));
        Serial.println(F("2: Set a pump's flowrate"));
        Serial.println(F("3: Turn on/off a single pump"));
        Serial.println(F("4: Turn on/off all pumps"));
        Serial.println(F("5: Set a pump mode"));
        Serial.println(F("\n- Valve Functions -"));
        Serial.println(F("6: Close Differential Valve"));
        Serial.println(F("7: Open Differential Valve"));
        Serial.println(F("\n- Alarm Functions -"));
        Serial.println(("8: Change Experiment Duration"));
        Serial.println(F("\n- Network Functions -"));
        Serial.println(("9: Refresh network connections"));
        Serial.println(F("\n- Differential Feedback Functions -"));
        Serial.println(("0: Change Differential Pressure Setpoint\n"));
        break;
    case STATE_HELP:
        Serial.println(F("\n--- Current State: Help --- "));
        Serial.println(F("s: Enter Set Parameters State - A neutral default state to adjust parameters in."));
        Serial.println(F("f: Function State - This intermediate state can be called at any time to adjust settings."));
        Serial.println(F("e: Enter Experiment State - State to start recording and displaying live data."));
        Serial.println(F("d: Enter Differential State - State to run experiment with automatic differential pressure adjustment"));
        Serial.println(F("r: Reset - Stops all pumps and opens differential valve. Then returns to Set Parameters.\n"));

        break;
    default:
        Serial.println(F("\nUnknown State Requested!\n"));
        break;
    }
}

/* -- Print Info --
    Function to update and print out the current readings of all the peripherals
*/
void PressureControllerSerialUI::_updateAllInfo()
{
    _checkNetworkStates();
    if (_pumpsResponding)
    {
        _network.updatePumpData(_pumpASettings, _pumpBSettings);
        _printPumpInfo();
    }

    if (_diffSensorResponding)
    {
        _network.updateSensorData(_diffPressSensor);
        _printSensorInfo();

        if ((_diffPressSensor.differentialPressure <= -0.9) || _diffPressSensor.differentialPressure >= 9.9)
        {
            _currentState = STATE_RESET;
            Serial.println(F("\n### WARNING: Differential sensor range about to be exceeded!"));
            Serial.println(F("### WARNING: Experiment cancelling..."));
        }
    }

    if (_pumpsResponding || _diffSensorResponding)
    {
        Serial.println(); // Move to the next line
    }
}

/* -- Print Headings --
    Function to print out headings based on which peripherals are available
*/
void PressureControllerSerialUI::_printHeadings()
{
    // Declare Strings
    createSafeString(unitMessage, 64);
    createSafeString(pressureUnit, 10);
    createSafeString(flowUnit, 10);

    // Create strings for units
    _enumToSafeString(_currentPressureUnit, pressureUnit);
    _enumToSafeString(_currentFlowUnit, flowUnit);

    unitMessage = unitMessage.concat(F("Pressure in ")).concat(pressureUnit).concat(F(". Flow in ")).concat(flowUnit);
    Serial.print(unitMessage);
    unitMessage = unitMessage.clear().concat(F(". Differential Pressure in BAR. Temperature in Celcius."));
    Serial.println(unitMessage);

    if (_pumpsResponding)
    {
        Serial.print(F("Pressure A"));
        Serial.print("\t");
        Serial.print(F("Flow Rate A"));
        Serial.print("\t");
        Serial.print(F("Volume A"));
        Serial.print("\t");
        Serial.print(F("Pressure B"));
        Serial.print("\t");
        Serial.print(F("Flow Rate B"));
        Serial.print("\t");
        Serial.print(F("Volume B"));
        Serial.print("\t");
    }

    if (_diffSensorResponding)
    {
        Serial.print(F("Diff Pressure"));
        Serial.print("\t");
        Serial.print(F("Temperature"));
    }

    if (_pumpsResponding || _diffSensorResponding)
    {
        Serial.println(); // Move to the next line
    }
}

/* -- Print Pump Info --
    Function to print out pump data
*/
void PressureControllerSerialUI::_printPumpInfo()
{
    // Pressure A
    Serial.print(_pumpASettings.pressure, 2);
    Serial.print(F("/"));
    Serial.print(_pumpASettings.pressureSetpoint, 2);
    Serial.print("\t");

    // Flow A
    Serial.print(_pumpASettings.flowRate, 2);
    Serial.print(F("/"));
    Serial.print(_pumpASettings.flowRateSetpoint, 2);
    Serial.print("\t");

    // Volume A
    Serial.print(_pumpASettings.volumeRemaining, 2);
    Serial.print("\t\t");

    // Pressure B
    Serial.print(_pumpBSettings.pressure, 2);
    Serial.print(F("/"));
    Serial.print(_pumpBSettings.pressureSetpoint, 2);
    Serial.print("\t");

    // Flow B
    Serial.print(_pumpBSettings.flowRate, 2);
    Serial.print(F("/"));
    Serial.print(_pumpBSettings.flowRateSetpoint, 2);
    Serial.print("\t");

    // Volume B
    Serial.print(_pumpBSettings.volumeRemaining, 2);
    Serial.print("\t\t");
}

/* -- Print Sensor Info --
    Function to print out sensor data
*/
void PressureControllerSerialUI::_printSensorInfo()
{
    Serial.print(_diffPressSensor.differentialPressure, 2);
    Serial.print(F("/"));
    Serial.print(_diffPressSensor.diffPressSetpoint, 2);
    Serial.print("\t");

    Serial.print(_diffPressSensor.temperature, 2);
    Serial.print("\t");
}

// ~~~ Subsection: Peripheral Functions ~~~ //

// Air Valve Functions
/* -- Close Differential Mode Valve --
    Function to close the air-operated valve and enter differential mode
*/
void PressureControllerSerialUI::_closeDiffValve()
{
    _network.switchAirValve(IN, true);
    delay(3000);
    _network.switchAirValve(OUT, false);
    delay(100);
    _airPurged = false;
}

/* -- Open Differential Mode Valve --
    Function to release the air from the air valve
*/
void PressureControllerSerialUI::_openDiffValve()
{
    _network.switchAirValve(IN, false);
    delay(100);
    _network.switchAirValve(OUT, true);
    delay(3000);
    _network.switchAirValve(OUT, false);
    delay(100);
    _airPurged = true;
}

// ~~~ Subsection: Configuration Functions ~~~ //
void PressureControllerSerialUI::_instantiatePumps()
{
    // Skip instantiation if pump communication fails
    if (!_pumpsResponding)
    {
        return;
    }

    // Read units:
    _currentPressureUnit = _network.readPressureUnit();

    if (_currentPressureUnit != BAR)
    {
        Serial.println(F("### Warning: System units have been automatically set to BAR."));
        _currentPressureUnit = BAR;
        _network.changePressureUnit(BAR);
    }

    _currentFlowUnit = _network.readFlowUnit();

    // Read modes:
    _pumpASettings.mode = _network.readPumpMode(A);
    _pumpBSettings.mode = _network.readPumpMode(B);

    // Populate pump data:
    _network.updatePumpData(_pumpASettings, _pumpBSettings);

    pumpsInstantiated = true; // Mark pumps as instantiated!
    Serial.println(F("Done instantiating pumps!"));
}

/* -- Check Network States --
    Function to respond to changes in network availibility
*/
void PressureControllerSerialUI::_checkNetworkStates()
{
    // Test pump network connection
    _pumpsResponding = _network.pumpNetworkActive;
    if (_prevPumpsResponding != _pumpsResponding) // Responding state has changed
    {
        if (_pumpsResponding) // Pumps have started responding
        {
            Serial.println(F("Pump network connection made."));
            if (!pumpsInstantiated)
            {
                Serial.println(F("Instantiating pumps..."));
                _instantiatePumps();
            }
        }
        else // Pumps have stopped responding
        {
            Serial.println(F("### Pump network connection lost!"));
        }
        _prevPumpsResponding = _pumpsResponding;
    }

    else if (!_pumpsResponding) // Failed to connect to pumps
    {
        Serial.println(F("### Failed to connect to pump network!"));
    }

    // Test sensor network connection
    _diffSensorResponding = _network.sensorNetworkActive;
    if (_prevDiffSensorResponding != _diffSensorResponding) // Responding state has changed
    {
        if (_diffSensorResponding) // Sensor has started responding
        {
            Serial.println(F("Differential sensor network connection made."));
        }
        else // Sensor has stopped responding
        {
            Serial.println(F("### Differential sensor network connection lost!"));
        }
        _prevDiffSensorResponding = _diffSensorResponding;
    }

    else if (!_prevDiffSensorResponding) // Failed to connect to sensor
    {
        Serial.println(F("### Failed to connect to to differential sensor network!"));
    }

    // Exit experiment if necessary
    if (_experimentStarted && (!_pumpsResponding || !_diffSensorResponding))
    {
        Serial.println(F("### WARNING: Exiting experiment due to network connection failure!"));
        _currentState = STATE_RESET;
    }
}

/* -- Test Networks --
    Function that sends test commands to each of the modbus networks to check for the presence of the device
*/
void PressureControllerSerialUI::_testNetworks()
{
    // Functions to check connection
    _network.readPressureUnit();
    _network.readTemperature();
    _setupSD();
}

// ~~~ Subsection: Command Utilites ~~~ //
/* -- Change Pump Setting --
    Function to facilitate parameter-setting pump functions (setting floating point values to registers)

    INPUTS:
        isPressure      - Boolean to identidy whether the function controls pressure or flow
        command         - The desired pump command
*/
void PressureControllerSerialUI::_changePumpSetting(PumpCommand command, bool isPressure = true)
{
    // Skip this function if the pump controller is not responding
    if (!_pumpsResponding)
    {
        Serial.println(F("Skipped function: 'Change Pump Setting' (Network not responding.)"));
        return;
    }

    // Shared Strings
    createSafeString(messageString, 70);
    createSafeString(functionString, 13);

    // State Strings
    createSafeString(stateString, 22);

    // Parameter Strings
    createSafeString(modeString, 9);
    createSafeString(unitString, 8);

    // Initialise input containers
    char pumpChar = '\0';
    bool state = false;
    float value = 0.0;

    // Identify command type:
    bool isStateCommand = false;
    if ((command == SWITCH_PUMP_STATE) || (command == PUMP_MODE))
    {
        isStateCommand = true;
    }

    // Print correct message:
    if (isStateCommand)
    {
        Serial.print(F("Enter pump (A or B) and desired action state"));
        if (command == PUMP_MODE)
        {
            Serial.println(F(" (P or F)"));
        }
        else
        {
            Serial.println(F(" (T or F)"));
        }

        _requestSerialData(&pumpChar, &state);
    }
    else
    {
        // Assign content to the mode and unit strings
        modeString = isPressure ? "pressure" : "flow rate";
        isPressure ? _enumToSafeString(_currentPressureUnit, unitString) : _enumToSafeString(_currentFlowUnit, unitString);

        // Create and send the data request message
        messageString = messageString.concat("Enter pump (A or B) and ").concat(modeString).concat(" (in ").concat(unitString).concat("):");
        Serial.println(messageString);
        messageString = "";

        _requestSerialData(&pumpChar, &value);
    }

    // Process the serial input to get the pump:
    PumpLetter pump = _pumpCharToEnum(pumpChar);

    // Verify pump letter was read successfully:
    if (pump == N)
    {
        Serial.println(F("### Error: Failed to parse pump letter."));
        return;
    }

    // Identify specific command and print out a custom message:
    if (isStateCommand)
    {
        switch (command)
        {
        case SWITCH_PUMP_STATE:
            turnPumpToState(pump, state);
            stateString = (state) ? "ON" : "OFF";
            functionString = " switched ";
            break;
        case PUMP_MODE:
            setPumpMode(pump, state);
            stateString = (state) ? "Constant Pressure Mode" : "Constant Flow Mode";
            functionString = " switched to ";
            break;
        default:
            Serial.println(F("### Invalid pump state command!"));
            break;
        }

        messageString = messageString.concat("Pump ").concat(pumpChar).concat(functionString).concat(stateString).concat("\n");
        Serial.println(messageString);
    }
    else
    {
        // Verify command matches mode
        // Warn the user if the function doesn't match the setting of their pump:
        bool isPumpA = (pump == _pumpASettings.pumpLetter); // If true, we're looking at pump A

        // First get the current modes, in case They have changed:
        _pumpASettings.mode = _network.readPumpMode(A);
        _pumpBSettings.mode = _network.readPumpMode(B);

        // Then run verification
        PumpMode currPumpMode = (isPumpA ? _pumpASettings.mode : _pumpBSettings.mode); // Find what the current mode is based on what pump we're looking at
        bool isCP = (currPumpMode == CONSTANT_PRESSURE);                               // If true, we're in constant pressure mode
        if ((isPressure && !isCP) || (!isPressure && isCP))
        {
            Serial.println(F("### Error: Parameter does not match pump mode! Please change pump mode first."));
            return;
        }

        switch (command)
        {
        case SET_PRESSURE:
            setPumpPressure(pump, value);
            break;
        case SET_MAX_PRESSURE:
            functionString = "maximum";
            break;
        case SET_MIN_PRESSURE:
            functionString = "minimum";
            break;
        case SET_FLOW_RATE:
            setPumpFlowrate(pump, value);
            break;
        case SET_MAX_FLOW_RATE:
            functionString = "maximum";
            break;
        case SET_MIN_FLOW_RATE:
            functionString = "minimum";
            break;
        default:
            Serial.println(F("### Invalid setting action!"));
            break;
        }

        messageString = messageString.concat("Pump ").concat(pumpChar).concat(" ").concat(functionString).concat(modeString).concat(" setpoint set to ").concat(value).concat(" ").concat(unitString).concat("\n");
        Serial.println(messageString);
    }
}

/* -- Set All Pumps State --
    Function to facilitate state-setting functions applied to all pumps (setting coil values)

    INPUTS:
        command         - The desired pump command
*/
void PressureControllerSerialUI::_setAllPumpsState(PumpCommand command)
{
    // Skip this function if the pump controller is not responding
    if (!_pumpsResponding)
    {
        Serial.println(F("Skipped function: 'Set All Pumps State' (Network not responding.)"));
        return;
    }

    // Blank strings for presenting what the function did
    createSafeString(messageString, 64, "");
    createSafeString(functionString, 9, "");
    createSafeString(stateString, 4, "");

    // Send the data request message
    Serial.println(F("Enter desired action state (T or F)"));

    bool state = false; // Initialise with zero as false

    // Process the serial input to get the pump and value:
    _requestSerialData(&state);

    // Set the pump's parameter based on the action
    switch (command)
    {
    case SWITCH_ALL_PUMPS:
        turnAllPumpsToState(state);
        stateString = (state) ? "ON" : "OFF";
        functionString = "switched ";
        break;
    default:
        Serial.println(F("### Invalid all-pumps state command!"));
        break;
    }

    // Print out a custom message based on the action performed:
    messageString = messageString.concat("All pumps ").concat(functionString).concat(stateString).concat("\n");
    Serial.println(messageString);
}

/* -- Change Differential Pressure Setpoint --
    Function to change the differential pressure setpoint
*/
void PressureControllerSerialUI::_changeDiffPressSetpoint()
{
    // Obtain new desired setpoint
    Serial.print(F("Current differential pressure setpoint (in BAR): "));
    Serial.println(_diffPressSensor.diffPressSetpoint);
    Serial.println(F("Enter new setpoint:"));
    _requestSerialData(&_diffPressSensor.diffPressSetpoint);

    // Apply new setpoint
    Serial.print(F("New differential pressure setpoint: "));
    Serial.print(_diffPressSensor.diffPressSetpoint);
    Serial.println(F(" BAR"));

    // Reset the PID controller
    _newControlCycle = true;
}

/* -- Run Differential Controller --
    Function to automatically adjust the pump B pressure
*/
void PressureControllerSerialUI::_runDifferentialController()
{
    // Initialise static variables
    static float previousError = 0;
    static float errorIntegral = 0;

    if (!_feedbackTimer.TRIGGERED) // Only drive controller if there's fresh data
    {
        return;
    }

    if (_newControlCycle) // Refresh static variables for a new control cycle
    {
        previousError = 0;
        errorIntegral = 0;
        _newControlCycle = false;
    }
    // TO DO:
    // Tune the constants in real life - give the controller more power if it's not making it up to the setpoint in a few seconds
    // Make this code compatible with other pressure units in the pump:
    // Check current unit setting, then convert it to bar for use in here, the convert it back before writing it to the pump

    // Coefficients
    const float Kp = 0.1;  // Proportional constant
    const float Ki = 0.05; // Integral constant
    const float Kd = 0.1;
    const float dt = 1; // Time increment to integrate over
    const float integralLimit = 2;
    const float maximumPressure = 300; // in bar

    // Calculate Error
    float error = _diffPressSensor.diffPressSetpoint - _diffPressSensor.differentialPressure; // Calculate the error

    // Calculate Integral of Error
    errorIntegral = errorIntegral + (error * dt); // Calculate the error integral

    // Cap the value of the integral
    if (abs(errorIntegral) > integralLimit)
    {
        errorIntegral = (errorIntegral > 0) ? integralLimit : -integralLimit; // Depending on the sign of the intergral, set to the limiting value accordingly
    }

    // Calculate Differential of Error
    float errorDifferential = (error - previousError) / dt;
    previousError = error;

    // Recalculating the new pressure setpoint
    float newPressure = _pumpBSettings.pressure + (Kp * error) + (Ki * errorIntegral) + (Kd * errorDifferential); // Modify the current pressure

    // Cap the value of the new pressure
    if (newPressure > maximumPressure)
    {
        newPressure = maximumPressure;
    }

    _network.setPumpPressure(B, newPressure); // Send the new pressure to pump B
}

/* -- Initialise Experiment --
    Function to startup experiment
*/
void PressureControllerSerialUI::_initialiseExperiment()
{
    if (!_experimentStarted)
    {
        // Verify that the networks are available before starting (if they were thought to be available)
        if (_pumpsResponding && _diffSensorResponding)
        {
            _testNetworks();
            _checkNetworkStates();
        }

        // If not responding, exit this state!
        if (!_pumpsResponding || !_diffSensorResponding)
        {
            Serial.println(F("### Error: Network fault. Refresh networks before starting experiment."));
            _currentState = STATE_SET_PARAMETERS;
            _printCurrentState();
            return;
        }

        // Set alarm for when to stop experiment
        unsigned long secsInDay = 86400;
        unsigned long seconds = _alarmDays * secsInDay;
        _setSecondsUntilNextAlarm(&seconds);

        // Write data stream headings
        _printHeadings();
        _printHeadingsToSD();

        // Mark experiment as started
        _experimentStarted = true;

        // Mark new control cycle
        _newControlCycle = true;
    }
}

/* -- Run Experiment --
    Function to run experiment
*/
void PressureControllerSerialUI::_runExperiment()
{
    // Update the data log on-screen and on SD
    if (_dataStreamTimer.TRIGGERED)
    {
        _updateAllInfo();
        _runSD();
    }
}

// Struct Updaters
/* -- Update All --
    Function to update data for all peripherals
*/
void PressureControllerSerialUI::_updateAllDataStructs()
{
    _updatePumpSetpoints();
    _updateCurrPumpValues();
    _updateCurrSensorValues();
}

/* -- Update Pump Setpoints --
    Function to update setpoints for each pump
*/
void PressureControllerSerialUI::_updatePumpSetpoints()
{
    // Pump A
    _pumpASettings.pressureSetpoint = _network.readPressureSetpoint(A);
    _pumpASettings.flowRateSetpoint = _network.readFlowSetpoint(A);

    // Pump B
    _pumpBSettings.pressureSetpoint = _network.readPressureSetpoint(B);
    _pumpBSettings.flowRateSetpoint = _network.readFlowSetpoint(B);
}

/* -- Update Current Pump Values --
    Function to update current values for each pump
*/
void PressureControllerSerialUI::_updateCurrPumpValues()
{
    // Pump A
    _pumpASettings.pressure = _network.readCurrentPressure(A);
    _pumpASettings.flowRate = _network.readCurrentFlow(A);

    // Pump B
    _pumpBSettings.pressure = _network.readCurrentPressure(B);
    _pumpBSettings.flowRate = _network.readCurrentFlow(B);
}

/* -- Update Current Sensor Values --
    Function to update current values for each sensor
*/
void PressureControllerSerialUI::_updateCurrSensorValues()
{
    // Differential Pressure Sensor
    _diffPressSensor.differentialPressure = _network.readDiffPressure();
    _diffPressSensor.temperature = _network.readTemperature();
}

// Formatting Functions
/* -- Enum To Safe String --
    Function to convert an enum into a safe string, for use in printing

    INPUTS:
        unit       - A pressure or flow unit
*/
void PressureControllerSerialUI::_enumToSafeString(PressureUnit &unit, SafeString &safeUnit)
{
    switch (unit)
    {
    case ATM:
        safeUnit = "ATM";
        break;
    case BAR:
        safeUnit = "BAR";
        break;
    case KPA:
        safeUnit = "kPa";
        break;
    case PSI:
        safeUnit = "PSI";
        break;
    default:
        safeUnit = "Unknown Unit";
        break;
    }
}

void PressureControllerSerialUI::_enumToSafeString(FlowUnit &unit, SafeString &safeUnit)
{
    switch (unit)
    {
    case ML_MIN:
        safeUnit = "mL/min";
        break;
    case ML_HR:
        safeUnit = "mL/hour";
        break;
    case UL_MIN:
        safeUnit = "uL/min";
        break;
    case UL_HR:
        safeUnit = "uL/hour";
        break;
    default:
        safeUnit = "Unknown Unit";
        break;
    }
}

void PressureControllerSerialUI::_enumToSafeString(PumpMode &mode, SafeString &safeUnit)
{
    switch (mode)
    {
    case CONSTANT_PRESSURE:
        safeUnit = "Constant Pressure";
        break;
    case CONSTANT_FLOW:
        safeUnit = "Constant Flow";
        break;
    default:
        safeUnit = "Unknown Mode";
        break;
    }
}

char PressureControllerSerialUI::_enumToChar(PumpLetter &letter)
{
    switch (letter)
    {
    case A:
        return 'A';
        break;
    case B:
        return 'B';
        break;
    default:
        return 'B';
        break;
    }
}

/* -- Pump Char to Enum --
    Function to convert a pump character back into an enum

    INPUTS:
        pump       - The character of a pump
    OUTPUTS:
        Returns the character as a pump letter enum
*/
PumpLetter PressureControllerSerialUI::_pumpCharToEnum(char pump)
{
    switch (pump)
    {
    case 'a':
    case 'A':
        return A;
        break;
    case 'b':
    case 'B':
        return B;
        break;
    default:
        Serial.println(F("### Error: Failed to convert pump character to PumpLetter"));
        return N;
        break;
    }
}

void PressureControllerSerialUI::_charToBool(bool *state, char boolChar)
{
    switch (boolChar)
    {
    case 'T':
    case 'P':
        *state = true;
        break;
    case 'F':
        *state = false;
        break;
    default:
        Serial.println(F("Unrecognised state character. State set to false by default"));
        *state = false;
        break;
    }
}
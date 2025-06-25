//Rack Robotics, Inc.
//Powercore Development Firmware
//For use with Rarduino-Pico Plugin by EarlePhilhower
//Copyright (c) 2025 Rack Robotics, Inc.
//All rights reserved.
//This firmware is for Powercore hardware revision E. 

// Software Version
const String SOFTWARE_VERSION = "1.0.0-beta";

//=============================================================================
// INCLUDES
//=============================================================================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h> 
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"

//=============================================================================
// ENUMERATIONS
//=============================================================================
// Device States
enum DeviceState {
    STARTUP,                            // Device is in startup state
    PERIPHERAL_MANAGEMENT,              // Device is in peripheral management state
    FAULT,                              // Device is in fault state
    OPERATING,                          // Device is in operating in a mode
    IDLE                                // Device is in idle state
};
// Modes of Operation of device
enum ModeOfOperation {
    EDM_ISOFREQUENCY_MODE,              // EDM operating in isofrequency mode, where the frequency of discharges is fixed to the machining frequency
    EDGE_DETECTION_MODE                 // Detect the edge of a conductive workpiece with a single pulse; report the edge via the EDM_FEEDBACK pin
};

// Fault State Types
enum FaultStateType {
    PMM_FAULT_TYPE,                     // Fault state due to the PMM current exceeding the maximum allowed current
    BUCK_CONVERTER_PGOOD_FAULT,         // Fault state due to the buck converter PGOOD pin being low
    BOOST_CONVERTER_PGOOD_FAULT,        // Fault state due to the boost converter PGOOD pin being low
    POWER_OUT_OF_RANGE_FAULT,           // Fault state due to the power input exceeding the maximum allowed power
    HIGH_VOLTAGE_PHASE_SETUP_FAULT      // Fault state due to the high voltage phase setup being incorrect
};

//=============================================================================
// STRUCTURES
//=============================================================================
// Command Parameter Types
struct OutputParameters {
    int dischargeCountTarget;
    float dutyCycle;
    float frequency;
    float initVoltage;
};

// PWM Output Configuration Structure
struct PWMOutput {
    uint32_t slice;
    uint32_t channel;
};

//=============================================================================
// PIN DEFINITIONS
//=============================================================================
// Digital Input/Output Pins
const int EDM_ENABLE              =   2;                // Digital input
const int EDM_FEEDBACK            =   3;                // PWM output for communicating with external motion controller
const int OUTPUT_OVERCURRENT      =   12;               // Reports when detected current exceeds current limit threshold on output current sensor
const int BUCK_POWER_GOOD         =   13;               // Reports when buck-converter module is providing power to system, active-LOW when fault is detected
const int BUCK_ENABLE             =   14;               // Enables buck-controller on buck-converter-module
const int BUCK_ADJ_PWM            =   15;               // UNUSED

const int I2C_SDA_PIN             =   16;               // I2C data pin
const int I2C_SCL_PIN             =   17;               // I2C clock pin

const int BOOST_PGOOD             =   18;               // Reports when boost-converter module is providing power to system, active-LOW when fault is detected
const int PMM_FAULT               =   20;               // Reports problems with power-management module, active-LOW when fault is detected
const int PMM_DIAG_EN             =   21;               // Enables reporting of faults from power-management module
const int PMM_ENABLE              =   22;               // Enables power-management module high-side switch, providing power to system
const int STATUS_LED              =   25;               // Status LED on Pico

// PWM Output Pins
const int SW_HIGH_VOLTAGE_PHASE   =   8;                // PWM #4 Channel A, Switch for connecting high-voltage phase of dual-converter to output. Controlled by P-channel MOSFET (logic is inverted, HIGH = OFF LOW = ON)
const int OUTPUT_OVERCURRENT_SET  =   9;                // PWM #4 Channel B, Sets current limit threshold using PWM, on output current sensor TMCS1133
const int SW_ENABLE               =   10;               // PWM #5 Channel A, Switch for connecting output (-)-electrode to GROUND through output current sensor. Controlled by N-channel MOSFET (Logic is not inverted, HIGH = ON LOW = OFF)
const int SW_HIGH_CURRENT_PHASE   =   11;               // PWM #5 Channel B, Switch for connecting high-current phase of dual-converter to output. Controlled by P-channel MOSFET (logic is inverted, HIGH = OFF LOW = ON)

// Analog Input Pins
const int PMM_ISENSE              =   26;               // Reports current from power-management module current sensor as analog voltage
const int OUTPUT_VSENSE           =   27;               // Reports voltage from output voltage sensor as analog voltage
const int OUTPUT_ISENSE           =   28;               // Reports current from output current sensor as analog voltage

//=============================================================================
// Variables
//=============================================================================

// Machining Parameters for EDM Isofrequency Mode
volatile double machiningDutyCycle = 0.10f;             // Duty cycle for machining (0.0 to 1.0)
volatile int machiningFrequency = 10000;                // Frequency in Hz
volatile int machiningInitVoltage = 80;                 // Initiation voltage in volts

// Discharge Success Rate Threshold
volatile double maxDischargeSuccessRateThreshold = 0.80f; // Maximum discharge success rate threshold

// Feature Flags
const bool sendPeriodicTelemetryEnabled = true;         // If true, the device will send periodic telemetry over serial
const bool allowPulseSkipping = true;                   // If true, the device will allow pulse skipping in order to attempt to clear/reduce faults

// Boost Converter Module Globals 
const int MIN_HIGH_VOLTAGE_PHASE_VOLTS = 64;            // Minimum safe high voltage phase voltage in volts
const int MAX_HIGH_VOLTAGE_PHASE_VOLTS = 150;           // Maximum safe high voltage phase voltage in volts
const int MIN_BOOST_CONVERTER_DPOT_POSITION = 0;        // Minimum position of the digital potentiometer on the boost converter module for safe operation
const int MAX_BOOST_CONVERTER_DPOT_POSITION = 110;      // Maximum position of the digital potentiometer on the boost converter module for safe operation
const int DEFAULT_BOOST_CONVERTER_DPOT_POSITION = 1;    // Default position of the digital potentiometer on the boost converter module
const int BOOST_CONVERTER_RAMP_SPACING_MS = 10;         // Spacing between steps in the digital potentiometer, in milliseconds

// Look up table for boost converter module digital potentiometer voltages
volatile int dpotVoltageTable[MAX_BOOST_CONVERTER_DPOT_POSITION + 1];

// I2C Configuration Constants
#define I2C_PORT i2c0                                   // I2C port for the digital potentiometer
#define DPOT_ADDR 0x3E                                  // Address of the digital potentiometer
#define DPOT_REG 0x00                                   // Register address of the digital potentiometer
#define I2C_BAUD_RATE_HZ 10000                          // Baud rate for I2C communication

// System Configuration Constants
const int ADC_RESOLUTION_BITS = 12;                     // ADC resolution in bits (0-4095)
const uint32_t PWM_BASE_CLOCK_FREQ = 200000000;         // Base clock frequency for PWM (200MHz)
const int MAX_PARAMETERS = 4;                           // Maximum number of parameters for command processing
volatile uint32_t edmFeedbackPwmFrequency_Hz = 10000;   // Frequency for EDM_FEEDBACK PWM in Hz
const uint16_t EDM_FEEDBACK_PWM_CLOCK_DIVIDER = 10;     // Set to 10 to fit within uint8_t

// Timing Constants
const int PMM_INRUSH_DELAY_MS = 500;                    // Delay after enabling PMM to allow in-rush current to settle
const unsigned long SERIAL_INIT_TIMEOUT_MS = 2000;      // Timeout duration for serial initialization (2 seconds)
const int serialInitializationBlinkInterval_MS = 500;   // Interval for serial initialization (500 ms)
const int periodicTelemetryInterval_MS = 1000;          // Interval for sending periodic telemetry (1000 ms)
volatile int dischargeRateCalculationInterval_MICROS = 0;  // Interval for calculating discharge success rate (is calculated from machining frequency)

//  Parameter Limits
const float MIN_ON_TIME_MICROS = 0.625f;                // Minimum on-time in microseconds
const float MAX_ON_TIME_MICROS = 160.0f;                // Maximum on-time in microseconds
const float MIN_OFF_TIME_MICROS = 10.625f;              // Minimum off-time in microseconds
const float MIN_MACHINING_FREQUENCY_HZ = 5000.0f;       // Minimum machining frequency in Hz
const float MAX_MACHINING_FREQUENCY_HZ = 64000.0f;      // Maximum machining frequency in Hz
const float MIN_DUTY_CYCLE = 0.01f;                     // Minimum duty cycle (1%)
const float MAX_DUTY_CYCLE = 0.15f;                     // Maximum duty cycle for pi-filter module
const float MIN_INIT_VOLTAGE = MIN_HIGH_VOLTAGE_PHASE_VOLTS;    // Minimum initiation voltage (exclusive)
const float MAX_INIT_VOLTAGE = MAX_HIGH_VOLTAGE_PHASE_VOLTS;    // Maximum initiation voltage in volts
const float MAX_SAFE_INPUT_CURRENT = 7.3f;              // Maximum safe input current in amps

// PMM_ISENSE Monitoring Constants
const float ADC_TO_VOLTS = 3.3f / 4095.0f;             // Constant for converting 12-bit ADC into volts
const float PMM_ISENSE_V_PER_AMP = 0.200f;             // 200 mV per amp for PMM current sensor
const int DEVICE_CURRENT_BUFFER_SIZE = 100;            // Size of buffer for device current running average
const int PMM_CALIBRATION_SAMPLES = 100;               // Number of samples to take for PMM current sensor calibration
const int PMM_CALIBRATION_DELAY_MS = 1;                // Delay between PMM calibration samples in milliseconds

// Output Current Sensor Constants
const float ISENSE_OUTPUT_V_PER_AMP = 0.025f;           // 25 mV per amp for output current sensor (TMCS1133C1A)
const int ISENSE_OUTPUT_AMPS_PER_VOLT = 40;             // 40 amps per volt for output current sensor (TMCS1133C1A)
const float OUTPUT_CURRENT_ZERO_OFFSET = 0.33f;         // Zero current offset of output current sensor (TMCS1133C1A), in volts
const int DEFAULT_OUTPUT_CURRENT_THRESHOLD_A = 8;       // Default threshold for output current sensor (40 A)
const int OUTPUT_CURRENT_SENSOR_CALIBRATION_SAMPLES = 100;  // Number of samples to take for output current sensor calibration
const int OUTPUT_CURRENT_SENSOR_CALIBRATION_DELAY_MS = 1;  // Delay between output current sensor calibration samples in milliseconds

// Output Voltage Sensor Constants
const float OUTPUT_VOLTAGE_SCALE = 0.08090972f;        // Pre-computed (3.3/4095) * 100.4 for output voltage scaling with 99.6:1 voltage divider

// Fault Handling Configuration
const bool PMM_FAULT_IN_USE = true;                     // Should always be true
const bool HIGH_CURRENT_PGOOD_IN_USE = false;           // Set to false if Pi-Filter module is used in high-current phase
const bool HIGH_VOLTAGE_PGOOD_IN_USE = true;            // Set to true if boost-converter module is used in high-voltage phase
const bool dynamicFaultHandlingEnabled = true;          // If true, the device will dynamically handle faults based on the fault type

// Device State Variables
volatile DeviceState deviceState = STARTUP;             // Current device state
volatile DeviceState oldDeviceState = STARTUP;          // Previous device state, used for temporary state storage
volatile ModeOfOperation modeOfOperation = EDM_ISOFREQUENCY_MODE;  // Current mode of operation
volatile FaultStateType FaultStateType = PMM_FAULT_TYPE;// Current fault type
volatile bool modePreparationComplete = false;          // Flag to track if mode preparation is complete
volatile bool isInFaultState = false;                   // Flag indicating if device is in fault state
volatile bool enablePortStatus = false;                 // Status of the enable port

// Timing Variables
volatile unsigned long currentTime_MS = 0;              // Current time in milliseconds
volatile unsigned long lastTelemetryEventTime_MS = 0;   // Time of last telemetry event
volatile unsigned long lastPeripheralManagementEventTime_MS = 0;  // Time of last peripheral management event
volatile unsigned long inrushStartTime_MS = 0;          // Time when inrush started
volatile unsigned long serialInitStartTime_MS = 0;      // Time when serial initialization started
volatile int peripheralManagementInterval_MS = 10;      // Interval for peripheral management in milliseconds
volatile int inrushDelay_MS = 500;                      // Delay for inrush in milliseconds
volatile int timeSinceLastCalculation_MICROS = 0;       // Time elapsed since last discharge success rate calculation

// Discharge Control Variables
volatile int dischargeCountTarget = 0;                  // Target count of discharges (0 for infinite, positive integer for finite)
volatile int currentDischargeCount = 0;                 // Counter for number of discharges completed
volatile int dischargesSinceOperationStarted = 0;       // Counter for number of discharges since operation started
volatile double dischargeSuccessRate = 0.00f;           // Current discharge success rate
volatile bool requestedDischargesReached = false;       // Flag to indicate if the requested number of discharges has been reached
volatile double newDischargeSuccessRate = 0.00f;        // New discharge success rate

// Pulse Skipping Variables
volatile bool pulseSkippingEnabled = true;              // Flag to enable pulse skipping safety feature
volatile int dischargesPerCalculationInterval = 5;      // Number of discharges per calculation interval

// Current Measurement Variables
volatile double inputCurrentReading = 0.00f;            // Current reading from PMM_ISENSE
volatile int pmmISENSEAdcValue = 0;                     // Raw ADC reading from PMM current sensor
volatile float deviceInputCurrentBuffer[DEVICE_CURRENT_BUFFER_SIZE];  // Circular buffer to store device current samples
volatile int deviceInputCurrentBufferIndex = 0;         // Current index in the circular buffer
volatile bool deviceInputCurrentBufferFull = false;     // Flag to indicate if buffer has been filled at least once
volatile float avgDeviceInputCurrent = 0.0f;            // Running average of device input current (from PMM_ISENSE)
volatile int pmmZeroCurrentOffset = 0;                  // ADC value when no current is flowing
volatile long pmmCalibrationSum = 0;                    // Sum of ADC samples during PMM current sensor calibration
volatile float inputCurrentBufferSum = 0.0f;            // Sum of input current buffer values

// Fault Measurement Variables
volatile double inputCurrentDuringFault = 0.0f;         // Input current reading during fault condition
volatile double outputVoltageDuringFault = 0.0f;        // Output voltage reading during fault condition
volatile double outputCurrentDuringFault = 0.0f;        // Output current reading during fault condition

// Output Current Sensor Calibration Variables
volatile int outputCurrentSensorCalibrationSum = 0;     // Sum of ADC samples during output current sensor calibration
volatile int outputCurrentSensorZeroCurrentOffset = 0;  // ADC value when no current is flowing through output current sensor

// Edge Detection Mode Variables
volatile uint32_t edgeDetectionWrapValue = 0;           // Wrap value for edge detection mode PWM
volatile uint32_t edgeDetectionLevelValue = 0;          // Level value for edge detection mode PWM
volatile bool edgeDetected = false;                     // Flag indicating if an edge has been detected in edge detection mode

// Global PWM Output Configuration
PWMOutput pwmOutputs[4];                                // Array to hold all PWM output configurations

// PWM Setup Values
volatile uint32_t pwmWrapValue = 0;                     // Wrap value for PWM configuration
volatile uint32_t pwmLevelValue = 0;                    // Level value for PWM configuration
volatile uint32_t pwmHighVoltageLevelValue = 0;         // Level value for high voltage phase PWM

// EDM Isofrequency Mode Settings
volatile uint32_t EDMIsofrequencyModeWrapValue = 0;                     // Wrap value for EDM isofrequency mode PWM
volatile uint32_t EDMIsofrequencyModeLevelValue = 0;                    // Level value for EDM isofrequency mode PWM
volatile uint32_t EDMIsofrequencyModeHighVoltageLevelValue = 0;         // Level value for high voltage phase PWM in EDM isofrequency mode
const uint32_t EDMIsofrequencyModeHighVoltagePulseOnTimeMicros = 1;     // Fixed 1μs on-time for high-voltage pulse. Used to calculate level for high voltage phase PWM output. Prvents shoot-through failure. 
const double EDMIsofrequencyModeHighVoltagePwmOffsetValue = 0.25f;      // Offset value for high voltage phase PWM output. Used to calculate level for high voltage phase PWM output. Prevents shoot-through failure.
volatile unsigned long lastDischargeSuccessRateCalculationTime = 0;     // Time when discharge success rate was last calculated

// Discharge Measurement Variables
volatile bool newDischargeDetected = false;              // Flag to indicate when a new discharge is detected
volatile double newDischargeCurrent = 0.0f;              // Current value of the most recent discharge
volatile int newDischargeCurrent_ADC = 0;                // ADC reading for the current of the most recent discharge
volatile double newDischargeVoltage = 0.0f;              // Voltage value of the most recent discharge
volatile int newDischargeVoltage_ADC = 0;                // ADC reading for the voltage of the most recent discharge
volatile double avgDischargeCurrent = 0.0f;              // Running average of discharge current
volatile double avgDischargeVoltage = 0.0f;              // Running average of discharge voltage

// Output Current Sensor Threshold Variables
volatile float outputOvercurrentThresholdVoltage = 0.0f;                // Voltage threshold for output current sensor
volatile int outputOvercurrentThresholdLevel = 0;                       // PWM value for output current sensor threshold

// PWM Variables
PWMOutput edmFeedbackPWM;                              // PWM configuration for EDM_FEEDBACK pin
volatile uint32_t edmFeedbackWrapValue = 0;            // Wrap value for EDM_FEEDBACK PWM
volatile float edmFeedbackPwmDutyCycle = 0.0f;         // Current duty cycle of EDM_FEEDBACK PWM (0.0 to 1.0)
volatile uint32_t edmFeedbackPwmLevel = 0;             // Current PWM level for EDM_FEEDBACK (0 to edmFeedbackWrapValue)
volatile double feedbackPortDutyCycle = 0.00;          // Calculated duty cycle for feedback port (0.0 to 1.0)

//=============================================================================
// FUNCTION DEFINITIONS
//=============================================================================
// Function Definitions for void setup()
//----------------------------------------------------
void setupPinModes() {
    // Configure PWM pins
    gpio_set_function(SW_HIGH_VOLTAGE_PHASE, GPIO_FUNC_PWM);            // Switch for high-voltage phase
    gpio_set_function(SW_ENABLE, GPIO_FUNC_PWM);                        // Switch for connecting output (-)-electrode
    gpio_set_function(SW_HIGH_CURRENT_PHASE, GPIO_FUNC_PWM);            // Switch for high-current phase
    gpio_set_function(OUTPUT_OVERCURRENT_SET, GPIO_FUNC_PWM);           // PWM for setting output overcurrent threshold
    gpio_set_function(EDM_FEEDBACK, GPIO_FUNC_PWM);                     // PWM for EDM feedback
    
    // Digital Pins
    pinMode(EDM_ENABLE, INPUT);       
    pinMode(OUTPUT_OVERCURRENT, INPUT);           
    pinMode(BUCK_POWER_GOOD, INPUT);              
    pinMode(BUCK_ENABLE, OUTPUT);                 
    pinMode(BUCK_ADJ_PWM, OUTPUT);                
    pinMode(BOOST_PGOOD, INPUT);                  
    pinMode(PMM_FAULT, INPUT);                    
    pinMode(PMM_DIAG_EN, OUTPUT);                 
    pinMode(PMM_ENABLE, OUTPUT);    
    pinMode(STATUS_LED, OUTPUT);              
    
    // Analog Pins
    pinMode(PMM_ISENSE, INPUT);                   
    pinMode(OUTPUT_VSENSE, INPUT);                
    pinMode(OUTPUT_ISENSE, INPUT);                
}
void setupInitialPinStates() {
    // Function to set initial pin states to be safe
    gpio_put(PMM_ENABLE, false);                           // dissable PMM_ENABLE switch, preventing current flow between input and output.
    gpio_put(PMM_DIAG_EN, true);                           // enable PMM fault reporting
    setEDMFeedbackDutyCycle(0.00);                         // Set EDM_FEEDBACK to 0% duty cycle
    gpio_put(BUCK_ENABLE, false);                          // dissable BUCK_ENABLE switch, preventing current flow between input and output.
    gpio_put(BUCK_ADJ_PWM, false);                         // dissable BUCK_ADJ_PWM pin, not used.
}
void setupADC() {
    analogReadResolution(ADC_RESOLUTION_BITS); // Set ADC resolution to 12 bits (0-4095)
}
void setupOutputPWM(double requestedMachiningDutyCycle, int requestedMachiningFrequency_Hz, int outputCurrentThreshold_A, bool enableEnableSwitch, bool enableHighCurrentPhase, bool enableHighVoltagePhase, int highVoltagePulseOnTimeMicros, double highVoltagePwmOffsetValue) {
    // Function to setup the output PWM for all modes of operation

    // Configure PWM pins as PWM outputs (in case they were being used as GPIOs)
    gpio_set_function(SW_HIGH_VOLTAGE_PHASE, GPIO_FUNC_PWM);    // Switch for high-voltage phase
    gpio_set_function(SW_ENABLE, GPIO_FUNC_PWM);                // Switch for connecting output (-)-electrode
    gpio_set_function(SW_HIGH_CURRENT_PHASE, GPIO_FUNC_PWM);    // Switch for high-current phase
    gpio_set_function(OUTPUT_OVERCURRENT_SET, GPIO_FUNC_PWM);   // PWM for setting output overcurrent threshold
    gpio_set_function(EDM_FEEDBACK, GPIO_FUNC_PWM);             // PWM for EDM feedback

    // PWM Timing Calculations for Phase-Correct Mode:
    // In phase-correct mode, the counter counts up to wrap_value then down to 0
    // For 10kHz (100μs period) we need:
    // wrap_value = (125MHz / 10kHz) / 2 = 6250
    // The /2 is because counting up+down doubles the effective period
    pwmWrapValue = PWM_BASE_CLOCK_FREQ / (requestedMachiningFrequency_Hz * 2);

    // Duty Cycle Calculation:
    // In phase-correct mode, the output is HIGH when counter <= level during both up and down count
    // The hardware automatically handles the symmetrical behavior
    // For 10% duty cycle, we want level_value = wrap_value * 0.10
    pwmLevelValue = pwmWrapValue * requestedMachiningDutyCycle;

    // The high voltage level is calculated based on the high voltage pulse on time and the wrap value.
    pwmHighVoltageLevelValue = (PWM_BASE_CLOCK_FREQ * 0.5f) * (highVoltagePulseOnTimeMicros / 1000000.0f);

    //Get the slice and channel for each PWM pin
    pwmOutputs[0].slice = pwm_gpio_to_slice_num(SW_ENABLE);
    pwmOutputs[0].channel = pwm_gpio_to_channel(SW_ENABLE);
    pwmOutputs[1].slice = pwm_gpio_to_slice_num(SW_HIGH_CURRENT_PHASE);
    pwmOutputs[1].channel = pwm_gpio_to_channel(SW_HIGH_CURRENT_PHASE);
    pwmOutputs[2].slice = pwm_gpio_to_slice_num(SW_HIGH_VOLTAGE_PHASE);
    pwmOutputs[2].channel = pwm_gpio_to_channel(SW_HIGH_VOLTAGE_PHASE);
    pwmOutputs[3].slice = pwm_gpio_to_slice_num(OUTPUT_OVERCURRENT_SET);
    pwmOutputs[3].channel = pwm_gpio_to_channel(OUTPUT_OVERCURRENT_SET);

    // Configure output polarity
    // SW_HIGH_CURRENT_PHASE and SW_ENABLE share a channel and counter
    // SW_HIGH_CURRENT_PHASE is inverted, but SW_ENABLE is not  
    pwm_set_output_polarity(pwmOutputs[1].slice, false, true);
    // SW_HIGH_VOLTAGE_PHASE and OUTPUT_OVERCURRENT_SET share a channel and counter
    // SW_HIGH_VOLTAGE_PHASE is inverted, but OUTPUT_OVERCURRENT_SET is not
    pwm_set_output_polarity(pwmOutputs[2].slice, true, false); 

    // Disable all PWM outputs before configuration
    pwm_set_enabled(pwmOutputs[0].slice, false);
    pwm_set_enabled(pwmOutputs[1].slice, false);
    pwm_set_enabled(pwmOutputs[2].slice, false);
    pwm_set_enabled(pwmOutputs[3].slice, false);

    // Set the wrap value
    pwm_set_wrap(pwmOutputs[0].slice, pwmWrapValue);
    pwm_set_wrap(pwmOutputs[1].slice, pwmWrapValue);
    pwm_set_wrap(pwmOutputs[2].slice, pwmWrapValue);
    pwm_set_wrap(pwmOutputs[3].slice, pwmWrapValue);

    // Set the clock divider to 1
    pwm_set_clkdiv_int_frac(pwmOutputs[0].slice, 1, 0);
    pwm_set_clkdiv_int_frac(pwmOutputs[1].slice, 1, 0);
    pwm_set_clkdiv_int_frac(pwmOutputs[2].slice, 1, 0);
    pwm_set_clkdiv_int_frac(pwmOutputs[3].slice, 1, 0);

    // Set the phase correct mode to true
    pwm_set_phase_correct(pwmOutputs[0].slice, true);
    pwm_set_phase_correct(pwmOutputs[1].slice, true);
    pwm_set_phase_correct(pwmOutputs[2].slice, true);
    pwm_set_phase_correct(pwmOutputs[3].slice, true);

    // Configure PWM levels for each output
    pwm_set_chan_level(pwmOutputs[0].slice, pwmOutputs[0].channel, pwmLevelValue);
    pwm_set_chan_level(pwmOutputs[1].slice, pwmOutputs[1].channel, pwmLevelValue);
    if (enableHighVoltagePhase) {
        // Set level for the high voltage phase
        pwm_set_chan_level(pwmOutputs[2].slice, pwmOutputs[2].channel, pwmHighVoltageLevelValue);
    } else {
        // Disable high voltage phase by setting level to 0
        pwm_set_chan_level(pwmOutputs[2].slice, pwmOutputs[2].channel, 0);
    }
    // Calculate output current sensor threshold level based on the current threshold in amps
    outputOvercurrentThresholdVoltage = (ISENSE_OUTPUT_V_PER_AMP * outputCurrentThreshold_A) / 2.5f;
    outputOvercurrentThresholdLevel = ((outputOvercurrentThresholdVoltage / 3.3f) * pwmWrapValue);
    pwm_set_chan_level(pwmOutputs[3].slice, pwmOutputs[3].channel, outputOvercurrentThresholdLevel);

    // Reset all counters to zero before enabling
    pwm_set_counter(pwmOutputs[0].slice, 0); // Counter for SW_ENABLE
    pwm_set_counter(pwmOutputs[1].slice, 0); // Counter for SW_HIGH_CURRENT_PHASE
    pwm_set_counter(pwmOutputs[2].slice, 0); // Counter for SW_HIGH_VOLTAGE_PHASE
    pwm_set_counter(pwmOutputs[3].slice, 0); // Counter for OUTPUT_OVERCURRENT_SET

    // Adjust counter for high-voltage phase for offset (also effects overcurrent threshold set, because it's the same counter)
    pwm_set_counter(pwmOutputs[2].slice, (pwmWrapValue * highVoltagePwmOffsetValue));

    // Enable all PWM outputs simultaneously as specified by the boolean flags
    pwm_set_enabled(pwmOutputs[0].slice, enableEnableSwitch); 
    pwm_set_enabled(pwmOutputs[1].slice, enableHighCurrentPhase);
    pwm_set_enabled(pwmOutputs[2].slice, enableHighVoltagePhase);
    pwm_set_enabled(pwmOutputs[3].slice, true);
}
void setupPowerManagementModule() {
    // Function to initialize and enable the power management module

    // Enable PMM fault reporting
    gpio_put(PMM_DIAG_EN, true);

    // Calibrate PMM current sensor before enabling power
    pmmCalibrationSum = 0;
    for(int i = 0; i < PMM_CALIBRATION_SAMPLES; i++) {
        // Take reading and add to sum
        pmmCalibrationSum += analogRead(PMM_ISENSE);
        // Delay between readings
        delay(PMM_CALIBRATION_DELAY_MS);
    }
    // Calculate the zero current offset as the average of the readings
    pmmZeroCurrentOffset = pmmCalibrationSum / PMM_CALIBRATION_SAMPLES;

    // Enable PMM high-side switch to allow capacitors to charge. In-rush current protection is provided by PMM
    gpio_put(PMM_ENABLE, true);
    // Wait for in-rush current to settle with inrushDelayMs, inrushStartTime is set to currentTime
    inrushStartTime_MS = millis();
    while (millis() - inrushStartTime_MS < inrushDelay_MS) {
        // Do nothing
    }
}
void calibrateOutputCurrentSensor() {
    // Function to calibrate the output current sensor

    // Reset the sum of the ADC samples
    outputCurrentSensorCalibrationSum = 0;
    // Take the average of the ADC samples
    for (int i = 0; i < OUTPUT_CURRENT_SENSOR_CALIBRATION_SAMPLES; i++) {
        outputCurrentSensorCalibrationSum += analogRead(OUTPUT_ISENSE);
        delay(OUTPUT_CURRENT_SENSOR_CALIBRATION_DELAY_MS);
    }
    outputCurrentSensorZeroCurrentOffset = (int)(outputCurrentSensorCalibrationSum / OUTPUT_CURRENT_SENSOR_CALIBRATION_SAMPLES);
    
}
void setupInterrupts() {
    // Function to setup interrupts
    
    if (PMM_FAULT_IN_USE) {
        attachInterrupt(digitalPinToInterrupt(PMM_FAULT), []() {handlePMMFault();}, FALLING);
    }
    if (HIGH_CURRENT_PGOOD_IN_USE) {
        attachInterrupt(digitalPinToInterrupt(BUCK_POWER_GOOD), []() {handleHighCurrentPhaseFault();}, FALLING);
    }   
    if (HIGH_VOLTAGE_PGOOD_IN_USE) {
        attachInterrupt(digitalPinToInterrupt(BOOST_PGOOD), []() {handleHighVoltagePhaseFault();}, FALLING);
    }
    // Set up overcurrent interrupt
    attachInterrupt(digitalPinToInterrupt(OUTPUT_OVERCURRENT), outputOvercurrentHandler, FALLING);
}
void setupSerialCommunication() {
    // Initialize serial communication at 115200 baud rate
    Serial.begin(115200);
    // Wait for serial to be ready with timeout
    serialInitStartTime_MS = millis();

    // Wait for 1 second
    while ( millis() - serialInitStartTime_MS < 1000) {
        // Do nothing
    }
}
void setupEDMFeedbackPWM() {
    // Get PWM slice and channel for EDM_FEEDBACK
    edmFeedbackPWM.slice = pwm_gpio_to_slice_num(EDM_FEEDBACK);
    edmFeedbackPWM.channel = pwm_gpio_to_channel(EDM_FEEDBACK);
    
    // Disable PWM before configuration
    pwm_set_enabled(edmFeedbackPWM.slice, false);
    
    // Calculate wrap value for edmFeedbackPwmFrequency_Hz, adjusted for new clock divider
    edmFeedbackWrapValue = PWM_BASE_CLOCK_FREQ / (edmFeedbackPwmFrequency_Hz * EDM_FEEDBACK_PWM_CLOCK_DIVIDER);
    pwm_set_wrap(edmFeedbackPWM.slice, edmFeedbackWrapValue);
    
    // Set clock divider to 250 and use fractional divider of 0
    pwm_set_clkdiv_int_frac(edmFeedbackPWM.slice, EDM_FEEDBACK_PWM_CLOCK_DIVIDER, 0);
    
    // Set initial duty cycle to 0%
    pwm_set_chan_level(edmFeedbackPWM.slice, edmFeedbackPWM.channel, 0);
    
    // Enable PWM
    pwm_set_enabled(edmFeedbackPWM.slice, true);
}
void setupI2C() {
    // Function to setup I2C for communication with the boost converter module, for the purpose of adjusting the output voltage
    i2c_init(I2C_PORT, I2C_BAUD_RATE_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
}

// Function Definitions for void loop()
//----------------------------------------------------
void handleFault() {
    // Function to handle faults for all fault types FaultStateType
    switch (FaultStateType) {
        case PMM_FAULT_TYPE:
            // Handle PMM fault
            handlePMMFault();
            break;
        case BUCK_CONVERTER_PGOOD_FAULT:
            // Handle BUCK fault
            handleHighCurrentPhaseFault();
            break;
        case BOOST_CONVERTER_PGOOD_FAULT:
            // Handle BOOST fault
            handleHighVoltagePhaseFault();
            break;
        case POWER_OUT_OF_RANGE_FAULT:
            // Handle POWER_OUT_OF_RANGE fault
            handlePowerOutOfRangeFault();
            break;
    }
}
void setEDMFeedbackDutyCycle(double dutyCycle) {
    // Clamp duty cycle between 0 and 1.00
    if (dutyCycle < 0.00) dutyCycle = 0.00;
    if (dutyCycle >= 1.00) dutyCycle = 1.00;

    // Calculate level from wrap value and duty cycle
    edmFeedbackPwmLevel = edmFeedbackWrapValue * dutyCycle;
    // Set the PWM level
    pwm_set_chan_level(edmFeedbackPWM.slice, edmFeedbackPWM.channel, edmFeedbackPwmLevel);
}
void handlePeripheralManagement() {
    // Function to handle peripherals like input current sensor, enable port, feedback port, etc. It is intended to be called every 10ms

    updateInputCurrentSensorBufferWithNewReading();
    updateDeviceStatistics();
    readEnablePort();

    // If the device state is operating, set the feedback port duty cycle to the inverse of the discharge success rate
    if (oldDeviceState == OPERATING) {
        // Calculate feedback duty cycle with proper precision
        feedbackPortDutyCycle = (double)(1.00 - dischargeSuccessRate);
        // Set the feedback port duty cycle
        setEDMFeedbackDutyCycle(feedbackPortDutyCycle);
    }
    else {
        // If the device state is FAULT, set the feedback port duty cycle to 0.00
        setEDMFeedbackDutyCycle(0.00);
    }    

    // Read for incoming commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processCommand(command);
    }

    // Change the device state back to the previous state
    deviceState = oldDeviceState;

}
void updateInputCurrentSensorBufferWithNewReading() {
    // Function to update the input current sensor buffer with a new ADC reading from the PMM_ISENSE pin 

    // Read the ADC value and convert to current in a single calculation
    pmmISENSEAdcValue = analogRead(PMM_ISENSE);
    // Serial print the ADC value
    //Serial.print("PMM_ISENSE_ADC_VALUE ");
    //Serial.println(pmmISENSEAdcValue);
    inputCurrentReading = (((pmmISENSEAdcValue - pmmZeroCurrentOffset) * ADC_TO_VOLTS) / PMM_ISENSE_V_PER_AMP);
    // Serial print the value of pmmZeroCurrentOffset
    //Serial.print("PMM_ZERO_CURRENT_OFFSET ");
    //Serial.println(pmmZeroCurrentOffset);
    // Serial print the input current reading
    //Serial.print("INPUT_CURRENT_READING ");
    //Serial.println(inputCurrentReading);
    // Update the circular buffer
    deviceInputCurrentBuffer[deviceInputCurrentBufferIndex] = inputCurrentReading;
    deviceInputCurrentBufferIndex = (deviceInputCurrentBufferIndex + 1) % DEVICE_CURRENT_BUFFER_SIZE;
    
    // Set buffer full flag when we've filled the buffer once
    if (deviceInputCurrentBufferIndex == 0) {
        deviceInputCurrentBufferFull = true;
    }
}
void updateDeviceStatistics() {
    // Function to update the device statistics

    // 1. Calculate the average device input current
    //----------------------------------------------------
    // If the buffer is not full, set the average device input current to 0.0f
    if (!deviceInputCurrentBufferFull) {
        avgDeviceInputCurrent = 0.0f;
        return;
    }
    // If the buffer is full, calculate the average device input current
    else {
        inputCurrentBufferSum = 0.0f;
        for(int i = 0; i < DEVICE_CURRENT_BUFFER_SIZE; i++) {
            inputCurrentBufferSum += deviceInputCurrentBuffer[i];
        }
        avgDeviceInputCurrent = inputCurrentBufferSum / DEVICE_CURRENT_BUFFER_SIZE;
    }

    // 2. Check if the average device input current is above the maximum safe input current
    //----------------------------------------------------
    if (avgDeviceInputCurrent > MAX_SAFE_INPUT_CURRENT) {
        // Set the device state to FAULT
        deviceState = FAULT;
        // Set the fault type to POWER_OUT_OF_RANGE_FAULT
        FaultStateType = POWER_OUT_OF_RANGE_FAULT;
    }
}
void readEnablePort() {
    // Function to read the status of the enable port and write it to a flag. The enablePortStatus flag is read by other functions as the program runs.
    
    // Read the enable port
    enablePortStatus = digitalRead(EDM_ENABLE);
    // If the enable port is high, set the device state to OPERATING
    if (enablePortStatus) {
        // If the old device state is IDLE, set the device state to OPERATING
        oldDeviceState = OPERATING;
    }
    // If the enable port is low, set the device state to IDLE
    else {
        // If the old device state is OPERATING, set the device state to IDLE
        oldDeviceState = IDLE;
    }
}
void handleOperatingState() {
    // Switch statement for mode of operation
    switch (modeOfOperation) {
        case EDM_ISOFREQUENCY_MODE:
            // Case for EDM_ISOFREQUENCY_MODE
            EDMIsofrequencyMode();
            break;
        case EDGE_DETECTION_MODE:
            // Case for EDGE_DETECTION_MODE
            EdgeDetectionMode();
            break;
    }

}

// Function Declarations for fault handling
//----------------------------------------------------
void handlePMMFault() {
    // Function to handle PMM faults

    // Clear the GPIO interrupt status 
    gpio_acknowledge_irq(PMM_FAULT, GPIO_IRQ_EDGE_FALL);
    // Disable the output stage
    disableOutputStage();

    // Save the old device state
    oldDeviceState = deviceState;
    // Change the device state to FAULT
    deviceState = FAULT;
    // Set the fault type
    FaultStateType = PMM_FAULT_TYPE;

    // Write feedback port to 0% duty cycle
    setEDMFeedbackDutyCycle(0.00);

    // Using millis, wait for 100 ms
    unsigned long startTime = millis();
    while (millis() - startTime < 500) {
        // Do nothing
    }

    // Turn on status LED 
    gpio_put(STATUS_LED, true);
    // Print that the fault has been cleared
    Serial.println("PMM Fault Cleared");
    // Fault has been cleared if PMM_FAULT pin is high. Return the device to its previous state
    deviceState = oldDeviceState;
}
void handleHighCurrentPhaseFault() {
    // Function to handle BUCK faults

    // Disable the output stage
    disableOutputStage();
       
    // Set the fault type
    FaultStateType = BUCK_CONVERTER_PGOOD_FAULT; 
    // Set the device state to FAULT
    deviceState = FAULT;

    // Write feedback port to 0% duty cycle
    setEDMFeedbackDutyCycle(0.00);
    // Turn off status LED 
    gpio_put(STATUS_LED, false);
    // Serial print that the function is starting
    Serial.println("High Current Phase Fault");

    while (true) {
        // If serial is available, process commands
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            processCommand(command);
        }
    }
}
void handleHighVoltagePhaseFault() {
    // Function to handle BOOST faults

    // Clear the GPIO interrupt status 
    gpio_acknowledge_irq(BOOST_CONVERTER_PGOOD_FAULT, GPIO_IRQ_EDGE_FALL);
    // Disable the output stage
    disableOutputStage();

    // Save the old device state
    oldDeviceState = deviceState;
    // Set the fault type
    FaultStateType = BOOST_CONVERTER_PGOOD_FAULT;
    // Set the device state to FAULT
    deviceState = FAULT;

    // Write feedback port to 0% duty cycle
    setEDMFeedbackDutyCycle(0.00);
    // Turn off status LED 
    gpio_put(STATUS_LED, false);

    // While the fault pin is LOW, do nothing
    while (gpio_get(BOOST_CONVERTER_PGOOD_FAULT) == false) {
        // If serial is available, process commands
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            processCommand(command);
        }
    }
    // Turn on status LED 
    gpio_put(STATUS_LED, true);
    // Serial print that the fault has been cleared
    Serial.println("High Voltage Phase Fault Cleared");
    // Fault has been cleared if PMM_FAULT pin is high. Return the device to its previous state
    deviceState = oldDeviceState;
}
void handleHighVoltagePhaseSetupFault() {
    // Function to handle high voltage phase setup faults

    // Clear the GPIO interrupt status 
    gpio_acknowledge_irq(BOOST_CONVERTER_PGOOD_FAULT, GPIO_IRQ_EDGE_FALL);
    // Disable the output stage
    disableOutputStage();

    // Set the fault type
    FaultStateType = HIGH_VOLTAGE_PHASE_SETUP_FAULT;
    // Set the device state to FAULT
    deviceState = FAULT;

    // Write feedback port to 0% duty cycle
    setEDMFeedbackDutyCycle(0.0f);
    // Turn off status LED 
    gpio_put(STATUS_LED, false);
    // Serial print that the function is starting
    Serial.println("High Voltage Phase Setup Fault");

    while (true) {
        // If serial is available, process commands
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            processCommand(command);
        }
    }
}
void handlePowerOutOfRangeFault() {
    // Function to handle power out of range faults

    // Disable the output stage
    disableOutputStage();

    // Set the fault type
    FaultStateType = POWER_OUT_OF_RANGE_FAULT;
    // Set the device state to FAULT
    deviceState = FAULT;

    // Write feedback port to 0% duty cycle
    setEDMFeedbackDutyCycle(0.0f);
    // Turn off status LED 
    gpio_put(STATUS_LED, false);
    // Serial print that the function is starting
    Serial.println("Power Out of Range Fault");

    while (true) {
        // If serial is available, process commands
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            processCommand(command);
        }
    }
}

// Function Declarations for miscellaneous functions
//----------------------------------------------------
void disableOutputStage() {
    // Function to disable the output, which are all PWM outputs

    // --- Ensure pins are set to known OFF states after disabling PWM ---
    // Set pin mode to OUTPUT and set logic level for OFF
    // SW_ENABLE (N-channel): OFF = LOW
    pinMode(SW_ENABLE, OUTPUT);
    gpio_put(SW_ENABLE, false);
    // SW_HIGH_CURRENT_PHASE (P-channel): OFF = HIGH
    pinMode(SW_HIGH_CURRENT_PHASE, OUTPUT);
    gpio_put(SW_HIGH_CURRENT_PHASE, true);
    // SW_HIGH_VOLTAGE_PHASE (P-channel): OFF = HIGH
    pinMode(SW_HIGH_VOLTAGE_PHASE, OUTPUT);
    gpio_put(SW_HIGH_VOLTAGE_PHASE, true);

    // Mode preparation is not complete
    modePreparationComplete = false;
    // Set the feedback port to 0% duty cycle
    setEDMFeedbackDutyCycle(0.00);
}
void outputOvercurrentHandler() {
    // Function to handle output overcurrent based on what mode the device is in

    // Switch statement to handle the different modes of operation
    switch (modeOfOperation) {
        case EDM_ISOFREQUENCY_MODE:
            // Case for EDM_ISOFREQUENCY_MODE
            outputOvercurrentHandler_EDMIsofrequencyMode();
            break;
        case EDGE_DETECTION_MODE:
            // Case for EDGE_DETECTION_MODE
            outputOvercurrentHandler_EdgeDetectionMode();
            break;
        default:
            // Case for default
            disableOutputStage();
            // print that we are in the output overcurrent handler
            Serial.println("OUTPUT OVERCURRENT HANDLER ERROR");
            break;
    }
}

// Function Declarations for the boost converter module which provides the high-voltage for the high-voltage phase
//----------------------------------------------------
uint8_t readBoostConverterDigitalPotentiometer() {
    // Function to read the digital potentiometer on the boost converter module and returns the position
    
    uint8_t dpotValue = 0;
    uint8_t reg = DPOT_REG;
    i2c_write_blocking(I2C_PORT, DPOT_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, DPOT_ADDR, &dpotValue, 1, false);

    // Serial print the value of the digital potentiometer
    //Serial.print("Value Read from DPOT: ");
    //Serial.println(dpotValue);

    // Return the value of the digital potentiometer
    return dpotValue;
}
void writeBoostConverterDigitalPotentiometer(int position) {
    // Function to write the digital potentiometer on the boost converter module to the desired position

    // Serial print that the function is starting
    //Serial.println("Writing to the digital potentiometer");

    // clamp the position to the minimum and maximum safe positions
    if (position < MIN_BOOST_CONVERTER_DPOT_POSITION) {
        position = MIN_BOOST_CONVERTER_DPOT_POSITION;
    }
    else if (position > MAX_BOOST_CONVERTER_DPOT_POSITION) {
        position = MAX_BOOST_CONVERTER_DPOT_POSITION;
    }

    uint8_t data[2] = {DPOT_REG, (uint8_t)position};
    i2c_write_blocking(I2C_PORT, DPOT_ADDR, data, 2, false);

    // Serial print the value of the digital potentiometer
    //Serial.print("Value Written to DPOT: ");
    //Serial.println(position);
}
int readHighVoltagePhaseVoltage() {
    // Function to read the voltage of the high voltage phase and returns the voltage in volts
    // Returns the voltage in volts

    int rawReading = 0;
    int voltage = 0;

    // Take ten readings of the voltage 
    for (int i = 0; i < 10; i++) {
        // Read the voltage of the high voltage phase
        rawReading = analogRead(OUTPUT_VSENSE);
        // Serial print the raw reading
        //Serial.print("Reading: ");
        //Serial.println(rawReading);
        // Add the reading to the total voltage
        voltage += rawReading * OUTPUT_VOLTAGE_SCALE;
    }

    // Take the average of the ten readings
    voltage = voltage / 10;

    // Serial print the voltage
    //Serial.print("High Voltage Phase Voltage: ");
    //Serial.println(voltage);

    // Return the voltage
    return voltage;
}
void setBoostConverterDpotFromSerial(int position) {
    // Function to set the boost converter digital potentiometer via serial command
    // Clamp position to safe range
    if (position < MIN_BOOST_CONVERTER_DPOT_POSITION) position = MIN_BOOST_CONVERTER_DPOT_POSITION;
    if (position > MAX_BOOST_CONVERTER_DPOT_POSITION) position = MAX_BOOST_CONVERTER_DPOT_POSITION;
    writeBoostConverterDigitalPotentiometer(position);
    Serial.print("DPOT set to: ");
    Serial.println(position);
}
void READ_HVP_VOLTAGE() {
    // Reads and averages 10 samples from OUTPUT_VSENSE, applies scaling, and prints the result.
    float sum = 0.0f;
    for (int i = 0; i < 10; i++) {
        sum += analogRead(OUTPUT_VSENSE) * OUTPUT_VOLTAGE_SCALE;
        // Delay for 100 microseconds to allow the ADC to settle
        delayMicroseconds(100);
    }
    float avg = sum / 10.0f;
    Serial.print("HVP_AVG_VOLTAGE ");
    Serial.println(avg, 3);
}
void updateBoostConverterDpotVoltageTable() {
    // Function to update the look up table for the boost converter digital potentiometer voltages

    // Serial print that the function is starting
    Serial.println("Updating boost converter DPOT voltage table...");

    // Wait for 500 ms to allow the voltage to stabilize
    delay(500);

    // Set the output PWM such that no pulses are generated, but the output high voltage phase is enabled
    setupOutputPWM(
        0.00f,       // 0% duty cycle
        10000,       // 10 kHz frequency (will be ignored)
        8,           // 8 amps current limit for detection of discharges
        false,       // Disable enable switch
        false,       // Disable high current phase
        true,        // Enable high voltage phase
        1,           // 1 microsecond high voltage pulse on time
        0.25f        // 25% high voltage phase offset
    );

    // Serial print that the DPOT is being set to the minimum position
    Serial.println("Setting DPOT to minimum position...");

    // Set the DPOT to the 64 position, which is the device default position on startup
    writeBoostConverterDigitalPotentiometer(64);

    // read the position of the DPOT and make sure it is 64
    int currentDPOTPosition = readBoostConverterDigitalPotentiometer();
    if (currentDPOTPosition != 64) {
        Serial.println("ERROR: DPOT is not at expected position");
        // Call a fault
        handleHighVoltagePhaseSetupFault();
    }
    
    // Ramp to the minimum position
    for (int i = 64; i >= MIN_BOOST_CONVERTER_DPOT_POSITION; i--) {
        writeBoostConverterDigitalPotentiometer(i);
        delay(BOOST_CONVERTER_RAMP_SPACING_MS);
    }

    // Serial print that we are waiting for the voltage to stabilize after setting the output PWM
    Serial.println("Waiting for voltage to stabilize...");

    // wait for the voltage to stabilize after setting the output PWM
    delay(250);

    // Serial print that we are starting to read the voltages
    Serial.println("Reading voltages for DPOT lookup table...");

    // For every position in the look up table, set the digital potentiometer to the position and read the voltage. Then store the voltage in the look up table.
    for (int i = 0; i <= MAX_BOOST_CONVERTER_DPOT_POSITION; i++) {
        writeBoostConverterDigitalPotentiometer(i);
        // delay for spacing between steps
        delay(BOOST_CONVERTER_RAMP_SPACING_MS);
        dpotVoltageTable[i] = readHighVoltagePhaseVoltage();
    }
    // Serial print the look up table
    Serial.println("Boost Converter Digital Potentiometer Voltage Table:");
    for (int i = 0; i <= MAX_BOOST_CONVERTER_DPOT_POSITION; i++) {
        Serial.print(i);
        Serial.print(": ");
        Serial.println(dpotVoltageTable[i]);
    }
    // Find the maximum voltage in the look up table
    int maxVoltage = 0;
    for (int i = 0; i <= MAX_BOOST_CONVERTER_DPOT_POSITION; i++) {
        if (dpotVoltageTable[i] > maxVoltage) {
            maxVoltage = dpotVoltageTable[i];
        }
    }
    
    // If the maximum voltage in the table is less than the maximum safe voltage, call a fault
    if (maxVoltage < MAX_HIGH_VOLTAGE_PHASE_VOLTS) {
        Serial.println("ERROR: DPOT Voltage range is less than expected range");
        // Call a fault for the high voltage phase
        handleHighVoltagePhaseSetupFault();
    }
    
    // Set the DPOT to the default position
    writeBoostConverterDigitalPotentiometer(DEFAULT_BOOST_CONVERTER_DPOT_POSITION);

    // Disable the output stage
    disableOutputStage();
}
void setBoostConverterDPOTfromVoltageTable(int targetVoltage, int spacing_ms) {
    // Function to set the boost converter digital potentiometer to the position that closest corresponds to the desired voltage in the look up table
    // The DPOT is ramped from its current position to the position that corresponds to the desired voltage in the look up table, with a spacing of spacing_ms between steps

    // Get the current position of the DPOT
    int currentDPOTPosition = readBoostConverterDigitalPotentiometer();
    int targetDPOTPosition = 0;

    // Temporary array to store the error for each position
    int errorArray[MAX_BOOST_CONVERTER_DPOT_POSITION + 1];

    // For every position in the dpot voltage table, check the error between the voltage and the target voltage.
    for (int i = 0; i <= MAX_BOOST_CONVERTER_DPOT_POSITION; i++) {
        errorArray[i] = abs(dpotVoltageTable[i] - targetVoltage);
    }

    // Select the position with the least error in the error array. If the error is greater than 5 volts, set the target DPOT position to the minimum position, and call a fault
    int minError = 1000;
    for (int i = 0; i <= MAX_BOOST_CONVERTER_DPOT_POSITION; i++) {
        if (errorArray[i] < minError) {
            minError = errorArray[i];
            targetDPOTPosition = i;
        }
    }

    // If the error is greater than 5 volts, set the target DPOT position to the minimum position, and call a fault
    if (minError > 5) {
        targetDPOTPosition = MIN_BOOST_CONVERTER_DPOT_POSITION;
        // Set the DPOT to the minimum position
        writeBoostConverterDigitalPotentiometer(MIN_BOOST_CONVERTER_DPOT_POSITION);
        // Serial print the error
        Serial.println("ERROR: Difference between target voltage and DPOT voltage is greater than 5 volts");
        // Call a fault for the high voltage phase
        handleHighVoltagePhaseSetupFault();
    }

    // If the current DPOT position is equal to the target DPOT position, do not ramp
    if (currentDPOTPosition == targetDPOTPosition) {
        writeBoostConverterDigitalPotentiometer(targetDPOTPosition);
    }
    else if (currentDPOTPosition < targetDPOTPosition) {
        // Ramp up
        for (int i = currentDPOTPosition; i <= targetDPOTPosition; i++) {
            writeBoostConverterDigitalPotentiometer(i);
            delay(spacing_ms);
        }
    }
    else if (currentDPOTPosition > targetDPOTPosition) {
        // Ramp down
        for (int i = currentDPOTPosition; i >= targetDPOTPosition; i--) {
            writeBoostConverterDigitalPotentiometer(i);
            delay(spacing_ms);
        }
    }
}

// Function Declarations for processing commands
//----------------------------------------------------
// Function to get fault type as string
String getFaultType() {
    switch (FaultStateType) {
        case PMM_FAULT_TYPE:
            return "PMM_FAULT";
        case BUCK_CONVERTER_PGOOD_FAULT:
            return "BUCK_CONVERTER_PGOOD_FAULT";
        case BOOST_CONVERTER_PGOOD_FAULT:
            return "BOOST_CONVERTER_PGOOD_FAULT";
        case POWER_OUT_OF_RANGE_FAULT:
            return "POWER_OUT_OF_RANGE_FAULT";
        default:
            return "UNKNOWN_FAULT";
    }
}
void sendTelemetry() {
    // Function to send telemetry data over serial
    // Print software version
    Serial.print("FIRMWARE_VERSION ");
    Serial.println(SOFTWARE_VERSION);
    // Build fault string
    String faultStr = "NONE";
    // if the device is in a fault state, build the fault string
    if (isInFaultState) {
        // Report the fault type
        faultStr = getFaultType();
    }
    // Print the device state
    Serial.print("STATE ");
    switch (oldDeviceState) {
        case STARTUP:
            Serial.println("STARTUP");
            break;
        case PERIPHERAL_MANAGEMENT:
            Serial.println("PERIPHERAL_MANAGEMENT");
            break;
        case FAULT:
            Serial.println("FAULT");
            break;
        case OPERATING:
            // Print the mode of operation
            Serial.print("OPERATING: ");
            break;
        case IDLE:
            Serial.println("IDLE");
            break;
        default:
            Serial.println("UNKNOWN");
            break;
    }
    Serial.print("FAULT ");
    Serial.println(faultStr);
    Serial.print("INPUT_CURRENT ");
    Serial.print(avgDeviceInputCurrent, 2);  // 2 decimal places
    Serial.println(" A");
    Serial.print("INPUT_POWER ");
    Serial.print((int)(avgDeviceInputCurrent * 48));  // 2 decimal places
    Serial.println("W ");

    // The telemetry sent is talored to the current mode of operation
    if (modeOfOperation == EDGE_DETECTION_MODE) {
        // print that an edge has been detected
        if (edgeDetected) {
            Serial.println("EDGE DETECTED");
        }
        else {
            Serial.println("NO EDGE DETECTED YET");
        }
    }
    // If the mode is isofrequency mode or isopulse mode, print the telemetry
    else if (modeOfOperation == EDM_ISOFREQUENCY_MODE) {
        Serial.print("AVG_DISCHARGE_CURRENT:");
        Serial.print(avgDischargeCurrent, 2);
        Serial.println(" A");
        Serial.print("AVG_DISCHARGE_VOLTAGE:");
        Serial.print(avgDischargeVoltage, 2);
        Serial.println(" V");
        Serial.print("DISCHARGES_SINCE_OPERATION_STARTED:");
        Serial.println(dischargesSinceOperationStarted);
        Serial.print("DISCHARGE_SUCCESS_RATE:");
        Serial.print((int)(dischargeSuccessRate * 100));
        Serial.println("%");
        Serial.print("DISCHARGE_SUCCESS_RATE_THRESHOLD:");
        Serial.print((int)(maxDischargeSuccessRateThreshold * 100));
        Serial.println("%");
        Serial.println("ISOPULSE PARAMETERS: ");
        if (dischargeCountTarget == 0) {
            Serial.println("Infinite Discharges Requested");
        }
        else {
            Serial.print("Discharges Requested:");
            Serial.println(dischargeCountTarget);
        }
        Serial.print("Machining Duty Cycle:");
        Serial.println(machiningDutyCycle, 3);  // 3 decimal places
        Serial.print("Machining Frequency:");
        Serial.print(machiningFrequency, 1);  // 1 decimal place for Hz
        Serial.println(" Hz");
        Serial.print("Initiation Voltage:");
        Serial.print(machiningInitVoltage, 1);   // 1 decimal place for voltage
        Serial.println(" V");
        Serial.println("--------------------------------");
    }
}
void processCommand(String command) {
    // Function to process incoming commands from the serial port

    // Remove leading/trailing whitespace
    command.trim();

    // If the command is to send telemetry
    if (command == "SEND_TELEMETRY") {
        // Self-contained command for sending telemetry data over serial when called
        sendTelemetry();
    }
    else if (command.startsWith("SET_ALL_PARAMETERS")) {
        // Parse parameters from command string
        // Format: SET_ALL_PARAMETERS <discharges> <dutyCycle> <frequency> <initVoltage>
        
        // Split the command into tokens
        String tokens[MAX_PARAMETERS + 1];  // +1 for the command itself
        int tokenCount = 0;
        int currentPosition = 0;
        
        while (tokenCount < MAX_PARAMETERS + 1 && currentPosition < command.length()) {
            int nextSpace = command.indexOf(' ', currentPosition);
            if (nextSpace == -1) {
                tokens[tokenCount++] = command.substring(currentPosition);
                break;
            }
            tokens[tokenCount++] = command.substring(currentPosition, nextSpace);
            currentPosition = nextSpace + 1;
        }
        
        if (tokenCount != MAX_PARAMETERS + 1) {
            Serial.println("ERROR: Invalid number of parameters");
            Serial.println("Expected 4 parameters: <discharges> <dutyCycle> <frequency> <initVoltage>");
            return;
        }
        
        // Parse parameters into structured format
        OutputParameters params;
        params.dischargeCountTarget = tokens[1].toInt();
        params.dutyCycle = tokens[2].toFloat();
        params.frequency = tokens[3].toFloat();
        params.initVoltage = tokens[4].toFloat();
        
        // Set the parameters
        if (setEDMParameters(params.dischargeCountTarget, params.dutyCycle, params.frequency, 
                                params.initVoltage)) {
            Serial.println("OK: Parameters set");
        }
    }
    else if (command.startsWith("EDGE_DETECTION_MODE")) {
        // Disable the output stage
        disableOutputStage();
        // Set mode to EDGE_DETECTION_MODE
        modeOfOperation = EDGE_DETECTION_MODE;
    }
    else if (command == "RESET_DEVICE") {
        // Disable the output stage
        disableOutputStage();
        // Serial print the command
        Serial.println("Rebooting device");
        // Reboot the device
        rp2040.reboot();
    }
    else if (command.startsWith("SET_DPOT ")) {
        int pos = command.substring(9).toInt();
        setBoostConverterDpotFromSerial(pos);
    }
    else if (command == "READ_HVP_VOLTAGE") {
        READ_HVP_VOLTAGE();
    }
    else if (command == "UPDATE_DPOT_VOLTAGE_TABLE") {
        updateBoostConverterDpotVoltageTable();
    }
    else if (command.startsWith("SET_DPOT_FROM_VTABLE ")) {
        int targetVoltage = command.substring(20).toInt();
        if (targetVoltage < MIN_HIGH_VOLTAGE_PHASE_VOLTS || targetVoltage > MAX_HIGH_VOLTAGE_PHASE_VOLTS) {
            Serial.print("ERROR: Target voltage out of range (");
            Serial.print(MIN_HIGH_VOLTAGE_PHASE_VOLTS);
            Serial.print("-");
            Serial.print(MAX_HIGH_VOLTAGE_PHASE_VOLTS);
            Serial.println(" V)");
        } else {
            setBoostConverterDPOTfromVoltageTable(targetVoltage, BOOST_CONVERTER_RAMP_SPACING_MS);
        }
    }
    else if (command.startsWith("SET_FEEDBACK_DUTY ")) {
        // Parse duty cycle from command string
        float duty = command.substring(18).toFloat();
        if (duty < 0.0 || duty > 1.0) {
            Serial.println("ERROR: Duty cycle must be between 0.0 and 1.0");
        } else {
            setEDMFeedbackDutyCycle(duty);
            delay(2000);
            Serial.print("EDM Feedback Duty Cycle set to: ");
            Serial.println(duty, 3);
        }
    }
    else if (command == "EDM_ISOFREQUENCY_MODE") {
        disableOutputStage();
        modeOfOperation = EDM_ISOFREQUENCY_MODE;
        Serial.println("EDM_ISOFREQUENCY_MODE entered");
    }
    else {
        Serial.println("ERROR: Unknown command");
        Serial.println("Accepted commands:");
        Serial.println("  SEND_TELEMETRY - Send current device status and parameters over serial");
        Serial.println("  SET_ALL_PARAMETERS <discharges> <dutyCycle> <frequency> <initVoltage> - Set all isopulse parameters at once");
        Serial.println("  EDGE_DETECTION_MODE - Enter edge detection mode");
        Serial.println("  RESET_DEVICE - Reset the device");
        //Serial.println("  SET_DPOT <position> - Set the boost converter digital potentiometer");
        //Serial.println("  READ_HVP_VOLTAGE - Read and average 10 samples from OUTPUT_VSENSE, apply scaling, and print result");
        //Serial.println("  UPDATE_DPOT_VOLTAGE_TABLE - Update the boost converter digital potentiometer voltage table");
        //Serial.println("  SET_DPOT_FROM_VTABLE <voltage> - Set DPOT to position for target voltage using voltage table");
        //Serial.println("  SET_FEEDBACK_DUTY <dutyCycle> - Set the EDM feedback duty cycle");
        Serial.println("  EDM_ISOFREQUENCY_MODE - Enter EDM Isofrequency Mode");
    }
}
bool setEDMParameters(int discharges, float dutyCycle, float frequency, float initV) {
    // Function to validate and set isopulse parameters 
    // Is called by processCommand() when a new isopulse command is received

    // Validate parameters against safe operating ranges
    if (discharges < 0) {
        Serial.println("ERROR: Invalid discharges value");
        return false;
    }
    
    if (dutyCycle < MIN_DUTY_CYCLE || dutyCycle > MAX_DUTY_CYCLE) {
        Serial.print("ERROR: Invalid duty cycle (must be between ");
        Serial.print(MIN_DUTY_CYCLE);
        Serial.print(" and ");
        Serial.print(MAX_DUTY_CYCLE);
        Serial.println(")");
        return false;
    }
    
    if (frequency < MIN_MACHINING_FREQUENCY_HZ || frequency > MAX_MACHINING_FREQUENCY_HZ) {
        Serial.print("ERROR: Invalid frequency (must be between ");
        Serial.print(MIN_MACHINING_FREQUENCY_HZ);
        Serial.print(" and ");
        Serial.print(MAX_MACHINING_FREQUENCY_HZ);
        Serial.println(" Hz)");
        return false;
    }
    
    if (initV < MIN_INIT_VOLTAGE || initV > MAX_INIT_VOLTAGE) {
        Serial.print("ERROR: Invalid initiation voltage (must be between ");
        Serial.print(MIN_INIT_VOLTAGE);
        Serial.print(" and ");
        Serial.print(MAX_INIT_VOLTAGE);
        Serial.println(" V)");
        return false;
    }
    
    // Calculate timing parameters locally
    float periodMicros = 1000000.0 / frequency;
    float onTimeMicros = periodMicros * dutyCycle;
    float offTimeMicros = periodMicros - onTimeMicros;
    
    // Validate calculated times
    if (onTimeMicros < MIN_ON_TIME_MICROS || onTimeMicros > MAX_ON_TIME_MICROS) {
        Serial.print("ERROR: Calculated on-time ("); 
        Serial.print(onTimeMicros);
        Serial.println(" us) is outside valid range");
        return false;
    }
    
    if (offTimeMicros < MIN_OFF_TIME_MICROS) {
        Serial.print("ERROR: Calculated off-time ("); 
        Serial.print(offTimeMicros);
        Serial.println(" us) is too short");
        return false;
    }
    
    // All parameters valid, update the values
    dischargeCountTarget = discharges;
    machiningDutyCycle = dutyCycle;
    machiningFrequency = frequency;
    machiningInitVoltage = initV;
    // Reset the mode preparation complete flag
    modePreparationComplete = false;
    return true;
}

// Function Declarations for idle state
//----------------------------------------------------
void handleIdleState() {
    // Function to handle the idle state of the device
    disableOutputStage();
    // Reset the discharges since operation started counter
    dischargesSinceOperationStarted = 0;
    // wait to prevent GPIO jitter from calling disableOutputStage() multiple times in a row
    delay(100);
}

// Function Definitions for Edge Detection Mode
//----------------------------------------------------
// Used to detect the edge of a workpiece with the electrodes, and report the edge detection over the feedback port.
void EdgeDetectionMode() {
    // Function to handle the edge detection mode of the device. It sets up the device for edge detection, and then does nothing.

    // If preparation for this operating mode is not complete
    if (!modePreparationComplete) {
        // Set the feedback port to 0% duty cycle
        setEDMFeedbackDutyCycle(0.0f);
        // 5% duty cycle, 10 KHz, do not enable the high current phase
        setupOutputPWMForEdgeDetection();
        // Set the mode preparation complete flag to true
        modePreparationComplete = true;
    }
}
void outputOvercurrentHandler_EdgeDetectionMode() {
    // Function to handle the output overcurrent handler for the edge detection mode
    if (!edgeDetected) {
        // Set the edge detected flag
        edgeDetected = true;
        // Clear the GPIO interrupt status using gpio_acknowledge_irq()
        gpio_acknowledge_irq(OUTPUT_OVERCURRENT, GPIO_IRQ_EDGE_FALL);
        // Disable the output stage
        disableOutputStage();
        // Set the feedback port to 100% duty cycle
        setEDMFeedbackDutyCycle(1.00f);
        // Send a message over serial
        Serial.println("EDGE DETECTED");
        // wait 1 second
        delay(1000);
        // Write the feedback port to 0% duty cycle
        setEDMFeedbackDutyCycle(0.00f);
        // wait 1 second
        delay(1000);
        // Reset the edge detected flag
        edgeDetected = false;
    }
}
void setupOutputPWMForEdgeDetection() {
    // Function to setup the output PWM for the edge detection mode of the device, in phase-correct mode

    // Set the high voltage phase to the minimum safe voltage using the voltage lookup table
    setBoostConverterDPOTfromVoltageTable(MIN_HIGH_VOLTAGE_PHASE_VOLTS, BOOST_CONVERTER_RAMP_SPACING_MS);

    // Setup the PWM with the values for edge detection mode
    setupOutputPWM(
        0.05f,       // 5% duty cycle
        10000,       // 10 kHz frequency
        DEFAULT_OUTPUT_CURRENT_THRESHOLD_A,       
        true,        // Enable enable switch
        true,        // Enable high current phase
        false,       // Disable high voltage phase
        1,           // 1 microsecond high voltage pulse on time
        0.25f           
    );
}

// Function Definitions for EDM Iso-frequency Mode
// In Isofrequency mode, the device outputs pulses at a constant frequency. The discharge duration may vary slightly due to ignition delays
//----------------------------------------------------
void EDMIsofrequencyMode() {
    // Function to handle the EDM isofrequency mode of the device. In Isofrequency mode, the device outputs pulses at a constant frequency of pulses every second. 

    // If a discharge has been detected,
    if (newDischargeDetected) {
        // Reset the newDischargeDetected flag
        newDischargeDetected = false;
        // Increment counters
        dischargesSinceOperationStarted++;
        currentDischargeCount++;
        // Calculate the average discharge current
        newDischargeCurrent = ((newDischargeCurrent_ADC - outputCurrentSensorZeroCurrentOffset) * ADC_TO_VOLTS * ISENSE_OUTPUT_AMPS_PER_VOLT);
        // Calculate the average discharge voltage
        newDischargeVoltage = (double)(newDischargeVoltage_ADC * OUTPUT_VOLTAGE_SCALE);
        
        // Clamp the discharge current to a minimum of 0 amps
        if (newDischargeCurrent < 0) {
            newDischargeCurrent = 0;
        }
        // Clamp the discharge voltage to a minimum of 0 volts
        if (newDischargeVoltage < 0) {
            newDischargeVoltage = 0;
        }
        // Calculate the average discharge current with 90/10 weighting
        avgDischargeCurrent = (0.90 * avgDischargeCurrent) + (0.10 * newDischargeCurrent);
        // Calculate the average discharge voltage with 90/10 weighting
        avgDischargeVoltage = (0.90 * avgDischargeVoltage) + (0.10 * newDischargeVoltage);
    }
    // Calculate the discharge success rate, but only if the setup is complete
    if (micros() - lastDischargeSuccessRateCalculationTime > dischargeRateCalculationInterval_MICROS) {
        // Calculate how long time has really passed since the last discharge success rate calculation
        timeSinceLastCalculation_MICROS = micros() - lastDischargeSuccessRateCalculationTime;
        // Calculate the new discharge success rate as the ratio of the number of discharges detected to the number of discharges requested
        dischargeSuccessRate = (double)(currentDischargeCount) / (double)(dischargesPerCalculationInterval);
        // Clamp the discharge success rate to a minimum of 0 and a maximum of 1
        if (dischargeSuccessRate < 0.00) {
            dischargeSuccessRate = 0.00;
        }
        else if (dischargeSuccessRate > 1.00) {
            dischargeSuccessRate = 1.00;
        }
        // Reset the current discharge count
        currentDischargeCount = 0;
        // Update the last discharge success rate calculation time
        lastDischargeSuccessRateCalculationTime = micros();
        // If the discharge succcess rate is greater than the maximum discharge success rate threshold, and allowPulseSkipping is true,
        if (dischargeSuccessRate > maxDischargeSuccessRateThreshold && allowPulseSkipping) {
            // Disable the output stage
            disableOutputStage();
            // Using millis, wait for 10 ms
            unsigned long startTime = millis();
            while (millis() - startTime < 50) {
                // Do nothing
            }
            // Set the discharge success rate to zero
            dischargeSuccessRate = 0;
        }
    }
    // If the number of completed discharges has reached the requested number, and the requested number is not 0 (infinite)
    if (dischargesSinceOperationStarted >= dischargeCountTarget && dischargeCountTarget != 0) {
        // Serial print that the number of discharges since operation started has been reached
        Serial.print("Number of discharges requested reached: ");
        Serial.println(dischargesSinceOperationStarted);

        // If requestedDischargesReached = false, disable the output stage
        if (!requestedDischargesReached) {
            disableOutputStage();
        }
        // Write the feedback port to 0% duty cycle
        setEDMFeedbackDutyCycle(0.0f);
        // Set flag for reaching the requested number of discharges
        requestedDischargesReached = true;
    }
    // The target number of discharges has not been reached, or the number of requested discharges is set to zero (infinite discharges)
    else {
        // Set flag for not reaching the requested number of discharges
        requestedDischargesReached = false;
        // If the number of requested discharges has not been reached
        if (!modePreparationComplete) {
            // Go through the preparation steps for this operating mode
            setupOutputPWMForEDMIsofrequencyMode(machiningDutyCycle, machiningFrequency);
            // Set the mode preparation complete flag to true
            modePreparationComplete = true;
            // Print that isofrequency mode has been setup
            //Serial.println("ISOFREQUENCY MODE SETUP COMPLETE");
        }
    }
}
void outputOvercurrentHandler_EDMIsofrequencyMode() {
    // Function to handle the output overcurrent handler for the EDM isofrequency mode of the device

    // Read the ADC value for the output current sensor
    newDischargeCurrent_ADC = analogRead(OUTPUT_ISENSE);
    // Read the ADC value for the output voltage sensor
    newDischargeVoltage_ADC = analogRead(OUTPUT_VSENSE);
    // Set the newDischargeDetected flag to true, so that during the EDMIsofrequencyMode() function the discharge count is incremented
    newDischargeDetected = true;
    // Clear the GPIO interrupt status using gpio_acknowledge_irq()
    gpio_acknowledge_irq(OUTPUT_OVERCURRENT, GPIO_IRQ_EDGE_FALL);
}
void setupOutputPWMForEDMIsofrequencyMode(double requestedMachiningDutyCycle, int requestedMachiningFrequency) {
    // Function to setup the output PWM for the EDM isofrequency mode of the device, in phase-correct mode

    // Disable the output stage
    disableOutputStage();

    // Calculate the discharge rate calculation interval from the requested machining frequency and a constant multiplier (10)
    dischargeRateCalculationInterval_MICROS = (int)(dischargesPerCalculationInterval * (1000000 / requestedMachiningFrequency));

    // Set the high voltage phase to the desired voltage using the voltage lookup table
    setBoostConverterDPOTfromVoltageTable(machiningInitVoltage, BOOST_CONVERTER_RAMP_SPACING_MS);   

    // Setup the PWM with the calculated wrap and level values. Do not enable the high-voltage phase.
    setupOutputPWM(
        requestedMachiningDutyCycle, 
        requestedMachiningFrequency, 
        DEFAULT_OUTPUT_CURRENT_THRESHOLD_A, 
        true,  // Enable enable switch
        true,  // Enable high current phase
        true,  // Enable high voltage phase
        EDMIsofrequencyModeHighVoltagePulseOnTimeMicros, // Specific to EDM Isofrequency Mode
        EDMIsofrequencyModeHighVoltagePwmOffsetValue     // Specific to EDM Isofrequency Mode
    );
}
// Void Setup()
void setup() {
    // Current time in milliseconds
    currentTime_MS = millis();
    // Device state is set to STARTUP
    deviceState = STARTUP;
    // Setup pin modes for all pins 
    setupPinModes();    
    // Setup initial pin states
    setupInitialPinStates();
    // Setup ADC
    setupADC();
    // Setup EDM Feedback PWM
    setupEDMFeedbackPWM();
    // Disable the output stage
    disableOutputStage();
    // Initialize Power Management Module
    setupPowerManagementModule();
    // Calibrate the output current sensor
    calibrateOutputCurrentSensor();
    // Initialize Serial communication
    setupSerialCommunication();
    // Setup interrupts
    setupInterrupts();  
    // Setup I2C
    setupI2C();
    // Update the boost converter dpot voltage table
    updateBoostConverterDpotVoltageTable();
    // Turn on status LED 
    gpio_put(STATUS_LED, true);
    // Serial print that the setup has completed
    Serial.println("Setup complete");
    // Before entering void loop(), set device state to IDLE
    deviceState = IDLE;
}

// Void Loop()
void loop() {
     // Record the current time in milliseconds
    currentTime_MS = millis();
    // If the device is in the FAULT state, run the fault handler function. It will act in accordance with the fault state type
    if (deviceState == FAULT) {
        handleFault();
    }
    // Switch statement for device states of secondary priority
    switch (deviceState) {
        case PERIPHERAL_MANAGEMENT:
            // Case for PERIPHERAL_MANAGEMENT state
            handlePeripheralManagement();
            break;
        case OPERATING:
            // Case for OPERATING state
            handleOperatingState();
            break;
        case IDLE:
            // Case for IDLE state
            handleIdleState();
            break;
    }
    // If periodic telemetry is enabled, and the time since the last telemetry event is greater than the telemetry interval, send telemetry
    if (sendPeriodicTelemetryEnabled && currentTime_MS - lastTelemetryEventTime_MS >= periodicTelemetryInterval_MS) {
        sendTelemetry();
        lastTelemetryEventTime_MS = currentTime_MS;
    }
    // If the time since the last peripheral management event is greater than the peripheral management interval, set the device state to PERIPHERAL_MANAGEMENT
    if (currentTime_MS - lastPeripheralManagementEventTime_MS >= peripheralManagementInterval_MS) {
        // Store the current state of the device, so that after the peripheral management loop, the device state can be restored to the previous state to continue what it was doing before
        oldDeviceState = deviceState;
        // Set the device state to PERIPHERAL_MANAGEMENT, so that we can enter the peripheral management loop
        deviceState = PERIPHERAL_MANAGEMENT;
        // Update the last peripheral management event time
        lastPeripheralManagementEventTime_MS = currentTime_MS;
    }
}
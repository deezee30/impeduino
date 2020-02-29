/***********************************************
 * Final Year Project: Brain-Machine Interface *
 *   National University of Ireland, Galway    *
 *              Deniss Zerkalijs               *
 *                                             *
 * Reads impedance values from the AD5933 over *
 * I2C and prints them serially                *
 ***********************************************/

#include "AD5933.h"
#include "Wire.h"

// AD5933 configuration
const float     FREQ_START  = 10000;            // linear frequency sweep start (Hz) [5 kHz < f < 100 kHz]
const float     FREQ_INC    = 1000;             // distance between successive frequency points (Hz)
const uint8_t   INC_NUM     = 70;               // number of increments along the sweep [< 511]
const uint16_t  REF_RESIST  = 22000;            // resistance used during calibration (Ohms)
const uint8_t   SWEEP_NUM   = 1;                // sweep each point multiple times, for increased accuracy

// Variables for internal use
boolean         cache       = false;            // whether or not to store frequency sweep data
float           temperature = 0.F;              // measured temperature of the AD5933
boolean         calibrated  = false;            // whether or not multi-point gain is calculated yet
boolean         swept       = false;            // whether or not calibrated frequency sweep was performed yet
String          command;                        // serial input command for AD5933 function
uint32_t        lastMillis;                     // used for timing heavy operations (for single-threaded use only)

// Impedance calibration and frequency sweep data
float           gain[INC_NUM + 1];              // calibration gains for each frequency point
float           phaseShift[INC_NUM + 1];        // calibration phase shifts for each frequency point, stored in radians
int32_t         *real       = 0;                // real impedance components for each frequency point
int32_t         *imag       = 0;                // imaginary impedance components for each frequency point

// Internal function definitions
bool frequencySweep(uint8_t n,                  // perform a frequency sweep and record complex impedance data across
                    uint8_t avgNum,             // n points. Can be used for calibration and final sweep. Each point
                    bool calibration,           // is swept avgNum times for increased accuracy and the result is
                    bool print);                // averaged out.
bool frequencySweep(uint8_t n,                  // similar as above, but each point gets measured once
                    bool calibration,           
                    bool print);
float complexMagnitude(float, float);           // computes impedance magnitude from real and imaginary components
float complexPhase(float, float);               // computes impedance phase angle from real and imaginary components
float complexReal(float, float);                // computes real component from impedance magnitude and phase angle
float complexImaginary(float, float);           // computes imaginary component from impedance magnitude and phase angle

void printCSVHeader(bool);                      // prints the calibration or measurement header to the CSV report
void printCSVLine(uint8_t, int32_t, int32_t,    // prints CSV impedance data at a specific frequency point
                  float, float);                // to the serial monitor
void printCSVFooter(uint8_t);                   // prints the footer to the CSV report

void showCommands();                            // prints a list of available commands to the serial monitor
void startTimer();                              // starts timing an operation (used for debugging)
uint32_t stopTimer();                           // stops timing an operation (used for debugging)

void setup() {
    // Join I2C bus, initialise Wire library and start serial communication at 9600 bps
    Wire.begin();
    Serial.begin(9600);

    Serial.println(F("\n-------- Start --------"));

    // Perform initial configuration. Fail if any one of these fail
    if (!(AD5933::reset() &&
          AD5933::setInternalClock(true) &&
          AD5933::setStartFrequency(FREQ_START) &&
          AD5933::setIncrementFrequency(FREQ_INC) &&
          AD5933::setNumberIncrements(INC_NUM) &&
          AD5933::setPGAGain(PGA_GAIN_X1) &&
          AD5933::setSettlingCycles(15, NUM_ST_CYCLES_DEFAULT))) {
        Serial.println(F("Failed in initialization!"));
        while (true) continue;
    }

    // Decide whether or not to store the swept impedance data.
    // ATmega328P has 2 kb of internal SRAM. The storage of real and imaginary
    // values after a frequency sweep is limited to 100;
    if (INC_NUM <= 100) {
        cache = true;

        // Allocate space for both arrays
        real = (int32_t*) malloc((INC_NUM + 1) * sizeof(int32_t));
        imag = (int32_t*) malloc((INC_NUM + 1) * sizeof(int32_t));
    } else {
        Serial.print(F("Warning: Number of frequency increments is set to be "));
        Serial.print(INC_NUM);
        Serial.println(F(". Storage of swept data will be disabled."));
        Serial.println(F("Swept data will be dynamically outputted during sweep."));
    }

    showCommands();

    delay(50);
}

void loop() {
    delay(50);

    // Monitor the serial console each tick and expect a received byte
    if (Serial.available() > 0) {
        String input = Serial.readString();
        
        input.toLowerCase();
        input.trim();

        // If the input command is empty space (e.g. a new line or a space),
        // then fire the previously used command (if one exists)
        if (input.length() == 0) {
            if (command.length() == 0)
                return;
        } else {
            command = input;
        }

        // Output the typed command back to the serial console
        Serial.print(F("\n> "));
        Serial.println(command);

        // Handle commands.
        // If debugging is enabled, time each command to evaluate performance.
        // Notify the serial monitor if any operation is unsuccessful

        // Handle temperature measurement
        if (command.equals(F("temp"))) {
            startTimer();

            // Typical
            if (AD5933::getTemperature(&temperature)) {
                Serial.print(F("Temperature: "));
                Serial.println(temperature);
            } else {
                Serial.println(F("Temperature measurement failed."));
            }

            stopTimer();
        // Handle calibration (multi-point gain calculation)
        } else if (command.equals(F("cal"))) {
            // Enforce temperature measurement before calibrating
            if (temperature == 0.F) {
                Serial.println(F("Type \"temp\" to perform a temperature measurement first."));
                return;
            }

            startTimer();

            if (frequencySweep(INC_NUM, SWEEP_NUM, true, !cache)) {
                Serial.print(INC_NUM);
                Serial.println(F(" points calibrated."));
            } else {
                Serial.println(F("Calibration failed."));
            }

            stopTimer();

        // Handle frequency sweep (impedance measurement)
        } else if (command.equals(F("sweep"))) {
            // Enforce calibration before performing sweep
            if (!calibrated) {
                Serial.println(F("Type \"cal\" to perform a calibration sweep first."));
                return;
            }

            startTimer();

            swept = frequencySweep(INC_NUM, SWEEP_NUM, false, !cache);
            if (swept) {
                Serial.print(F("Impedance measured at "));
                Serial.print(INC_NUM);
                Serial.println(F(" frequency points."));
            } else {
                Serial.println(F("Frequency sweep failed."));
            }

            stopTimer();

        // Handle exporting values to CSV file
        } else if (command.equals(F("exp"))) {

            // If NUM_INC > 100, then caching of frequency sweep data (with the
            // exception of gain) is disabled. This is due to the small amount
            // of memory the ATmega328P has (SRAM = 2 kb). Instead, the results
            // are printed to the serial monitor as they're being swept.
            if (!cache) {
                Serial.println(F("Number of increments too large. Consider sweeping less than 100 points."));
                return;
            }

            // Enforce calibration before performing sweep
            if (!calibrated) {
                Serial.println(F("Type \"cal\" to perform a calibration sweep first."));
                return;
            }

            // TODO: Method of exporting CSV to file?
            // TODO: Perhaps make a serial monitor listener program, parse and save
            
            printCSVHeader(!swept);
            uint8_t i = 0;
            for (; i < INC_NUM; i++) {
                printCSVLine(i, real[i], imag[i], gain[i], phaseShift[i]);
            }
            printCSVFooter(i);

        // Handle incorrect commands
        } else {
            Serial.print(F("Incorrect command \""));
            Serial.print(command);
            Serial.println(F("\". Try the following:"));
            showCommands();
        }
    }
}

bool frequencySweep(uint8_t n, bool calibration, bool print) {
    return frequencySweep(n, 1, calibration, print);
}

// perform a frequency sweep and record complex impedance data across
// the sweep for n points. If used for calibration, raw real and
// imaginary numbers are used for gain factor and phase shift calculation.
// Otherwise, the gain factor and phase shift are applied to the new sweep data.
// Optionally, the data can be printed as it is being processed if print == true.
// The data can be measured multiple times per point for an average, set by avgNum.
bool frequencySweep(uint8_t n, uint8_t avgNum, bool calibration, bool print) {
    // Begin by issuing a sequence of commands
    // If the commands aren't taking hold, add a brief delay
    if (!(AD5933::setPowerMode(POWER_STANDBY) &&             // place in standby
          AD5933::setControlMode(CTRL_INIT_START_FREQ) &&    // init start freq
          AD5933::setControlMode(CTRL_START_FREQ_SWEEP))) {  // begin frequency sweep
        Serial.println(F("Could not initialize frequency sweep."));
        return false;
    }

    // If printing is enabled, print the CSV header
    if (print) printCSVHeader(calibration);

    int32_t realPoint;
    int32_t imagPoint;
    float magPoint;
    float phasePoint;

    // Perform the sweep
    uint8_t i = 0;
    while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
        // Make sure we aren't exceeding the bounds of our buffer
        if (i == n)
            continue;

        // Get the data for this frequency point and store it
        if (!AD5933::getComplexData(&realPoint, &imagPoint, avgNum)) {
            Serial.print(F("Could not get raw frequency data for loop "));
            Serial.println(i);
            return false;
        }

        magPoint = complexMagnitude(realPoint, imagPoint);
        phasePoint = complexPhase(realPoint, imagPoint);

        // If calibrating, calculate the gain factor and phase shift for current point
        if (calibration) {
            gain[i] = float(1. / (REF_RESIST * magPoint));
            phaseShift[i] = phasePoint;
        } else {
            // Otherwise, apply calibration gain factor and phase shift
            magPoint = float(1. / (gain[i] * magPoint));
            phasePoint -= phaseShift[i];

            // Adjust recorded real and imaginary points to new calibrated values
            realPoint = complexReal(magPoint, phasePoint);
            imagPoint = complexImaginary(magPoint, phasePoint);
        }

        // If printing is enabled, print in CSV format
        if (print) {
            printCSVLine(i, realPoint, imagPoint, gain[i], phaseShift[i]);
        }

        // If impedance data caching is enabled, store in global array
        if (cache) {
            real[i] = realPoint;
            imag[i] = imagPoint;
        }

        // Increment the frequency and loop index
        i++;
        AD5933::setControlMode(CTRL_INCREMENT_FREQ);
    }

    // If printing is enabled, finish by printing the CSV footer
    if (print) printCSVFooter(i);

    // Set AD5933 power mode to standby when finished
    return calibrated = AD5933::setPowerMode(POWER_STANDBY);
}

float complexMagnitude(float real, float imaginary) {
    return sqrt(square(real) + square(imaginary));
}

float complexPhase(float real, float imaginary) {
    float phase = atan2(imaginary, real);   // raw angle

    if (real >= 0 && imaginary >= 0) {
        ; // do nothing                     // 1st quadrant: abs. angle = raw phase
    } else if (real < 0 && imaginary > 0) {
        phase += PI;                        // 2nd quadrant: abs. angle = 180 + phase
    } else if (real < 0 && imaginary < 0) {
        phase -= PI;                        // 3rd quadrant: abs. angle = -180 + phase
    } else {
        phase += 2 * PI;                    // 4th quadrant: abs. angle = 360 + phase
    }

    return phase;
}

float complexReal(float magnitude, float phase) {
    return magnitude * cos(phase);
}

float complexImaginary(float magnitude, float phase) {
    return magnitude * sin(phase);
}

void printCSVHeader(bool cal) {
    Serial.print(F("================= "));
    Serial.print(cal ? F("Calibration") : F("Frequency Sweep"));
    Serial.println(F(" Data (CSV) ================="));
    Serial.println(F("Iteration,\tFrequency,\tResistance,\tReactance,\tMagnitude,\tPhase Angle,\tGain Factor,\t\tPhase Shift"));
    Serial.println(F("i,\t\tf (Hz),\t\tR (Ohm),\tX (Ohm),\t|Z| (Ohm),\tθ (Degrees),\tG,\t\t\tTheta (Degrees)"));
}

void printCSVLine(uint8_t iter, int32_t real, int32_t imag,
                  float gain, float phaseShift) {
    Serial.print(++iter);                           Serial.print(F(",\t\t"));   // iteration
    Serial.print(FREQ_START + iter * FREQ_INC, 0);  Serial.print(F(",\t\t"));   // point frequency
    Serial.print(real);                             Serial.print(F(",\t\t"));   // resistance (real impedance component)
    Serial.print(imag);                             Serial.print(F(",\t\t"));   // reactance (imag impedance component)
    Serial.print(complexMagnitude(real, imag));     Serial.print(F(",\t"));     // impedance magnitude
    Serial.print(atan2(imag, real) * 180 / PI, 3);  Serial.print(F(",\t\t"));   // impedance phase angle in degrees
    Serial.print(gain, 15);                         Serial.print(F(",\t"));     // gain factor (calibration)
    Serial.println(phaseShift * 180 / PI, 4);                                   // phase shift in degrees (calibration)
}

void printCSVFooter(uint8_t i) {
    Serial.print(F("Performed "));
    Serial.print(i);
    Serial.println(F(" measurements.\n"));
}

void showCommands() {
    Serial.println(F("COMMNAD\t\tDESCRIPTION"));
    Serial.println(F("- temp\t\tMeasure AD5933 temperature."));
    Serial.println(F("- cal\t\tCalculate impedance gain factor (Calibration)."));
    Serial.println(F("- sweep\t\tPerform a frequency sweep."));
    Serial.println(F("- exp\t\tExport to CSV file."));
}

void startTimer() {
    Serial.println(F("Working..."));
#if TIMING
    lastMillis = millis();
#endif
}

uint32_t stopTimer() {
#if TIMING
    uint32_t work = millis() - lastMillis;

    Serial.print(F("Done in "));
    Serial.print(work);
    Serial.println(F(" ms."));

    return work;
#else
    return 0L;
#endif
}
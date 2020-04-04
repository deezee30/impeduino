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

#define STIM_OUT 13 // PWM LED

// AD5933 program parameters
const float     FREQ_START  = 10000.F;          // linear frequency sweep start (Hz) [5 kHz < f < 100 kHz]
const float     FREQ_INC    = 1000.F;           // distance between successive frequency points (Hz)
const uint8_t   INC_NUM     = 71;               // number of increments along the sweep [< 511]
const uint16_t  REF_RESIST  = 2700;             // resistance used during calibration (Ohms)
const uint8_t   SWEEP_NUM   = 1;                // sweep each point multiple times, for increased accuracy
// PWM stimulation parameters
const uint32_t  STIM_FREQ   = 10000;            // stimulation pulse frequency (Hz)
const float     STIM_LENGTH = 0.500F;           // total duration of stimulation signal (s)
const float     STIM_POWER  = 0.5;              // PWM duty cycle of the stimulation signal

// Variables for internal use
bool            cache       = false;            // whether or not to store frequency sweep data
float           temperature = 0.F;              // measured temperature of the AD5933
bool            calibrated  = false;            // whether or not multi-point gain is calculated yet
bool            swept       = false;            // whether or not calibrated frequency sweep was performed yet
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
bool stimulate(uint32_t, float, float);         // stimulate neurons via PWM with frequency, duration and duty cycle
bool stimulate(float, float, float);            // stimulate neurons via PWM with high, low and overall durations
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

/**
 * Initialisation
 */
void setup() {
    // Set up digital neuron stimulation voltage output
    pinMode(STIM_OUT, OUTPUT);

    // Join I2C bus, initialise Wire library and start serial communication at 9600 bps
    Wire.begin();
    Serial.begin(9600);

    // Run the clock at 400 kHz (fast mode) instead of 100 kHz (standard mode)
    Wire.setClock(400000);

    Serial.println(F("\n-------- Start --------"));

    // Perform initial configuration. Fail if any one of these fail
    if (!(AD5933::reset() &&
          AD5933::setInternalClock(true) &&
          AD5933::setStartFrequency(FREQ_START) &&
          AD5933::setIncrementFrequency(FREQ_INC) &&
          AD5933::setNumberIncrements(INC_NUM) &&
          AD5933::setPGAGain(CTRL_PGA_GAIN_X1) &&
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

/**
 * Main program tick
 */
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

            printCSVHeader(!swept);
            uint8_t i = 0;
            for (; i < INC_NUM; i++) {
                printCSVLine(i, real[i], imag[i], gain[i], phaseShift[i]);
            }
            printCSVFooter(i);

        // Handle tissue stimulation
        } else if (command.equals(F("stim"))) {

            startTimer();

            // Stimulate material with default parameters
            if (!stimulate(STIM_FREQ, STIM_LENGTH, STIM_POWER)) {
                Serial.println(F("Stimulation failed. Use different parameters."));
                return;
            }

            stopTimer();

        // Handle incorrect commands
        } else {
            Serial.print(F("Incorrect command \""));
            Serial.print(command);
            Serial.println(F("\". Try the following:"));
            showCommands();
        }
    }
}

/**
 * Perform a frequency sweep and record complex impedance data across the sweep
 * for n points.
 * 
 * If used for calibration, raw real and imaginary numbers are used for gain
 * factor and phase shift calculation. Otherwise, the gain factor and phase
 * shift are applied to the new sweep data.
 * 
 * Optionally, the data can be printed as it is being processed if @param print
 * is set to @code{true}.
 * 
 * @param n           number of points in the sweep
 * @param calibration whether or not the sweep is used for calibrating the
 *                    system (as opposed to doing the actual measurement)
 * @param print       whether or not to print data to serial during sweep
 * @return @code{true} if successful, @code{false} if not
 * 
 * @see frequencySweep(uint8_t, uint8_t, bool, bool) if data is to be measured
 * multiple times per point and averaged out.
 */
bool frequencySweep(uint8_t n, bool calibration, bool print) {
    return frequencySweep(n, 1, calibration, print);
}

/**
 * Perform a frequency sweep and record complex impedance data across the sweep
 * for n points.
 * 
 * If used for calibration, raw real and imaginary numbers are used for gain
 * factor and phase shift calculation. Otherwise, the gain factor and phase
 * shift are applied to the new sweep data.
 * 
 * Optionally, the data can be printed as it is being processed if @param print
 * is set to @code{true}.
 * 
 * @param n           number of points in the sweep
 * @param avgNum      amount of measurements to take for each complex impedance
 *                    point for averaged out results
 * @param calibration whether or not the sweep is used for calibrating the
 *                    system (as opposed to doing the actual measurement)
 * @param print       whether or not to print data to serial during sweep
 * @return @code{true} if successful, @code{false} if not
 */
bool frequencySweep(uint8_t n, uint8_t avgNum, bool calibration, bool print) {
    // Begin by issuing a sequence of commands
    // If the commands aren't taking hold, add a brief delay
    if (!(AD5933::setControlMode(CTRL_STANDBY_MODE) &&       // place in standby
          AD5933::setControlMode(CTRL_INIT_START_FREQ) &&    // init start freq
          AD5933::setControlMode(CTRL_START_FREQ_SWEEP))) {  // begin frequency sweep
        Serial.println(F("Could not initialize frequency sweep."));
        return false;
    }

    // If printing is enabled, print the CSV header
    if (print) printCSVHeader(calibration);

#if DEBUG
    Serial.print(F("[ "));
#endif

    int32_t realPoint;
    int32_t imagPoint;
    float magPoint;
    float phasePoint;

    // Perform the sweep
    uint8_t i = 0;
    while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
        // Make sure we aren't exceeding the bounds of our buffer
        if (i == n)
            break;

        // Get the data for this frequency point and store it
        if (!AD5933::getComplexData(&realPoint, &imagPoint, avgNum)) {
            Serial.print(F("Could not get raw frequency data for loop "));
            Serial.println(i);
            break;
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

#if DEBUG
        // Update progress at 10% intervals
        if (fmod(i, 0.1 * n) == 0) {
            Serial.print(F("="));
        }
#endif
    }

#if DEBUG
    Serial.print(F(" ]"));
#endif

    // If printing is enabled, finish by printing the CSV footer
    if (print) printCSVFooter(i);

    // Set AD5933 power mode to standby when finished
    return calibrated = AD5933::setControlMode(CTRL_STANDBY_MODE);
}

/**
 * Initiates a Pulse Width Modulation (PWM) signal to stimulate tissue. Wired to
 * @code{STIM_OUT} to produce a digital signal.
 * 
 * The duty cycle can be represented as the ratio of high to low duration
 * portions of PWN signal.
 * 
 * Processed in microseconds, so timings are accurate to the nearest microsecond. 
 * 
 * @param frequency frequency of signal in Hz. Max. 1 MHz
 * @param duration  total stimulation length in seconds
 * @param dutyCycle ratio of durations of pin state high to low during a single
 *                  cycle, such that [0 < dutyCycle < 1]. Default is 0.5
 * @return @code{true} if successful, @code{false} if not
 * @see stimulate(float, float, duration) for alternative inputs
 */
bool stimulate(uint32_t frequency, float duration, float dutyCycle = 0.5) {
    if (dutyCycle < 0 || dutyCycle > 1) return false;
    if (frequency > 1E6) return false; // Allow max. 1 MHz frequency

    // Convert duty cycle to absolute durations of high and low voltage output
    // The period of the pulse is the sum of high and low state lengths
    float period = 1.F / frequency;
    return stimulate(dutyCycle * period, (1 - dutyCycle) * period, duration);
}

/**
 * Initiates a Pulse Width Modulation (PWM) signal to stimulate tissue. Wired to
 * @code{STIM_OUT} to produce a digital signal.
 * 
 * The duty cycle can be represented as the ratio of high to low duration
 * portions of PWN signal.
 * 
 * Processed in microseconds, so timings are accurate to the nearest microsecond.
 * Maximum @param high and @param low values set to 4294 seconds so as to avoid
 * microsecond overflow.
 * 
 * @param high     duration of pin state set to high, in seconds. Max. 4294 s
 * @param low      duration of pin state set to low, in seconds. Max. 4294 s
 * @param duration total stimulation length in seconds
 * @return @code{true} if successful, @code{false} if not
 */
bool stimulate(float high, float low, float duration) {
    // Max. pulse width of 4294 seconds to avoid future microsecond overflow
    if (high > 4294 || low > 4294) return false;
    if (high < 0 || low < 0 || duration <= 0) return false;

    // Number of pulses to generate
    uint32_t fin = floor(duration / (high + low));

    // Convert high and low timings to micros
    uint32_t highMicros = floor(high * 1E6);
    uint32_t lowMicros = floor(low * 1E6);

#if DEBUG
    Serial.print(F("Stimulating for "));
    Serial.print(fin);
    Serial.println(F(" pulses..."));

    Serial.print(F("[ "));
#endif

    uint32_t i = 0;
    while (i++ < fin) {
        // High
        digitalWrite(STIM_OUT, HIGH);
        delayMicroseconds(highMicros);

        // Low
        digitalWrite(STIM_OUT, LOW);
        delayMicroseconds(lowMicros);

#if DEBUG
        // Update progress at 10% intervals
        if (fmod(i, 0.1 * fin) == 0) {
            Serial.print(F("="));
        }
#endif
    }

#if DEBUG
    Serial.println(F(" ]"));
#endif

    return true;
}

/**
 * Returns the magnitude of a complex impedance.
 * 
 * @param real real component of a complex impedance (Ohms)
 * @param imag imaginary component of a complex impedance (Ohms)
 * @return the magnitude of a complex impedance (Ohms)
 */
float complexMagnitude(float real, float imag) {
    return sqrt(square(real) + square(imag));
}

/**
 * Returns the phase angle of a complex impedance via tan2 method.
 * 
 * @param real real component of a complex impedance (Ohms)
 * @param imag imaginary component of a complex impedance (Ohms)
 * @return the phase angle of a complex impedance (rad)
 */
float complexPhase(float real, float imag) {
    return atan2(imag, real);
}

/**
 * Returns the real component of a polar impedance.
 * 
 * @param magnitude magnitude of a polar impedance (Ohms)
 * @param phase phase angle of a polar impedance (rad)
 * @return real component of a polar impedance (Ohm)
 */
float complexReal(float magnitude, float phase) {
    return magnitude * cos(phase);
}

/**
 * Returns the imaginary component of a polar impedance.
 * 
 * @param magnitude magnitude of a polar impedance (Ohms)
 * @param phase phase angle of a polar impedance (rad)
 * @return imaginary component of a polar impedance (Ohm)
 */
float complexImaginary(float magnitude, float phase) {
    return magnitude * sin(phase);
}

/**
 * Prints table header of comma-separated value (CSV) format to serial console.
 * Also compatible with tab-separated value (TSV) format.
 * 
 * @param cal whether or not the table will contain calibration data as opposed
 *            to measurement data
 */
void printCSVHeader(bool cal) {
    Serial.print(F("================= "));
    Serial.print(cal ? F("Calibration") : F("Frequency Sweep"));
    Serial.println(F(" Data (CSV) ================="));
    Serial.println(F("Iteration,\tFrequency,\tResistance,\tReactance,\tMagnitude,\tPhase Angle,\tGain Factor,\t\tPhase Shift"));
    Serial.println(F("i,\t\tf (Hz),\t\tR (Ohm),\tX (Ohm),\t|Z| (Ohm),\tθ (Degrees),\tG,\t\t\tTheta (Degrees)"));
}

/**
 * Prints complex impedance measurement result entry of comma-separated value
 * (CSV) format to serial console. Also compatible with tab-separated value
 * (TSV) format.
 * 
 * @param iter iteration number
 * @param real complex impedance real counterpart
 * @param imag complex impedance imaginary counterpart
 * @param gain calibration gain factor used for corrected impedance calculation
 * @param phaseShift phase shift used for corrected phase angle calculation
 */
void printCSVLine(uint8_t iter, int32_t real, int32_t imag,
                  float gain, float phaseShift) {
    Serial.print(iter + 1);                         Serial.print(F(",\t\t"));   // iteration
    Serial.print(FREQ_START + iter * FREQ_INC, 0);  Serial.print(F(",\t\t"));   // point frequency
    Serial.print(real);                             Serial.print(F(",\t\t"));   // resistance (real impedance component)
    Serial.print(imag);                             Serial.print(F(",\t\t"));   // reactance (imag impedance component)
    Serial.print(complexMagnitude(real, imag));     Serial.print(F(",\t"));     // impedance magnitude
    Serial.print(atan2(imag, real) * 180 / PI, 3);  Serial.print(F(",\t\t"));   // impedance phase angle in degrees
    Serial.print(gain, 15);                         Serial.print(F(",\t"));     // gain factor (calibration)
    Serial.println(phaseShift * 180 / PI, 4);                                   // phase shift in degrees (calibration)
}

/**
 * Prints table footer for impedance table data.
 * @param i number of measurements performed
 */
void printCSVFooter(uint8_t i) {
    Serial.print(F("Performed "));
    Serial.print(i);
    Serial.println(F(" measurements.\n"));
}

/**
 * Displays a command list for interaction with ATmega328P via serial console.
 */
void showCommands() {
    Serial.println(F("COMMNAD\t\tDESCRIPTION"));
    Serial.println(F("- temp\t\tMeasure AD5933 temperature."));
    Serial.println(F("- cal\t\tCalculate impedance gain factor (Calibration)."));
    Serial.println(F("- sweep\t\tPerform a frequency sweep."));
    Serial.println(F("- exp\t\tExport to CSV file."));
    Serial.println(F("- stim\t\tOutput pulsatile stimulation"));
}

/**
 * Initiates timer for heavy tasks if timing feature is enabled.
 */
void startTimer() {
    Serial.println(F("Working..."));
#if TIMING
    lastMillis = millis();
#endif
}

/**
 * Stops timer after heavy tasks and prints timing if timing feature is enabled.
 */
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
#ifndef AD5933_h
#define AD5933_h

// Includes
#include "Arduino.h"
#include "Wire.h"

// AD5933 Register Map - based on datasheet p. 23

// Device address and address pointer
#define AD5933_ADDR                 0x0D                    // 7-bit I2C serial bus address to AD5933 impedance analyser
#define ADDR_PTR                    0xB0                    // pointer to address of the register
// Control Register
#define CTRL_REG1                   0x80                    // [D15 - D8]  16-bit read/write
#define CTRL_REG2                   0x81                    // [D7  - D0]  16-bit read/write
// Start Frequency Register
#define START_FREQ_1                0x82                    // [D23 - D16] 24-bit read/write register 1
#define START_FREQ_2                0x83                    // [D15 - D8]  24-bit read/write register 2
#define START_FREQ_3                0x84                    // [D7  - D0]  24-bit read/write register 3
// Frequency increment register
#define INC_FREQ_1                  0x85                    // [D23 - D16] 24-bit read/write register 1
#define INC_FREQ_2                  0x86                    // [D15 - D8]  24-bit read/write register 2
#define INC_FREQ_3                  0x87                    // [D7  - D0]  24-bit read/write register 3
// Number of increments register
#define NUM_INC_1                   0x88                    // [D15 - D9]  9-bit  read/write register 1
#define NUM_INC_2                   0x89                    // [D7  - D0]  9-bit  read/write register 2
// Number of settling time cycles register
#define NUM_SCYCLES_1               0x8A                    // [D15 - D8]  16-bit read/write register 1
#define NUM_SCYCLES_2               0x8B                    // [D7  - D0]  16-bit read/write register 2
// Status register
#define STATUS_REG                  0x8F                    // [D7  - D0]  8-bit  read status register
// Temperature data register
#define TEMP_DATA_1                 0x92                    // [D15 - D8]  16-bit read register 1
#define TEMP_DATA_2                 0x93                    // [D7  - D0]  16-bit read register 2
// Real data register
#define REAL_DATA_1                 0x94                    // [D15 - D8]  16-bit read register 1
#define REAL_DATA_2                 0x95                    // [D7  - D0]  16-bit read register 2
// Imaginary data register
#define IMAG_DATA_1                 0x96                    // [D15 - D8]  16-bit read register 1
#define IMAG_DATA_2                 0x97                    // [D7  - D0]  16-bit read register 2

// Constants for use with the AD5933 library class.

// Control register commands
#define CTRL_NO_OPERATION           0b00000000              // [D15 D14 D13 D12] => [0 0 0 0]
#define CTRL_INIT_START_FREQ        0b00010000              // [D15 D14 D13 D12] => [0 0 0 1]
#define CTRL_START_FREQ_SWEEP       0b00100000              // [D15 D14 D13 D12] => [0 0 1 0]
#define CTRL_INCREMENT_FREQ         0b00110000              // [D15 D14 D13 D12] => [0 0 1 1]
#define CTRL_REPEAT_FREQ            0b01000000              // [D15 D14 D13 D12] => [0 1 0 0]
#define CTRL_TEMP_MEASURE           0b10010000              // [D15 D14 D13 D12] => [1 0 0 1]
#define CTRL_POWER_DOWN_MODE        0b10100000              // [D15 D14 D13 D12] => [1 0 1 0]
#define CTRL_STANDBY_MODE           0b10110000              // [D15 D14 D13 D12] => [1 0 1 1]
#define CTRL_RESET                  0b00010000              // [D4] => [1]
#define CTRL_CLOCK_EXTERNAL         0b00001000              // [D3] => [1]
#define CTRL_CLOCK_INTERNAL         0b00000000              // [D3] => [0]
#define CTRL_PGA_GAIN_X1            0b00000001              // [D8] => [1]
#define CTRL_PGA_GAIN_X5            0b00000000              // [D8] => [5]
// Block read and write
#define BLOCK_WRITE                 0b10100000
#define BLOCK_READ                  0b10100001
// Number of settling time cycles
#define NUM_ST_CYCLES_DEFAULT       0b00000000              // [D10 D9] => [0 0]
#define NUM_ST_CYCLES_X2            0b00000001              // [D10 D9] => [0 1]
#define NUM_ST_CYCLES_X4            0b00000011              // [D10 D9] => [1 1]
// I2C result
#define I2C_RESULT_SUCCESS          0                       // I2C transmission successful
#define I2C_RESULT_DATA_TOO_LONG    1                       // I2C error: data too long
#define I2C_RESULT_ADDR_NACK        2                       // I2C error: address sent, NACK received
#define I2C_RESULT_DATA_NACK        3                       // I2C error: data sent, NACK received
#define I2C_RESULT_OTHER_FAIL       4                       // other I2C error (lost bus arbitration, bus error...)
// Status register options
#define STATUS_TEMP_VALID           0b00000001              // valid temperature measurement
#define STATUS_DATA_VALID           0b00000010              // valid real/imaginary data
#define STATUS_SWEEP_DONE           0b00000100              // frequency sweep complete
#define STATUS_ERROR                0b11111111              // operation unsuccessful / unknown status

// Loggers and debuggers
#define DEBUG                       true                   // keep false unless debugging
#define TIMING                      true                    // records and prints timing of heavy operations

// AD5933 Library class
// Contains mainly functions for interfacing with the AD5933.
class AD5933 {
   public:

    // Read registers
    static uint8_t readRegister(uint8_t);
    static uint8_t readStatusRegister();
    static uint8_t readControlRegister();

    // Resets the board.
    static bool reset();

    // Set control mode register CTRL_REG1
    static bool setControlMode(uint8_t);

    // Clock
    static bool setClockSource(uint8_t);
    static bool setInternalClock(bool);

    // No. of settline time cycles
    static bool setSettlingCycles(uint16_t, uint8_t);

    // Frequency sweep configuration
    static bool setStartFrequency(uint32_t);
    static bool setIncrementFrequency(uint32_t);
    static bool setNumberIncrements(uint16_t);
    inline static bool setFrequencyRange(uint32_t startFreq, uint32_t incFreq, uint16_t numInc) {
        return setStartFrequency(startFreq) &&
               setIncrementFrequency(incFreq) &&
               setNumberIncrements(numInc);
    }

    // Gain multiplier configuration
    static bool setPGAGain(uint8_t);

    // Temperature measurement
    static bool getTemperature(float*);
    static bool getTemperatureExperimental(float*);

    // Impedance data
    inline static bool getComplexData(int32_t* real, int32_t* imag) {
        return getComplexData(real, imag, 1);
    }
    static bool getComplexData(int32_t*, int32_t*, uint8_t avgNum);

   private:
    // Private data
    static const uint32_t clockSpeed = 16776000L;

    // Sending/receiving byte methods, for easy re-use
    inline static bool readByte(uint8_t address, uint8_t* value) {
        uint8_t val[1];
        if (readBytes(address, 1, val)) {
            *value = val[0];
            return true;
        }
        return false;
    }
    static bool readBytes(uint8_t, uint8_t, uint8_t[]);
    inline static bool blockReadBytes(uint8_t address, uint8_t blocks,
                                      uint8_t values[]) {
        return blockReadBytes(address, blocks, 1, values);
    }
    static bool blockReadBytes(uint8_t, uint8_t, uint8_t, uint8_t[]);
    static bool writeByte(uint8_t, uint8_t);
    static bool blockWriteBytes(uint8_t, uint8_t, uint8_t[]);
};

#endif
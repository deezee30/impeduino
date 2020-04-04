#include "AD5933.h"
#include "Math.h"

/**
 * Reads AD5933 register and returns its value. Directly calls @link{readByte()}.
 * 
 * @param reg the register to read
 * @return the register value if successful, or error code if one occurred.
 */
uint8_t AD5933::readRegister(uint8_t reg) {
    uint8_t val;
    if (readByte(reg, &val)) {
        return val;
    } else {
        return STATUS_ERROR;
    }
}

/**
 * Reads AD5933 status register and returns its value. Directly calls
 * @link{readRegister()}.
 * @return the value of the status register if successful, or error code if one
 *         occurred.
 */
uint8_t AD5933::readStatusRegister() {
    return readRegister(STATUS_REG);
}

/**
 * Reads both AD5933 control registers and returns their 2CS combined value.
 * Directly calls @link{readRegister()} on both control registers.
 * @return the value of the control registers if successful, or error code if
 *         one occurred.
 */
uint8_t AD5933::readControlRegister() {
    return ((readRegister(CTRL_REG1) << 8) | readRegister(CTRL_REG2)) & 0xFFFF;
}

/**
 * Sets the control mode of the AD5933 to a value specified by @param mode.
 * 
 * Note that the "Control" register should not be written to as part of a
 * "Block-Write" command.
 * 
 * @param mode the control mode to set
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::setControlMode(uint8_t mode) {
    // Get the current value of the "Control" register
    uint8_t val;
    if (!readByte(CTRL_REG1, &val))
        return false;

    // Wipe out the top 4 bits. Mode bits are bits 5 through 8.
    val &= 0x0F;

    // Set the top 4 bits appropriately
    val |= mode;

    // Write back to the register
    return writeByte(CTRL_REG1, val);
}

/**
 * Resets the AD5933.
 * 
 * Does not reset any programmed values associated with sweep, (that is,
 * "Start Frequency", "Frequency Increment" and "Number of Increments").
 * 
 * After a "Reset" command, an "Initialize with Start Frequency" command must
 * be issued to the "Control" register to restart the frequency sweep sequence.
 */
bool AD5933::reset() {
    // Get the current value of the "Control" register
    uint8_t val;
    if (!readByte(CTRL_REG2, &val))
        return false;

    // Set bit D4 for reset
    val |= CTRL_RESET;

    // Send byte back
    return writeByte(CTRL_REG2, val);
}

/**
 * Obtains the current temperature of the AD5933 as measured from both registers
 * using the block-read technique. Accurate for temperatures -40 < T < +125 C.
 * 
 * See datasheet for detailed process description for temperature measurement.
 * 
 * @param temp output variable to assign the measured temperature to
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::getTemperature(float *temp) {
    // Set temperature mode
    if (!setControlMode(CTRL_TEMP_MEASURE))
        return false;

    // Wait for a valid temperature to be ready
    while ((readStatusRegister() & STATUS_TEMP_VALID) != STATUS_TEMP_VALID)
        continue;

    // Block-read raw temperature values from both temperature registers
    uint8_t rawTemp[2];
    if (blockReadBytes(TEMP_DATA_1, 2, rawTemp)) {

        // Combine raw temperature bytes into an interger. The ADC
        // returns a 14-bit 2's C value where the 14th bit is a sign
        // bit. As such, we only need to keep the bottom 13 bits.
        int16_t tempVal = (int16_t(rawTemp[0]) << 8 | rawTemp[1]) & 0x1FFF;

        // Convert into celcius using the formula given in the
        // datasheet. There is a different formula depending on the sign
        // bit, which is the 5th bit of the byte in TEMP_DATA_1.
        if (bitRead(rawTemp[0], 5) == 0) {
            // Positive (msb = 0)
            *temp = tempVal / 32.0;
        } else {
            // Negative (msb = 1)
            *temp = (tempVal - 16384) / 32.0;
        }

        return true;
    }

    return false;
}

/**
 * Sets internal/external master clock source.
 * @param source internal/external source
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::setClockSource(uint8_t source) {
    // Determine what source was selected and set it appropriately
    switch (source) {
        case CTRL_CLOCK_EXTERNAL:
        case CTRL_CLOCK_INTERNAL:
            return writeByte(CTRL_REG2, source);
        default:
            return false;
    }
}

/**
 * Sets whether or not to use an internal clock source.
 * @param source @code{true} for internal source, @code{false} for external
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::setInternalClock(bool internal) {
    // This function is essentially a wrapper for setClockSource()
    return setClockSource(internal ? CTRL_CLOCK_INTERNAL : CTRL_CLOCK_EXTERNAL);
}

/**
 * Updates both AD5933 settling time cycle registers with the number of settling
 * time cycles to use for sweeping.
 * 
 * @param nstc       number of settling time cycles: 9 bits, max. value 511
 * @param multiplier NSTC multiplier
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::setSettlingCycles(uint16_t nstc, uint8_t multiplier) {
    // nstc is a word, aka 9 bits long -> max. number is 511
    if (nstc > 511 || multiplier > 4) return false;

    // Ensure multiplier is valid
    switch (multiplier) {
        case NUM_ST_CYCLES_DEFAULT:
        case NUM_ST_CYCLES_X2:
        case NUM_ST_CYCLES_X4:
            break;
        default:
            return false;
    }

    // D15 to D11 are don't cares. The next 2 is the multiplier
    // and the rest are the 9-bit number of cycles
    uint8_t highByte = (multiplier << 1) | ((nstc >> 8) & 0xFF);
    uint8_t lowByte  = nstc & 0xFF;

    return writeByte(NUM_SCYCLES_1, highByte)
        && writeByte(NUM_SCYCLES_2, lowByte);
}

/**
 * Updates low, mid and high AD5933 start frequency registers with the initial
 * frequency to start sweeping with.
 * @param start initial frequency to sweep with
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::setStartFrequency(uint32_t start) {
    // Page 24 of the Datasheet gives the following formula to represent the
    // start frequency.
    uint32_t freqHex = start * pow(2, 29) / clockSpeed;
    if (freqHex > 0xFFFFFF) {
        return false; // overflow
    }

    // freqHex should be a 24-bit value. We need to break it up into 3 bytes.
    uint8_t highByte = (freqHex >> 16) & 0xFF;
    uint8_t midByte  = (freqHex >> 8)  & 0xFF;
    uint8_t lowByte  = (freqHex)       & 0xFF;

    // Attempt sending all three bytes
    return writeByte(START_FREQ_1, highByte)
        && writeByte(START_FREQ_2, midByte)
        && writeByte(START_FREQ_3, lowByte);
}

/**
 * Updates low, mid and high AD5933 increment frequency registers with the
 * frequency to increase each sweeping iteration with.
 * @param increment delta frequency to increase by
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::setIncrementFrequency(uint32_t increment) {
    // Page 25 of the Datasheet gives the following formula to represent the
    // increment frequency.
    uint32_t freqHex = increment * pow(2, 29) / clockSpeed;
    if (freqHex > 0xFFFFFF) {
        return false;  // overflow
    }

    // freqHex should be a 24-bit value. We need to break it up into 3 bytes.
    uint8_t highByte = (freqHex >> 16) & 0xFF;
    uint8_t midByte  = (freqHex >> 8)  & 0xFF;
    uint8_t lowByte  = (freqHex)       & 0xFF;

    // Attempt sending all three bytes
    return writeByte(INC_FREQ_1, highByte)
        && writeByte(INC_FREQ_2, midByte)
        && writeByte(INC_FREQ_3, lowByte);
}

/**
 * Updates both AD5933 increment number registers with the number of points
 * to be recorded during the entire sweep.
 * @param num number of data points in the sweep
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::setNumberIncrements(uint16_t num) {
    // Check that the number sent in is valid.
    if (num > 511) {
        return false;
    }

    // Divide the 9-bit integer into 2 bytes.
    uint8_t highByte = (num >> 8) & 0xFF;
    uint8_t lowByte  = (num)      & 0xFF;

    // Write to register.
    return writeByte(NUM_INC_1, highByte)
        && writeByte(NUM_INC_2, lowByte);
}

/**
 * Sets PGA gain to either x1 (default) or x5.
 * @param gain @code{1} or @code{5} PGA gain
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::setPGAGain(uint8_t gain) {
    // Get the current value of the "Control" register
    uint8_t val;
    if (!readByte(CTRL_REG1, &val))
        return false;

    // Clear out the bottom bit, D8, which is the PGA gain set bit
    val &= 0xFE;

    // Determine what gain factor was selected
    if (gain == CTRL_PGA_GAIN_X1 || gain == 1) {
        // Set PGA gain to x1 in CTRL_REG1
        val |= CTRL_PGA_GAIN_X1;
    } else if (gain == CTRL_PGA_GAIN_X5 || gain == 5) {
        // Set PGA gain to x5 in CTRL_REG1
        val |= CTRL_PGA_GAIN_X5;
    } else {
        return false;
    }

    return writeByte(CTRL_REG1, val);
}

/**
 * Obtains complex impedance data at a specific point during a frequency sweep.
 * 
 * The block-read technique is used to read four registers at one time to
 * speed the reading process. If the measurement cannot be performed or gives
 * invalid complex data three times, then the operation is cancelled and
 * @code{false} is returned.
 * 
 * If @code{avgNum > 1}, then each data point is re-measured multiple times
 * and averaged out to increase mean precision of results.
 *
 * @param real   output real component value of the impedance at a point
 * @param imag   output imaginary component value of the impedance at a point
 * @param avgNum number of times to measure the point for averaging, default is 1
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::getComplexData(int32_t *real, int32_t *imag, uint8_t avgNum = 1) {
    // Wait for a measurement to be available
    while ((readStatusRegister() & STATUS_DATA_VALID) != STATUS_DATA_VALID)
        continue;

    // Contains block-read data from 2's C impedance component registers
    // [0, 1] -> real register, [2, 3] -> imaginary register
    uint8_t rawImp[4];

    uint8_t err = 0;
    int32_t sumReal = 0;
    int32_t sumImag = 0;
    int8_t n = 0;
    for (;;) {
        // after 3 fails, escape the loop
        if (err == 3) break;

        // Block-read 4 raw real and imaginary impedance components, starting from REAL_DATA_1
        if (blockReadBytes(REAL_DATA_1, 4, rawImp)) {

            // 16-bit processing by 2's complement system
            sumReal += (rawImp[0] << 8) | rawImp[1];
            sumImag += (rawImp[2] << 8) | rawImp[3];

            if (++n == avgNum) {
                break;
            } else {
                // repeat frequency
                if (!setControlMode(CTRL_REPEAT_FREQ)) {
                    err++;
                }
            }
        } else {
            // if fail, then skip this loop and move to the next one
            err++;
        }
    }

    if (err == 3) {
        *real = -1;
        *imag = -1;
        return false;
    }

    *real = sumReal / n;
    *imag = sumImag / n;

    return true;
}

/**
 * Reads a byte using the address pointer register. Supports multiple byte
 * readings during one transmission, if provided by the slave.
 * 
 * @param address the address to read bytes from
 * @param bytes   amount of bytes expected to read
 * @param values  output bytes read from the given address, if successful
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::readBytes(uint8_t address, uint8_t bytes, uint8_t values[]) {
    if (bytes > 32)
        return false;
    
#if DEBUG
    Serial.print(F("Awaiting "));
    Serial.print(bytes);
    Serial.print(F(" byte(s) from 0x"));
    Serial.println(address, HEX);
#endif

    // transmit address pointer to slave address and heck that it's successful
    if (!writeByte(ADDR_PTR, address)) {
        Serial.println(F("I2C address pointer error"));
        return false;
    }

    // Read and handle the bytes from the slave address
    Wire.requestFrom((uint8_t) AD5933_ADDR, (uint8_t) bytes);
    uint8_t count = 0;
    while (count < bytes && Wire.available()) {
        values[count++] = Wire.read();

#if DEBUG
        Serial.print(F("  Byte "));
        Serial.print(count);
        Serial.print(F(": "));
        Serial.println(values[count - 1]);
#endif
    }

#if DEBUG
    Serial.print(F("  Received "));
    Serial.print(count);
    Serial.println(F(" byte(s)."));
#endif

    return bytes == count;
}

/**
 * Reads a block of registers with the possibility of each having multiple bytes
 * in one transmission using the address pointer register. The read must start
 * at numerically the lowest address index and must increment by 1 within the
 * same block.
 * 
 * @param address       the address to read bytes from
 * @param blocks        amount of blocks expected to read
 * @param bytesPerBlock amount of bytes each block will have
 * @param values        output bytes read from the given address, if successful
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::blockReadBytes(uint8_t address,       uint8_t blocks,
                            uint8_t bytesPerBlock, uint8_t values[]) {
    if (bytesPerBlock > 32 || blocks > 8)
        return false;

#if DEBUG
    Serial.print(F("Awaiting "));
    Serial.print(blocks);
    Serial.print(F(" blocks of "));
    Serial.print(bytesPerBlock);
    Serial.print(F(" byte(s) starting from 0x"));
    Serial.println(address, HEX);
#endif

    // transmit address pointer to slave address and heck that it's successful
    if (!writeByte(ADDR_PTR, address)) {
        Serial.println(F("I2C address pointer error"));
        return false;
    }

    // transmit the block read command and block size and check that it's successful
    if (!writeByte(BLOCK_READ, blocks)) {
        Serial.println(F("I2C block read error."));
        return false;
    }

    for (int x = 0; x < blocks; x++) {
        Wire.requestFrom((uint8_t) AD5933_ADDR, (uint8_t) bytesPerBlock);
        
#if DEBUG
        Serial.print(F("  Block "));
        Serial.print(x + 1);
        Serial.print(F(" @ 0x"));
        Serial.print(address + x, HEX);
        Serial.println(F(":"));
#endif

        uint8_t pos = x * bytesPerBlock;
        uint8_t y = 0;
        while (y < bytesPerBlock && Wire.available()) {
            values[pos + y++] = Wire.read();
#if DEBUG
            Serial.print(F("    Byte "));
            Serial.print(y);
            Serial.print(F(": "));
            Serial.println(values[pos + y - 1]);
#endif
        }
    }

#if DEBUG
    Serial.print(F("  Received "));
    Serial.print(blocks * bytesPerBlock);
    Serial.println(F(" byte(s) in total."));
#endif

    return true;
}

/**
 * Writes a byte to address and finalises transmission.
 * 
 * @param address the address to write the byte to
 * @param value   the byte to send to the address
 * @return @code{true} if successful, @code{false} if not
 */
bool AD5933::writeByte(uint8_t address, uint8_t value) {
    Wire.beginTransmission(AD5933_ADDR); // transmit to slave address
    Wire.write(address); // address to slave address
    Wire.write(value); // write byte to slave address

    return Wire.endTransmission() == I2C_RESULT_SUCCESS;
}

// TODO: Block write.
bool AD5933::blockWriteBytes(uint8_t address, uint8_t bytes, uint8_t values[]) {
    return false;
}
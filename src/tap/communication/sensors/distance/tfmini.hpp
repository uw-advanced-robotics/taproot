/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_TFMINI_HPP_
#define TAPROOT_TFMINI_HPP_

#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "modm/architecture/interface/uart.hpp"

namespace tap::communication::sensors::distance
{
/**
 * Object for interpreting and decoding the serial communications of a TFMini
 *
 * Takes in bytes one at a time and appropriately interprets as a TFMini message. Handles dropped
 * bytes by using the robustness in the TFMini protocol (unique start sequence and final checksum).
 *
 * Datasheet (which contains serial protocol) can be found online by searching "TFMini Lidar"
 *
 * This class is stateful and expects serial bytes, and thus should not be used to decode two
 * different TFMini streams at the same time.
 */
class TFMiniDecoder
{
public:
    /**
     * Process the next data byte received from the tfmini
     *
     * @param data the byte to be processed
     *
     * @return `true` iff this byte was the last in a valid message frame (i.e.: new data
     * is now available through the getter methods). `false` otherwise (either message is still in
     * progress or message completed but failed checksum)
     */
    bool processNextByte(uint8_t data)
    {
        if (sequenceNum < 2)
        {
            // we wait for two 0x59's in a row as that indicates
            // start of message
            if (data == 0x59)
            {
                buffer[sequenceNum] = data;
                sequenceNum++;
            }
        }
        else
        {
            buffer[sequenceNum] = data;
            sequenceNum++;
        }

        // Parse buffer once full
        if (sequenceNum == FRAME_SIZE)
        {
            sequenceNum = 0;
            return parseBuffer();
        }
        else
        {
            return false;
        }
    }

private:
    static constexpr size_t FRAME_SIZE = 9;

    /**
     * Parses the contents of the buffer and stores them into appropriate member variables if
     * successful.
     *
     * @return `true` iff the buffer was successfully parsed as a valid TFMini message frame with
     * a valid checksum
     */
    bool parseBuffer()
    {
        if (isChecksumValid())
        {
            // buffer[0] and buffer[1] are start of frame indicators
            dist = buffer[2] | buffer[3] << 8;
            strength = buffer[4] | buffer[5] << 8;
            integrationTime = buffer[6];
            // buffer[7] is reserved
            // buffer[8] is checksum
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * @returns `true` iff the checksum in this->buffer is correct. More precisely, checks that
     * buffer[8] is equal to the lowest byte of sum(buffer[0..7]), i.e.: if the checksum is valid.
     */
    inline bool isChecksumValid()
    {
        uint8_t checksum = buffer[8];

        int ourChecksum = 0;
        // Checksum is last 8 bits of sum of first 8 bytes of message
        for (int i = 0; i < 8; i++)
        {
            ourChecksum += buffer[i];
        }

        return checksum == (ourChecksum & 0xFF);
    }

    /**
     * Index in the message frame of the next byte to be received (also points to next position for
     * byte to be inserted into this->buffer). Tracks where we are in the message frame. A full
     * frame from the TFMini has 9 bytes, so this should always be < 9.
     */
    int sequenceNum = 0;

    /**
     * Stores the current message frame as it's being built. buffer[0..sequenceNum-1] are bytes that
     * have been read for the current in-progress message.
     */
    uint8_t buffer[9];

    /**
     * Distance reported by the sensor in centimeters (cm)
     */
    uint16_t dist = 0;

    /**
     * "Signal strength" according to the datasheet. No idea what this actually means :/
     * TODO: find out what this means. Probably not relevant for most use cases though
     */
    uint16_t strength = 0;

    /**
     * "Integration time". TODO: find out units and meaning.
     */
    uint8_t integrationTime = 0;
};

/**
 * Handles communication with a TFMini over a single UART port
 *
 * @tparam UartPort the uart port the TFMini is bound to
 */
template <::tap::communication::serial::Uart::UartPort TFMINI_UART_PORT>
class TFMiniUartDriver
{
public:
    static constexpr int TFMINI_BAUDRATE = 115200;

    TFMiniUartDriver(::tap::Drivers* drivers) : drivers(drivers) {}

    /**
     * Initialize the driver.
     *
     * @warning Call before using this object!
     */
    void initialize()
    {
        // Initialize the uart
        drivers->uart.init<
            TFMINI_UART_PORT,
            TFMINI_BAUDRATE,
            ::tap::communication::serial::Uart::Parity::Disabled>();
    }

    /**
     * Reads a byte from UART buffer if available and parses it.
     *
     * @return `true` iff a valid message was completed by this call and new data is available.
     */
    bool update()
    {
        uint8_t data;
        if (drivers->uart.read(TFMINI_UART_PORT, &data))
        {
            return decoder.processNextByte(data);
        }
        else
        {
            return false;
        }
    }

private:
    /**
     * Decodes messages
     */
    TFMiniDecoder decoder;

    tap::Drivers* const drivers;
};

}  // namespace tap::communication::sensors::distance

#endif  // TAPROOT_TFMINI_HPP_

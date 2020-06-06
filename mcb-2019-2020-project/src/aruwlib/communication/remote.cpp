/**
 * Remote DBUS modm interface
 * A simple API that allows users to easily interact and
 * communicate with the remote dbus communication via
 * UART with modm. This impletation allows for the
 * initialization of the remote and easy access for 
 * reading user inputs.
 */

#include "remote.hpp"
#include "aruwlib/communication/serial/uart.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/Drivers.hpp"
#include "aruwlib/algorithms/math_user_utils.hpp"

using namespace aruwlib::serial;

namespace aruwlib {
    // Enables and initializes Usart1 communication
    void Remote::initialize() {
        Drivers::uart.init<Uart::Uart1, 100000, Uart::Parity::Even>();
    }

    // Reads/parses the current buffer and updates the current remote info states
    void Remote::read() {
        // Check disconnect timeout
        if (aruwlib::arch::clock::getTimeMilliseconds() - lastRead > REMOTE_DISCONNECT_TIMEOUT) {
           connected = false;  // Remote no longer connected
           reset();  // Reset current remote values
        }
        uint8_t data;  // Next byte to be read
        // Read next byte if available and more needed for the current packet
        while (Drivers::uart.read(Uart::UartPort::Uart1, &data)
            && currentBufferIndex < REMOTE_BUF_LEN) {
            rxBuffer[currentBufferIndex] = data;
            currentBufferIndex++;
            lastRead = aruwlib::arch::clock::getTimeMilliseconds();
        }
        // Check read timeout
        if (aruwlib::arch::clock::getTimeMilliseconds() - lastRead > REMOTE_READ_TIMEOUT) {
            clearRxBuffer();
        }
        // Parse buffer if all 18 bytes are read
        if (currentBufferIndex >= REMOTE_BUF_LEN) {
            connected = true;
            parseBuffer();
            clearRxBuffer();
        }
    }

    // Returns if the remote is connected
    bool Remote::isConnected() const {
        return connected;
    }

    // Returns the value of the given channel
    float Remote::getChannel(Channel ch) const {
        switch (ch) {
            case Channel::RIGHT_HORIZONTAL: return remote.rightHorizontal / STICK_MAX_VALUE;
            case Channel::RIGHT_VERTICAL: return remote.rightVertical / STICK_MAX_VALUE;
            case Channel::LEFT_HORIZONTAL: return remote.leftHorizontal / STICK_MAX_VALUE;
            case Channel::LEFT_VERTICAL: return remote.leftVertical / STICK_MAX_VALUE;
        }
        return 0;
    }

    // Returns the value of the given switch
    Remote::SwitchState Remote::getSwitch(Switch sw) const {
        switch (sw) {
            case Switch::LEFT_SWITCH: return remote.leftSwitch;
            case Switch::RIGHT_SWITCH: return remote.rightSwitch;
        }
        return SwitchState::UNKNOWN;
    }

    // Returns the current mouse x value
    int16_t Remote::getMouseX() const {
        return remote.mouse.x;
    }

    // Returns the current mouse y value
    int16_t Remote::getMouseY() const {
        return remote.mouse.y;
    }

    // Returns the current mouse z value
    int16_t Remote::getMouseZ() const {
        return remote.mouse.z;
    }

    // Returns the current mouse l value
    bool Remote::getMouseL() const {
        return remote.mouse.l;
    }

    // Returns the current mouse r value
    bool Remote::getMouseR() const {
        return remote.mouse.r;
    }

    bool Remote::keyPressed(Key key) const {
        return (remote.key & (1 << (uint8_t) key)) != 0;
    }

    // Returns the value of the wheel
    int16_t Remote::getWheel() const {
        return remote.wheel;
    }

    // Parses the current rxBuffer
    void Remote::parseBuffer() {
        // values implemented by shifting bits across based on the dr16
        // values documentation and code created last year
        remote.rightHorizontal = (rxBuffer[0] | rxBuffer[1] << 8) & 0x07FF;
        remote.rightHorizontal -= 1024;
        remote.rightVertical = (rxBuffer[1] >> 3 | rxBuffer[2] << 5) & 0x07FF;
        remote.rightVertical -= 1024;
        remote.leftHorizontal = (rxBuffer[2] >> 6 | rxBuffer[3] << 2 | rxBuffer[4] << 10) & 0x07FF;
        remote.leftHorizontal -= 1024;
        remote.leftVertical = (rxBuffer[4] >> 1 | rxBuffer[5] << 7) & 0x07FF;
        remote.leftVertical -= 1024;
        // the first 6 bytes refer to the remote channel values

        // switches on the dji remote - their input is registered
        switch (((rxBuffer[5] >> 4) & 0x000C) >> 2) {
            case 1:
                remote.leftSwitch = SwitchState::UP;
                break;
            case 3:
                remote.leftSwitch = SwitchState::MID;
                break;
            case 2:
                remote.leftSwitch = SwitchState::DOWN;
                break;
            default:
                remote.leftSwitch = SwitchState::UNKNOWN;
                break;
        }

        switch ((rxBuffer[5] >> 4) & 0x003) {
            case 1:
                remote.rightSwitch = SwitchState::UP;
                break;
            case 3:
                remote.rightSwitch = SwitchState::MID;
                break;
            case 2:
                remote.rightSwitch = SwitchState::DOWN;
                break;
            default:
                remote.rightSwitch = SwitchState::UNKNOWN;
                break;
        }

        // remaining 12 bytes (based on the DBUS_BUF_LEN variable
        // being 18) use mouse and keyboard data
        // 660 is the max value from the remote, so gaining a higher
        // value would be impractical.
        // as such, the method returns null, exiting the method.
        if ((abs(remote.rightHorizontal) > 660) || \
            (abs(remote.rightVertical) > 660) || \
            (abs(remote.leftHorizontal) > 660) || \
            (abs(remote.leftVertical) > 660))
        {
            return;
        }

        // mouse input
        remote.mouse.x = rxBuffer[6] | (rxBuffer[7] << 8);  // x axis
        remote.mouse.y = rxBuffer[8] | (rxBuffer[9] << 8);  // y axis
        remote.mouse.z = rxBuffer[10] | (rxBuffer[11] << 8);  // z axis
        remote.mouse.l = static_cast<bool>(rxBuffer[12]);  // left button click
        remote.mouse.r = static_cast<bool>(rxBuffer[13]);  // right button click

        // keyboard capture
        remote.key = rxBuffer[14] | rxBuffer[15] << 8;
        // Remote wheel
        remote.wheel = (rxBuffer[16] | rxBuffer[17] << 8) - 1024;

        Drivers::ioMapper.handleKeyStateChange(
            remote.key, remote.leftSwitch, remote.rightSwitch);

        remote.updateCounter++;
    }

    // Clears the current rxBuffer
    void Remote::clearRxBuffer() {
        // Reset bytes read counter
        currentBufferIndex = 0;
        // Clear remote rxBuffer
        for (int i = 0; i < REMOTE_BUF_LEN; i++) {
            rxBuffer[i] = 0;
        }
        // Clear Usart1 rxBuffer
        Drivers::uart.discardReceiveBuffer(Uart::UartPort::Uart1);
    }

    // Resets the current remote info
    void Remote::reset() {
        remote.rightHorizontal = 0;
        remote.rightVertical = 0;
        remote.leftHorizontal = 0;
        remote.leftVertical = 0;
        remote.leftSwitch = SwitchState::UNKNOWN;
        remote.rightSwitch = SwitchState::UNKNOWN;
        remote.mouse.x = 0;
        remote.mouse.y = 0;
        remote.mouse.z = 0;
        remote.mouse.l = 0;
        remote.mouse.r = 0;
        remote.key = 0;
        remote.wheel = 0;
        clearRxBuffer();
    }

    uint32_t Remote::getUpdateCounter() const
    {
        return remote.updateCounter;
    }
}  // namespace aruwlib

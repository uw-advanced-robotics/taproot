/**
 * Remote DBUS modm interface
 * A simple API that allows users to easily interact and
 * communicate with the remote dbus communication via
 * UART with modm. This impletation allows for the
 * initialization of the remote and easy access for 
 * reading user inputs.
 */

#include "remote.hpp"

namespace aruwlib {
    // The current remote information
    Remote::RemoteInfo Remote::remote;

    // Remote connection state
    bool Remote::connected = false;

    // uart recieve buffer
    uint8_t Remote::rxBuffer[REMOTE_BUF_LEN];

    // Timestamp when last byte was read
    modm::Timestamp Remote::lastRead;

    // Current count of bytes read
    uint8_t Remote::currentBufferIndex = 0;

    // Enables and initializes Usart1 communication
    void Remote::initialize() {
        Usart1::connect<GpioB7::Rx>();
        Usart1::initialize<Board::SystemClock, 100000>(REMOTE_INT_PRI, Usart1::Parity::Even);
    }

    // Reads/parses the current buffer and updates the current remote info states
    void Remote::read() {
        // Check disconnect timeout
        if (modm::Clock::now().getTime() - lastRead.getTime() > REMOTE_DISCONNECT_TIMEOUT) {
           connected = false;  // Remote no longer connected
           reset();  // Reset current remote values
        }
        uint8_t data;  // Next byte to be read
        // Read next byte if available and more needed for the current packet
        while (Usart1::read(data) && currentBufferIndex < REMOTE_BUF_LEN) {
            rxBuffer[currentBufferIndex] = data;
            currentBufferIndex++;
            lastRead = modm::Clock::now();
        }
        // Check read timeout
        if (modm::Clock::now().getTime() - lastRead.getTime() > REMOTE_READ_TIMEOUT) {
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
    bool Remote::isConnected() {
        return connected;
    }

    // Returns the value of the given channel
    int16_t Remote::getChannel(Channel ch) {
        switch (ch) {
            case Channel::RIGHT_HORIZONTAL: return remote.rightHorizontal;
            case Channel::RIGHT_VERTICAL: return remote.rightVertical;
            case Channel::LEFT_HORIZONTAL: return remote.leftHorizontal;
            case Channel::LEFT_VERTICAL: return remote.leftVertical;
        }
        return 0;
    }

    // Returns the value of the given switch
    Remote::SwitchState Remote::getSwitch(Switch sw) {
        switch (sw) {
            case Switch::LEFT_SWITCH: return remote.leftSwitch;
            case Switch::RIGHT_SWITCH: return remote.rightSwitch;
        }
        return SwitchState::UNKNOWN;
    }

    // Returns the current mouse x value
    int16_t Remote::getMouseX() {
        return remote.mouse.x;
    }

    // Returns the current mouse y value
    int16_t Remote::getMouseY() {
        return remote.mouse.y;
    }

    // Returns the current mouse z value
    int16_t Remote::getMouseZ() {
        return remote.mouse.z;
    }

    // Returns the current mouse l value
    bool Remote::getMouseL() {
        return remote.mouse.l;
    }

    // Returns the current mouse r value
    bool Remote::getMouseR() {
        return remote.mouse.r;
    }

    bool Remote::keyPressed(Key key) {
        return (remote.key & (1 << (uint8_t) key)) != 0;
    }

    // Returns the value of the wheel
    int16_t Remote::getWheel() {
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
        Usart1::discardReceiveBuffer();
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
}  // namespace aruwlib
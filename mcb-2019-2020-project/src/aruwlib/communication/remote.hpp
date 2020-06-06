/**
 * Remote DBUS modm interface
 * A simple API that allows users to easily interact and
 * communicate with the remote dbus communication via
 * UART with modm. This impletation allows for the
 * initialization of the remote and easy access for
 * reading user inputs.
 */

#ifndef __REMOTE_HPP__
#define __REMOTE_HPP__

#include <cstdint>

#ifndef ENV_SIMULATOR
#include <modm/platform.hpp>
#endif

namespace aruwlib
{
class Remote
{
public:
    Remote() = default;
    Remote(const Remote &) = delete;
    Remote &operator=(const Remote &) = default;

    enum class Channel
    {
        RIGHT_HORIZONTAL,
        RIGHT_VERTICAL,
        LEFT_HORIZONTAL,
        LEFT_VERTICAL
    };

    enum class Switch
    {
        LEFT_SWITCH,
        RIGHT_SWITCH
    };

    enum class SwitchState
    {
        UNKNOWN,
        UP,
        MID,
        DOWN
    };

    enum class Key
    {
        W = 0,
        S,
        A,
        D,
        SHIFT,
        CTRL,
        Q,
        E,
        R,
        F,
        G,
        Z,
        X,
        C,
        V,
        B
    };

    // Enables and initializes Usart1 communication
    void initialize();

    // Reads/parses the current buffer and updates the current remote info states
    void read();

    // Returns if the remote is connected
    bool isConnected() const;

    // Returns the value of the given channel
    float getChannel(Channel ch) const;

    // Returns the state of the given switch
    SwitchState getSwitch(Switch sw) const;

    // Returns the current mouse x value
    int16_t getMouseX() const;

    // Returns the current mouse y value
    int16_t getMouseY() const;

    // Returns the current mouse z value
    int16_t getMouseZ() const;

    // Returns the current mouse l value
    bool getMouseL() const;

    // Returns the current mouse r value
    bool getMouseR() const;

    // Returns whether or not the given key is pressed
    bool keyPressed(Key key) const;

    // Returns the value of the wheel
    int16_t getWheel() const;

    uint32_t getUpdateCounter() const;

private:
#define REMOTE_BUF_LEN 18              // Length of the remote recieve buffer
#define REMOTE_READ_TIMEOUT 6          // Timeout delay between valid packets
#define REMOTE_DISCONNECT_TIMEOUT 100  // Timeout delay for remote disconnect
#define REMOTE_INT_PRI 12              // Interrupt priority

    static constexpr float STICK_MAX_VALUE = 660.0f;

    // The current remote information
    struct RemoteInfo
    {
        uint32_t updateCounter = 0;
        int16_t rightHorizontal;
        int16_t rightVertical;
        int16_t leftHorizontal;
        int16_t leftVertical;
        SwitchState leftSwitch;
        SwitchState rightSwitch;
        struct
        {  // Mouse information
            int16_t x;
            int16_t y;
            int16_t z;
            bool l;
            bool r;
        } mouse;
        uint16_t key;   // Keyboard information
        int16_t wheel;  // Remote wheel information
    };

    RemoteInfo remote;

    // Remote connection state
    bool connected = false;

    // uart recieve buffer
    uint8_t rxBuffer[REMOTE_BUF_LEN]{0};

    // Timestamp when last byte was read (milliseconds)
    uint32_t lastRead = 0;

    // Current count of bytes read
    uint8_t currentBufferIndex = 0;

    // Parses the current rxBuffer
    void parseBuffer();

    // Clears the current rxBuffer
    void clearRxBuffer();

    // Resets the current remote info
    void reset();
};

}  // namespace aruwlib

#endif  // __REMOTE_HPP__

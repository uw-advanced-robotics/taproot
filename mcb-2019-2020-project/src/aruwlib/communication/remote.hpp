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

namespace aruwlib {

class Remote {
 public:
    enum class Channel { RIGHT_HORIZONTAL, RIGHT_VERTICAL, LEFT_HORIZONTAL, LEFT_VERTICAL };

    enum class Switch { LEFT_SWITCH, RIGHT_SWITCH };

    enum class SwitchState { UNKNOWN, UP, MID, DOWN };

    enum class Key { W = 0, S, A, D, SHIFT, CTRL, Q, E, R, F, G, Z, X, C, V, B };

    // Enables and initializes Usart1 communication
    static void initialize(void);

    // Reads/parses the current buffer and updates the current remote info states
    static void read(void);

    // Returns if the remote is connected
    static bool isConnected(void);

    // Returns the value of the given channel
    static float getChannel(Channel ch);

    // Returns the state of the given switch
    static SwitchState getSwitch(Switch sw);

    // Returns the current mouse x value
    static int16_t getMouseX(void);

    // Returns the current mouse y value
    static int16_t getMouseY(void);

    // Returns the current mouse z value
    static int16_t getMouseZ(void);

    // Returns the current mouse l value
    static bool getMouseL(void);

    // Returns the current mouse r value
    static bool getMouseR(void);

    // Returns whether or not the given key is pressed
    static bool keyPressed(Key key);

    // Returns the value of the wheel
    static int16_t getWheel(void);

    static uint32_t getUpdateCounter();

 private:
    #define REMOTE_BUF_LEN 18  // Length of the remote recieve buffer
    #define REMOTE_READ_TIMEOUT 6  // Timeout delay between valid packets
    #define REMOTE_DISCONNECT_TIMEOUT 100  // Timeout delay for remote disconnect
    #define REMOTE_INT_PRI 12  // Interrupt priority

    static constexpr float STICK_MAX_VALUE = 660.0f;

    // The current remote information
    static struct RemoteInfo {
        uint32_t updateCounter = 0;
        int16_t rightHorizontal;
        int16_t rightVertical;
        int16_t leftHorizontal;
        int16_t leftVertical;
        SwitchState leftSwitch;
        SwitchState rightSwitch;
        struct {  // Mouse information
            int16_t x;
            int16_t y;
            int16_t z;
            bool l;
            bool r;
        } mouse;
        uint16_t key;  // Keyboard information
        int16_t wheel;  // Remote wheel information
    } remote;

    // Remote connection state
    static bool connected;

    // uart recieve buffer
    static uint8_t rxBuffer[REMOTE_BUF_LEN];

    // Timestamp when last byte was read (milliseconds)
    static uint32_t lastRead;

    // Current count of bytes read
    static uint8_t currentBufferIndex;

    // Parses the current rxBuffer
    static void parseBuffer(void);

    // Clears the current rxBuffer
    static void clearRxBuffer(void);

    // Resets the current remote info
    static void reset(void);
};

}  // namespace aruwlib

#endif  // __REMOTE_HPP__

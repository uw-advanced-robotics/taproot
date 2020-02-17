#ifndef __DJI_MOTOR_HPP__
#define __DJI_MOTOR_HPP__

#include <modm/processing/timer/timeout.hpp>
#include <rm-dev-board-a/board.hpp>
#include "src/aruwlib/communication/can/can_rx_handler.hpp"

namespace aruwlib
{

namespace motor
{
    // for declaring a new motor, must be one of these motor
    // identifiers
    enum MotorId
    {
        MOTOR1 = 0X201,
        MOTOR2 = 0x202,
        MOTOR3 = 0x203,
        MOTOR4 = 0x204,
        MOTOR5 = 0x205,
        MOTOR6 = 0x206,
        MOTOR7 = 0x207,
        MOTOR8 = 0x208,
    };

// extend the CanRxListner class, which allows one to connect a
// motor to the receive handler and use the class's built in
// receive handler
class DjiMotor : public aruwlib::can::CanRxListner
{
 public:
    // 0 - 8191 for dji motors
    static constexpr uint16_t ENC_RESOLUTION = 8191;

    // construct new motor
    DjiMotor(MotorId desMotorIdentifier, aruwlib::can::CanBus motorCanBus, bool isInverted);

    ~DjiMotor();

    // Data structure for storing encoder values. DjiMotor class may call
    // update(), which increments or decrements encoder revolutions and
    // sets the current wrapped encoder value to the updated input.
    class EncoderStore
    {
     public:
        int64_t getEncoderUnwrapped() const;

        uint16_t getEncoderWrapped() const;
     private:
        friend class DjiMotor;

        explicit EncoderStore(
            uint16_t encWrapped = (ENC_RESOLUTION + 1) / 2,
            int64_t encRevolutions = 0
        ) : encoderWrapped(encWrapped),
        encoderRevolutions(encRevolutions)
        {}

        void updateValue(uint16_t newEncWrapped);

        uint16_t encoderWrapped;

        int64_t encoderRevolutions;
    };

    // delete copy constructor
    DjiMotor(const DjiMotor&) = delete;

    // whenever you process a message, this callback is meant to be used by a subclass
    void motorReceiveMessageCallback() {}

    // overrides virtual method in the can class, called every time a message is
    // received by the can receive handler
    void processMessage(const modm::can::Message& message) override
    {
        parseCanRxData(message);
        motorReceiveMessageCallback();
    }

    // Accept a larger value in case someone is stupid and gave something smaller or greater
    // than 2^16, then limit it.
    // Limiting should typically be done on a motor by motor basis in a wrapper class, this
    // is simply a sanity check.
    void setDesiredOutput(int32_t desiredOutput);

    bool isMotorOnline();

    // Serializes send data and deposits it in a message to be sent.
    void serializeCanSendData(modm::can::Message* txMessage) const;

    // getter functions
    int16_t getOutputDesired() const;

    uint32_t getMotorIdentifier() const;

    int8_t getTemperature() const;

    int16_t getTorque() const;

    int16_t getShaftRPM() const;

    int16_t getCurrentActual() const;

    bool isMotorInverted() const;

    aruwlib::can::CanBus getCanBus() const;

    EncoderStore encStore;

    int32_t encw;

 private:
    // wait time before the motor is considered disconnected, in milliseconds
    static const uint32_t MOTOR_DISCONNECT_TIME = 100;

    // Parses receive data given message with the correct identifier.
    void parseCanRxData(const modm::can::Message& message);

    uint32_t motorIdentifier;

    aruwlib::can::CanBus motorCanBus;

    int16_t desiredOutput;

    int16_t currentActual;

    int16_t shaftRPM;

    int8_t temperature;

    int16_t torque;

    bool motorInverted;

    modm::ShortTimeout motorDisconnectTimeout;
};

}  // namespace motor

}  // namespace aruwlib

#endif

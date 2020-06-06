/*
 * This file is part of ARUW's repository.
 *
 * Uses modm can interface to send CAN packets to dji motors. Receiving
 * data is done on the motor level in conjunction with 
 * can_receive_interface.hpp.
 *
 * When a motor is instantiated, a CAN interface and a motor number is
 * requested. This class checks when a motor is created and kills
 * the board if multiple identical motors are created.
 * 
 * To use this class properly, declare a motor somewhere, preferibly
 * in static memory, and then call the initialize method, which allows
 * one to start interacting with a motor connected via CAN bus. When
 * initialize is called, the motor class is added to a list of motors
 * which a handler runs through, sending and receiving messages for all
 * motors.
 * 
 * To receive messages, create a new motor and call the initialize
 * function. This adds the motor to a queue of receive message handlers
 * that is updated by calling the pollCanData function.
 * 
 * To send messages, call processCanSendMessage.
 * 
 * Currently, no error handling is implemented if you attempt to use
 * a motor without initialization. This must be implemented when
 * error handling is complete.
 */

#ifndef __MOTOR_MANAGER_HPP__
#define __MOTOR_MANAGER_HPP__

#include <limits.h>
#include "dji_motor.hpp"

namespace aruwlib
{

namespace motor
{

#define DJI_MOTOR_NORMALIZED_ID(id) ((uint32_t) (id - aruwlib::motor::MotorId::MOTOR1))

class DjiMotorTxHandler
{
 public:
    DjiMotorTxHandler() = default;
    // delete copy constructor.
    DjiMotorTxHandler(const DjiMotorTxHandler&) = delete;
    // We will need this to reset.
    DjiMotorTxHandler &operator=(const DjiMotorTxHandler&) = default;

    // Called when a motor is created, adds to the motor manager
    // and checks to make sure the motor is not already being used.
    void addMotorToManager(DjiMotor* motor);

    void processCanSendData();

    void removeFromMotorManager(const DjiMotor& motor);

    DjiMotor const* getCan1MotorData(MotorId motorId);

    DjiMotor const* getCan2MotorData(MotorId motorId);

    static DjiMotorTxHandler &getMainTxHandler();

 private:
    static const int DJI_MOTORS_PER_CAN = 8;

    DjiMotor* can1MotorStore[DJI_MOTORS_PER_CAN] = {0};
    DjiMotor* can2MotorStore[DJI_MOTORS_PER_CAN] = {0};

    void addMotorToManager(DjiMotor** canMotorStore, DjiMotor*const motor);

    void serializeMotorStoreSendData(
        DjiMotor** canMotorStore,
        modm::can::Message* messageLow,
        modm::can::Message* messageHigh
    );

    void removeFromMotorManager(const DjiMotor& motor, DjiMotor** motorStore);

    void zeroTxMessage(modm::can::Message* message);
};

}  // namespace motor

}  // namespace aruwlib

#endif

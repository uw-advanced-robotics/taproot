#ifndef __AGITATOR_SUBSYSTEM_HPP__
#define __AGITATOR_SUBSYSTEM_HPP__

#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include <modm/math/filter/pid.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"

#include "robot_type.hpp"

namespace aruwsrc
{
namespace agitator
{
class AgitatorSubsystem : public aruwlib::control::Subsystem
{
public:
#if defined(TARGET_SOLDIER) || defined(TARGET_OLD_SOLDIER)
    // position pid terms
    // pid terms for soldier
    static constexpr float PID_17MM_P = 170000.0f;
    static constexpr float PID_17MM_I = 0.0f;
    static constexpr float PID_17MM_D = 80.0f;
    static constexpr float PID_17MM_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_MAX_OUT = 16000.0f;

    static constexpr aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
    static constexpr aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

    static constexpr bool isAgitatorInverted = false;

#elif defined(TARGET_SENTINEL)
    // position pid terms
    // pid terms for sentinel
    static constexpr float PID_17MM_P = 170000.0f;
    static constexpr float PID_17MM_I = 0.0f;
    static constexpr float PID_17MM_D = 80.0f;
    static constexpr float PID_17MM_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_MAX_OUT = 16000.0f;

    static constexpr float PID_17MM_KICKER_P = 170000.0f;
    static constexpr float PID_17MM_KICKER_I = 0.0f;
    static constexpr float PID_17MM_KICKER_D = 80.0f;
    static constexpr float PID_17MM_KICKER_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_KICKER_MAX_OUT = 16000.0f;

    static constexpr aruwlib::motor::MotorId AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
    static constexpr aruwlib::motor::MotorId SENTINEL_KICKER_MOTOR_ID = aruwlib::motor::MOTOR8;
    static constexpr aruwlib::can::CanBus AGITATOR_MOTOR_CAN_BUS = aruwlib::can::CanBus::CAN_BUS1;

#elif defined(TARGET_HERO)
    /// \todo tune all the things
    // pid terms for hero agitator 1
    static constexpr float PID_HERO1_P = 1500.0f;
    static constexpr float PID_HERO1_I = 500.0f;
    static constexpr float PID_HERO1_D = 7000.0f;
    static constexpr float PID_HERO1_MAX_ERR_SUM = 0.0f;

    // pid terms for hero agitator 2
    static constexpr float PID_HERO2_P = 1500.0f;
    static constexpr float PID_HERO2_I = 500.0f;
    static constexpr float PID_HERO2_D = 7000.0f;
    static constexpr float PID_HERO2_MAX_ERR_SUM = 0.0f;

    static constexpr aruwlib::motor::MotorId HERO1_AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR7;
    static constexpr aruwlib::can::CanBus HERO1_AGITATOR_MOTOR_CAN_BUS =
        aruwlib::can::CanBus::CAN_BUS1;

    static constexpr aruwlib::motor::MotorId HERO2_AGITATOR_MOTOR_ID = aruwlib::motor::MOTOR6;
    static constexpr aruwlib::can::CanBus HERO2_AGITATOR_MOTOR_CAN_BUS =
        aruwlib::can::CanBus::CAN_BUS1;
#endif

    // agitator gear ratio, for determining shaft rotation angle
    static constexpr float AGITATOR_GEAR_RATIO_M2006 = 36.0f;
    static constexpr float AGITATOR_GEAR_RATIO_GM3508 = 19.0f;

    explicit AgitatorSubsystem(
        float kp,
        float ki,
        float kd,
        float maxIAccum,
        float maxOutput,
        float agitatorGearRatio,
        aruwlib::motor::MotorId agitatorMotorId,
        aruwlib::can::CanBus agitatorCanBusId,
        bool isAgitatorInverted);

    void refresh() override;

    void setAgitatorDesiredAngle(const float& newAngle);

    float getAgitatorAngle() const;

    float getAgitatorDesiredAngle() const;

    bool agitatorCalibrateHere();

    void armAgitatorUnjamTimer(const uint32_t& predictedRotateTime);

    void disarmAgitatorUnjamTimer();

    bool isAgitatorJammed() const;

    // Returns the velocity of the agitator in units of degrees per second
    float getAgitatorVelocity() const;

    bool isAgitatorCalibrated() const;

private:
    // we add on this amount of "tolerance" to the predicted rotate time since some times it
    // takes longer than predicted and we only want to unjam when we are actually jammed
    // measured in ms
    static const uint32_t JAMMED_TOLERANCE_PERIOD = 150;

    // pid controller for running postiion pid on unwrapped agitator angle (in radians)
    aruwsrc::algorithms::TurretPid agitatorPositionPid;

    aruwlib::motor::DjiMotor agitatorMotor;

    // The user desired angle, measured in radians.
    // The agitator uses unwrapped angle.
    float desiredAgitatorAngle;

    // You can calibrate the agitator, which will set the current agitator angle
    // to zero radians.
    float agitatorCalibratedZeroAngle;

    // Whether or not the agitator has been calibrated yet. You should calibrate the
    // agitator before using it.
    bool agitatorIsCalibrated;

    // A timeout that is used to determine whether or not the agitator is jammed.
    // If the agitator has not reached the desired position in a certain time, the
    // agitator is considered jammed.
    // units: milliseconds
    aruwlib::arch::MilliTimeout agitatorJammedTimeout;

    // the current agitator timeout time, in milliseconds
    uint32_t agitatorJammedTimeoutPeriod;

    // motor gera ratio, so we use shaft angle rather than encoder angle
    float gearRatio;

    void agitatorRunPositionPid();

    float getUncalibratedAgitatorAngle() const;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif

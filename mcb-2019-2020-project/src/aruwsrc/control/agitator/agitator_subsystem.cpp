#include <modm/math/filter/pid.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>
#include <aruwlib/errors/create_errors.hpp>
#include "agitator_subsystem.hpp"
#include "agitator_rotate_command.hpp"

using namespace aruwlib::motor;

namespace aruwsrc
{

namespace agitator
{
    AgitatorSubsystem::AgitatorSubsystem(
        float kp,
        float ki,
        float kd,
        float maxIAccum,
        float maxOutput,
        float agitatorGearRatio,
        aruwlib::motor::MotorId agitatorMotorId,
        aruwlib::can::CanBus agitatorCanBusId,
        bool isAgitatorInverted
    ) :
        agitatorPositionPid(kp, ki, kd, maxIAccum, maxOutput, 1.0f, 0.0f, 1.0f, 0.0f),
        agitatorMotor(agitatorMotorId, agitatorCanBusId, isAgitatorInverted, "agitator motor"),
        desiredAgitatorAngle(0.0f),
        agitatorCalibratedZeroAngle(0.0f),
        agitatorIsCalibrated(false),
        agitatorJammedTimeout(0),
        agitatorJammedTimeoutPeriod(0),
        gearRatio(agitatorGearRatio)
    {
        agitatorJammedTimeout.stop();
    }

    void AgitatorSubsystem::armAgitatorUnjamTimer(const uint32_t& predictedRotateTime)
    {
        if (predictedRotateTime == 0)
        {
            RAISE_ERROR("The predicted rotate time is 0, this is physically impossible",
                    aruwlib::errors::SUBSYSTEM,
                    aruwlib::errors::ZERO_DESIRED_AGITATOR_ROTATE_TIME);
        }
        agitatorJammedTimeoutPeriod = predictedRotateTime + JAMMED_TOLERANCE_PERIOD;
        agitatorJammedTimeout.restart(agitatorJammedTimeoutPeriod);
    }

    void AgitatorSubsystem::disarmAgitatorUnjamTimer()
    {
        agitatorJammedTimeout.stop();
    }

    bool AgitatorSubsystem::isAgitatorJammed() const
    {
        return agitatorJammedTimeout.isExpired();
    }

    void AgitatorSubsystem::refresh()
    {
        if (agitatorIsCalibrated)
        {
            agitatorRunPositionPid();
        }
        else
        {
            agitatorCalibrateHere();
        }
    }

    void AgitatorSubsystem::agitatorRunPositionPid()
    {
        if (!agitatorIsCalibrated)
        {
            agitatorPositionPid.reset();
        }
        else
        {
            agitatorPositionPid.runController(desiredAgitatorAngle - getAgitatorAngle(),
                    getAgitatorVelocity());
            agitatorMotor.setDesiredOutput(agitatorPositionPid.getOutput());
        }
    }

    bool AgitatorSubsystem::agitatorCalibrateHere()
    {
        if (!agitatorMotor.isMotorOnline())
        {
            return false;
        }
        agitatorCalibratedZeroAngle = getUncalibratedAgitatorAngle();
        agitatorIsCalibrated = true;
        return true;
    }

    float AgitatorSubsystem::getAgitatorAngle() const
    {
        if (!agitatorIsCalibrated)
        {
            return 0.0f;
        }
        return getUncalibratedAgitatorAngle() - agitatorCalibratedZeroAngle;
    }

    float AgitatorSubsystem::getUncalibratedAgitatorAngle() const
    {
        // position is equal to the following equation:
        // position = 2 * PI / encoder resolution * unwrapped encoder value / gear ratio
        return (2.0f * aruwlib::algorithms::PI / static_cast<float>(DjiMotor::ENC_RESOLUTION)) *
            agitatorMotor.encStore.getEncoderUnwrapped() / gearRatio;
    }

    void AgitatorSubsystem::setAgitatorDesiredAngle(const float& newAngle)
    {
        desiredAgitatorAngle = newAngle;
    }

    float AgitatorSubsystem::getAgitatorDesiredAngle() const
    {
        return desiredAgitatorAngle;
    }

    float AgitatorSubsystem::getAgitatorVelocity() const
    {
        return 6.0f * static_cast<float>(agitatorMotor.getShaftRPM()) / gearRatio;
    }

    bool AgitatorSubsystem::isAgitatorCalibrated() const
    {
        return agitatorIsCalibrated;
    }

}  // namespace agitator

}  // namespace aruwsrc

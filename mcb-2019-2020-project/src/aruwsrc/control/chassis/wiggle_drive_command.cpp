#include "wiggle_drive_command.hpp"

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/clock.hpp>
#include <aruwlib/communication/remote.hpp>
#include <aruwlib/communication/sensors/mpu6500/mpu6500.hpp>

using namespace aruwlib::algorithms;
using namespace aruwlib::sensors;
using aruwlib::Drivers;

namespace aruwsrc
{
namespace chassis
{
void WiggleDriveCommand::initialize()
{
    float turretCurAngle = turret->getYawAngleFromCenter();

    // We don't apply the sine wave if we are out of the angle of the sine wave.
    outOfCenter = fabsf(turretCurAngle) > WIGGLE_MAX_ROTATE_ANGLE;

    turretCurAngle = aruwlib::algorithms::limitVal(
        turretCurAngle,
        -WIGGLE_MAX_ROTATE_ANGLE,
        WIGGLE_MAX_ROTATE_ANGLE);

    // We use the limited current turret angle to calculate a time offset in
    // a angle vs. time graph so when we start at some angle from center we
    // are still in phase.
    startTimeForAngleOffset = asinf(turretCurAngle / WIGGLE_MAX_ROTATE_ANGLE) * WIGGLE_PERIOD /
                              (2.0f * aruwlib::algorithms::PI);

    // The offset so when we start calculating a rotation angle, the initial
    // time is zero.
    timeOffset = aruwlib::arch::clock::getTimeMilliseconds();
}

float WiggleDriveCommand::wiggleSin(float t)
{
    return WIGGLE_MAX_ROTATE_ANGLE * sinf((2.0f * aruwlib::algorithms::PI / WIGGLE_PERIOD) * t);
}

void WiggleDriveCommand::execute()
{
    float r;
    float x = Drivers::controlOperatorInterface.getChassisXInput() *
              ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    float y = Drivers::controlOperatorInterface.getChassisYInput() *
              ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

    // We only wiggle when the turret is online.
    if (turret->isTurretOnline())
    {
        float curTime =
            static_cast<float>(aruwlib::arch::clock::getTimeMilliseconds() - timeOffset) -
            startTimeForAngleOffset;
        float desiredAngleError;
        float turretYawAngle = turret->getYawAngleFromCenter();

        if (outOfCenter)
        {
            outOfCenter = fabsf(turretYawAngle) > WIGGLE_MAX_ROTATE_ANGLE;
            if (!outOfCenter)
            {
                initialize();
            }
        }

        if (!outOfCenter)
        {
            desiredAngleError = wiggleSin(curTime) + turretYawAngle;
        }
        else
        {
            desiredAngleError = aruwlib::algorithms::limitVal(
                turretYawAngle,
                -WIGGLE_OUT_OF_CENTER_MAX_ROTATE_ERR,
                WIGGLE_OUT_OF_CENTER_MAX_ROTATE_ERR);
        }

        // Wrapping between -180 and 180.
        ContiguousFloat rotationError(desiredAngleError, -180.0f, 180.0f);
        r = chassis->chassisSpeedRotationPID(rotationError.getValue(), WIGGLE_ROTATE_KP);
        x *= TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING;
        y *= TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING;
        // Apply a rotation matrix to the user input so you drive turret
        // relative while wiggling.
        aruwlib::algorithms::rotateVector(&x, &y, -degreesToRadians(turretYawAngle));
    }
    else
    {
        r = Drivers::controlOperatorInterface.getChassisRInput() *
            ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    }

    chassis->setDesiredOutput(x, y, r);
}

void WiggleDriveCommand::end(bool) { chassis->setDesiredOutput(0.0f, 0.0f, 0.0f); }

bool WiggleDriveCommand::isFinished() const { return false; }

}  // namespace chassis

}  // namespace aruwsrc

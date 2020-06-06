#include "friction_wheel_subsystem.hpp"

#include <aruwlib/architecture/clock.hpp>

namespace aruwsrc
{
namespace launcher
{
void FrictionWheelSubsystem::setDesiredRpm(float desRpm) { desiredRpmRamp.setTarget(desRpm); }

void FrictionWheelSubsystem::refresh()
{
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    desiredRpmRamp.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    prevTime = currTime;

    velocityPidLeftWheel.update(desiredRpmRamp.getValue() - leftWheel.getShaftRPM());
    leftWheel.setDesiredOutput(static_cast<int32_t>(velocityPidLeftWheel.getValue()));
    velocityPidRightWheel.update(desiredRpmRamp.getValue() - rightWheel.getShaftRPM());
    rightWheel.setDesiredOutput(static_cast<int32_t>(velocityPidRightWheel.getValue()));
}
}  // namespace launcher

}  // namespace aruwsrc

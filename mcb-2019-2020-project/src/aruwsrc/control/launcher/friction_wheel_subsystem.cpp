#include "friction_wheel_subsystem.hpp"

namespace aruwsrc
{

namespace launcher
{
    void FrictionWheelSubsystem::setDesiredRpm(float desRpm)
    {
        desiredRpmRamp.setTarget(desRpm);
    }

    void FrictionWheelSubsystem::refresh()
    {
        uint32_t currTime = modm::Clock::now().getTime();
        desiredRpmRamp.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
        prevTime = modm::Clock::now().getTime();

        velocityPidLeftWheel.update(desiredRpmRamp.getValue() - leftWheel.getShaftRPM());
        leftWheel.setDesiredOutput(static_cast<int32_t>(velocityPidLeftWheel.getValue()));
        velocityPidRightWheel.update(desiredRpmRamp.getValue() - rightWheel.getShaftRPM());
        rightWheel.setDesiredOutput(static_cast<int32_t>(velocityPidRightWheel.getValue()));
    }
}  // namespace launcher

}  // namespace aruwsrc

#include "control_operator_interface.hpp"
#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/communication/remote.hpp"

using namespace aruwlib;
using namespace aruwlib::algorithms;

namespace aruwlib
{

namespace control
{
    uint32_t ControlOperatorInterface::prevUpdateCounterX = 0;
    uint32_t ControlOperatorInterface::prevUpdateCounterY = 0;
    uint32_t ControlOperatorInterface::prevUpdateCounterZ = 0;

    aruwlib::algorithms::LinearInterpolation ControlOperatorInterface::chassisXInput;
    aruwlib::algorithms::LinearInterpolation ControlOperatorInterface::chassisYInput;
    aruwlib::algorithms::LinearInterpolation ControlOperatorInterface::chassisRInput;

    float ControlOperatorInterface::getChassisXInput()
    {
        if (prevUpdateCounterX != Remote::getUpdateCounter()) {
            chassisXInput.update(Remote::getChannel(Remote::Channel::LEFT_VERTICAL));
        }
        prevUpdateCounterX = Remote::getUpdateCounter();
        return aruwlib::algorithms::limitVal<float>(
                chassisXInput.getInterpolatedValue(modm::Clock::now().getTime())
                + static_cast<float>(Remote::keyPressed(Remote::Key::W))
                - static_cast<float>(Remote::keyPressed(Remote::Key::S)),
                -1.0f, 1.0f);
    }

    float ControlOperatorInterface::getChassisYInput()
    {
        if (prevUpdateCounterY != Remote::getUpdateCounter()) {
            chassisYInput.update(Remote::getChannel(Remote::Channel::LEFT_HORIZONTAL));
        }
        prevUpdateCounterY = Remote::getUpdateCounter();
        return aruwlib::algorithms::limitVal<float>(
                chassisYInput.getInterpolatedValue(modm::Clock::now().getTime())
                + static_cast<float>(Remote::keyPressed(Remote::Key::A))
                - static_cast<float>(Remote::keyPressed(Remote::Key::D)),
                -1.0f, 1.0f);
    }

    float ControlOperatorInterface::getChassisRInput()
    {
        if (prevUpdateCounterZ != Remote::getUpdateCounter()) {
            chassisRInput.update(Remote::getChannel(Remote::Channel::RIGHT_HORIZONTAL));
        }
        prevUpdateCounterZ = Remote::getUpdateCounter();
        return aruwlib::algorithms::limitVal<float>(
                chassisRInput.getInterpolatedValue(modm::Clock::now().getTime())
                + static_cast<float>(Remote::keyPressed(Remote::Key::Q))
                - static_cast<float>(Remote::keyPressed(Remote::Key::E)),
                -1.0f, 1.0f);
    }

    float ControlOperatorInterface::getTurretYawInput()
    {
        return -static_cast<float>(Remote::getChannel(Remote::Channel::RIGHT_HORIZONTAL))
                + static_cast<float>(Remote::getMouseX()) * USER_MOUSE_YAW_SCALAR;
    }

    float ControlOperatorInterface::getTurretPitchInput()
    {
        return static_cast<float>(Remote::getChannel(Remote::Channel::RIGHT_VERTICAL))
                + static_cast<float>(Remote::getMouseY()) * USER_MOUSE_PITCH_SCALAR;
    }

    float ControlOperatorInterface::getSentinelSpeedInput()
    {
        return aruwlib::Remote::getChannel(aruwlib::Remote::Channel::LEFT_HORIZONTAL)
                * USER_STICK_SENTINEL_DRIVE_SCALAR;
    }
}  // namespace control

}  // namespace aruwlib

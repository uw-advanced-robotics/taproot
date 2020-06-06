#include "control_operator_interface.hpp"
#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/Drivers.hpp"

using namespace aruwlib;
using namespace aruwlib::algorithms;

namespace aruwlib
{

namespace control
{
    float ControlOperatorInterface::getChassisXInput()
    {
        if (prevUpdateCounterX != Drivers::remote.getUpdateCounter()) {
            chassisXInput.update(
                Drivers::remote.getChannel(Remote::Channel::LEFT_VERTICAL));
        }
        prevUpdateCounterX = Drivers::remote.getUpdateCounter();
        return aruwlib::algorithms::limitVal<float>(
                chassisXInput.getInterpolatedValue(aruwlib::arch::clock::getTimeMilliseconds())
                + static_cast<float>(Drivers::remote.keyPressed(Remote::Key::W))
                - static_cast<float>(Drivers::remote.keyPressed(Remote::Key::S)),
                -1.0f, 1.0f);
    }

    float ControlOperatorInterface::getChassisYInput()
    {
        if (prevUpdateCounterY != Drivers::remote.getUpdateCounter()) {
            chassisYInput.update(
                Drivers::remote.getChannel(Remote::Channel::LEFT_HORIZONTAL));
        }
        prevUpdateCounterY = Drivers::remote.getUpdateCounter();
        return aruwlib::algorithms::limitVal<float>(
                chassisYInput.getInterpolatedValue(aruwlib::arch::clock::getTimeMilliseconds())
                + static_cast<float>(Drivers::remote.keyPressed(Remote::Key::A))
                - static_cast<float>(Drivers::remote.keyPressed(Remote::Key::D)),
                -1.0f, 1.0f);
    }

    float ControlOperatorInterface::getChassisRInput()
    {
        if (prevUpdateCounterZ != Drivers::remote.getUpdateCounter()) {
            chassisRInput.update(
                Drivers::remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL));
        }
        prevUpdateCounterZ = Drivers::remote.getUpdateCounter();
        return aruwlib::algorithms::limitVal<float>(
                chassisRInput.getInterpolatedValue(aruwlib::arch::clock::getTimeMilliseconds())
                + static_cast<float>(Drivers::remote.keyPressed(Remote::Key::Q))
                - static_cast<float>(Drivers::remote.keyPressed(Remote::Key::E)),
                -1.0f, 1.0f);
    }

    float ControlOperatorInterface::getTurretYawInput()
    {
        return -static_cast<float>(
            Drivers::remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL))
                + static_cast<float>(Drivers::remote.getMouseX())
                * USER_MOUSE_YAW_SCALAR;
    }

    float ControlOperatorInterface::getTurretPitchInput()
    {
        return static_cast<float>(
            Drivers::remote.getChannel(Remote::Channel::RIGHT_VERTICAL))
                + static_cast<float>(Drivers::remote.getMouseY())
                * USER_MOUSE_PITCH_SCALAR;
    }

    float ControlOperatorInterface::getSentinelSpeedInput()
    {
        return aruwlib::Drivers::remote.getChannel(
            aruwlib::Remote::Channel::LEFT_HORIZONTAL)
                * USER_STICK_SENTINEL_DRIVE_SCALAR;
    }
}  // namespace control

}  // namespace aruwlib

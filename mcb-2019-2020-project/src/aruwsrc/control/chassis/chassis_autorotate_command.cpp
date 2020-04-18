#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/communication/remote.hpp>
#include <aruwlib/control/control_operator_interface.hpp>
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "chassis_autorotate_command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace chassis
{

void ChassisAutorotateCommand::initialize()
{}

void ChassisAutorotateCommand::execute()
{
    // calculate pid for chassis rotation
    // returns a chassis rotation speed
    float chassisRotationDesiredWheelspeed = chassis->chassisSpeedRotationPID(
            turret->getYawAngleFromCenter(), CHASSIS_AUTOROTATE_PID_KP);

    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain
            = chassis->calculateRotationTranslationalGain(chassisRotationDesiredWheelspeed);

    float chassisXDesiredWheelspeed =
            aruwlib::algorithms::limitVal<float>(
                    ControlOperatorInterface::getChassisXInput(),
                    -rTranslationalGain, rTranslationalGain)
            * ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

    float chassisYDesiredWheelspeed =
            aruwlib::algorithms::limitVal<float>(
                    ControlOperatorInterface::getChassisYInput(),
                    -rTranslationalGain, rTranslationalGain)
            * ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

    chassis->setDesiredOutput(chassisXDesiredWheelspeed,
        chassisYDesiredWheelspeed, chassisRotationDesiredWheelspeed);
}

// NOLINTNEXTLINE
void ChassisAutorotateCommand::end(bool)
{}

bool ChassisAutorotateCommand::isFinished() const
{
    return false;
}

}  // namespace chassis

}  // namespace aruwsrc

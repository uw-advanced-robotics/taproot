#include "chassis_drive_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"

namespace aruwsrc
{

namespace chassis
{

void ChassisDriveCommand::initialize()
{}

void ChassisDriveCommand::execute()
{
    float chassisRotationDesiredWheelspeed = ChassisSubsystem::getChassisR()
        * ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain
        = chassis->calculateRotationTranslationalGain(chassisRotationDesiredWheelspeed);

    float chassisXDesiredWheelspeed =
        aruwlib::algorithms::limitVal<float>(ChassisSubsystem::getChassisX(),
        -rTranslationalGain, rTranslationalGain)
        * ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    float chassisYDesiredWheelspeed =
        aruwlib::algorithms::limitVal<float>(ChassisSubsystem::getChassisY(),
        -rTranslationalGain, rTranslationalGain)
        * ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

    chassis->setDesiredOutput(chassisXDesiredWheelspeed,
        chassisYDesiredWheelspeed, chassisRotationDesiredWheelspeed);
}

void ChassisDriveCommand::end(bool interrupted)
{
    if (interrupted)
    {
        chassis->setDesiredOutput(0.0f, 0.0f, 0.0f);
    }
    chassis->setDesiredOutput(0.0f, 0.0f, 0.0f);
}

bool ChassisDriveCommand::isFinished() const
{
    return false;
}

}  // namespace chassis

}  // namespace aruwsrc
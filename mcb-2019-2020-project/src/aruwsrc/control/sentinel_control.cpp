#include "robot-type/robot_type.hpp"
#include "src/aruwlib/control/controller_mapper.hpp"
#include "agitator/agitator_subsystem.hpp"
#include "agitator/agitator_calibrate_command.hpp"
#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_init_command.hpp"
#include "turret/turret_manual_command.hpp"
#include "turret/turret_world_relative_position_command.hpp"
#include "sentinel/sentinel_auto_drive_command.hpp"
#include "sentinel/sentinel_drive_manual_command.hpp"
#include "sentinel/sentinel_drive_subsystem.hpp"
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"

#if defined(TARGET_SENTINEL)

using namespace aruwsrc::agitator;
using namespace aruwsrc::launcher;

namespace aruwsrc
{

namespace control
{

/* define subsystems --------------------------------------------------------*/
AgitatorSubsystem agitator(AgitatorSubsystem::PID_17MM_P,
                           AgitatorSubsystem::PID_17MM_I,
                           AgitatorSubsystem::PID_17MM_D,
                           AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
                           AgitatorSubsystem::PID_17MM_MAX_OUT,
                           AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
                           AgitatorSubsystem::AGITATOR_MOTOR_ID,
                           AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
                           false);

AgitatorSubsystem kickerMotor(AgitatorSubsystem::PID_17MM_KICKER_P,
                              AgitatorSubsystem::PID_17MM_KICKER_I,
                              AgitatorSubsystem::PID_17MM_KICKER_D,
                              AgitatorSubsystem::PID_17MM_KICKER_MAX_ERR_SUM,
                              AgitatorSubsystem::PID_17MM_KICKER_MAX_OUT,
                              AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
                              AgitatorSubsystem::SENTINEL_KICKER_MOTOR_ID,
                              AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
                              false);

aruwsrc::control::SentinelDriveSubsystem sentinelDrive;

FrictionWheelSubsystem upperFrictionWheels(aruwlib::motor::MOTOR3, aruwlib::motor::MOTOR4);

FrictionWheelSubsystem lowerFrictionWheels;

/* define commands ----------------------------------------------------------*/
ShootFastComprisedCommand agitatorShootSlowCommand(&agitator);

AgitatorCalibrateCommand agitatorCalibrateCommand(&agitator);

AgitatorRotateCommand agitatorKickerCommand(&kickerMotor, 3.0f, 1, 0, false);

AgitatorCalibrateCommand agitatorCalibrateKickerCommand(&kickerMotor);

SentinelAutoDriveCommand sentinelAutoDrive(&sentinelDrive);

SentinelDriveManualCommand sentinelDriveManual(&sentinelDrive);

FrictionWheelRotateCommand spinUpperFrictionWheels(&upperFrictionWheels,
        FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand spinLowerFrictionWheels(&lowerFrictionWheels,
        FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand stopUpperFrictionWheels(&upperFrictionWheels, 0);

FrictionWheelRotateCommand stopLowerFrictionWheels(&lowerFrictionWheels, 0);

/* register subsystems here -------------------------------------------------*/
void registerSentinelSubsystems()
{
    CommandScheduler::getMainScheduler().registerSubsystem(&agitator);
    CommandScheduler::getMainScheduler().registerSubsystem(&kickerMotor);
    CommandScheduler::getMainScheduler().registerSubsystem(&sentinelDrive);
    sentinelDrive.initLimitSwitches();
    CommandScheduler::getMainScheduler().registerSubsystem(&upperFrictionWheels);
    CommandScheduler::getMainScheduler().registerSubsystem(&lowerFrictionWheels);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentinelCommands()
{
    sentinelDrive.setDefaultCommand(&sentinelDriveManual);
    upperFrictionWheels.setDefaultCommand(&spinUpperFrictionWheels);
    lowerFrictionWheels.setDefaultCommand(&spinLowerFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentinelCommands()
{
    CommandScheduler::getMainScheduler().addCommand(&agitatorCalibrateCommand);
    CommandScheduler::getMainScheduler().addCommand(&agitatorCalibrateKickerCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSentinelIoMappings()
{
    IoMapper::addHoldRepeatMapping(
            IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
            &agitatorShootSlowCommand);

    IoMapper::addHoldRepeatMapping(
            IoMapper::newKeyMap(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
            &agitatorKickerCommand);

    IoMapper::addHoldRepeatMapping(
            IoMapper::newKeyMap(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN),
            &sentinelAutoDrive);

    IoMapper::addHoldMapping(
            IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
            &stopLowerFrictionWheels);

    IoMapper::addHoldMapping(
            IoMapper::newKeyMap(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN),
            &stopUpperFrictionWheels);
}

void initSubsystemCommands()
{
    registerSentinelSubsystems();
    setDefaultSentinelCommands();
    startSentinelCommands();
    registerSentinelIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif

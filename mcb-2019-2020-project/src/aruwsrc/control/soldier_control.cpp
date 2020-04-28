#include <aruwlib/control/controller_mapper.hpp>
#include "robot_type.hpp"
#include "agitator/agitator_subsystem.hpp"
#include "agitator/agitator_calibrate_command.hpp"
#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/wiggle_drive_command.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_world_relative_position_command.hpp"
#include "hopper-cover/hopper_subsystem.hpp"
#include "hopper-cover/open_hopper_command.hpp"
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"

#if defined(TARGET_SOLDIER)

using namespace aruwsrc::agitator;
using namespace aruwsrc::chassis;
using namespace aruwsrc::launcher;
using namespace aruwsrc::turret;

namespace aruwsrc
{

namespace control
{

/* define subsystems --------------------------------------------------------*/
TurretSubsystem turret;

ChassisSubsystem chassis;

AgitatorSubsystem agitator(AgitatorSubsystem::PID_17MM_P,
                           AgitatorSubsystem::PID_17MM_I,
                           AgitatorSubsystem::PID_17MM_D,
                           AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
                           AgitatorSubsystem::PID_17MM_MAX_OUT,
                           AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
                           AgitatorSubsystem::AGITATOR_MOTOR_ID,
                           AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
                           AgitatorSubsystem::isAgitatorInverted);

HopperSubsystem hopperCover(aruwlib::gpio::Pwm::W,
                            HopperSubsystem::SOLDIER_HOPPER_OPEN_PWM,
                            HopperSubsystem::SOLDIER_HOPPER_CLOSE_PWM,
                            HopperSubsystem::SOLDIER_PWM_RAMP_SPEED);

FrictionWheelSubsystem frictionWheels;

/* define commands ----------------------------------------------------------*/
ChassisDriveCommand chassisDriveCommand(&chassis);

ChassisAutorotateCommand chassisAutorotateCommand(&chassis, &turret);

WiggleDriveCommand wiggleDriveCommand(&chassis, &turret);

TurretWorldRelativePositionCommand turretWorldRelativeCommand(&turret, &chassis);

AgitatorCalibrateCommand agitatorCalibrateCommand(&agitator);

ShootFastComprisedCommand agitatorShootFastCommand(&agitator);

OpenHopperCommand openHopperCommand(&hopperCover);

FrictionWheelRotateCommand spinFrictionWheels(&frictionWheels,
                                              FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand stopFrictionWheels(&frictionWheels, 0);

/// \todo add cv turret

/* register subsystems here -------------------------------------------------*/
void registerSoldierSubsystems()
{
    CommandScheduler::getMainScheduler().registerSubsystem(&agitator);
    CommandScheduler::getMainScheduler().registerSubsystem(&chassis);
    CommandScheduler::getMainScheduler().registerSubsystem(&turret);
    CommandScheduler::getMainScheduler().registerSubsystem(&hopperCover);
    CommandScheduler::getMainScheduler().registerSubsystem(&frictionWheels);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSoldierCommands()
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSoldierCommands()
{
    CommandScheduler::getMainScheduler().addCommand(&agitatorCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSoldierIoMappings()
{
    IoMapper::addHoldRepeatMapping(
            IoMapper::newKeyMap(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
            &agitatorShootFastCommand);

    IoMapper::addHoldRepeatMapping(
            IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID),
            &chassisAutorotateCommand);

    IoMapper::addHoldMapping(
            IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
            &chassisDriveCommand);

    IoMapper::addHoldMapping(
            IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
            &openHopperCommand);

    IoMapper::addHoldMapping(
            IoMapper::newKeyMap(Remote::SwitchState::DOWN, Remote::SwitchState::DOWN),
            &stopFrictionWheels);

    IoMapper::addHoldMapping(
            IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
            &wiggleDriveCommand);

    /// \todo left switch up is cv command
}

void initSubsystemCommands()
{
    registerSoldierSubsystems();
    setDefaultSoldierCommands();
    startSoldierCommands();
    registerSoldierIoMappings();
}


}  // namespace control

}  // namespace aruwsrc

#endif

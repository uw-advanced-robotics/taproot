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

#if defined(TARGET_OLD_SOLDIER)

using namespace aruwsrc::agitator;
using namespace aruwsrc::chassis;
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
                            HopperSubsystem::OLD_SOLDIER_HOPPER_OPEN_PWM,
                            HopperSubsystem::OLD_SOLDIER_HOPPER_CLOSE_PWM,
                            HopperSubsystem::OLD_SOLDIER_PWM_RAMP_SPEED);

/* define commands ----------------------------------------------------------*/
ChassisDriveCommand chassisDriveCommand(&chassis);

ChassisAutorotateCommand chassisAutorotateCommand(&chassis, &turret);

WiggleDriveCommand wiggleDriveCommand(&chassis, &turret);

TurretWorldRelativePositionCommand turretWorldRelativeCommand(&turret, &chassis);

AgitatorCalibrateCommand agitatorCalibrateCommand(&agitator);

ShootFastComprisedCommand agitatorShootFastCommand(&agitator);

OpenHopperCommand openHopperCommand(&hopperCover);

/* register subsystems here -------------------------------------------------*/
void registerOldSoldierSubsystems()
{
    CommandScheduler::getMainScheduler().registerSubsystem(&agitator);
    CommandScheduler::getMainScheduler().registerSubsystem(&chassis);
    CommandScheduler::getMainScheduler().registerSubsystem(&turret);
    CommandScheduler::getMainScheduler().registerSubsystem(&hopperCover);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultOldSoldierCommands()
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startOldSoldierCommands()
{
    CommandScheduler::getMainScheduler().addCommand(&agitatorCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerOldSoldierIoMappings()
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
            IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
            &wiggleDriveCommand);
}

void initSubsystemCommands()
{
    registerOldSoldierSubsystems();
    setDefaultOldSoldierCommands();
    startOldSoldierCommands();
    registerOldSoldierIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif

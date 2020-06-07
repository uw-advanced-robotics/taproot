#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/command_mapper.hpp>

#include "agitator/agitator_calibrate_command.hpp"
#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "agitator/agitator_subsystem.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "chassis/wiggle_drive_command.hpp"
#include "hopper-cover/hopper_subsystem.hpp"
#include "hopper-cover/open_hopper_command.hpp"
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_world_relative_position_command.hpp"

#include "robot_type.hpp"

#if defined(TARGET_SOLDIER)

using namespace aruwsrc::agitator;
using namespace aruwsrc::chassis;
using namespace aruwsrc::launcher;
using namespace aruwsrc::turret;
using aruwlib::Drivers;
using aruwlib::control::CommandMapper;

namespace aruwsrc
{
namespace control
{
/* define subsystems --------------------------------------------------------*/
TurretSubsystem turret;

ChassisSubsystem chassis;

AgitatorSubsystem agitator(
    AgitatorSubsystem::PID_17MM_P,
    AgitatorSubsystem::PID_17MM_I,
    AgitatorSubsystem::PID_17MM_D,
    AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::AGITATOR_MOTOR_ID,
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    AgitatorSubsystem::isAgitatorInverted);

HopperSubsystem hopperCover(
    aruwlib::gpio::Pwm::W,
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

FrictionWheelRotateCommand spinFrictionWheels(
    &frictionWheels,
    FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand stopFrictionWheels(&frictionWheels, 0);

/// \todo add cv turret

/* register subsystems here -------------------------------------------------*/
void registerSoldierSubsystems()
{
    Drivers::commandScheduler.registerSubsystem(&agitator);
    Drivers::commandScheduler.registerSubsystem(&chassis);
    Drivers::commandScheduler.registerSubsystem(&turret);
    Drivers::commandScheduler.registerSubsystem(&hopperCover);
    Drivers::commandScheduler.registerSubsystem(&frictionWheels);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSoldierCommands()
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSoldierCommands() { Drivers::commandScheduler.addCommand(&agitatorCalibrateCommand); }

/* register io mappings here ------------------------------------------------*/
void registerSoldierIoMappings()
{
    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
        &agitatorShootFastCommand);

    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID),
        &chassisAutorotateCommand);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
        &chassisDriveCommand);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
        &openHopperCommand);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper::newKeyMap(Remote::SwitchState::DOWN, Remote::SwitchState::DOWN),
        &stopFrictionWheels);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
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

/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#if defined(TARGET_HERO)

#include "aruwlib/control/command_mapper.hpp"
#include "aruwlib/control/hold_command_mapping.hpp"
#include "aruwlib/control/hold_repeat_command_mapping.hpp"
#include "aruwlib/control/press_command_mapping.hpp"
#include "aruwlib/control/setpoint/commands/calibrate_command.hpp"
#include "aruwlib/control/setpoint/commands/move_absolute_command.hpp"
#include "aruwlib/control/setpoint/commands/move_command.hpp"
#include "aruwlib/control/setpoint/commands/move_unjam_comprised_command.hpp"
#include "aruwlib/control/toggle_command_mapping.hpp"
#include "aruwlib/drivers_singleton.hpp"

#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "agitator/double_agitator_subsystem.hpp"
#include "agitator/limit_switch_agitator_subsystem.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "chassis/wiggle_drive_command.hpp"
#include "client-display/client_display_command.hpp"
#include "client-display/client_display_subsystem.hpp"
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_world_relative_position_command.hpp"

using aruwlib::DoNotUse_getDrivers;
using namespace aruwlib::control::setpoint;
using namespace aruwsrc::agitator;
using namespace aruwsrc::chassis;
using namespace aruwsrc::launcher;
using namespace aruwsrc::control::turret;
using namespace aruwlib::control;
using namespace aruwsrc::display;
using aruwlib::DoNotUse_getDrivers;
using aruwlib::Remote;
using aruwlib::control::CommandMapper;
using aruwlib::control::RemoteMapState;
/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwlib::driversFunc drivers = aruwlib::DoNotUse_getDrivers;

namespace hero_control
{
/* define subsystems --------------------------------------------------------*/
TurretSubsystem turret(drivers());

ChassisSubsystem chassis(drivers());

// Hero has two agitators, one waterWheel and then a kicker
LimitSwitchAgitatorSubsystem waterWheelAgitator(
    drivers(),
    AgitatorSubsystem::PID_HERO_WATERWHEEL_P,
    AgitatorSubsystem::PID_HERO_WATERWHEEL_I,
    AgitatorSubsystem::PID_HERO_WATERWHEEL_D,
    AgitatorSubsystem::PID_HERO_WATERWHEEL_MAX_ERR_SUM,
    AgitatorSubsystem::PID_HERO_WATERWHEEL_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::HERO_WATERWHEEL_MOTOR_ID,
    AgitatorSubsystem::HERO_WATERWHEEL_MOTOR_CAN_BUS,
    AgitatorSubsystem::HERO_WATERWHEEL_INVERTED,
    AgitatorSubsystem::JAM_DISTANCE_TOLERANCE_WATERWHEEL,
    AgitatorSubsystem::JAM_TEMPORAL_TOLERANCE_WATERWHEEL,
    LimitSwitchAgitatorSubsystem::WATERWHEEL_LIMIT_PIN);

DoubleAgitatorSubsystem kickerSubsystem(
    drivers(),
    AgitatorSubsystem::PID_HERO_KICKER_P,
    AgitatorSubsystem::PID_HERO_KICKER_I,
    AgitatorSubsystem::PID_HERO_KICKER_D,
    AgitatorSubsystem::PID_HERO_KICKER_MAX_ERR_SUM,
    AgitatorSubsystem::PID_HERO_KICKER_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::HERO_KICKER1_MOTOR_ID,
    AgitatorSubsystem::HERO_KICKER1_MOTOR_CAN_BUS,
    AgitatorSubsystem::HERO_KICKER2_MOTOR_ID,
    AgitatorSubsystem::HERO_KICKER2_MOTOR_CAN_BUS,
    AgitatorSubsystem::HERO_KICKER_INVERTED,
    DoubleAgitatorSubsystem::JAM_DISTANCE_TOLERANCE,
    DoubleAgitatorSubsystem::JAM_TEMPORAL_TOLERANCE,
    false);

FrictionWheelSubsystem frictionWheels(drivers());

ClientDisplaySubsystem clientDisplay(drivers());

/* define commands ----------------------------------------------------------*/
ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

CalibrateCommand calibrateDoubleAgitator(&kickerSubsystem);

ChassisAutorotateCommand chassisAutorotateCommand(drivers(), &chassis, &turret);
WiggleDriveCommand wiggleDriveCommand(drivers(), &chassis, &turret);
TurretWorldRelativePositionCommand turretWorldRelativeCommand(drivers(), &turret, &chassis);

WaterwheelLoadCommand42mm waterwheelLoadCommand(drivers(), &waterWheelAgitator);

ShootCommand42mm kickerShootHeatLimitedCommand(drivers(), &kickerSubsystem, true);
ShootCommand42mm kickerShootUnlimitedCommand(drivers(), &kickerSubsystem, false);

FrictionWheelRotateCommand spinFrictionWheels(
    &frictionWheels,
    FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);
FrictionWheelRotateCommand stopFrictionWheels(&frictionWheels, 0);

ClientDisplayCommand clientDisplayCommand(
    drivers(),
    &clientDisplay,
    nullptr,
    &chassisAutorotateCommand,
    &wiggleDriveCommand,
    &chassisDriveCommand);

/* define command mappings --------------------------------------------------*/
// Remote related mappings
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&wiggleDriveCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping rightSwitchUp(
    drivers(),
    {&kickerShootHeatLimitedCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// Keyboard/Mouse related mappings
ToggleCommandMapping rToggled(drivers(), {&chassisDriveCommand}, RemoteMapState({Remote::Key::R}));

ToggleCommandMapping fToggled(drivers(), {&wiggleDriveCommand}, RemoteMapState({Remote::Key::F}));

HoldCommandMapping leftMousePressed(
    drivers(),
    {&kickerShootHeatLimitedCommand},
    RemoteMapState(RemoteMapState::MouseButton::LEFT));

HoldRepeatCommandMapping rightMousePressed(
    drivers(),
    {&kickerShootUnlimitedCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));

HoldCommandMapping leftSwitchUp(
    drivers(),
    {&calibrateDoubleAgitator},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    turret.initialize();
    chassis.initialize();
    waterWheelAgitator.initialize();
    kickerSubsystem.initialize();
    frictionWheels.initialize();
    clientDisplay.initialize();
    drivers()->xavierSerial.attachChassis(&chassis);
    drivers()->xavierSerial.attachTurret(&turret);
}

/* register subsystems here -------------------------------------------------*/
void registerHeroSubsystems(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&waterWheelAgitator);
    drivers->commandScheduler.registerSubsystem(&kickerSubsystem);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultHeroCommands(aruwlib::Drivers *)
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
    waterWheelAgitator.setDefaultCommand(&waterwheelLoadCommand);
    clientDisplay.setDefaultCommand(&clientDisplayCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startHeroCommands(aruwlib::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerHeroIoMappings(aruwlib::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rToggled);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&leftMousePressed);
    drivers->commandMapper.addMap(&rightMousePressed);
    drivers->commandMapper.addMap(&leftSwitchUp);
}
}  // namespace hero_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwlib::Drivers *drivers)
{
    hero_control::initializeSubsystems();
    hero_control::registerHeroSubsystems(drivers);
    hero_control::setDefaultHeroCommands(drivers);
    hero_control::startHeroCommands(drivers);
    hero_control::registerHeroIoMappings(drivers);
}
}  // namespace aruwsrc::control

#endif

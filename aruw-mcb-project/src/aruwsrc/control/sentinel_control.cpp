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

#include <aruwlib/DriversSingleton.hpp>
#include <aruwlib/control/CommandMapper.hpp>
#include <aruwlib/control/HoldCommandMapping.hpp>
#include <aruwlib/control/HoldRepeatCommandMapping.hpp>
#include <aruwlib/control/PressCommandMapping.hpp>
#include <aruwlib/control/ToggleCommandMapping.hpp>

#include "agitator/agitator_calibrate_command.hpp"
#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "agitator/agitator_subsystem.hpp"
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "sentinel/sentinel_auto_drive_command.hpp"
#include "sentinel/sentinel_drive_manual_command.hpp"
#include "sentinel/sentinel_drive_subsystem.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_init_command.hpp"
#include "turret/turret_manual_command.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_world_relative_position_command.hpp"

#if defined(TARGET_SENTINEL)

using namespace aruwsrc::agitator;
using namespace aruwsrc::launcher;
using namespace aruwlib::control;
using aruwlib::DoNotUse_getDrivers;
using aruwlib::Remote;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwlib::driversFunc drivers = aruwlib::DoNotUse_getDrivers;

namespace aruwsrc
{
namespace control
{
/* define subsystems --------------------------------------------------------*/
AgitatorSubsystem agitator(
    drivers(),
    AgitatorSubsystem::PID_17MM_P,
    AgitatorSubsystem::PID_17MM_I,
    AgitatorSubsystem::PID_17MM_D,
    AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::AGITATOR_MOTOR_ID,
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    false);

AgitatorSubsystem kickerMotor(
    drivers(),
    AgitatorSubsystem::PID_17MM_KICKER_P,
    AgitatorSubsystem::PID_17MM_KICKER_I,
    AgitatorSubsystem::PID_17MM_KICKER_D,
    AgitatorSubsystem::PID_17MM_KICKER_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_KICKER_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::SENTINEL_KICKER_MOTOR_ID,
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    false);

aruwsrc::control::SentinelDriveSubsystem sentinelDrive(drivers());

FrictionWheelSubsystem upperFrictionWheels(
    drivers(),
    aruwlib::motor::MOTOR3,
    aruwlib::motor::MOTOR4);

FrictionWheelSubsystem lowerFrictionWheels(drivers());

/* define commands ----------------------------------------------------------*/
ShootFastComprisedCommand17MM agitatorShootSlowCommand(drivers(), &agitator);

AgitatorCalibrateCommand agitatorCalibrateCommand(&agitator);

AgitatorRotateCommand agitatorKickerCommand(&kickerMotor, 3.0f, 1, 0, false);

AgitatorCalibrateCommand agitatorCalibrateKickerCommand(&kickerMotor);

SentinelAutoDriveCommand sentinelAutoDrive(&sentinelDrive);

SentinelDriveManualCommand sentinelDriveManual(drivers(), &sentinelDrive);

FrictionWheelRotateCommand spinUpperFrictionWheels(
    &upperFrictionWheels,
    FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand spinLowerFrictionWheels(
    &lowerFrictionWheels,
    FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand stopUpperFrictionWheels(&upperFrictionWheels, 0);

FrictionWheelRotateCommand stopLowerFrictionWheels(&lowerFrictionWheels, 0);

/* define command mappings --------------------------------------------------*/
HoldRepeatCommandMapping leftSwitchUp(
    drivers(),
    {&agitatorShootSlowCommand, &agitatorKickerCommand},
    aruwlib::control::RemoteMapState(
        aruwlib::Remote::Switch::LEFT_SWITCH,
        aruwlib::Remote::SwitchState::UP));
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&stopLowerFrictionWheels, &stopUpperFrictionWheels},
    aruwlib::control::RemoteMapState(
        aruwlib::Remote::Switch::LEFT_SWITCH,
        aruwlib::Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchDown(
    drivers(),
    {&sentinelAutoDrive},
    aruwlib::control::RemoteMapState(
        aruwlib::Remote::Switch::RIGHT_SWITCH,
        aruwlib::Remote::SwitchState::DOWN));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    agitator.initialize();
    kickerMotor.initialize();
    sentinelDrive.initialize();
    upperFrictionWheels.initialize();
    lowerFrictionWheels.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerSentinelSubsystems(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&kickerMotor);
    drivers->commandScheduler.registerSubsystem(&sentinelDrive);
    drivers->commandScheduler.registerSubsystem(&upperFrictionWheels);
    drivers->commandScheduler.registerSubsystem(&lowerFrictionWheels);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSentinelCommands(aruwlib::Drivers *)
{
    sentinelDrive.setDefaultCommand(&sentinelDriveManual);
    upperFrictionWheels.setDefaultCommand(&spinUpperFrictionWheels);
    lowerFrictionWheels.setDefaultCommand(&spinLowerFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSentinelCommands(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&agitatorCalibrateCommand);
    drivers->commandScheduler.addCommand(&agitatorCalibrateKickerCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSentinelIoMappings(aruwlib::Drivers *drivers)
{
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchDown);
}

void initSubsystemCommands(aruwlib::Drivers *drivers)
{
    initializeSubsystems();
    registerSentinelSubsystems(drivers);
    setDefaultSentinelCommands(drivers);
    startSentinelCommands(drivers);
    registerSentinelIoMappings(drivers);
}
}  // namespace control
}  // namespace aruwsrc

#endif

/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/command_mapper.hpp>

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
using aruwlib::Drivers;
using aruwlib::control::CommandMapper;

namespace aruwsrc
{
namespace control
{
/* define subsystems --------------------------------------------------------*/
AgitatorSubsystem agitator(
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
    AgitatorSubsystem::PID_17MM_KICKER_P,
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

FrictionWheelRotateCommand spinUpperFrictionWheels(
    &upperFrictionWheels,
    FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand spinLowerFrictionWheels(
    &lowerFrictionWheels,
    FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand stopUpperFrictionWheels(&upperFrictionWheels, 0);

FrictionWheelRotateCommand stopLowerFrictionWheels(&lowerFrictionWheels, 0);

/* register subsystems here -------------------------------------------------*/
void registerSentinelSubsystems()
{
    Drivers::commandScheduler.registerSubsystem(&agitator);
    Drivers::commandScheduler.registerSubsystem(&kickerMotor);
    Drivers::commandScheduler.registerSubsystem(&sentinelDrive);
    Drivers::commandScheduler.registerSubsystem(&upperFrictionWheels);
    Drivers::commandScheduler.registerSubsystem(&lowerFrictionWheels);
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
    Drivers::commandScheduler.addCommand(&agitatorCalibrateCommand);
    Drivers::commandScheduler.addCommand(&agitatorCalibrateKickerCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSentinelIoMappings()
{
    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::LEFT_SWITCH,
            aruwlib::Remote::SwitchState::UP),
        &agitatorShootSlowCommand);

    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::RIGHT_SWITCH,
            aruwlib::Remote::SwitchState::UP),
        &agitatorKickerCommand);

    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::RIGHT_SWITCH,
            aruwlib::Remote::SwitchState::DOWN),
        &sentinelAutoDrive);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::LEFT_SWITCH,
            aruwlib::Remote::SwitchState::DOWN),
        &stopLowerFrictionWheels);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::RIGHT_SWITCH,
            aruwlib::Remote::SwitchState::DOWN),
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

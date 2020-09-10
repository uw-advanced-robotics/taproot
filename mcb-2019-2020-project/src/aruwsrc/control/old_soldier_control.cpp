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
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "chassis/wiggle_drive_command.hpp"
#include "hopper-cover/hopper_subsystem.hpp"
#include "hopper-cover/open_hopper_command.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_world_relative_position_command.hpp"

#if defined(TARGET_OLD_SOLDIER)

using namespace aruwsrc::agitator;
using namespace aruwsrc::chassis;
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
    Drivers::commandScheduler.registerSubsystem(&agitator);
    Drivers::commandScheduler.registerSubsystem(&chassis);
    Drivers::commandScheduler.registerSubsystem(&turret);
    Drivers::commandScheduler.registerSubsystem(&hopperCover);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultOldSoldierCommands()
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startOldSoldierCommands() { Drivers::commandScheduler.addCommand(&agitatorCalibrateCommand); }

/* register io mappings here ------------------------------------------------*/
void registerOldSoldierIoMappings()
{
    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::RIGHT_SWITCH,
            aruwlib::Remote::SwitchState::UP),
        &agitatorShootFastCommand);

    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::LEFT_SWITCH,
            aruwlib::Remote::SwitchState::MID),
        &chassisAutorotateCommand);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::LEFT_SWITCH,
            aruwlib::Remote::SwitchState::DOWN),
        &chassisDriveCommand);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::LEFT_SWITCH,
            aruwlib::Remote::SwitchState::DOWN),
        &openHopperCommand);

    Drivers::commandMapper.addHoldMapping(
        CommandMapper::newKeyMap(
            aruwlib::Remote::Switch::LEFT_SWITCH,
            aruwlib::Remote::SwitchState::UP),
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

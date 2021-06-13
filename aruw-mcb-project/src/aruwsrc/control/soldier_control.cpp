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
#if defined(TARGET_SOLDIER)

#include <aruwlib/DriversSingleton.hpp>
#include <aruwlib/control/CommandMapper.hpp>
#include <aruwlib/control/HoldCommandMapping.hpp>
#include <aruwlib/control/HoldRepeatCommandMapping.hpp>
#include <aruwlib/control/PressCommandMapping.hpp>
#include <aruwlib/control/ToggleCommandMapping.hpp>

#include "agitator/agitator_calibrate_command.hpp"
#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "agitator/agitator_subsystem.hpp"
#include "chassis/beyblade_command.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "client-display/client_display_command.hpp"
#include "client-display/client_display_subsystem.hpp"
#include "hopper-cover/hopper_commands.hpp"
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_world_relative_position_command.hpp"

#ifdef PLATFORM_HOSTED
#include "aruwlib/communication/can/can.hpp"
#include "aruwlib/motor/motorsim/motor_sim.hpp"
#include "aruwlib/motor/motorsim/sim_handler.hpp"
#endif

using namespace aruwsrc::agitator;
using namespace aruwsrc::chassis;
using namespace aruwsrc::launcher;
using namespace aruwsrc::turret;
using namespace aruwlib::control;
using namespace aruwsrc::display;
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
TurretSubsystem turret(drivers(), false);

ChassisSubsystem chassis(drivers());

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
    AgitatorSubsystem::isAgitatorInverted);

// TODO: validate and tune these constexpr parameters for hopper lid motor
// also find out what kind of motor hopper lid uses lol
AgitatorSubsystem hopperCover(
    drivers(),
    AgitatorSubsystem::PID_17MM_P,
    AgitatorSubsystem::PID_17MM_I,
    AgitatorSubsystem::PID_17MM_D,
    AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::HOPPER_COVER_MOTOR_ID,
    AgitatorSubsystem::HOPPER_COVER_MOTOR_CAN_BUS,
    AgitatorSubsystem::IS_HOPPER_COVER_INVERTED);

FrictionWheelSubsystem frictionWheels(drivers());

ClientDisplaySubsystem clientDisplay(drivers());

/* define commands ----------------------------------------------------------*/
ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

ChassisAutorotateCommand chassisAutorotateCommand(drivers(), &chassis, &turret);

BeybladeCommand beybladeCommand(drivers(), &chassis, &turret);

TurretWorldRelativePositionCommand turretWorldRelativeCommand(drivers(), &turret, &chassis);

TurretCVCommand turretCVCommand(drivers(), &turret);

AgitatorCalibrateCommand agitatorCalibrateCommand(&agitator);

ShootFastComprisedCommand17MM agitatorShootFastLimited(drivers(), &agitator);

ShootFastComprisedCommand17MM agitatorShootFastNotLimited(drivers(), &agitator, false);

SoldierOpenHopperCommand openHopperCommand(&hopperCover);

SoldierCloseHopperCommand closeHopperCommand(&hopperCover);

FrictionWheelRotateCommand spinFrictionWheels(
    &frictionWheels,
    FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);

FrictionWheelRotateCommand stopFrictionWheels(&frictionWheels, 0);

ClientDisplayCommand clientDisplayCommand(
    drivers(),
    &clientDisplay,
    &beybladeCommand,
    &chassisAutorotateCommand,
    nullptr,
    &chassisDriveCommand);

/* define command mappings --------------------------------------------------*/
// Remote related mappings
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&openHopperCommand, &stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&agitatorShootFastLimited},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&beybladeCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&chassisDriveCommand, &turretCVCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Keyboard/Mouse related mappings
ToggleCommandMapping rToggled(
    drivers(),
    {&openHopperCommand, &stopFrictionWheels},
    RemoteMapState({Remote::Key::R}));
ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));
HoldRepeatCommandMapping leftMousePressedShiftNotPressed(
    drivers(),
    {&agitatorShootFastLimited},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::SHIFT}));
HoldRepeatCommandMapping leftMousePressedShiftPressed(
    drivers(),
    {&agitatorShootFastNotLimited},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::SHIFT}));
HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));

/* register subsystems here -------------------------------------------------*/
void registerSoldierSubsystems(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&hopperCover);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);

#ifdef PLATFORM_HOSTED
    // Register the motor sims for the Agitator subsystem
    // TODO: Create simulator for correct motor
    aruwlib::motorsim::SimHandler::registerSim(
        aruwlib::motorsim::MotorSim::MotorType::M3508,
        aruwsrc::agitator::AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
        aruwsrc::agitator::AgitatorSubsystem::AGITATOR_MOTOR_ID);

    // Register the motor sims for the Chassis subsystem
    aruwlib::motorsim::MotorSim::MotorType CHASSIS_MOTOR_TYPE =
        aruwlib::motorsim::MotorSim::MotorType::M3508;
    aruwlib::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        chassis::ChassisSubsystem::LEFT_FRONT_MOTOR_ID);
    aruwlib::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        chassis::ChassisSubsystem::LEFT_BACK_MOTOR_ID);
    aruwlib::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        chassis::ChassisSubsystem::RIGHT_FRONT_MOTOR_ID);
    aruwlib::motorsim::SimHandler::registerSim(
        CHASSIS_MOTOR_TYPE,
        aruwsrc::chassis::ChassisSubsystem::CAN_BUS_MOTORS,
        chassis::ChassisSubsystem::RIGHT_BACK_MOTOR_ID);

    // Register the motor sims for the turret subsystem
    aruwlib::motorsim::SimHandler::registerSim(
        aruwlib::motorsim::MotorSim::MotorType::GM6020,
        aruwsrc::turret::TurretSubsystem::CAN_BUS_MOTORS,
        aruwsrc::turret::TurretSubsystem::PITCH_MOTOR_ID);
    aruwlib::motorsim::SimHandler::registerSim(
        aruwlib::motorsim::MotorSim::MotorType::GM6020,
        aruwsrc::turret::TurretSubsystem::CAN_BUS_MOTORS,
        aruwsrc::turret::TurretSubsystem::YAW_MOTOR_ID);

    // Register the motor sims for the Hopper Cover (There aren't any)
    // Register the motor sims for the Friction Wheels
    aruwlib::motorsim::SimHandler::registerSim(
        aruwlib::motorsim::MotorSim::MotorType::M3508,
        aruwsrc::launcher::FrictionWheelSubsystem::CAN_BUS_MOTORS,
        aruwsrc::launcher::FrictionWheelSubsystem::LEFT_MOTOR_ID);
    aruwlib::motorsim::SimHandler::registerSim(
        aruwlib::motorsim::MotorSim::MotorType::M3508,
        aruwsrc::launcher::FrictionWheelSubsystem::CAN_BUS_MOTORS,
        aruwsrc::launcher::FrictionWheelSubsystem::RIGHT_MOTOR_ID);
#endif  // PLATFORM_HOSTED
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    turret.initialize();
    chassis.initialize();
    agitator.initialize();
    frictionWheels.initialize();
    hopperCover.initialize();
    clientDisplay.initialize();
    drivers()->xavierSerial.attachChassis(&chassis);
    drivers()->xavierSerial.attachTurret(&turret);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultSoldierCommands(aruwlib::Drivers *)
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
    clientDisplay.setDefaultCommand(&clientDisplayCommand);
    hopperCover.setDefaultCommand(&closeHopperCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startSoldierCommands(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&agitatorCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerSoldierIoMappings(aruwlib::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&rToggled);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&leftMousePressedShiftNotPressed);
    drivers->commandMapper.addMap(&leftMousePressedShiftPressed);
    drivers->commandMapper.addMap(&rightMousePressed);
}

void initSubsystemCommands(aruwlib::Drivers *drivers)
{
    initializeSubsystems();
    registerSoldierSubsystems(drivers);
    setDefaultSoldierCommands(drivers);
    startSoldierCommands(drivers);
    registerSoldierIoMappings(drivers);
}

}  // namespace control

}  // namespace aruwsrc

#endif

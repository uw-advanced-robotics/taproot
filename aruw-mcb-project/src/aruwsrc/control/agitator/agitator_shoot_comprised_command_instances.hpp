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

#ifndef AGITATOR_SHOOT_COMPRISED_COMMAND_INSTANCES_HPP_
#define AGITATOR_SHOOT_COMPRISED_COMMAND_INSTANCES_HPP_

#include "aruwlib/Drivers.hpp"

#include "agitator_shoot_comprised_command.hpp"
#include "limited_agitator_subsystem.hpp"

namespace aruwsrc
{
namespace agitator
{
/**
 * A class that extends the shoot comprised command and defines the system parameters of the
 * comprised command. The constants are choosen for fast rotation speed for a soldier robot's
 * agitator.
 */
class ShootFastComprisedCommand17MM : public ShootComprisedCommand
{
public:
    ShootFastComprisedCommand17MM(
        aruwlib::Drivers* drivers,
        AgitatorSubsystem* agitator17mm,
        bool heatLimiting = true)
        : ShootComprisedCommand(
              drivers,
              agitator17mm,
              aruwlib::algorithms::PI / 5.0f,
              aruwlib::algorithms::PI / 2.0f,
              40,
              10),
          drivers(drivers),
          heatLimiting(heatLimiting)
    {
    }

    void initialize() override;

private:
    /// Buffer from max heat limit in which limiting occurs (3 bullet buffer)
    static constexpr uint16_t HEAT_LIMIT_BUFFER = 30;

    aruwlib::Drivers* drivers;

    const bool heatLimiting;
};  // class ShootFastComprisedCommand

/**
 * A class that extends the shoot comprised command and defines the system parameters of the
 * comprised command. The constants are choosen for slow rotation speed for a soldier robot's
 * agitator.
 */
class ShootSlowComprisedCommand17MM : public ShootComprisedCommand
{
public:
    ShootSlowComprisedCommand17MM(
        aruwlib::Drivers* drivers,
        AgitatorSubsystem* agitator17mm,
        bool heatLimiting = true)
        : ShootComprisedCommand(
              drivers,
              agitator17mm,
              aruwlib::algorithms::PI / 5.0f,
              aruwlib::algorithms::PI / 2.0f,
              300,
              100),
          drivers(drivers),
          heatLimiting(heatLimiting)
    {
    }

    void initialize() override;

private:
    /// Buffer from max heat limit in which limiting occurs (2 bullet buffer)
    static constexpr uint16_t HEAT_LIMIT_BUFFER = 20;

    aruwlib::Drivers* drivers;

    const bool heatLimiting;
};  // class ShootSlowComprisedCommand

class WaterwheelLoadCommand42mm : public ShootComprisedCommand
{
public:
    // Angle the command tries to move the agitator whenever it is scheduled
    static constexpr float WATERWHEEL_42MM_CHANGE_ANGLE = aruwlib::algorithms::PI / 6;
    // Max angle the agitator will move while unjamming
    static constexpr float WATERWHEEL_42MM_MAX_UNJAM_ANGLE = aruwlib::algorithms::PI / 6;
    // Expected time for the water wheel to rotate the specified angle in ms
    static constexpr uint32_t WATERWHEEL_42MM_ROTATE_TIME = 1000;
    // How long the command should wait after reaching the target angle
    static constexpr uint32_t WATERWHEEL_42MM_PAUSE_AFTER_ROTATE_TIME = 10;

    WaterwheelLoadCommand42mm(
        aruwlib::Drivers* drivers,
        aruwsrc::agitator::LimitedAgitatorSubsystem* waterwheel);

    bool isReady() override;

    bool isFinished() const override;

private:
    // Store instance of drivers to be able to access digital
    aruwlib::Drivers* drivers;

    // Store pointer to limited agitator subsystem with derived class type
    aruwsrc::agitator::LimitedAgitatorSubsystem* waterwheel;

};  // class Waterwheel42mmLoadCommand

class ShootComprisedCommand42mm : public ShootComprisedCommand
{
public:
    // Angle the command tries to move the agitator whenever it is scheduled
    static constexpr float KICKER_42MM_CHANGE_ANGLE = aruwlib::algorithms::PI / 6;
    // Max angle the agitator will move while unjamming
    static constexpr float KICKER_42MM_MAX_UNJAM_ANGLE = aruwlib::algorithms::PI / 6;
    // Expected time for the water wheel to rotate the specified angle in ms
    static constexpr uint32_t KICKER_42MM_ROTATE_TIME = 300;
    // How long the command should wait after reaching the target angle
    static constexpr uint32_t KICKER_42MM_PAUSE_AFTER_ROTATE_TIME = 10;

    ShootComprisedCommand42mm(
        aruwlib::Drivers* drivers,
        aruwsrc::agitator::AgitatorSubsystem* kicker,
        bool heatLimiting = true);

    // Override for heat limiting logic
    bool isReady() override;

private:
    // Buffer from max heat limit in which limiting occurs, for hero 100 is one shot.
    static constexpr uint16_t HEAT_LIMIT_BUFFER = 100;

    aruwlib::Drivers* drivers;

    bool heatLimiting;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_SHOOT_COMPRISED_COMMAND_INSTANCES_HPP_

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

#ifndef __FRICTION_WHEEL_ROTATE_COMMAND_HPP__
#define __FRICTION_WHEEL_ROTATE_COMMAND_HPP__

#include "aruwlib/control/command.hpp"

namespace aruwsrc
{
namespace launcher
{
class FrictionWheelSubsystem;

/**
 * Command which sets a given friction wheel subsystem to a set speed.
 */
class FrictionWheelRotateCommand : public aruwlib::control::Command
{
public:
    FrictionWheelRotateCommand(FrictionWheelSubsystem* subsystem, int speed);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "friction wheel rotate"; }

#if defined(TARGET_SOLDIER)
    static constexpr int16_t DEFAULT_WHEEL_RPM = 4500;
#elif defined(TARGET_HERO)
    static constexpr int16_t DEFAULT_WHEEL_RPM = 7000;
#elif defined(TARGET_SENTINEL)
    static const int16_t DEFAULT_WHEEL_RPM = 5150;
#else
    static constexpr int16_t DEFAULT_WHEEL_RPM = 0;
#endif

private:
    FrictionWheelSubsystem* frictionWheelSubsystem;

    int speed;
};

}  // namespace launcher

}  // namespace aruwsrc

#endif

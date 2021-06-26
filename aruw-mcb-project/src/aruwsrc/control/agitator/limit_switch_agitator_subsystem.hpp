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

#ifndef LIMITED_AGITATOR_SUBSYSTEM_HPP_
#define LIMITED_AGITATOR_SUBSYSTEM_HPP_

#include "aruwlib/communication/gpio/digital.hpp"

#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "modm/math/filter/debounce.hpp"

namespace aruwlib
{
class Drivers;
}

namespace aruwsrc
{
namespace agitator
{
/**
 * An agitator subsystem that contains an agitator and a limit switch. The limit switch is
 * located between the agitator and the 42mm barrel. When the limit switch is tripped, this
 * indicates that a ball has passed into the space between it and the barrel and the number of balls
 * queued between the limit switch and barrel is incremented. When a projectile is fired through the
 * barrel, this indicates that the number of balls between the limit switch and barrel has decreased
 * by 1.
 *
 * This subsystem has all the functionality that the AgitatorSubsystem does but keeps track of
 * this limit switch logic.
 */
class LimitSwitchAgitatorSubsystem : public AgitatorSubsystem
{
public:
#if defined(TARGET_HERO)
    /**
     * The pin to check for the limit switch input, needs to be set by someone who
     * knows what they're doing.
     */
    static constexpr aruwlib::gpio::Digital::InputPin WATERWHEEL_LIMIT_PIN =
        aruwlib::gpio::Digital::InputPin::B;
#endif

    /**
     * @param[in] limitSwitchPin the pin to be checked for limit switch input.
     * @note for all params before `limitSwitchPin` see `AgitatorSubsystem.hpp`
     * @note for all debounce params see `debounce.hpp` for reference
     */
    LimitSwitchAgitatorSubsystem(
        aruwlib::Drivers* drivers,
        float kp,
        float ki,
        float kd,
        float maxIAccum,
        float maxOutput,
        float agitatorGearRatio,
        aruwlib::motor::MotorId agitatorMotorId,
        aruwlib::can::CanBus agitatorCanBusId,
        bool isAgitatorInverted,
        float distanceTolerance,
        uint32_t temporalTolerance,
        aruwlib::gpio::Digital::InputPin limitSwitchPin);

    void refresh() override;

    inline int getBallsInTube() const { return ballsInTube; }

private:
    /**
     * The heat will increase by this amount every time you fire a 42mm projectile.
     */
    static constexpr uint16_t FIRING_HEAT_INCREASE_42 = 100;

    /**
     * The pin that the agitator checks for limiting
     */
    aruwlib::gpio::Digital::InputPin limitSwitchPin;

    /**
     * A pointer to the digital class to get GPIO state from
     */
    aruwlib::gpio::Digital* digital;

    bool limitSwitchPressed;

    int ballsInTube;

    float prevHeat42;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif  // LIMITED_AGITATOR_SUBSYSTEM_HPP_

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
 * An agitator subsystem that also checks a limit switch every
 * refresh cycle, debouncing the limit switch input.
 */
class LimitedAgitatorSubsystem : public AgitatorSubsystem
{
public:
#if defined(TARGET_HERO)
    /**
     * The pin to check for the limit switch input, needs to be set by someone who
     * knows what they're doing.
     */
    static constexpr aruwlib::gpio::Digital::InputPin WATERWHEEL_LIMIT_PIN =
        aruwlib::gpio::Digital::InputPin::A;
    static constexpr uint8_t WATERWHEEL_DEBOUNCE_MAX_SUM = 100;
    static constexpr uint8_t WATERWHEEL_DEBOUNCE_LOWER_BOUND = 30;
    static constexpr uint8_t WATERWHEEL_DEBOUNCE_UPPER_BOUND = 70;
#endif

    /**
     * @param[in] limitSwitchPin the pin to be checked for limit switch input.
     * @note for all params before `limitSwitchPin` see `AgitatorSubsystem.hpp`
     * @note for all debounce params see `debounce.hpp` for reference
     */
    LimitedAgitatorSubsystem(
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
        aruwlib::gpio::Digital::InputPin limitSwitchPin,
        uint8_t debounceMaxSum,
        uint8_t debounceLowerBound,
        uint8_t debounceUpperBound);

    void refresh() override;

    /**
     * Get the debounced state of the limit switch
     */
    bool isLimitSwitchPressed() const;

private:
    /**
     * The pin that the agitator checks for limiting
     */
    aruwlib::gpio::Digital::InputPin limitSwitchPin;

    /**
     * A pointer to the digital class to get GPIO state from
     */
    aruwlib::gpio::Digital* digital;

    /**
     * debounce filter for limit switch input
     */
    modm::filter::Debounce<uint8_t> debounceFilter;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif  // LIMITED_AGITATOR_SUBSYSTEM_HPP_

/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "pwm.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/rm-dev-board-a/board.hpp"

using namespace Board;
using namespace tap::algorithms;

namespace tap
{
namespace gpio
{
void Pwm::init()
{
#ifndef PLATFORM_HOSTED
    Timer8::connect<PWMOutPinW::Ch1, PWMOutPinX::Ch2, PWMOutPinY::Ch3, PWMOutPinZ::Ch4>();
    Timer8::enable();
    Timer8::setMode(Timer8::Mode::UpCounter);
    timer8CalculatedOverflow =
        Timer8::setPeriod<Board::SystemClock>(1'000'000 / DEFAULT_TIMER8_FREQUENCY);
    Timer8::start();
    Timer8::enableOutput();

    Timer12::connect<PWMOutPinBuzzer::Ch1>();
    Timer12::enable();
    Timer12::setMode(Timer12::Mode::UpCounter);
    timer12CalculatedOverflow =
        Timer12::setPeriod<Board::SystemClock>(1'000'000 / DEFAULT_TIMER12_FREQUENCY);
    Timer12::start();
    Timer12::enableOutput();
#endif
    // Set all out pins to 0 duty
    writeAll(0.0f);
}

void Pwm::writeAll(float duty)
{
    write(duty, Pin::W);
    write(duty, Pin::X);
    write(duty, Pin::Y);
    write(duty, Pin::Z);
    write(duty, Pin::BUZZER);
}

void Pwm::write(float duty, Pin pin)
{
#ifndef PLATFORM_HOSTED
    duty = limitVal(duty, 0.0f, 1.0f);
    if (pin == BUZZER)
    {
        Timer12::configureOutputChannel(
            BUZZER_CHANNEL,
            Timer12::OutputCompareMode::Pwm2,
            duty * timer8CalculatedOverflow);
    }
    else
    {
        Timer8::configureOutputChannel(
            static_cast<int>(pin),
            Timer8::OutputCompareMode::Pwm,
            duty * timer12CalculatedOverflow);
    }
#endif
}

void Pwm::setTimerFrequency(Timer timer, uint32_t frequency)
{
#ifndef PLATFORM_HOSTED
    switch (timer)
    {
        case TIMER_8:
            Timer8::setPeriod<Board::SystemClock>(1'000'000 / frequency);
            break;
        case TIMER_12:
            Timer12::setPeriod<Board::SystemClock>(1'000'000 / frequency);
            break;
    }
#endif
}

void Pwm::pause(Timer timer)
{
#ifndef PLATFORM_HOSTED
    switch (timer)
    {
        case TIMER_8:
            Timer8::pause();
            break;
        case TIMER_12:
            Timer12::pause();
            break;
    }
#endif
}

void Pwm::start(Timer timer)
{
#ifndef PLATFORM_HOSTED
    switch (timer)
    {
        case TIMER_8:
            Timer8::start();
            break;
        case TIMER_12:
            Timer12::start();
            break;
    }
#endif
}
}  // namespace gpio

}  // namespace tap

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

#ifndef PWM_MOCK_HPP_
#define PWM_MOCK_HPP_

#include <aruwlib/communication/gpio/pwm.hpp>
#include <gmock/gmock.h>

namespace aruwlib
{
namespace mock
{
class PwmMock : public aruwlib::gpio::Pwm
{
public:
    PwmMock();
    virtual ~PwmMock();

    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(void, writeAll, (float duty), (override));
    MOCK_METHOD(void, write, (float duty, aruwlib::gpio::Pwm::Pin pin), (override));
};  // class PwmMock
}  // namespace mock
}  // namespace aruwlib

#endif  //  PWM_MOCK_HPP_

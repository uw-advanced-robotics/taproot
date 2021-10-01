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

#ifndef DOUBLE_DJI_MOTOR_HPP_
#define DOUBLE_DJI_MOTOR_HPP_

#include "dji_motor.hpp"
#include "motor_interface.hpp"

namespace tap::motor
{
class DoubleDjiMotor : public MotorInterface
{
public:
    DoubleDjiMotor(
        Drivers* drivers,
        MotorId desMotorIdentifierOne,
        MotorId desMotorIdentifierTwo,
        tap::can::CanBus motorCanBusOne,
        tap::can::CanBus motorCanBusTwo,
        bool isInvertedOne,
        bool isInvertedTwo,
        const char* nameOne,
        const char* nameTwo,
        uint16_t encWrapped = DjiMotor::ENC_RESOLUTION / 2,
        int64_t encRevolutions = 0);

    void initialize() override;
    int64_t getEncoderUnwrapped() const override;
    uint16_t getEncoderWrapped() const override;
    void setDesiredOutput(int32_t desiredOutput) override;
    bool isMotorOnline() const override;
    int16_t getOutputDesired() const override;
    int8_t getTemperature() const override;
    int16_t getTorque() const override;
    int16_t getShaftRPM() const override;

private:
    DjiMotor motorOne;
    DjiMotor motorTwo;
};
}  // namespace tap::motor

#endif  // DOUBLE_DJI_MOTOR_HPP_

/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_WRAPPED_ENCODER_HPP_
#define TAPROOT_WRAPPED_ENCODER_HPP_

#include "tap/communication/sensors/encoder/encoder_interface.hpp"
#include "tap/util_macros.hpp"

namespace tap::encoder
{
class WrappedEncoder : public EncoderInterface
{
public:
    WrappedEncoder(
        bool isInverted,
        uint32_t encoderResolution,
        float gearRatio = 1,
        tap::algorithms::WrappedFloat encoderHomePosition = tap::algorithms::WrappedFloat(0, 0, 1));

    void initialize() override{};

    tap::algorithms::WrappedFloat getPosition() const override;

    float getVelocity() const override;

    void alignWith(EncoderInterface* other) override;

    /**
     * Resets this motor's current encoder home position to the current encoder position reported by
     * CAN messages, and resets this motor's encoder revolutions to 0.
     */
    void resetEncoderValue() override;

    DISALLOW_COPY_AND_ASSIGN(WrappedEncoder)

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    bool isOnline() const override { return true; }
#else
protected:
#endif

    /**
     * Updates the stored encoder value given a newly received encoder value
     * special logic necessary for keeping track of unwrapped encoder value.
     */
    void updateEncoderValue(uint32_t encoderActual);

    tap::algorithms::WrappedFloat encoder;

    tap::algorithms::WrappedFloat position;

    /**
     * If `false` the positive rotation direction of the shaft is counter-clockwise when
     * looking at the shaft from the side opposite the motor. If `true` then the positive
     * rotation direction will be clockwise.
     */
    bool inverted;

private:
    uint32_t encoderResolution;

    float gearRatio;

    /**
     * The actual encoder wrapped value received from CAN messages where this motor
     * is considered to have an encoder value of 0. encoderHomePosition is 0 by default.
     */
    tap::algorithms::WrappedFloat encoderHomePosition;

    tap::algorithms::WrappedFloat pastPosition;

    uint64_t lastUpdateTime;
};

}  // namespace tap::encoder

#endif  // TAPROOT_WRAPPED_ENCODER_HPP_

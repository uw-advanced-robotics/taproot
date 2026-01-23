/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_FALLBACK_ENCODER_HPP_
#define TAPROOT_FALLBACK_ENCODER_HPP_

#include "multi_encoder.hpp"

namespace tap::encoder
{
/**
 * Relies on best encoder readings if best encoder is online, otherwise averages remaining encoder
 * measurements
 */
template <uint32_t COUNT>
class FallbackEncoder : public MultiEncoder<COUNT>
{
public:
    FallbackEncoder(std::array<EncoderInterface*, COUNT> encoders) : MultiEncoder<COUNT>(encoders) {}

    tap::algorithms::WrappedFloat getPosition() const override
    {
        const_cast<FallbackEncoder<COUNT>*>(this)->syncEncoders();
        if (this->validEncoder(0))
        {
            return this->encoders[0]->getPosition();
        }
        else
        {
            int onlineEncoders = 0;
            float position = 0;
            for (uint32_t i = 1; i < COUNT; i++)
            {
                if (this->validEncoder(i))
                {
                    position += this->encoders[i]->getPosition().getUnwrappedValue();
                    onlineEncoders++;
                }
            }
            return tap::algorithms::WrappedFloat(
                onlineEncoders == 0 ? 0 : position / onlineEncoders,
                0,
                static_cast<float>(M_TWOPI));
        }
    }

    float getVelocity() const override
    {
        const_cast<FallbackEncoder<COUNT>*>(this)->syncEncoders();
        if (this->validEncoder(0))
        {
            return this->encoders[0]->getVelocity();
        }
        else
        {
            int onlineEncoders = 0;
            float velocity = 0;
            for (uint32_t i = 1; i < COUNT; i++)
            {
                if (this->validEncoder(i))
                {
                    velocity += this->encoders[i]->getVelocity();
                    onlineEncoders++;
                }
            }
            return onlineEncoders == 0 ? 0 : velocity / onlineEncoders;
        }
    }
};
}  // namespace tap::encoder

#endif
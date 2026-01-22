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

#ifndef TAPROOT_ALIGNING_ENCODER_HPP_
#define TAPROOT_ALIGNING_ENCODER_HPP_


#include "multi_encoder.hpp"

namespace tap::encoder {

/**
 * Takes in COUNT encoders and chooses one to align the rest of the encoders to
 */
template <uint32_t COUNT>
class AligningEncoder : public MultiEncoder
{
public:
    AlignEncoder(std::array<EncoderInterface*, COUNT> encoders, int index)
        : MultiEncoder(encoders), index(index) {}

    tap::algorithms::WrappedFloat getPosition() const override {
        alignWithEncoder();
        tap::algorithms::WrappedFloat position = MultiEncoder::getPosition(!aligned);
        aligned = false; 
        return position;
    }

    float getVelocity() const override {
        alignWithEncoder();
        float velocity = MultiEncoder::getVelocity(!aligned);
        aligned = false;
        return velocity;
    }

private:
    uint32_t index;
    bool aligned = false;

    void alignWithEncoder() {
        if (this->validEncoder(index)) {
            for (int i = 0; i < COUNT; i++) {
                if (i != index) {
                    bool online = this->encoders[i] != nullptr && this->encoders[i]->isOnline();
                    if (online && !this->seenEncoder(i)) {
                        this->seenEncoders |= 1 << i;
                        encoders[i]->alignWith(encoders[index]);
                        aligned = true;
                    } else if (validEncoder(i) && !seenEncoder(index)) {
                        this->seenEncoders |= 1 << index;
                        encoders[index]->alignWith(encoders[i]);
                        aligned = true;
                    } else if (!online) {
                        this->seenEncoders &= ~(1 << i);
                    }
                }
            }
        }
    }
};
}

#endif
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

#include "can_encoder.hpp"

#include "modm/architecture/interface/can_message.hpp"

namespace tap::encoder
{
CanEncoder::CanEncoder(
    Drivers* drivers,
    CanEncoderId id,
    tap::can::CanBus canBus,
    bool isInverted,
    float gearRatio,
    uint32_t encoderHomePosition)
    : CanRxListener(drivers, id, canBus),
      WrappedEncoder(isInverted, ENCODER_RESOLUTION, gearRatio, encoderHomePosition),
      gauss(0)
{
}

bool CanEncoder::isOnline() const { return !this->encoderDisconnectTimeout.isExpired(); }

void CanEncoder::initialize() { attachSelfToRxHandler(); }

void CanEncoder::processMessage(const modm::can::Message& message)
{
    uint16_t encoder = (message.data[1] << 8) | message.data[0];
    this->updateEncoderValue(encoder);

    this->gauss = (message.data[3] << 8) | message.data[2];
    this->encoderDisconnectTimeout.restart(DISCONNECT_TIME);
}

}  // namespace tap::encoder
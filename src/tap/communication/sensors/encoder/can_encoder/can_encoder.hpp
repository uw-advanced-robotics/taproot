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

#ifndef TAPROOT_CAN_ENCODER_HPP_
#define TAPROOT_CAN_ENCODER_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/sensors/encoder/wrapped_encoder.hpp"

namespace tap::encoder
{
enum CanEncoderId
{
    ID0 = 0x1E0,
    ID1 = 0x1E1,
    ID2 = 0x1E2,
    ID3 = 0x1E3,
    ID4 = 0x1E4,
    ID5 = 0x1E5,
    ID6 = 0x1E6,
    ID7 = 0x1E7
};

class CanEncoder : public can::CanRxListener, public WrappedEncoder
{
public:
    static constexpr uint32_t ENCODER_RESOLUTION = 4096;

    CanEncoder(
        Drivers* drivers,
        CanEncoderId id,
        tap::can::CanBus canBus,
        bool isInverted = false,
        float gearRatio = 1,
        uint32_t encoderHomePosition = 0);

    void initialize() override;
    bool isOnline() const override;

    DISALLOW_COPY_AND_ASSIGN(CanEncoder)

    void processMessage(const modm::can::Message& message) override;

    mockable float getGauss() const { return this->gauss; };

private:
    // wait time before the encoder is considered disconnected, in milliseconds
    static const uint32_t DISCONNECT_TIME = 100;

    tap::arch::MilliTimeout encoderDisconnectTimeout{DISCONNECT_TIME};

    float gauss;
};

}  // namespace tap::encoder

#endif  // TAPROOT_CAN_ENCODER_HPP_

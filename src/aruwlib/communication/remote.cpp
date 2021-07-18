/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "remote.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/communication/serial/uart.hpp"
#include "aruwlib/drivers.hpp"

using namespace aruwlib::arch::clock;

#ifndef PLATFORM_HOSTED
namespace modm::platform
{
void uart1IrqHandler() { aruwlib::Remote::RemoteDmaTxISRHandler::dmaRXISR(); }
}  // namespace modm::platform
#endif

namespace aruwlib
{
void Remote::initialize()
{
#ifndef PLATFORM_HOSTED
    Dma2::enableRcc();

    RemoteDma::connect<GpioB7::Rx>();

    // Only configure RX stream
    Dma2::Stream2::configure(
        DmaBase::ChannelSelection::CHANNEL_4,
        DmaBase::DataTransferDirection::PERIPH_TO_MEM,
        DmaBase::PeripheralIncrementMode::FIXED,
        DmaBase::MemoryIncrementMode::INCREMENT,
        DmaBase::PeripheralDataSize::BYTE,
        DmaBase::MemoryDataSize::BYTE,
        DmaBase::ControlMode::CIRCULAR,
        DmaBase::PriorityLevel::LOW,
        DmaBase::FifoMode::DISABLED,
        DmaBase::FifoThreshold::QUARTER_FULL,
        DmaBase::MemoryBurstTransfer::SINGLE,
        DmaBase::PeripheralBurstTransfer::SINGLE);

    RemoteDma::initialize<Board::SystemClock, 100000>(
        false,
        12,
        modm::platform::UartBase::Parity::Even);

    // Disable UART TX
    modm::platform::UsartHal1::setTransmitterEnable(false);

    // Disalbe RX interrupt, we only want the idle interrupt
    modm::platform::UsartHal1::enableInterrupt(modm::platform::UsartHal1::Interrupt::RxIdle);
    modm::platform::UsartHal1::disableInterrupt(modm::platform::UsartHal1::Interrupt::RxNotEmpty);

    RemoteDma::clearIdleFlag();
#endif
}

void Remote::read()
{
    if (RemoteDmaTxISRHandler::readBuffer(rxBuffer))
    {
        parseBuffer();
    }
    else if (!isConnected())
    {
        reset();
    }
}

bool Remote::isConnected() const { return getTimeMilliseconds() - lastRead <= 2 * RECEIVE_PERIOD; }

float Remote::getChannel(Channel ch) const
{
    switch (ch)
    {
        case Channel::RIGHT_HORIZONTAL:
            return remote.rightHorizontal / STICK_MAX_VALUE;
        case Channel::RIGHT_VERTICAL:
            return remote.rightVertical / STICK_MAX_VALUE;
        case Channel::LEFT_HORIZONTAL:
            return remote.leftHorizontal / STICK_MAX_VALUE;
        case Channel::LEFT_VERTICAL:
            return remote.leftVertical / STICK_MAX_VALUE;
    }
    return 0;
}

Remote::SwitchState Remote::getSwitch(Switch sw) const
{
    switch (sw)
    {
        case Switch::LEFT_SWITCH:
            return remote.leftSwitch;
        case Switch::RIGHT_SWITCH:
            return remote.rightSwitch;
    }
    return SwitchState::UNKNOWN;
}

int16_t Remote::getMouseX() const { return remote.mouse.x; }

int16_t Remote::getMouseY() const { return remote.mouse.y; }

int16_t Remote::getMouseZ() const { return remote.mouse.z; }

bool Remote::getMouseL() const { return remote.mouse.l; }

bool Remote::getMouseR() const { return remote.mouse.r; }

bool Remote::keyPressed(Key key) const { return (remote.key & (1 << (uint8_t)key)) != 0; }

int16_t Remote::getWheel() const { return remote.wheel; }

void Remote::parseBuffer()
{
    remote.rightHorizontal = (rxBuffer[0] | rxBuffer[1] << 8) & 0x07FF;
    remote.rightHorizontal -= 1024;
    remote.rightVertical = (rxBuffer[1] >> 3 | rxBuffer[2] << 5) & 0x07FF;
    remote.rightVertical -= 1024;
    remote.leftHorizontal = (rxBuffer[2] >> 6 | rxBuffer[3] << 2 | rxBuffer[4] << 10) & 0x07FF;
    remote.leftHorizontal -= 1024;
    remote.leftVertical = (rxBuffer[4] >> 1 | rxBuffer[5] << 7) & 0x07FF;
    remote.leftVertical -= 1024;
    // the first 6 bytes refer to the remote channel values

    // switches on the dji remote - their input is registered
    switch (((rxBuffer[5] >> 4) & 0x000C) >> 2)
    {
        case 1:
            remote.leftSwitch = SwitchState::UP;
            break;
        case 3:
            remote.leftSwitch = SwitchState::MID;
            break;
        case 2:
            remote.leftSwitch = SwitchState::DOWN;
            break;
        default:
            remote.leftSwitch = SwitchState::UNKNOWN;
            break;
    }

    switch ((rxBuffer[5] >> 4) & 0x003)
    {
        case 1:
            remote.rightSwitch = SwitchState::UP;
            break;
        case 3:
            remote.rightSwitch = SwitchState::MID;
            break;
        case 2:
            remote.rightSwitch = SwitchState::DOWN;
            break;
        default:
            remote.rightSwitch = SwitchState::UNKNOWN;
            break;
    }

    // remaining 12 bytes (based on the DBUS_BUF_LEN variable
    // being 18) use mouse and keyboard data
    // STICK_MAX_VALUE is the max value from the remote, so gaining a higher
    // value would be impractical.
    // as such, the method returns null, exiting the method.
    if ((abs(remote.rightHorizontal) > STICK_MAX_VALUE) ||
        (abs(remote.rightVertical) > STICK_MAX_VALUE) ||
        (abs(remote.leftHorizontal) > STICK_MAX_VALUE) ||
        (abs(remote.leftVertical) > STICK_MAX_VALUE))
    {
        return;
    }

    // mouse input
    remote.mouse.x = rxBuffer[6] | (rxBuffer[7] << 8);    // x axis
    remote.mouse.y = rxBuffer[8] | (rxBuffer[9] << 8);    // y axis
    remote.mouse.z = rxBuffer[10] | (rxBuffer[11] << 8);  // z axis
    remote.mouse.l = static_cast<bool>(rxBuffer[12]);     // left button click
    remote.mouse.r = static_cast<bool>(rxBuffer[13]);     // right button click

    // keyboard capture
    remote.key = rxBuffer[14] | rxBuffer[15] << 8;
    // Remote wheel
    remote.wheel = (rxBuffer[16] | rxBuffer[17] << 8) - 1024;

    remote.updateCounter++;
    lastRead = getTimeMilliseconds();

    drivers->commandMapper.handleKeyStateChange(
        remote.key,
        remote.leftSwitch,
        remote.rightSwitch,
        remote.mouse.l,
        remote.mouse.r);
}

void Remote::reset()
{
    remote.rightHorizontal = 0;
    remote.rightVertical = 0;
    remote.leftHorizontal = 0;
    remote.leftVertical = 0;
    remote.leftSwitch = SwitchState::UNKNOWN;
    remote.rightSwitch = SwitchState::UNKNOWN;
    remote.mouse.x = 0;
    remote.mouse.y = 0;
    remote.mouse.z = 0;
    remote.mouse.l = 0;
    remote.mouse.r = 0;
    remote.key = 0;
    remote.wheel = 0;
}

uint32_t Remote::getUpdateCounter() const { return remote.updateCounter; }
}  // namespace aruwlib

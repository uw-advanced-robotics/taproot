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

#ifndef REMOTE_HPP_
#define REMOTE_HPP_

#include <cstdint>

#ifndef PLATFORM_HOSTED
#include "modm/platform.hpp"
#include "modm/platform/uart/uart_1.hpp"
#endif

#include "aruwlib/util_macros.hpp"

#include "dma/dma_uart_rx_irqhandler.hpp"

namespace aruwlib
{
class Drivers;
/**
 * Communication with the remote via UART + DMA
 */
class Remote
{
private:
    static constexpr int DMA_BUFF_SIZE = 50;       /// Size of DMA buffer used when requesting data.
    static constexpr int REMOTE_PACKET_SIZE = 18;  /// Length of the remote recieve buffer.
    static constexpr float STICK_MAX_VALUE = 660.0f;  /// Max value received by one of the sticks.
    static constexpr uint32_t RECEIVE_PERIOD = 14;    /// Time between message receivals, in ms

public:
#ifndef PLATFORM_HOSTED
    using RemoteDma = modm::platform::Usart1<
        modm::platform::Dma2::Stream7,
        modm::platform::DmaBase::ChannelSelection::CHANNEL_4,
        modm::platform::Dma2::Stream2,
        modm::platform::DmaBase::ChannelSelection::CHANNEL_4>;

    using RemoteDmaTxISRHandler =
        aruwlib::dma::DmaRXISR<REMOTE_PACKET_SIZE, RemoteDma, modm::platform::Dma2::Stream2>;
#else
    using RemoteDmaTxISRHandler = aruwlib::dma::DmaRXISRStub;
#endif

    Remote(Drivers *drivers) : drivers(drivers) {}
    DISALLOW_COPY_AND_ASSIGN(Remote)
    mockable ~Remote() = default;

    /**
     * Specifies a particular joystick.
     */
    enum class Channel
    {
        RIGHT_HORIZONTAL,
        RIGHT_VERTICAL,
        LEFT_HORIZONTAL,
        LEFT_VERTICAL
    };

    /**
     * Specifies a particular switch.
     */
    enum class Switch
    {
        LEFT_SWITCH,
        RIGHT_SWITCH
    };

    /**
     * Different switch orientations.
     */
    enum class SwitchState
    {
        UNKNOWN,
        UP,
        MID,
        DOWN
    };

    /**
     * A list of the particular keys to interact with, in bit order.
     */
    enum class Key
    {
        W = 0,
        S,
        A,
        D,
        SHIFT,
        CTRL,
        Q,
        E,
        R,
        F,
        G,
        Z,
        X,
        C,
        V,
        B
    };

    /**
     * Enables and initializes `Uart::Uart1` communication.
     */
    mockable void initialize();

    /**
     * Reads/parses the current buffer and updates the current remote info state
     * and `CommandMapper` state.
     */
    mockable void read();

    /**
     * @return `true` if the remote is connected, `false` otherwise.
     * @note A timer is used to determine if the remote is disconnected, so expect a
     *      second or so of delay from disconnecting the remote to this function saying
     *      the remote is disconnected.
     */
    mockable bool isConnected() const;

    /**
     * @return The value of the given channel, between [-1, 1].
     */
    mockable float getChannel(Channel ch) const;

    /**
     * @return The state of the given switch.
     */
    mockable SwitchState getSwitch(Switch sw) const;

    /**
     * @return The current mouse x value.
     */
    mockable int16_t getMouseX() const;

    /**
     * @return The current mouse y value.
     */
    mockable int16_t getMouseY() const;

    /**
     * @return The current mouse z value.
     */
    mockable int16_t getMouseZ() const;

    /**
     * @return The current mouse l value.
     */
    mockable bool getMouseL() const;

    /**
     * @return The current mouse r value.
     */
    mockable bool getMouseR() const;

    /**
     * @return `true` if the given `key` is pressed, `false` otherwise.
     */
    mockable bool keyPressed(Key key) const;

    /**
     * @return the value of the wheel, between `[-STICK_MAX_VALUE, STICK_MAX_VALUE]`.
     */
    mockable int16_t getWheel() const;

    /**
     * @return the number of times remote info has been received.
     */
    mockable uint32_t getUpdateCounter() const;

private:
    /**
     * The current remote information
     */
    struct RemoteInfo
    {
        uint32_t updateCounter = 0;
        int16_t rightHorizontal = 0;
        int16_t rightVertical = 0;
        int16_t leftHorizontal = 0;
        int16_t leftVertical = 0;
        SwitchState leftSwitch = SwitchState::UNKNOWN;
        SwitchState rightSwitch = SwitchState::UNKNOWN;
        struct
        {  /// Mouse information
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
            bool l = false;
            bool r = false;
        } mouse;
        uint16_t key = 0;   /// Keyboard information
        int16_t wheel = 0;  /// Remote wheel information
    };

    Drivers *drivers;

    RemoteInfo remote;

    uint8_t rxBuffer[DMA_BUFF_SIZE]{0};  /// UART recieve buffer.

    uint32_t lastRead = 0;  /// Timestamp when last byte was read (milliseconds).

    void parseBuffer();  /// Parses the current rxBuffer.

    void reset();  /// Resets the current remote info.
};

}  // namespace aruwlib

#endif  // REMOTE_HPP_

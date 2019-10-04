/*
 * Copyright (c) 2015-2018, Niklas Hauser
 * Copyright (c) 2017, Sascha Schade
 * Copyright (c) 2018, Antal Szabó
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_ROBOMASTER_DEV_BOARD_A_HPP
#define MODM_ROBOMASTER_DEV_BOARD_A_HPP

#include <modm/platform.hpp>
#include <modm/architecture/interface/clock.hpp>

using namespace modm::platform;

/// @ingroup TODO
namespace Board
{
    using namespace modm::literals;

/// STM32F427 running at 180MHz from the external 12MHz crystal
struct SystemClock
{
    static constexpr uint32_t Frequency = 180_MHz;
    static constexpr uint32_t Apb1 = Frequency / 4;
    static constexpr uint32_t Apb2 = Frequency / 2;

    static constexpr uint32_t Adc = Apb2;

    static constexpr uint32_t Spi1 = Apb2;
    static constexpr uint32_t Spi2 = Apb1;
    static constexpr uint32_t Spi3 = Apb1;
    static constexpr uint32_t Spi4 = Apb2;
    static constexpr uint32_t Spi5 = Apb2;
    static constexpr uint32_t Spi6 = Apb2;

    static constexpr uint32_t Usart1 = Apb2;
    static constexpr uint32_t Usart2 = Apb1;
    static constexpr uint32_t Usart3 = Apb1;
    static constexpr uint32_t Uart4  = Apb1;
    static constexpr uint32_t Uart5  = Apb1;
    static constexpr uint32_t Usart6 = Apb2;
    static constexpr uint32_t Uart7  = Apb1;
    static constexpr uint32_t Uart8  = Apb1;

    static constexpr uint32_t Can1 = Apb1;
    static constexpr uint32_t Can2 = Apb1;

    static constexpr uint32_t I2c1 = Apb1;
    static constexpr uint32_t I2c2 = Apb1;
    static constexpr uint32_t I2c3 = Apb1;

    static constexpr uint32_t Apb1Timer = Apb1;
    static constexpr uint32_t Apb2Timer = Apb2;
    static constexpr uint32_t Timer1  = Apb2Timer;
    static constexpr uint32_t Timer2  = Apb1Timer;
    static constexpr uint32_t Timer3  = Apb1Timer;
    static constexpr uint32_t Timer4  = Apb1Timer;
    static constexpr uint32_t Timer5  = Apb1Timer;
    static constexpr uint32_t Timer6  = Apb1Timer;
    static constexpr uint32_t Timer7  = Apb1Timer;
    static constexpr uint32_t Timer8  = Apb2Timer;
    static constexpr uint32_t Timer9  = Apb2Timer;
    static constexpr uint32_t Timer10 = Apb2Timer;
    static constexpr uint32_t Timer11 = Apb2Timer;
    static constexpr uint32_t Timer12 = Apb1Timer;
    static constexpr uint32_t Timer13 = Apb1Timer;
    static constexpr uint32_t Timer14 = Apb1Timer;

    static bool inline
    enable()
    {
        Rcc::enableExternalCrystal();  // 8 MHz
        Rcc::enablePll(
            Rcc::PllSource::ExternalCrystal,
            6,    // 12MHz / N=6 -> 2MHz
            180,  // 2MHz * M=180 -> 360MHz
            2     // 360MHz / P=2 -> 180MHz = F_cpu
        );

        Rcc::setFlashLatency<Frequency>();
        Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
        Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div2);
        Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div1);
        Rcc::updateCoreFrequency<Frequency>();

        return true;
    }
};


// initialize a button built into mcb
using Button = GpioInputB2;

// initialize 9 green Leds and 1 red LED
using Led1 = GpioOutputG1;
using Led2 = GpioOutputG2;
using Led3 = GpioOutputG3;
using Led4 = GpioOutputG4;
using Led5 = GpioOutputG5;
using Led6 = GpioOutputG6;
using Led7 = GpioOutputG7;
using Led8 = GpioOutputG8;
using Led9 = GpioOutputF14;
using LedRed = GpioOutputE11;

using Leds = SoftwareGpioPort
<
    Led1, Led2, Led3, Led4, Led5,
    Led6, Led7, Led8, Led9, LedRed
>;

// initialize 4 24V outputs
using PowerOut1 = GpioOutputH2;
using PowerOut2 = GpioOutputH3;
using PowerOut3 = GpioOutputH4;
using PowerOut4 = GpioOutputH5;

using PowerOuts = SoftwareGpioPort
<
    PowerOut1, PowerOut2,
    PowerOut3, PowerOut4
>;

// initilize 4 digital output pins
using DigitalOutPinS = GpioOutputA0;
using DigitalOutPinT = GpioOutputA1;
using DigitalOutPinU = GpioOutputA2;
using DigitalOutPinV = GpioOutputA3;

using DigitalOutPins = SoftwareGpioPort
<
    DigitalOutPinS, DigitalOutPinT,
    DigitalOutPinU, DigitalOutPinV
>;

// initiazlize 4 digital input pins
using DigitalInPinW = GpioInputI5;
using DigitalInPinX = GpioInputI6;
using DigitalInPinY = GpioInputI7;
using DigitalInPinZ = GpioInputI2;

using DigitalInPins = SoftwareGpioPort
<
    DigitalInPinW, DigitalInPinX,
    DigitalInPinY, DigitalInPinZ
>;

inline void
killAllGpioOutput()
{
    Leds::setOutput(modm::Gpio::High);
    PowerOuts::setOutput(modm::Gpio::Low);
    DigitalOutPins::setOutput(modm::Gpio::Low);
}

inline void
initialize()
{
    // init clock
    SystemClock::enable();
    SysTickTimer::initialize<SystemClock>();

    // init Leds
    Leds::setOutput(modm::Gpio::Low);
    // init 24V output
    PowerOuts::setOutput(modm::Gpio::High);
    // init digital out pins
    DigitalOutPins::setOutput(modm::Gpio::Low);
    // init digital in pins
    // interrupts disabled
    DigitalInPins::setInput();
    // init button on board
    Button::setInput();
}

}  // namespace Board

#endif  // MODM_ROBOMASTER_DEV_BOARD_A_HPP

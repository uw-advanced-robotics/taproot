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
	static constexpr uint32_t Apb1 = Frequency / 4; // this (Apb1) is obviously wrong, but uart2 doesn't work at 45 MHz and works fine with this at 90 MHz
	static constexpr uint32_t Apb2 = Frequency / 2; // I double checked the datasheet for uart2 and it appears to be 45 Mhz for apb1, so not sure what to do

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

	static constexpr uint32_t Apb1Timer = Apb1 * 2; // why is this * 2?
	static constexpr uint32_t Apb2Timer = Apb2 * 2;
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
		Rcc::enableExternalCrystal(); // 8 MHz
		Rcc::enablePll(
			Rcc::PllSource::ExternalCrystal,
			6,		// 12MHz / N=6 -> 2MHz
			180,	// 2MHz * M=180 -> 360MHz
			2		// 360MHz / P=2 -> 180MHz = F_cpu
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
using BUTTON = GpioInputB2;

// initialize 9 green LEDs and 1 red LED
using LED1 = GpioOutputG1;
using LED2 = GpioOutputG2;
using LED3 = GpioOutputG3;
using LED4 = GpioOutputG4;
using LED5 = GpioOutputG5;
using LED6 = GpioOutputG6;
using LED7 = GpioOutputG7;
using LED8 = GpioOutputG8;
using LED9 = GpioOutputF14;
using LED_RED = GpioOutputE11;

using LEDS = SoftwareGpioPort< LED1, LED2, LED3, LED4, LED5, LED6, LED7, LED8, LED9, LED_RED >;

// initialize 4 24V outputs
using POWER_OUT1 = GpioOutputH2;
using POWER_OUT2 = GpioOutputH3;
using POWER_OUT3 = GpioOutputH4;
using POWER_OUT4 = GpioOutputH5;

using POWER_OUTS = SoftwareGpioPort< POWER_OUT1, POWER_OUT2, POWER_OUT3, POWER_OUT4 >;

// initilize 4 digital output pins
using DIGITAL_OUT_PIN_S = GpioOutputA0;
using DIGITAL_OUT_PIN_T = GpioOutputA1;
using DIGITAL_OUT_PIN_U = GpioOutputA2;
using DIGITAL_OUT_PIN_V = GpioOutputA3;

using DIGITAL_OUT_PINS = SoftwareGpioPort< DIGITAL_OUT_PIN_S, DIGITAL_OUT_PIN_T, DIGITAL_OUT_PIN_U, DIGITAL_OUT_PIN_V >;

// initiazlize 4 digital input pins
using DIGITAL_IN_PIN_W = GpioInputI5;
using DIGITAL_IN_PIN_X = GpioInputI6;
using DIGITAL_IN_PIN_Y = GpioInputI7;
using DIGITAL_IN_PIN_Z = GpioInputI2;

using DIGITAL_IN_PINS = SoftwareGpioPort< DIGITAL_IN_PIN_W, DIGITAL_IN_PIN_X, DIGITAL_IN_PIN_Y, DIGITAL_IN_PIN_Z >;

// initialize 4 analog input pins
// TODO

/**
namespace l3g
{
using Int1 = GpioInputE0;	// MEMS_INT1 [L3GD20_INT1]: GPXTI0
using Int2 = GpioInputE1;	// MEMS_INT2 [L3GD20_DRDY/INT2]: GPXTI1

using Cs   = GpioOutputE3;	// CS_I2C/SPI [L3GD20_CS_I2C/SPI]
using Sck  = GpioOutputA5;	// SPI1_SCK [L3GD20_SCL/SPC]
using Mosi = GpioOutputA7;	// SPI1_MOSI [L3GD20_SDA/SDI/SDO]
using Miso = GpioInputA6;	// SPI1_MISO [L3GD20_SA0/SDO]

using SpiMaster = SpiMaster1;
}

namespace usb
{
using Dm = GpioOutputA11;		// OTG_FS_DM: USB_OTG_HS_DM
using Dp = GpioOutputA12;		// OTG_FS_DP: USB_OTG_HS_DP
using Id = GpioOutputA10;		// OTG_FS_ID: USB_OTG_HS_ID

//using Overcurrent = GpioC5;		// OTG_FS_OC [OTG_FS_OverCurrent]: GPXTI5
using Power = GpioOutputC0;		// OTG_FS_PSO [OTG_FS_PowerSwitchOn]
using VBus = GpioA9;			// VBUS_FS: USB_OTG_HS_VBUS
//using Device = UsbFs;
}
*/

inline void
killAllGpioOutput()
{
	LEDS::setOutput(modm::Gpio::High);
	POWER_OUTS::setOutput(modm::Gpio::Low);
	DIGITAL_OUT_PINS::setOutput(modm::Gpio::Low);
}

inline void
initialize()
{
	// init clock
	SystemClock::enable();
	SysTickTimer::initialize<SystemClock>();

	// init leds
	LEDS::setOutput(modm::Gpio::Low);
	// init 24V output
	POWER_OUTS::setOutput(modm::Gpio::High);
	// init digital out pins
	DIGITAL_OUT_PINS::setOutput(modm::Gpio::Low);
	// init digital in pins
	// interrupts disabled
	DIGITAL_IN_PINS::setInput();
	// init button on board
	BUTTON::setInput();
}

/*
inline void
initializeL3g()
{
	l3g::Int1::setInput();
	l3g::Int1::setInputTrigger(Gpio::InputTrigger::RisingEdge);
	l3g::Int1::enableExternalInterrupt();
//	l3g::Int1::enableExternalInterruptVector(12);

	l3g::Int2::setInput();
	l3g::Int2::setInputTrigger(Gpio::InputTrigger::RisingEdge);
	l3g::Int2::enableExternalInterrupt();
//	l3g::Int2::enableExternalInterruptVector(12);

	l3g::Cs::setOutput(modm::Gpio::High);

	l3g::SpiMaster::connect<l3g::Sck::Sck, l3g::Mosi::Mosi, l3g::Miso::Miso>();
	l3g::SpiMaster::initialize<SystemClock, 6.25_MHz>();
	l3g::SpiMaster::setDataMode(l3g::SpiMaster::DataMode::Mode3);
}
*/
}

#endif	// MODM_ROBOMASTER_DEV_BOARD_A_HPP

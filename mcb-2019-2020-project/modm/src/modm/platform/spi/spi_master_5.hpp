/*
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2011-2017, Niklas Hauser
 * Copyright (c) 2012, Georgi Grinshpun
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2014, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_SPI_MASTER5_HPP
#define MODM_STM32_SPI_MASTER5_HPP

#include <modm/architecture/interface/spi_master.hpp>
#include <modm/platform/gpio/connector.hpp>
#include "spi_hal_5.hpp"

namespace modm
{

namespace platform
{

/**
 * Serial peripheral interface (SPI5).
 *
 * Simple unbuffered implementation.
 *
 * @author	Niklas Hauser
 * @ingroup	modm_platform_spi modm_platform_spi_5
 */
class SpiMaster5 : public modm::SpiMaster
{
	static uint8_t state;
	static uint8_t count;
	static void *context;
	static ConfigurationHandler configuration;
public:
	using Hal = SpiHal5;

	/// Spi Data Mode, Mode0 is the most common mode
	enum class
	DataMode : uint32_t
	{
		Mode0 = 0b00,			///< clock normal,   sample on rising  edge
		Mode1 = SPI_CR1_CPHA,	///< clock normal,   sample on falling edge
		Mode2 = SPI_CR1_CPOL,	///< clock inverted, sample on falling  edge
		Mode3 = SPI_CR1_CPOL | SPI_CR1_CPHA
		///< clock inverted, sample on rising edge
	};

	/// Spi Data Order, MsbFirst is the most common mode
	enum class
	DataOrder : uint32_t
	{
		MsbFirst = 0b0,
		LsbFirst = SPI_CR1_LSBFIRST
	};

public:
	template< template<Peripheral _> class... Signals >
	static void
	connect()
	{
		using Connector = GpioConnector<Peripheral::Spi5, Signals...>;
		using Sck = typename Connector::template GetSignal<Gpio::Signal::Sck>;
		using Mosi = typename Connector::template GetSignal<Gpio::Signal::Mosi>;
		using Miso = typename Connector::template GetSignal<Gpio::Signal::Miso>;

		// Connector::disconnect();
		Sck::setOutput(Gpio::OutputType::PushPull);
		Mosi::setOutput(Gpio::OutputType::PushPull);
		Miso::setInput(Gpio::InputType::Floating);
		Connector::connect();
	}

	// start documentation inherited
	template< class SystemClock, baudrate_t baudrate, percent_t tolerance=pct(5) >
	static void
	initialize()
	{
		// calculate the nearest prescaler from the baudrate
		constexpr float desired = float(SystemClock::Spi5) / baudrate;
		constexpr uint16_t nearest = (
				(desired >= (128 * 4.f/3)) ? 256 : (
				(desired >= ( 64 * 4.f/3)) ? 128 : (
				(desired >= ( 32 * 4.f/3)) ?  64 : (
				(desired >= ( 16 * 4.f/3)) ?  32 : (
				(desired >= (  8 * 4.f/3)) ?  16 : (
				(desired >= (  4 * 4.f/3)) ?   8 : (
				(desired >= (  2 * 4.f/3)) ?   4 :
											   2
				)))))));

		// check if we found a prescaler which generates a baudrate within the tolerance
		assertBaudrateInTolerance<
				SystemClock::Spi5 / nearest,
				baudrate,
				tolerance >();

		// translate the prescaler into the bitmapping
		constexpr SpiHal5::Prescaler prescaler = (
				(nearest >= 256) ? SpiHal5::Prescaler::Div256 : (
				(nearest >= 128) ? SpiHal5::Prescaler::Div128 : (
				(nearest >=  64) ? SpiHal5::Prescaler::Div64  : (
				(nearest >=  32) ? SpiHal5::Prescaler::Div32  : (
				(nearest >=  16) ? SpiHal5::Prescaler::Div16  : (
				(nearest >=   8) ? SpiHal5::Prescaler::Div8   : (
				(nearest >=   4) ? SpiHal5::Prescaler::Div4   :
								   SpiHal5::Prescaler::Div2
				)))))));

		// initialize the Spi
		SpiHal5::initialize(prescaler);
		state = 0;
	}

	static modm_always_inline void
	setDataMode(DataMode mode)
	{
		SpiHal5::setDataMode(static_cast<SpiHal5::DataMode>(mode));
	}

	static modm_always_inline void
	setDataOrder(DataOrder order)
	{
		SpiHal5::setDataOrder(static_cast<SpiHal5::DataOrder>(order));
	}


	static uint8_t
	acquire(void *ctx, ConfigurationHandler handler = nullptr);

	static uint8_t
	release(void *ctx);


	static uint8_t
	transferBlocking(uint8_t data)
	{
		return RF_CALL_BLOCKING(transfer(data));
	}

	static void
	transferBlocking(uint8_t *tx, uint8_t *rx, std::size_t length)
	{
		RF_CALL_BLOCKING(transfer(tx, rx, length));
	}


	static modm::ResumableResult<uint8_t>
	transfer(uint8_t data);

	static modm::ResumableResult<void>
	transfer(uint8_t *tx, uint8_t *rx, std::size_t length);
	// end documentation inherited
};

} // namespace platform

} // namespace modm

#endif // MODM_STM32_SPI_MASTER5_HPP
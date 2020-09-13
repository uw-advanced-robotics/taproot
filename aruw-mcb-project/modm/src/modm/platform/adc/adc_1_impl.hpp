/*
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2014-2017, Niklas Hauser
 * Copyright (c) 2016, Fabian Greif
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_ADC1_HPP
#	error 	"Don't include this file directly, use 'adc_1.hpp' instead!"
#endif

#include <modm/platform/clock/rcc.hpp>

template< class SystemClock, modm::frequency_t frequency, modm::percent_t tolerance >
void
modm::platform::Adc1::initialize()
{
	constexpr float desired = float(SystemClock::Adc) / (frequency > 36000000 ? 36000000 : frequency);

	// respect the prescaler range of 2, 4, 6, 8
	constexpr uint8_t pre_ceil = (
			std::ceil(desired) > 6 ? 8 : (
			std::ceil(desired) > 4 ? 6 : (
			std::ceil(desired) > 2 ? 4 :
									 2
			)));
	constexpr uint8_t pre_floor = (
			std::floor(desired) < 4 ? 2 : (
			std::floor(desired) < 6 ? 4 : (
			std::floor(desired) < 8 ? 6 :
									  8
			)));

	// calculate the possible baudrates above and below the requested baudrate
	constexpr uint32_t baud_lower = SystemClock::Adc / pre_ceil;
	constexpr uint32_t baud_upper = SystemClock::Adc / pre_floor;

	// calculate the half-point between the upper and lower baudrate
	constexpr uint32_t baud_middle = (baud_upper + baud_lower) / 2;
	// decide which divisor is closer to a possible baudrate
	// lower baudrate means higher divisor!
	constexpr uint8_t pre = (frequency < baud_middle) ? pre_ceil : pre_floor;

	// check if within baudrate tolerance
	assertBaudrateInTolerance<
			SystemClock::Adc / pre,
			frequency,
			tolerance >();

	// translate the prescaler into the bitmapping
	constexpr Prescaler prescaler = (
			(pre >= 8) ? Prescaler::Div8 : (
			(pre >= 6) ? Prescaler::Div6 : (
			(pre >= 4) ? Prescaler::Div4 :
						 Prescaler::Div2
			)));

	Rcc::enable<Peripheral::Adc1>();
	ADC1->CR2 |= ADC_CR2_ADON;			// switch on ADC

	setPrescaler(prescaler);
}

void
modm::platform::Adc1::setPrescaler(const Prescaler prescaler)
{
	ADC->CCR = (ADC->CCR & ~(0b11 << 17)) | (uint32_t(prescaler) << 17);
}

void
modm::platform::Adc1::enableTemperatureRefVMeasurement()
{
	ADC->CCR |= ADC_CCR_TSVREFE;
}

void
modm::platform::Adc1::disableTemperatureRefVMeasurement()
{
	ADC->CCR &= ~ADC_CCR_TSVREFE;
}
void
modm::platform::Adc1::setLeftAdjustResult()
{
	ADC1->CR2 |= ADC_CR2_ALIGN;
}

void
modm::platform::Adc1::setRightAdjustResult()
{
	ADC1->CR2 &= ~ADC_CR2_ALIGN;
}

bool
modm::platform::Adc1::setChannel(const Channel channel,
									 const SampleTime sampleTime)
{
	if (uint32_t(channel) > 18) return false;
	// clear number of conversions in the sequence
	// and set number of conversions to 1
	ADC1->SQR1 = 0;
	ADC1->SQR2 = 0;
	ADC1->SQR3 = uint32_t(channel) & 0x1f;

	setSampleTime(channel, sampleTime);
	return true;
}

modm::platform::Adc1::Channel
modm::platform::Adc1::getChannel()
{
	return Channel(ADC1->SQR3 & 0x1f);
}

bool
modm::platform::Adc1::addChannel(const Channel channel,
									const SampleTime sampleTime)
{
	// read channel count
	uint8_t channel_count = (ADC1->SQR1 & ADC_SQR1_L) >> 20;
	++channel_count;
	if(channel_count > 0x0f) return false; // emergency exit
	// write channel number
	if(channel_count < 6) {
		ADC1->SQR3 |=
			(uint32_t(channel) & 0x1f) << (channel_count*5);
	} else 	if(channel_count < 12) {
		ADC1->SQR2 |=
			(uint32_t(channel) & 0x1f) << ((channel_count-6)*5);
	} else {
		ADC1->SQR1 |=
			(uint32_t(channel) & 0x1f) << ((channel_count-12)*5);
	}
	// update channel count
	ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_L) | (channel_count << 20);

	setSampleTime(channel, sampleTime);
	return true;
}

void
modm::platform::Adc1::setSampleTime(const Channel channel,
										const SampleTime sampleTime)
{
	if (uint32_t(channel) < 10) {
		ADC1->SMPR2 |= uint32_t(sampleTime)
								<< (uint32_t(channel) * 3);
	}
	else {
		ADC1->SMPR1 |= uint32_t(sampleTime)
								<< ((uint32_t(channel)-10) * 3);
	}
}

void
modm::platform::Adc1::enableFreeRunningMode()
{
	ADC1->CR2 |= ADC_CR2_CONT;	// set to continuous mode
}

void
modm::platform::Adc1::disableFreeRunningMode()
{
	ADC1->CR2 &= ~ADC_CR2_CONT;		// set to single mode
}

void
modm::platform::Adc1::disable()
{
	ADC1->CR2 &= ~(ADC_CR2_ADON);		// switch off ADC
	RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN; // stop ADC Clock
}

void
modm::platform::Adc1::startConversion()
{
	acknowledgeInterruptFlags(InterruptFlag::All);
	// starts single conversion for the regular group
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

bool
modm::platform::Adc1::isConversionFinished()
{
	return (ADC1->SR & ADC_SR_EOC);
}

uint16_t
modm::platform::Adc1::getValue()
{
	return ADC1->DR;
}


uint16_t
modm::platform::Adc1::readChannel(Channel channel)
{
	if (!setChannel(channel)) return 0;

	startConversion();
	// wait until the conversion is finished
	while (!isConversionFinished())
		;

	return getValue();
}

// ----------------------------------------------------------------------------
// TODO: move this to some shared header for all cortex m3 platforms
// Re-implemented here to save some code space. As all arguments in the calls
// below are constant the compiler is able to calculate everything at
// compile time.

#ifndef MODM_CUSTOM_NVIC_FUNCTIONS
#define MODM_CUSTOM_NVIC_FUNCTIONS

static modm_always_inline void
nvicEnableInterrupt(const IRQn_Type IRQn)
{
	NVIC->ISER[(uint32_t(IRQn) >> 5)] = (1 << (uint32_t(IRQn) & 0x1F));
}

static modm_always_inline void
nvicDisableInterrupt(IRQn_Type IRQn)
{
	NVIC_DisableIRQ(IRQn);
}

#endif // MODM_CUSTOM_NVIC_FUNCTIONS

void
modm::platform::Adc1::enableInterruptVector(const uint32_t priority,
												const bool enable)
{
	const IRQn_Type InterruptVector = ADC_IRQn;
	if (enable) {
		NVIC_SetPriority(InterruptVector, priority);
		nvicEnableInterrupt(InterruptVector);
	} else {
		NVIC_DisableIRQ(InterruptVector);
	}
}

void
modm::platform::Adc1::enableInterrupt(const Interrupt_t interrupt)
{
	ADC1->CR1 |= interrupt.value;
}

void
modm::platform::Adc1::disableInterrupt(const Interrupt_t interrupt)
{
	ADC1->CR1 &= ~interrupt.value;
}

modm::platform::Adc1::InterruptFlag_t
modm::platform::Adc1::getInterruptFlags()
{
	return InterruptFlag_t(ADC1->SR);
}

void
modm::platform::Adc1::acknowledgeInterruptFlags(const InterruptFlag_t flags)
{
	// Flags are cleared by writing a one to the flag position.
	// Writing a zero is ignored.
	ADC1->SR = flags.value;
}
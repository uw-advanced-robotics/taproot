/*
 * Copyright (c) 2015-2016, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "../device.hpp"
#include <modm/platform/clock/common.hpp>
#include "hardware_init.hpp"




extern "C"
{

void modm_fastcode
_delay_ns(uint16_t ns)
{
	// ns_per_loop = nanoseconds per cycle times cycles per loop (3 cycles)
	asm volatile (
		".syntax unified"       "\n\t"
		"muls.n	%2, %2, %1"     "\n\t"  // multiply the overhead cycles with the ns per cycle:  1-2 cycles on cm3, up to 32 cycles on cm0
		"subs.n	%0, %0, %2"     "\n\t"  // subtract the overhead in ns from the input:          1 cycle
	"1:  subs.n	%0, %0, %1"     "\n\t"  // subtract the ns per loop from the input:             1 cycle
		"bpl.n	1b"             "\n\t"  // keep doing that while result is still positive:      2 cycles (when taken)
	:: "r" (ns), "r" (modm::clock::ns_per_loop), "r" (8));
	// => loop is 3 cycles long
}

void modm_fastcode
_delay_us(uint16_t us)
{
	if (!us) return;    // 1 cycle, or 2 when taken

	uint32_t start = DWT->CYCCNT;
	// prefer this for cores with fast hardware multiplication
	int32_t delay = int32_t(modm::clock::fcpu_MHz) * us - 25;

	while (int32_t(DWT->CYCCNT - start) < delay)
		;
}

void modm_fastcode
_delay_ms(uint16_t ms)
{
	if (!ms) return;    // 1 cycle, or 2 when taken

	uint32_t start = DWT->CYCCNT;
	int32_t delay = int32_t(modm::clock::fcpu_kHz) * ms - 25;

	while (int32_t(DWT->CYCCNT - start) < delay)
		;
}

}

void
modm_dwt_enable(void)
{
	// Enable Tracing Debug Unit
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	// Enable CPU cycle counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

MODM_HARDWARE_INIT_ORDER(modm_dwt_enable, 100);

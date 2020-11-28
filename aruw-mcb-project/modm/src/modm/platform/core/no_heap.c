/*
 * Copyright (c) 2020, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <reent.h>
#include <modm/architecture/utils.hpp>

modm_weak
void __modm_initialize_memory(void)
{
	/* tumbleweed */
}

// ----------------------------------------------------------------------------
modm_weak modm_section(".Heap_is_not_implemented!__Please_include_the__modm:platform:heap__module_in_your_project!")
void* _sbrk_r(struct _reent *r, ptrdiff_t size)
{
	(void) r;
	(void) size;
	return NULL;
}
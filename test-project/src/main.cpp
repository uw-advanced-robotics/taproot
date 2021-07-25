/*
 * Copyright (c) 2020, Erik Henriksson
 * Copyright (c) 2020, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "tusb.h"

#include "aruwlib/rm-dev-board-a/board.hpp"
#include "modm/io.hpp"
#include "modm/processing.hpp"
#include "modm/platform.hpp"
#include "aruwlib/architecture/periodic_timer.hpp"

using namespace Board;
using namespace modm::literals;
using namespace modm::platform;

modm::IODeviceWrapper<UsbUart0, modm::IOBuffer::BlockIfFull> usb_io_device;
modm::IOStream usb_stream(usb_io_device);

aruwlib::arch::PeriodicMilliTimer tmr{2500};

// Invoked when device is mounted
void tud_mount_cb() { tmr.restart(1000); }
// Invoked when device is unmounted
void tud_umount_cb() { tmr.restart(250); }
// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool) { tmr.restart(2500); }
// Invoked when usb bus is resumed
void tud_resume_cb() { tmr.restart(1000); }

namespace usb
{
// using Vbus = GpioA9;
using Id = GpioA10;
using Dm = GpioA11;
using Dp = GpioA12;

using Overcurrent = GpioInputG7;	// OTG_FS_OverCurrent
using Power = GpioOutputG6;			// OTG_FS_PowerSwitchOn

using Device = modm::platform::UsbFs;
}

int main()
{
	Board::initialize();

	usb::Device::initialize<SystemClock>();
	usb::Device::connect<usb::Dm::Dm, usb::Dp::Dp, usb::Id::Id>();

	usb::Overcurrent::setInput();
	// usb::Vbus::setInput();
	// Force device mode
	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
	modm::delay_ms(25);
	// Enable VBUS sense (B device) via pin PA9
	// USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBDEN;

	tusb_init();

	uint8_t counter{0};
	while (true)
	{
		tud_task();

		if (tmr.execute())
		{
			Board::LedsPort::toggle();
			usb_stream << "Hello World from USB: " << (counter++) << modm::endl;
		}
	}

	return 0;
}
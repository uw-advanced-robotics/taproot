/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "Drivers.hpp"

namespace aruwlib
{
can::Can Drivers::can;
can::CanRxHandler Drivers::canRxHandler;
gpio::Analog Drivers::analog;
gpio::Digital Drivers::digital;
gpio::Leds Drivers::leds;
gpio::Pwm Drivers::pwm;
Remote Drivers::remote;
sensors::Mpu6500 Drivers::mpu6500;
serial::Uart Drivers::uart;
serial::XavierSerial Drivers::xavierSerial;
serial::RefSerial Drivers::refSerial;
control::CommandScheduler Drivers::commandScheduler;
control::ControlOperatorInterface Drivers::controlOperatorInterface;
control::CommandMapper Drivers::commandMapper;
errors::ErrorController Drivers::errorController;
motor::DjiMotorTxHandler Drivers::djiMotorTxHandler;

#ifdef ENV_SIMULATOR
void Drivers::reset()
{
    resetCan();
    resetCanRxHandler();
    resetAnalog();
    resetDigital();
    resetLeds();
    resetPwm();
    resetRemote();
    resetMpu6500();
    resetUart();
    resetXavierSerial();
    resetRefSerial();
    resetCommandScheduler();
    resetControlOperatorInterface();
    resetCommandMapper();
    resetErrorController();
    resetDjiMotorTxHandler();
}

void Drivers::resetCan() {}
void Drivers::resetCanRxHandler() {}
void Drivers::resetAnalog() {}
void Drivers::resetDigital() {}
void Drivers::resetLeds() {}
void Drivers::resetPwm() {}
void Drivers::resetRemote() {}
void Drivers::resetMpu6500() {}
void Drivers::resetUart() {}
void Drivers::resetXavierSerial() {}
void Drivers::resetRefSerial() {}
void Drivers::resetCommandScheduler() {}
void Drivers::resetControlOperatorInterface() {}
void Drivers::resetCommandMapper() {}
void Drivers::resetErrorController() {}
void Drivers::resetDjiMotorTxHandler() {}
#endif
}  // namespace aruwlib

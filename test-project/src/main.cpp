/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/drivers.hpp"

class Drivers : public tap::Drivers
{
};

class DriversSingleton
{
public:
    static ::Drivers drivers;
};

::Drivers DriversSingleton::drivers;

int main()
{
    Board::initialize();

    auto &dr(DriversSingleton::drivers);

    dr.analog.init();
    dr.can.initialize();
    dr.digital.init();
    dr.leds.init();
    dr.pwm.init();
    dr.refSerial.initialize();
    dr.remote.initialize();
    dr.terminalSerial.initialize();
    dr.schedulerTerminalHandler.init();
    dr.errorController.init();
    dr.djiMotorTerminalSerialHandler.init();

    tap::arch::PeriodicMilliTimer timer(2);

    int test{0};

    if (test == 0)
    {
        while (true)
        {
            // test digital in/out (read in digital input and output to digital out, use external
            // stimuli to test).
            bool a13(Board::DigitalInPinPA_13::read());
            Board::DigitalOutPinPC_14::set(a13);

            if (timer.execute())
            {
                dr.terminalSerial.hijack() << a13 << modm::endl;
            }

            modm::delay_us(10);
        }
    }
    else if (test == 1)
    {
        // test remote
        tap::arch::PeriodicMilliTimer timer(2);

        while (true)
        {
            dr.remote.read();

            if (timer.execute())
            {
                auto ch(dr.remote.getChannel(
                    tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL));
                dr.terminalSerial.hijack() << ch << modm::endl;
            }

            modm::delay_us(10);
        }
    }
    else if (test == 2)
    {
        // test referee serial
        tap::arch::PeriodicMilliTimer timer(2);

        while (true)
        {
            dr.refSerial.updateSerial();

            if (timer.execute())
            {
                auto health(dr.refSerial.getRobotData().currentHp);
                dr.terminalSerial.hijack() << health << modm::endl;
            }
        }
    }
    else if (test == 3)
    {
        // test terminal serial
        tap::arch::PeriodicMilliTimer timer(2);

        while (true)
        {
            if (timer.execute())
            {
                dr.terminalSerial.update();
            }
        }
    }
    else if (test == 4)
    {
        // test analog
        while (true)
        {
            dr.terminalSerial.hijack() << dr.analog.read(tap::gpio::Analog::PA_0) << modm::endl;

            modm::delay_ms(10);
        }
    }
    else if (test == 5)
    {
        // test can
        modm::can::Message msg(0x201, 8);
        msg.data[0] = 0xa5;

        while (true)
        {
            if (dr.can.isReadyToSend(tap::can::CanBus::CAN_BUS1))
            {
                dr.can.sendMessage(tap::can::CanBus::CAN_BUS1, msg);

                modm::can::Message rxMsg;
                if (dr.can.getMessage(tap::can::CanBus::CAN_BUS1, &rxMsg))
                {
                    dr.terminalSerial.hijack() << rxMsg.data[0] << modm::endl;
                }
            }

            modm::delay_ms(10);
        }
    }
    else if (test == 6)
    {
        // test pwm
        dr.pwm.write(0.5, tap::gpio::Pwm::PC_6);

        while (true)
        {
            modm::delay_ms(1);
        }
    }
    else if (test == 7)
    {
        // test motor control

        tap::motor::DjiMotor motor(
            &dr,
            tap::motor::MOTOR1,
            tap::can::CanBus::CAN_BUS1,
            false,
            "motor");

        motor.initialize();

        tap::algorithms::SmoothPidConfig cfg{
            .kp = 20.0f,
            .maxOutput = 10'000,
        };
        tap::algorithms::SmoothPid pid(cfg);

        while (true)
        {
            dr.canRxHandler.pollCanData();

            if (timer.execute())
            {
                // run velocity pid
                float targetRpm(1000);
                float actualRpm(motor.getShaftRPM());
                float error(targetRpm - actualRpm);
                float out(pid.runControllerDerivateError(error, 0.002f));
                motor.setDesiredOutput(out);

                dr.djiMotorTxHandler.encodeAndSendCanData();
            }

            modm::delay_us(10);
        }
    }

    return 0;
}

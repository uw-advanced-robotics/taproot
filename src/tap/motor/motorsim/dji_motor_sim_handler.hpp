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

#ifndef TAPROOT_DJI_MOTOR_SIM_HANDLER_HPP_
#define TAPROOT_DJI_MOTOR_SIM_HANDLER_HPP_

#ifdef PLATFORM_HOSTED

#include <map>
#include <memory>

#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor_tx_handler.hpp"

#include "motor_sim.hpp"

namespace tap::motor::motorsim
{
class DjiMotorSimHandler
{
public:
    static DjiMotorSimHandler* getInstance()
    {
        static DjiMotorSimHandler* handler = new DjiMotorSimHandler;
        return handler;
    }

    /**
     * Reset output stream for DjiMotorSimHandler as well as all of the MotorSim objects.
     */
    void resetMotorSims();

    /**
     * Registers a new MotorSim object for the given motor type
     * that will respond at the given position on the given CAN bus.
     *
     * Default torque load for this function is 0 N*m.
     */
    void registerSim(
        std::shared_ptr<MotorSim> motorSim,
        std::tuple<can::CanBus, motor::MotorId> canBusAndMotorId);

    /**
     * Allows the DjiMotorSimHandler to receive a given CAN message
     * and stream input values to the motor sims.
     * Returns true if data is processed (it always should be).
     */
    bool parseMotorMessage(tap::can::CanBus bus, const modm::can::Message& message);

    /**
     * Fills the given pointer with a new motor sim feedback message.
     * Returns true if successful (it always should be unless no motor sims have been registers).
     */
    bool encodeMessage(tap::can::CanBus bus, modm::can::Message* message);

    /// Updates all MotorSim objects (position, RPM, time values).
    void updateSims();

private:
    std::map<std::tuple<can::CanBus, MotorId>, std::shared_ptr<MotorSim>> motorIDToMotorSimMap;

    std::map<can::CanBus, MotorId> nextMotorIdToEncode;

    bool getNextMotorId(can::CanBus bus);
};
}  // namespace tap::motor::motorsim

#endif  // PLATFORM_HOSTED

#endif  // TAPROOT_DJI_MOTOR_SIM_HANDLER_HPP_

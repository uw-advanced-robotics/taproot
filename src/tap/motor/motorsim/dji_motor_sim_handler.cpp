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

#ifdef PLATFORM_HOSTED

#include "dji_motor_sim_handler.hpp"

#include <cassert>

#include "modm/architecture/interface/can_message.hpp"

#include "can_serializer.hpp"

using namespace tap::can;

namespace tap::motor::motorsim
{
void DjiMotorSimHandler::resetMotorSims()
{
    nextMotorIdToEncode[can::CanBus::CAN_BUS1] = MotorId::MOTOR1;
    nextMotorIdToEncode[can::CanBus::CAN_BUS2] = MotorId::MOTOR1;

    for (auto& it : motorIDToMotorSimMap)
    {
        it.second->reset();
    }
}

void DjiMotorSimHandler::registerSim(
    std::shared_ptr<MotorSim> motorSim,
    std::tuple<can::CanBus, motor::MotorId> canBusAndMotorId)
{
    assert(motorIDToMotorSimMap.find(canBusAndMotorId) == motorIDToMotorSimMap.end());

    motorIDToMotorSimMap[canBusAndMotorId] = motorSim;
}

static bool motorIDAssociatedWithCommandID(MotorId mid, uint32_t cmdId)
{
    return mid < MotorId::MOTOR5 ? cmdId == DjiMotorTxHandler::CAN_DJI_LOW_IDENTIFIER
                                 : cmdId == DjiMotorTxHandler::CAN_DJI_HIGH_IDENTIFIER;
}

bool DjiMotorSimHandler::parseMotorMessage(CanBus bus, const modm::can::Message& message)
{
    std::array<int16_t, 4> newInputs = CanSerializer::parseMessage(&message);

    bool found = false;

    for (auto& it : motorIDToMotorSimMap)
    {
        auto canBus = std::get<0>(it.first);
        auto mid = std::get<1>(it.first);
        auto normalizedID = DJI_MOTOR_TO_NORMALIZED_ID(mid);

        if (canBus == bus && motorIDAssociatedWithCommandID(mid, message.identifier))
        {
            it.second->setMotorInput(newInputs[normalizedID % 4]);
            found = true;
        }
    }

    return found;
}

bool DjiMotorSimHandler::encodeMessage(CanBus bus, modm::can::Message* message)
{
    if (message == nullptr) return false;

    bool foundNext = getNextMotorId(bus);

    if (!foundNext) return false;

    auto id = nextMotorIdToEncode[bus];
    auto busAndMotorID = std::tuple<can::CanBus, MotorId>(bus, id);

    assert(motorIDToMotorSimMap.find(busAndMotorID) != motorIDToMotorSimMap.end());

    const auto motorsim = motorIDToMotorSimMap[busAndMotorID];

    *message = CanSerializer::serializeFeedback(
        motorsim->getEnc(),
        motorsim->getRPM(),
        motorsim->getInput(),
        id);

    return true;
}

void DjiMotorSimHandler::updateSims()
{
    for (auto& it : motorIDToMotorSimMap)
    {
        it.second->update();
    }
}

bool DjiMotorSimHandler::getNextMotorId(can::CanBus bus)
{
    for (int i = 0; i < DjiMotorTxHandler::DJI_MOTORS_PER_CAN; i++)
    {
        auto nextId = (i + nextMotorIdToEncode[bus]) % DjiMotorTxHandler::DJI_MOTORS_PER_CAN;
        auto djiMotorId = NORMALIZED_ID_TO_DJI_MOTOR(nextId);
        auto key = std::tuple<can::CanBus, MotorId>(bus, djiMotorId);

        if (motorIDToMotorSimMap.find(key) != motorIDToMotorSimMap.end())
        {
            nextMotorIdToEncode[bus] = djiMotorId;
            return true;
        }
    }

    return false;
}

}  // namespace tap::motor::motorsim

#endif  // PLATFORM_HOSTED

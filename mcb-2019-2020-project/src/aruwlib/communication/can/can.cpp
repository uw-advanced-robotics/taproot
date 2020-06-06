#include "can.hpp"

#include <modm/platform.hpp>

#include <aruwlib/rm-dev-board-a/board.hpp>

#ifndef ENV_SIMULATOR
using namespace modm::platform;
#endif
using namespace modm::literals;

void aruwlib::can::Can::initialize() {
    #ifndef ENV_SIMULATOR
    CanFilter::setStartFilterBankForCan2(14);
    // initialize CAN 1
    Can1::connect<GpioD0::Rx, GpioD1::Tx>(Gpio::InputType::PullUp);
    Can1::initialize<Board::SystemClock, 1000_kbps>(9);
    // receive every message for CAN 1
    CanFilter::setFilter(0, CanFilter::FIFO0,
        CanFilter::StandardIdentifier(0),
        CanFilter::StandardFilterMask(0));
    Can2::connect<GpioB12::Rx, GpioB13::Tx>(Gpio::InputType::PullUp);
    Can2::initialize<Board::SystemClock, 1000_kbps>(12);
    // receive every message for CAN 2
    CanFilter::setFilter(14, CanFilter::FIFO0,
        CanFilter::StandardIdentifier(0),
        CanFilter::StandardFilterMask(0));
    #endif
}


bool aruwlib::can::Can::isMessageAvailable(aruwlib::can::CanBus bus) const {
    #ifdef ENV_SIMULATOR
    return false;
    #else
    switch (bus) {
        case CanBus::CAN_BUS1:
            return Can1::isMessageAvailable();
        case CanBus::CAN_BUS2:
            return Can2::isMessageAvailable();
        default:
            return false;
    }
    #endif
}

bool aruwlib::can::Can::getMessage(aruwlib::can::CanBus bus, modm::can::Message *message) {
    #ifdef ENV_SIMULATOR
    return false;
    #else
    switch (bus) {
        case CanBus::CAN_BUS1:
            return Can1::getMessage(*message);
        case CanBus::CAN_BUS2:
            return Can2::getMessage(*message);
        default:
            return false;
    }
    #endif
}

bool aruwlib::can::Can::isReadyToSend(CanBus bus) const {
    #ifdef ENV_SIMULATOR
    return false;
    #else
    switch (bus) {
        case CanBus::CAN_BUS1:
            return Can1::isReadyToSend();
        case CanBus::CAN_BUS2:
            return Can2::isReadyToSend();
        default:
            return false;
    }
    #endif
}

bool aruwlib::can::Can::sendMessage(CanBus bus, const modm::can::Message& message) {
    #ifdef ENV_SIMULATOR
    return false;
    #else
    switch (bus) {
        case CanBus::CAN_BUS1:
            return Can1::sendMessage(message);
        case CanBus::CAN_BUS2:
            return Can2::sendMessage(message);
        default:
            return false;
    }
    #endif
}

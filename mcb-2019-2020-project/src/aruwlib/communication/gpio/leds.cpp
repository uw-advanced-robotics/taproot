#include "leds.hpp"

#include "aruwlib/rm-dev-board-a/board.hpp"

using namespace Board;

namespace aruwlib
{
namespace gpio
{
void Leds::init()
{
#ifndef ENV_SIMULATOR
    // init Leds
    LedsPort::setOutput(modm::Gpio::Low);
#endif
}

void Leds::set(Leds::LedPin pin, bool isSet)
{
#ifndef ENV_SIMULATOR
    switch (pin)
    {
        case Leds::LedPin::A:
            LedA::set(isSet);
            break;

        case Leds::LedPin::B:
            LedB::set(isSet);
            break;

        case Leds::LedPin::C:
            LedC::set(isSet);
            break;

        case Leds::LedPin::D:
            LedD::set(isSet);
            break;

        case Leds::LedPin::E:
            LedE::set(isSet);
            break;

        case Leds::LedPin::F:
            LedF::set(isSet);
            break;

        case Leds::LedPin::G:
            LedG::set(isSet);
            break;

        case Leds::LedPin::H:
            LedH::set(isSet);
            break;

        case Leds::LedPin::Green:
            LedGreen::set(isSet);
            break;

        case Leds::LedPin::Red:
            LedRed::set(isSet);
            break;
    }
#endif
}
}  // namespace gpio

}  // namespace aruwlib

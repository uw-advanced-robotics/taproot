#include "digital.hpp"

#include "aruwlib/rm-dev-board-a/board.hpp"

using namespace Board;

namespace aruwlib
{
namespace gpio
{
void Digital::init()
{
#ifndef ENV_SIMULATOR
    // init digital out pins
    DigitalOutPins::setOutput(modm::Gpio::Low);
    // init digital in pins
    // interrupts disabled
    DigitalInPins::setInput();
#endif
}

void Digital::configureInputPullMode(Digital::InputPin pin, Digital::InputPullMode mode)
{
#ifndef ENV_SIMULATOR
    switch (pin)
    {
        case Digital::InputPin::A:
            DigitalInPinA::configure(mode);
            break;

        case Digital::InputPin::B:
            DigitalInPinB::configure(mode);
            break;

        case Digital::InputPin::C:
            DigitalInPinC::configure(mode);
            break;

        case Digital::InputPin::D:
            DigitalInPinD::configure(mode);
            break;
    }
#endif
}

void Digital::set(Digital::OutputPin pin, bool isSet)
{
#ifndef ENV_SIMULATOR
    switch (pin)
    {
        case Digital::OutputPin::E:
            DigitalOutPinE::set(isSet);
            break;

        case Digital::OutputPin::F:
            DigitalOutPinF::set(isSet);
            break;

        case Digital::OutputPin::G:
            DigitalOutPinG::set(isSet);
            break;

        case Digital::OutputPin::H:
            DigitalOutPinH::set(isSet);
            break;
    }
#endif
}

bool Digital::read(Digital::InputPin pin) const
{
#ifdef ENV_SIMULATOR
    return false;
#else
    switch (pin)
    {
        case Digital::InputPin::A:
            return DigitalInPinA::read();

        case Digital::InputPin::B:
            return DigitalInPinB::read();

        case Digital::InputPin::C:
            return DigitalInPinC::read();

        case Digital::InputPin::D:
            return DigitalInPinD::read();
    }
#endif
}
}  // namespace gpio

}  // namespace aruwlib

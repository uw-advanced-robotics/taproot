#include "analog.hpp"

#include "aruwlib/rm-dev-board-a/board.hpp"

using namespace Board;

namespace aruwlib
{

namespace gpio
{
    void Analog::init()
    {
        #ifndef ENV_SIMULATOR
        AnalogInPins::setAnalogInput();

        // Initial ADC/Timer setup
        Adc1::connect<AnalogInPinS::In0, AnalogInPinT::In1,
                    AnalogInPinU::In2, AnalogInPinV::In3>();
        Adc1::initialize<SystemClock, 22500000_Bd>();
        Adc1::setPinChannel<AnalogInPinS>();
        Adc1::setPinChannel<AnalogInPinT>();
        Adc1::setPinChannel<AnalogInPinU>();
        Adc1::setPinChannel<AnalogInPinV>();
        #endif
    }

    uint16_t Analog::read(Pin pin) {
        #ifdef ENV_SIMULATOR
        return 0;
        #else
        switch(pin) {
            case Pin::S :
                return Adc1::readChannel(Adc1::getPinChannel<AnalogInPinS>());

            case Pin::T :
                return Adc1::readChannel(Adc1::getPinChannel<AnalogInPinT>());

            case Pin::U :
                return Adc1::readChannel(Adc1::getPinChannel<AnalogInPinU>());

            case Pin::V :
                return Adc1::readChannel(Adc1::getPinChannel<AnalogInPinV>());

            default:
                return 0;
        }
        #endif
    }
}  // namespace gpio

}  // namespace aruwlib

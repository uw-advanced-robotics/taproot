#include <rm-dev-board-a/board.hpp>
#include "analog.hpp"

using namespace Board;

namespace aruwlib
{

namespace gpio
{
    void Analog::init()
    {
        // Initial ADC/Timer setup
        Adc1::connect<AnalogInPinS::In0, AnalogInPinT::In1,
                    AnalogInPinU::In2, AnalogInPinV::In3>();
        Adc1::initialize<SystemClock, 22500000_Bd>();
        Adc1::setPinChannel<AnalogInPinS>();
        Adc1::setPinChannel<AnalogInPinT>();
        Adc1::setPinChannel<AnalogInPinU>();
        Adc1::setPinChannel<AnalogInPinV>();
    }

    /*
    * Reads voltage across the specified pin. Units in mV.
    */
    uint16_t Analog::Read(Pin pin) {
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
    }
}  // namespace gpio

}  // namespace aruwlib

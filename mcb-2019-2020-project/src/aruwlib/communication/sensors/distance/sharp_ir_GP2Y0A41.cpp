#include "sharp_ir_GP2Y0A41.hpp"

namespace aruwlib {

namespace sensors {
    // Constructor to init Sharp IR analog pin
    // Uses preset values for boundary and distance conversion
    SharpIrGP2Y0A41::SharpIrGP2Y0A41(gpio::Analog::Pin pin) :
        AnalogDistanceSensor(SHARP_IR_MIN, SHARP_IR_MAX,
        SHARP_IR_M, SHARP_IR_B, SHARP_IR_OFFSET, pin) {}
}  // namespace sensors

}  // namespace aruwlib

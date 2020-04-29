// Subsystem for Y-Axis Mechanism

#include "xaxis_subsystem.hpp"

#include <aruwlib/communication/gpio/digital.hpp>

namespace aruwsrc
{

namespace engineer
{
    void XAxisSubsystem::refresh() {
        // for refresh, keep calling set(isExtended);
        aruwlib::gpio::Digital::set(aruwlib::gpio::Digital::OutputPin::E, isExtended);
    }

    void XAxisSubsystem::setXAxisExtended(bool isExtended) {
        this->isExtended = isExtended;
        aruwlib::gpio::Digital::set(aruwlib::gpio::Digital::OutputPin::E, isExtended);
    }

    bool XAxisSubsystem::getIsExtended() {
        return this->isExtended;
    }
}  // namespace engineer

}  // namespace aruwsrc

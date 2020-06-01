#include "xaxis_subsystem.hpp"

#include <aruwlib/communication/gpio/digital.hpp>

namespace aruwsrc
{

namespace engineer
{
    void XAxisSubsystem::setExtended(bool isExtended) {
        aruwlib::gpio::Digital::set(pin, extended);
        extended = isExtended;
    }

    bool XAxisSubsystem::isExtended() const {
        return extended;
    }
}  // namespace engineer

}  // namespace aruwsrc

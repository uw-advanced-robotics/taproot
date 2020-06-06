#include "xaxis_subsystem.hpp"

#include <aruwlib/Drivers.hpp>

using aruwlib::Drivers;

namespace aruwsrc
{

namespace engineer
{
    void XAxisSubsystem::setExtended(bool isExtended) {
        aruwlib::Drivers::digital.set(pin, extended);
        extended = isExtended;
    }

    bool XAxisSubsystem::isExtended() const {
        return extended;
    }
}  // namespace engineer

}  // namespace aruwsrc

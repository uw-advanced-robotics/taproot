// Subsystem for Y-Axis Mechanism

#include "xaxis_subsystem.hpp"

#include <aruwlib/rm-dev-board-a/board.hpp>

namespace aruwsrc
{

namespace engineer
{
    void XAxisSubsystem::refresh() {
        // for refresh, keep calling set(isExtended);
        xAxisDigitalOutPin::set(isExtended);
    }

    void XAxisSubsystem::setXAxisExtended(bool isExtended) {
        this->isExtended = isExtended;
        xAxisDigitalOutPin::set(isExtended);
    }

    bool XAxisSubsystem::getIsExtended() {
        return this->isExtended;
    }
}  // namespace engineer

}  // namespace aruwsrc

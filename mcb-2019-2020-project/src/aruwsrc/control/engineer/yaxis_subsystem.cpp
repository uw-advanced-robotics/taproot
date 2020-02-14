// Subsystem for Y-Axis Mechanism

#include "yaxis_subsystem.hpp"

#include <rm-dev-board-a/board.hpp>

namespace aruwsrc
{

namespace engineer
{
    void YAxisSubsystem::refresh() {
        // for refresh, keep calling set(isExtended);
        yAxisDigitalOutPin::set(isExtended);
    }

    void YAxisSubsystem::setYAxisExtended(bool isExtended) {
        this->isExtended = isExtended;
        yAxisDigitalOutPin::set(isExtended);
    }

    bool YAxisSubsystem::getIsExtended() {
        return this->isExtended;
    }
}  // namespace engineer

}  // namespace aruwsrc

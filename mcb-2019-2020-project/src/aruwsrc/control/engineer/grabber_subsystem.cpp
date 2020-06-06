#include "grabber_subsystem.hpp"

#include <aruwlib/Drivers.hpp>

namespace aruwsrc
{

namespace engineer
{
void GrabberSubsystem::setSqueezed(bool isGrabberSqueezed) {
    aruwlib::Drivers::digital.set(pin, isGrabberSqueezed);
    this->isGrabberSqueezed = isGrabberSqueezed;
}

bool GrabberSubsystem::isSqueezed() const {
    return isGrabberSqueezed;
}
}  // namespace engineer

}  // namespace aruwsrc

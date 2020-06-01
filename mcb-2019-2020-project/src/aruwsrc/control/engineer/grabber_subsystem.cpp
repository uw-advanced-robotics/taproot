#include "grabber_subsystem.hpp"

namespace aruwsrc
{

namespace engineer
{
void GrabberSubsystem::setSqueezed(bool isGrabberSqueezed) {
    aruwlib::gpio::Digital::set(pin, isGrabberSqueezed);
    this->isGrabberSqueezed = isGrabberSqueezed;
}

bool GrabberSubsystem::isSqueezed() const {
    return isGrabberSqueezed;
}
}  // namespace engineer

}  // namespace aruwsrc

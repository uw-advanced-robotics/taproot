#include "hopper_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
void HopperSubsystem::setOpen() { hopper.setTargetPwm(hopper.getMaxPWM()); }

void HopperSubsystem::setClose() { hopper.setTargetPwm(hopper.getMinPWM()); }

void HopperSubsystem::refresh() { hopper.updateSendPwmRamp(); }

float HopperSubsystem::getOpenPWM() { return hopper.getMaxPWM(); }

float HopperSubsystem::getClosePWM() { return hopper.getMinPWM(); }
}  // namespace control

}  // namespace aruwsrc

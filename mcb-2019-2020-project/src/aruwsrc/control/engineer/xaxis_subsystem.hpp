#ifndef XAXIS_SUBSYSTEM_HPP_
#define XAXIS_SUBSYSTEM_HPP_

#include <aruwlib/communication/gpio/digital.hpp>
#include <aruwlib/control/subsystem.hpp>

namespace aruwsrc
{

namespace engineer
{

/**
 * This is a subsystem code for x-axis movement (moving the
 * grabber back and forward). Connect this to a digital output
 * pin. This controls a solenoid, which actuates a piston.
 */
class XAxisSubsystem : public aruwlib::control::Subsystem
{
 public:
    explicit XAxisSubsystem(aruwlib::gpio::Digital::OutputPin pin)
        : pin(pin), extended(false) {}

    void setExtended(bool isExtended);

    bool isExtended() const;

 private:
    aruwlib::gpio::Digital::OutputPin pin;

    bool extended;
};  // class XAxisSubsystem

}  // namespace engineer

}  // namespace aruwsrc

#endif  // XAXIS_SUBSYSTEM_HPP_

#ifndef __PERIODIC_TIMER_HPP__
#define __PERIODIC_TIMER_HPP__

#include "aruwlib/architecture/timeout.hpp"

namespace aruwlib
{
namespace arch
{
class PeriodicMilliTimer
{
public:
    PeriodicMilliTimer();

    explicit PeriodicMilliTimer(uint32_t period);

    void restart();
    void restart(uint32_t period);

    void stop();

    bool execute();

    bool isStopped() const;

private:
    uint32_t period;
    aruwlib::arch::MilliTimeout timeout;
};

}  // namespace arch
}  // namespace aruwlib
#endif

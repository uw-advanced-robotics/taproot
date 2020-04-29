#include "periodic_timer.hpp"

#include "aruwlib/architecture/clock.hpp"

using namespace aruwlib::arch;

PeriodicMilliTimer::PeriodicMilliTimer() : period(0)
{
}

PeriodicMilliTimer::PeriodicMilliTimer(uint32_t period) : period(period), timeout(period)
{
}

void PeriodicMilliTimer::restart()
{
    timeout.restart(period);
}

void PeriodicMilliTimer::restart(uint32_t period)
{
    this->period = period;
    restart();
}

void PeriodicMilliTimer::stop()
{
    timeout.stop();
}

bool PeriodicMilliTimer::isStopped() const
{
    return timeout.isStopped();
}

bool PeriodicMilliTimer::execute()
{
    if (timeout.execute())
    {
        uint32_t now = clock::getTimeMilliseconds();

        do
        {
            timeout.expireTime += period;
        } while(timeout.expireTime <= now);

        timeout.isRunning = true;
        return true;
    }
    return false;
}

#include "timeout.hpp"

#include <aruwlib/architecture/clock.hpp>

namespace aruwlib
{
namespace arch
{
MilliTimeout::MilliTimeout()
{
    stop();
    this->expireTime = 0;
}

MilliTimeout::MilliTimeout(uint32_t timeout_millis) { restart(timeout_millis); }

void MilliTimeout::restart(uint32_t timeout_millis)
{
    this->isRunning = true;
    this->isExecuted = false;
    this->expireTime = clock::getTimeMilliseconds() + timeout_millis;
}

void MilliTimeout::stop()
{
    this->isRunning = false;
    this->isExecuted = false;
}

bool MilliTimeout::isExpired() const
{
    return this->isRunning && clock::getTimeMilliseconds() >= this->expireTime;
}

bool MilliTimeout::isStopped() const { return !this->isRunning; }

bool MilliTimeout::execute()
{
    if (!isExecuted && isExpired())
    {
        isExecuted = true;
        return true;
    }

    return false;
}
}  // namespace arch
}  // namespace aruwlib

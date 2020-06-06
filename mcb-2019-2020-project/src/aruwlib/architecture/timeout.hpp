#ifndef TIMEOUT_HPP
#define TIMEOUT_HPP

#include <stdint.h>

namespace aruwlib
{
namespace arch
{
class MilliTimeout
{
    friend class PeriodicMilliTimer;

private:
    bool isRunning;
    bool isExecuted;
    uint32_t expireTime;

public:
    MilliTimeout();
    explicit MilliTimeout(uint32_t timeout_millis);

    void restart(uint32_t timeout_millis);

    void stop();

    bool isStopped() const;

    bool isExpired() const;

    bool execute();
};

}  // namespace arch
}  // namespace aruwlib

#endif

#include "DriversSingleton.hpp"

namespace aruwlib
{
/**
 * Class that allows one to construct a Drivers instance because of frienship
 * with the Drivers class.
 */
class DriversSingleton
{
public:
    static Drivers drivers;
};  // class DriversSingleton

Drivers DriversSingleton::drivers;

Drivers *DoNotUse_getDrivers() { return &DriversSingleton::drivers; }
}  // namespace aruwlib

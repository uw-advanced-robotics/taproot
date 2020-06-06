#ifndef ERROR_CONTROLLER_HPP
#define ERROR_CONTROLLER_HPP

#include <aruwlib/architecture/timeout.hpp>
#include <modm/container.hpp>

#include "system_error.hpp"

namespace aruwlib
{
namespace errors
{
/**
 * Protocol description:
 * The 8 leds on the mcb are used to indicate a location and error type. LEDs A-D transmit
 * the error location and E-H the error type.
 * For location, the LSB is D, and for error type, the LSB is H
 * The other green led (next to the red led) comes on
 * when you have added an invalid error. The red led is always on (not used). Default, leds
 * A-H are always off if no errors are detected
 */
class ErrorController
{
public:
    ErrorController() : prevLedErrorChangeWait(ERROR_ROTATE_TIME) {}
    ErrorController(const ErrorController&) = delete;
    ErrorController& operator=(const ErrorController&) = default;

    void addToErrorList(SystemError error);

    void update();

private:
    static const int ERROR_ROTATE_TIME = 5000;

    static const unsigned ERROR_LIST_MAX_SIZE = 16;

    modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE> errorList;

    aruwlib::arch::MilliTimeout prevLedErrorChangeWait;

    int currentDisplayIndex = 0;

    bool getLedErrorCodeBits(Location location, ErrorType errorType, uint8_t* number);

    void setLedError(uint8_t binaryRep);

    void ledSwitch(uint8_t ledOnBoard, bool displayOnBoard);
};

}  // namespace errors

}  // namespace aruwlib

#endif

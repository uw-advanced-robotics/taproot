#ifndef ERROR_CONTROLLER_HPP
#define ERROR_CONTROLLER_HPP

#include <modm/container.hpp>
#include <modm/processing/timer.hpp>
#include <rm-dev-board-a/board.hpp>
#include "system_error.hpp"

namespace aruwlib
{

namespace errors
{

class ErrorController
{
 public:
    ErrorController() : prevLedErrorChangeWait(ERROR_ROTATE_TIME)
    {}

    void addToErrorList(const SystemError error);

    void removeCurrentDisplayedError(void);

    void update();

 private:
    const int ERROR_ROTATE_TIME = 2000;

    const unsigned ERROR_LIST_MAX_SIZE = 5;

    modm::LinkedList <SystemError> errorList;

    modm::ShortTimeout prevLedErrorChangeWait;

    bool getBinaryNumber(Location location, ErrorType errorType, uint8_t* number);

    void setLedError(uint8_t binaryRep);

    void ledSwitch(uint8_t ledLocation, bool display);
};

}  // namespace errors

}  // namespace aruwlib

#endif

#include <modm/container/linked_list.hpp>
#include "src/errors/error_controller.hpp"

// Overall method to use when receiving errors

namespace aruwlib
{
namespace errors
{
    // add an error to list of errors
    void ErrorController::addToErrorList(const SystemError error) {
        // only add error if it is not already added
        for (SystemError sysErr : errorList)
        {
            if (
                sysErr.getErrorType() == error.getErrorType()
                && sysErr.getLocation() == error.getLocation()
            ) {
                return;
            }
        }
        if (errorList.getSize() <= ERROR_LIST_MAX_SIZE)
        {
            errorList.append(error);
        }
    }

    void ErrorController::removeCurrentDisplayedError()
    {
        if (errorList.getSize() != 0) {
            errorList.removeFront();
        }
    }

    // Blink the list of errors in a loop on the board
    void ErrorController::update() {
        if (errorList.getSize() == 0) {
            setLedError(0);
            Board::LedGreen::setOutput(modm::Gpio::High);
            return;
        }

        // change error every ERROR_ROTATE_TIME time increment
        if (prevLedErrorChangeWait.execute()) {
            prevLedErrorChangeWait.restart(ERROR_ROTATE_TIME);
            SystemError currDisplayError = errorList.getFront();
            errorList.removeFront();
            errorList.append(currDisplayError);
        }

        uint8_t displayNum = 0;
        if (getBinaryNumber(errorList.getFront().getLocation(),
            errorList.getFront().getErrorType(), &displayNum)
        ) {
            setLedError(displayNum);
            Board::LedGreen::setOutput(modm::Gpio::High);
        } else {
            setLedError(0);
            Board::LedGreen::setOutput(modm::Gpio::Low);
        }
    }

    bool ErrorController::getBinaryNumber(Location location, ErrorType errorType, uint8_t* number) {
        // Limit location and error type
        // Check to make sure they are within bounds

        // find the bit mask for the location
        uint8_t locationMask = static_cast<uint8_t>(~(0xffu << ERROR_LOCATION_SIZE));
        uint8_t errorTypeMask = static_cast<uint8_t>(~(0xffu << ERROR_TYPE_SIZE));

        uint8_t locationMasked = static_cast<uint8_t>(location) & locationMask;
        uint8_t errorTypeMasked = static_cast<uint8_t>(errorType) & errorTypeMask;

        // set another error if this error is outside of the range of the error handler
        if (locationMasked != location || errorTypeMasked != errorType) {
            return false;
        }

        // Combine location and error type
        *number = location << ERROR_LOCATION_SIZE | errorType;
        return true;
    }

    void ErrorController::setLedError(uint8_t binaryRep) {
        // Mask number and determine if it is a 0 or a 1
        // If it is a 1, the LED corresponding will blink
        for (int i = 0; i < 8; i++) {
            bool display = (binaryRep >> i) & 1;
            ledSwitch(i, display);
        }
    }

    void ErrorController::ledSwitch(uint8_t ledLocation, bool display) {
        switch(ledLocation) {
            case 0: Board::LedA::setOutput(!display);
                break;
            case 1: Board::LedB::setOutput(!display);
                break;
            case 2: Board::LedC::setOutput(!display);
                break;
            case 3: Board::LedD::setOutput(!display);
                break;
            case 4: Board::LedE::setOutput(!display);
                break;
            case 5: Board::LedF::setOutput(!display);
                break;
            case 6: Board::LedG::setOutput(!display);
                break;
            case 7: Board::LedH::setOutput(!display);
                break;
        }
    }
}  // namespace errors

}  // namespace aruwlib



#include "uart.hpp"

#include "aruwlib/rm-dev-board-a/board.hpp"

using namespace Board;

namespace aruwlib
{

namespace serial
{

    bool Uart::read(UartPort port, uint8_t *data) {
        #ifdef ENV_SIMULATOR
        return false;
        #else
        switch (port) {
            case UartPort::Uart1:
                return Usart1::read(*data);
            case UartPort::Uart2:
                return Usart2::read(*data);
            case UartPort::Uart6:
                return Usart6::read(*data);
            default:
                return false;
        }
        #endif
    }

    std::size_t Uart::read(UartPort port, uint8_t *data, std::size_t length) {
        #ifdef ENV_SIMULATOR
        return 0;
        #else
        switch (port) {
            case UartPort::Uart1:
                return Usart1::read(data, length);
            case UartPort::Uart2:
                return Usart2::read(data, length);
            case UartPort::Uart6:
                return Usart6::read(data, length);
            default:
                return 0;
        }
        #endif
    }

    std::size_t Uart::discardReceiveBuffer(UartPort port) {
        #ifdef ENV_SIMULATOR
        return 0;
        #else
        switch (port) {
            case UartPort::Uart1:
                return Usart1::discardReceiveBuffer();
            case UartPort::Uart2:
                return Usart2::discardReceiveBuffer();
            case UartPort::Uart6:
                return Usart6::discardReceiveBuffer();
            default:
                return 0;
        }
        #endif
    }

    bool Uart::write(UartPort port, uint8_t data) {
        #ifdef ENV_SIMULATOR
        return false;
        #else
        switch (port) {
            case UartPort::Uart1:
                return Usart1::write(data);
            case UartPort::Uart2:
                return Usart2::write(data);
            case UartPort::Uart6:
                return Usart6::write(data);
            default:
                return false;
        }
        #endif
    }

    std::size_t Uart::write(UartPort port, const uint8_t *data, std::size_t length) {
        #ifdef ENV_SIMULATOR
        return 0;
        #else
        switch (port) {
            case UartPort::Uart1:
                return Usart1::write(data, length);
            case UartPort::Uart2:
                return Usart2::write(data, length);
            case UartPort::Uart6:
                return Usart6::write(data, length);
            default:
                return 0;
        }
        #endif
    }

    bool Uart::isWriteFinished(UartPort port) const {
        #ifdef ENV_SIMULATOR
        return false;
        #else
        switch (port) {
            case UartPort::Uart1:
                return Usart1::isWriteFinished();
            case UartPort::Uart2:
                return Usart2::isWriteFinished();
            case UartPort::Uart6:
                return Usart6::isWriteFinished();
            default:
                return false;
        }
        #endif
    }

}  // namespace serial

}  // namespace aruwlib

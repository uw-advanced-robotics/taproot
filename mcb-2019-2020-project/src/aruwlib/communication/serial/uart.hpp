#ifndef UART_HPP
#define UART_HPP

#include <stdint.h>

#ifndef ENV_SIMULATOR
#include <modm/platform.hpp>
#include <modm/platform/uart/uart_base.hpp>
#endif

#include "aruwlib/rm-dev-board-a/board.hpp"

namespace aruwlib
{

namespace serial
{

class Uart {
 public:
    enum UartPort
    {
        Uart1, Uart2, Uart6
    };

    #ifdef ENV_SIMULATOR
    enum Parity {
        Disabled, Even, Odd
    };
    #else
    using Parity = modm::platform::UartBase::Parity;
    #endif

    template<UartPort port, modm::baudrate_t baudrate, Parity parity = Parity::Disabled>
    static void init()
    {
        #ifndef ENV_SIMULATOR
        // TODO(kaelin): move pin definition to Board?
        if constexpr (port == UartPort::Uart1) {
            // TODO(kaelin): what's the TX pin on UART1?
            modm::platform::Usart1::connect<GpioB7::Rx>();
            modm::platform::Usart1::initialize<Board::SystemClock, baudrate>(12, parity);
        // NOLINTNEXTLINE
        } else if constexpr (port == UartPort::Uart2) {
            modm::platform::Usart2::connect<GpioD5::Tx, GpioD6::Rx>();
            modm::platform::Usart2::initialize<Board::SystemClock, baudrate>(12, parity);
        // NOLINTNEXTLINE
        } else if constexpr (port == UartPort::Uart6) {
            modm::platform::Usart6::connect<GpioG14::Tx, GpioG9::Rx>();
            modm::platform::Usart6::initialize<Board::SystemClock, baudrate>(12, parity);
        }
        #endif
    }

    static bool read(UartPort port, uint8_t *data);
    static std::size_t read(UartPort port, uint8_t *data, std::size_t length);

    static std::size_t discardReceiveBuffer(UartPort port);

    static bool write(UartPort port, uint8_t data);
    static std::size_t write(UartPort port, const uint8_t *data, std::size_t length);

    static bool isWriteFinished(UartPort port);
};

}  // namespace serial

}  // namespace aruwlib

#endif

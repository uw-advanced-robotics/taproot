#ifndef UART_HPP
#define UART_HPP

#include <cstdint>
#include <cstdlib>

#ifndef ENV_SIMULATOR
#include <modm/platform.hpp>
#include <modm/platform/uart/uart_base.hpp>
#endif

#include "aruwlib/rm-dev-board-a/board.hpp"

namespace aruwlib
{
namespace serial
{
/**
 * Class that wraps modm's Uart implementation.
 *
 * Currently only wraps the uart ports that we are generating modm
 * code for. If additional `UartPort`'s are added, they must be added
 * to this wrapper class here.
 */
class Uart
{
public:
    enum UartPort
    {
        Uart1,
        Uart2,
        Uart6
    };

#ifdef ENV_SIMULATOR
    enum Parity
    {
        Disabled,
        Even,
        Odd
    };
#else
    using Parity = modm::platform::UartBase::Parity;
#endif

    Uart() = default;
    Uart(const Uart &) = delete;
    Uart &operator=(const Uart &) = default;

    /**
     * .initializes a particular Uart with the pins particular to the RoboMaster type a board.
     *
     * @note follow covention in the functino when adding a `UartPort`.
     * @tparam port the particular port to initialize.
     * @tparam baudrate desired baud rate in Hz.
     * @tparam parity @see `Parity`.
     */
    template <UartPort port, modm::baudrate_t baudrate, Parity parity = Parity::Disabled>
    void init()
    {
#ifndef ENV_SIMULATOR
        // TODO(kaelin): move pin definition to Board?
        if constexpr (port == UartPort::Uart1)
        {
            // TODO(kaelin): what's the TX pin on UART1?
            modm::platform::Usart1::connect<GpioB7::Rx>();
            modm::platform::Usart1::initialize<Board::SystemClock, baudrate>(12, parity);
        }
        else if constexpr (port == UartPort::Uart2)
        {
            modm::platform::Usart2::connect<GpioD5::Tx, GpioD6::Rx>();
            modm::platform::Usart2::initialize<Board::SystemClock, baudrate>(12, parity);
        }
        else if constexpr (port == UartPort::Uart6)
        {
            modm::platform::Usart6::connect<GpioG14::Tx, GpioG9::Rx>();
            modm::platform::Usart6::initialize<Board::SystemClock, baudrate>(12, parity);
        }
#endif
    }

    /**
     * Read a single byte.
     *
     * @param[in] port the port to read from.
     * @param[out] data Byte read, if any.
     *
     * @return `true` if a byte was received, `false` otherwise.
     */
    bool read(UartPort port, uint8_t *data);

    /**
     * Read a block of bytes.
     *
     * @param[in] port the port to read from.
     * @param[out] data pointer to a buffer big enough to store `length` bytes
     * @param[in] length number of bytes to be read.
     *
     * @return number of bytes which could be read, maximal `length`.
     */
    std::size_t read(UartPort port, uint8_t *data, std::size_t length);

    /**
     * Empty the receive FIFO queue and hardware buffer.
     *
     * @param[in] port the port's buffer to discard.
     * @return the size of the deleted FIFO queue.
     */
    std::size_t discardReceiveBuffer(UartPort port);

    /**
     * Pushes a single byte into the buffer.
     *
     * @param[in] port the port to write to.
     * @return `true` if data has been successfully sent, `false` if buffer is full.
     * @note this writing is buffered.
     */
    bool write(UartPort port, uint8_t data);

    /**
     * Pushes a block of bytes into the buffer.
     *
     * @param[in] port the port to write to.
     * @param[in] data pointer to a buffer big enough to store `length` bytes.
     * @param[in] length number of bytes to be written.
     * @return the number of bytes that have been written.
     * @note this writing may be buffered.
     */
    std::size_t write(UartPort port, const uint8_t *data, std::size_t length);

    /**
     * Because the data is buffered, check here to see if the buffer is empty
     * (implying everything has been written).
     *
     * @param[in] port the port to see if writing is finished.
     * @return `true` if the buffer is empty and the last byte has been sent.
     */
    bool isWriteFinished(UartPort port) const;
};

}  // namespace serial

}  // namespace aruwlib

#endif

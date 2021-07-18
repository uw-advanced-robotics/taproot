#ifndef DMA_UART_RX_IRQHANDLER_HPP_
#define DMA_UART_RX_IRQHANDLER_HPP_

#include <cinttypes>
#include <cstring>

#include <modm/architecture/interface/atomic_lock.hpp>

namespace aruwlib::dma
{
#ifndef PLATFORM_HOSTED

template <int RX_BUFF_SIZE, class UART, class DMA_STREAM>
class DmaRXISR
{
public:
    static void initializeRX()
    {
        UART::read(buffer1, RX_BUFF_SIZE);
        buffer1InUse = true;
    }

    static void dmaRXISR()
    {
        if ((USART1->SR & USART_SR_IDLE) && (USART1->CR1 & USART_CR1_IDLEIE))
        {
            buffer1InUse = !buffer1InUse;
            newDataAvailable = true;
            UART::clearIdleFlag();
            UART::read(buffer1InUse ? buffer1 : buffer2, RX_BUFF_SIZE);
            DMA_STREAM::setDataLength(RX_BUFF_SIZE);
            DMA_STREAM::enable();
        }
    }

    static bool readBuffer(uint8_t *bufferLocation)
    {
        modm::atomic::Lock lock;
        if (newDataAvailable)
        {
            // Use buffer that isn't in use
            uint8_t *buffer = buffer1InUse ? buffer2 : buffer1;
            memcpy(bufferLocation, buffer, RX_BUFF_SIZE);
            newDataAvailable = false;
            return true;
        }
        return false;
    }

private:
    static inline volatile bool buffer1InUse{false};
    static inline volatile bool newDataAvailable{false};
    static uint8_t buffer1[RX_BUFF_SIZE];
    static uint8_t buffer2[RX_BUFF_SIZE];
};

template <int RX_BUFF_SIZE, class UART, class DMA_STREAM>
uint8_t DmaRXISR<RX_BUFF_SIZE, UART, DMA_STREAM>::buffer1[RX_BUFF_SIZE];

template <int RX_BUFF_SIZE, class UART, class DMA_STREAM>
uint8_t DmaRXISR<RX_BUFF_SIZE, UART, DMA_STREAM>::buffer2[RX_BUFF_SIZE];

#else

class DmaRXISRStub
{
public:
    static void initializeRX() {}
    static void dmaRXISR() {}
    static bool readBuffer(uint8_t *) { return false; }
};

#endif

}  // namespace aruwlib::dma

#endif  // DMA_UART_RX_IRQHANDLER_HPP_

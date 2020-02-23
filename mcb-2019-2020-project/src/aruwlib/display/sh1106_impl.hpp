#ifndef SH1106_HPP
    #error "Don't include this file directly, use 'sh1106.hpp' instead!"
#endif

#include "sh1106_defines.hpp"

template <typename SPI, typename A0, typename Reset,
    unsigned int Width, unsigned int Height, bool Flipped>
void aruwlib::display::Sh1106<SPI, A0, Reset, Width, Height, Flipped>::update()
{
    for(uint8_t y = 0; y < (Height / 8); ++y)
    {
        // command mode
        a0.reset();
        spi.transferBlocking(SH1106_PAGE_ADDRESS | y);  // Row select
        spi.transferBlocking(SH1106_COL_ADDRESS_MSB);  // Column select high

        if (Flipped) {
            spi.transferBlocking(SH1106_COL_ADDRESS_LSB | 4);  // Column select low
        } else {
            spi.transferBlocking(SH1106_COL_ADDRESS_LSB);  // Column select low
        }

        // switch to data mode
        a0.set();
        for(uint8_t x = 0; x < Width; ++x) {
            spi.transferBlocking(this->display_buffer[x][y]);
        }
    }
}

template <typename SPI, typename A0, typename Reset,
    unsigned int Width, unsigned int Height, bool Flipped>
void aruwlib::display::Sh1106<SPI, A0, Reset, Width, Height, Flipped>::setInvert(bool invert)
{
    a0.reset();

    if (invert) {
        spi.transferBlocking(SH1106_REVERSE);
    } else {
        spi.transferBlocking(SH1106_NORMAL);
    }
}

// ----------------------------------------------------------------------------
template <typename SPI, typename A0, typename Reset,
    unsigned int Width, unsigned int Height, bool Flipped>
void aruwlib::display::Sh1106<SPI, A0, Reset, Width, Height, Flipped>::initializeBlocking()
{
    a0.setOutput();
    reset.setOutput();

    // reset the controller
    reset.reset();
    modm::delayMilliseconds(20);
    reset.set();

    a0.reset();

    // View direction
    if (Flipped) {
        spi.transferBlocking(SH1106_ADC_NORMAL);
        spi.transferBlocking(SH1106_SCAN_DIR_NORMAL);
    } else {
        spi.transferBlocking(SH1106_ADC_REVERSE);
        spi.transferBlocking(SH1106_SCAN_DIR_REVERSE);
    }

    spi.transferBlocking(SH1106_ON);

    this->clear();
    this->update();
}

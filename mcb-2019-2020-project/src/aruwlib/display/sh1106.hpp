#ifndef SH1106_HPP
#define SH1106_HPP

#include <modm/architecture/interface/accessor_flash.hpp>
#include <modm/architecture/interface/delay.hpp>

#include <modm/ui/display/monochrome_graphic_display_buffered_vertical.hpp>

namespace aruwlib
{

namespace display
{
/**
 * Driver for SH1106 based OLED displays
 */
template <typename SPI, typename A0, typename Reset,
            unsigned int Width, unsigned int Height, bool Flipped>
class Sh1106 : public modm::MonochromeGraphicDisplayBufferedVertical<Width, Height>
{
 public:
    virtual ~Sh1106()
    {
    }

    void initializeBlocking();

    /**
     * Update the display with the content of the RAM buffer
     */
    virtual void
    update();

    // Invert the display content
    void
    setInvert(bool invert);

 protected:
    SPI spi;
    A0 a0;
    Reset reset;
};

}  // namespace display
}  // namespace aruwlib

#include "sh1106_impl.hpp"

#endif  // SH1106_HPP

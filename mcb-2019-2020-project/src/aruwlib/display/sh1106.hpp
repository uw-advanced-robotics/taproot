#ifndef SH1106_HPP
#define SH1106_HPP

#ifndef ENV_SIMULATOR
#include <modm/architecture/interface/accessor_flash.hpp>
#include <modm/architecture/interface/delay.hpp>
#endif


#include <modm/ui/display/monochrome_graphic_display_buffered_vertical.hpp>

namespace aruwlib
{

namespace display
{
/**
 * Driver for SH1106 based OLED displays
 */
template <
#ifndef ENV_SIMULATOR
            typename SPI, typename A0, typename Reset,
#endif
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
#ifndef ENV_SIMULATOR
    SPI spi;
    A0 a0;
    Reset reset;
#endif
};

}  // namespace display
}  // namespace aruwlib

#ifdef ENV_SIMULATOR
#include "sh1106_mock_impl.hpp"
#else
#include "sh1106_impl.hpp"
#endif

#endif  // SH1106_HPP

#ifndef SH1106_HPP
#error "Don't include this file directly, use 'sh1106.hpp' instead!"
#endif

template <unsigned int Width, unsigned int Height, bool Flipped>
void aruwlib::display::Sh1106<Width, Height, Flipped>::update()
{
    // no-op
}

template <unsigned int Width, unsigned int Height, bool Flipped>
void aruwlib::display::Sh1106<Width, Height, Flipped>::setInvert(bool invert)
{
    // no-op
}

// ----------------------------------------------------------------------------
template <unsigned int Width, unsigned int Height, bool Flipped>
void aruwlib::display::Sh1106<Width, Height, Flipped>::initializeBlocking()
{
    this->clear();
    this->update();
}

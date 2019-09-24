#include <rm-dev-board-a/board.hpp>

int main()
{
    Board::initialize();

    while (1)
    {
        Board::LEDS::toggle();
        modm::delayMilliseconds(1000);
    }
    return 0;
}

#include <modm/architecture/interface/spi_device.hpp>

template <class SpiMaster, class Cs>
class SdCardSpi : public modm::SpiDevice<SpiMaster, Cs>
{
public:
	SdCardSpi();

	modm::ResumableResult<bool> initialize();
	modm::ResumableResult<bool> readBlock(uint32_t blockAddr, uint8_t* buffer);

private:
	modm::ResumableResult<bool> sendCommand(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t& response);
	bool waitForDataToken(uint8_t token);

	uint8_t inBuffer[1];
	uint8_t outBuffer[6];
};

template <class SpiMaster, class Cs>
SdCardSpi<SpiMaster, Cs>::SdCardSpi()
{
    this->attachConfigurationHandler([]() {
        SpiMaster::setDataMode(SpiMaster::DataMode::Mode0);
        SpiMaster::setDataOrder(SpiMaster::DataOrder::MsbFirst);
    });
    Cs::setOutput(modm::Gpio::High);
}

template <class SpiMaster, class Cs>
modm::ResumableResult<bool>
SdCardSpi<SpiMaster, Cs>::sendCommand(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t& response)
{
	RF_BEGIN();

	RF_WAIT_UNTIL(this->acquireMaster());
	Cs::reset();

	outBuffer[0] = 0x40 | cmd;
	outBuffer[1] = (arg >> 24) & 0xFF;
	outBuffer[2] = (arg >> 16) & 0xFF;
	outBuffer[3] = (arg >> 8) & 0xFF;
	outBuffer[4] = arg & 0xFF;
	outBuffer[5] = crc;

	for (int i = 0; i < 6; i++) {
		SpiMaster::transfer(&outBuffer[i], &inBuffer[0], 1);
	}

	for (int i = 0; i < 10; i++) {
		SpiMaster::transfer(nullptr, &response, 1);
		if (!(response & 0x80)) break;
	}

	Cs::set();
	this->releaseMaster();

	RF_RETURN(!(response & 0x80));
	RF_END_RETURN(false);
}

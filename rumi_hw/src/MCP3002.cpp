#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rumi_hw/MCP3002.hpp"


constexpr uint32_t SPEED_HZ = 1'200'000;

MCP3002::MCP3002(int bus, int device, float voltage)
    : voltage(voltage)
{
	auto fileName = "/dev/spidev" + std::to_string(bus) + "." + std::to_string(device);
	deviceFile = open(fileName.c_str(), O_RDWR);
	if (deviceFile < 0)
	{
		throw std::runtime_error("MCP3002: Can't open device.");
	}
	if (-1 == ioctl(deviceFile, SPI_IOC_WR_MAX_SPEED_HZ, &SPEED_HZ))
	{
		throw std::runtime_error("MCP3002: Can't set max speed hz.");
	}
}

MCP3002::~MCP3002()
{
	close(deviceFile);
}

float MCP3002::get()
{
	// 0 - alignment
	// 1 - start bit
	// 1 - (SGL/DIFF) single mode
	// 0 - (ODD/SIGN): channel 0
	// 0 - (MSBF/LSBF) LSB first
	uint8_t tx[] = {0b01100000, 0};
	uint8_t rx[sizeof(tx)];

	transfer(tx, rx, sizeof(tx));

	uint16_t value_10bit = (rx[0] << 8) + rx[1];
	return voltage * static_cast<float>(value_10bit) / (1 << 10);
}

void MCP3002::transfer(const void* tx, void* rx, uint32_t len) const
{
	spi_ioc_transfer message = {
	        .tx_buf = (uintptr_t) tx,
	        .rx_buf = (uintptr_t) rx,
	        .len = len,
	        .speed_hz = SPEED_HZ,
	        .delay_usecs = 0,
	        .bits_per_word = 8,
	};

	if (-1 == ioctl(deviceFile, SPI_IOC_MESSAGE(1), &message))
	{
		throw std::runtime_error("MCP3002: Can't send spi message.");
	}
}

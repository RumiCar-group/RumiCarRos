#pragma once

#include <cstdint>

/**
 * ADC board
 * Provides 10 bits digital output basing on input and max voltages.
 */
class MCP3002
{
    float voltage;
    int deviceFile;

public:
    /**
     * @param bus SPI bus
     * @param device SPI device
     * @param voltage max voltage
     */
    MCP3002(int bus, int device, float voltage = 5.f);
    ~MCP3002();

    /**
     * @return input voltage
     */
    float get();

private:
    void transfer(const void* tx, void* rx, uint32_t len) const;
};

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

struct gpiod_chip;
struct gpiod_line;

/**
 * Simple GPIO control based on gpiod.
 */
class RumiGpio final
{
    std::string path;
    gpiod_chip* chip;
    std::unordered_map<uint, gpiod_line*> lines;

public:
    /// Opens the device.
    explicit RumiGpio(std::string devPath = "/dev/gpiochip0");

    /// Releases all pins and closes the device.
    ~RumiGpio();

    /// Toggles OUT pin on / off. Implicitly acquires pin, if necessary.
    bool togglePin(uint offset, bool on);

    /// Acquire pin - other applications won't access it.
    void acquirePin(uint offset);

    /// Release pin.
    void releasePin(uint offset);
};

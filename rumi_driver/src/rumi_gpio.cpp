#include <gpiod.h>
#include <error.h>
#include <stdlib.h>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include <rumi_gpio.hpp>

gpiod_line_request_config outConfig = {
    .consumer = "blink",
    .request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT,
    .flags = 0,
};

RumiGpio::RumiGpio(std::string devPath)
        : path(move(devPath))
        , chip(gpiod_chip_open(path.data()))
{
    if (path.size() == 0)
    {
        throw std::runtime_error("Wrong gpio path: " + path);
    }

    if (!chip)
    {
        throw std::runtime_error("Failed to open: " + path);
    }
}

RumiGpio::~RumiGpio()
{
    for (auto& offsetLine : lines) {
        gpiod_line_release(offsetLine.second);
    }
    gpiod_chip_close(chip);
}

bool RumiGpio::togglePin(uint offset, bool on)
{
    if (lines.find(offset) == lines.end())
    {
        acquirePin(offset);
    }
    return 0 == gpiod_line_set_value(lines[offset], on ? 1 : 0);
}

void RumiGpio::acquirePin(uint offset)
{
    gpiod_line* line = gpiod_chip_get_line(chip, offset);
    if (!line)
    {
        throw std::runtime_error("Failed to get pin: " + offset);
    }

    if (int error = gpiod_line_request(line, &outConfig, 0))
    {
        throw std::runtime_error("Failed to request pin " + offset);
    }
    lines.emplace(offset, line);
}

void RumiGpio::releasePin(uint offset)
{
    auto it = lines.find(offset);
    if (it == lines.end()) return;

    gpiod_line_release(it->second);
    lines.erase(it);
}

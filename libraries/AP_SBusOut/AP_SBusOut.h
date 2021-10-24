/*
 * AP_SBusOut.h
 *
 *  Created on: Aug 19, 2017
 *      Author: Mark Whitehorn
 */

#pragma once

#include <AP_Param/AP_Param.h>

class AP_SerialDevice;

class AP_SBusOut {
public:
    AP_SBusOut();

    /* Do not allow copies */
    AP_SBusOut(const AP_SBusOut &other) = delete;
    AP_SBusOut &operator=(const AP_SBusOut&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    void update();

    // public format function for use by IOMCU
    static void sbus_format_frame(uint16_t *channels, uint8_t num_channels, uint8_t buffer[25]);

private:

    AP_SerialDevice *sbus1_uart;

    void init(void);

    uint16_t sbus_frame_interval;   // microseconds

    AP_Int16 sbus_rate;
    bool initialised;
};

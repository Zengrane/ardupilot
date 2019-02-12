#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_ESP32/AP_HAL_ESP32.h>
#include <AP_HAL_ESP32/Semaphores.h>

#include "driver/uart.h"


class ESP32::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(uint8_t serial_num);

    void begin(uint32_t b) override;
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void end() override;
    void flush() override;
    bool is_initialized() override;
    void set_blocking_writes(bool blocking) override;
    bool tx_pending() override;

    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;
    void _timer_tick(void) override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
private:
    bool _initialized;
    const size_t TX_BUF_SIZE = 1024;
    const size_t RX_BUF_SIZE = 1024;
    uint8_t _buffer[32];
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    Semaphore _write_mutex;
    void read_data();
    void write_data();

    //hw configuration
    uart_port_t uart_num;
    uint8_t rx_pin;
    uint8_t tx_pin;

};

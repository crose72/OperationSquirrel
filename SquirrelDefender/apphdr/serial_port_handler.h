#ifndef SERIAL_PORT_HANDLER_H
#define SERIAL_PORT_HANDLER_H

#include "standard_libs.h"

void open_serial_port(void);
void write_serial_port(uint8_t* buffer, uint16_t len);
uint8_t read_serial_port(void);
void offset_buffer(uint8_t* buffer, uint16_t &len, mavlink_message_t &msg);
void clear_buffer(uint8_t* buffer, uint16_t len);
void close_serial_port(void);

#endif // SERIAL_PORT_HANDLER_H
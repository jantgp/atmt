#ifndef TASK_SAFE_WIRE_H
#define TASK_SAFE_WIRE_H

#include <Arduino.h>

void   IRAM_ATTR task_safe_wire_begin(uint8_t address);
void   IRAM_ATTR task_safe_wire_lock();
bool   IRAM_ATTR task_safe_wire_try_lock(uint32_t timeoutMs);
void   IRAM_ATTR task_safe_wire_unlock();
size_t IRAM_ATTR task_safe_wire_write(uint8_t value);
void   IRAM_ATTR task_safe_wire_restart();
uint8_t IRAM_ATTR task_safe_wire_request_from(uint8_t address, uint8_t quantity);
int    IRAM_ATTR task_safe_wire_read();
int    IRAM_ATTR task_safe_wire_available();
uint8_t IRAM_ATTR task_safe_wire_end();

#endif

#ifndef CPU_H
#define CPU_H

#include "stdint.h"

#define NOP 0x00
#define JP 0xC3

typedef struct {
    uint8_t AF[2]; // High-Low register pairs
    uint8_t BC[2];
    uint8_t DE[2];
    uint8_t HL[2];

    uint16_t SP;
    uint16_t PC;
} CPU_registers;

void initialize_CPU(CPU_registers*, uint8_t*);
uint8_t fetch_next_byte(CPU_registers*, uint8_t*);
void decode_execute_opcode(CPU_registers*, uint8_t, uint8_t*);

#endif
#ifndef CPU_H
#define CPU_H

#include "stdint.h"
#include <windows.h> // Multiplatform support -> unistd.h

#include "opcode.h"


#define FLAG_Z 0x80  // Zero flag
#define FLAG_N 0x40  // Subtraction flag
#define FLAG_H 0x20  // Half Carry flag
#define FLAG_C 0x10  // Carry flag

typedef struct {
    uint8_t AF[2]; // High-Low register pairs
    uint8_t BC[2];
    uint8_t DE[2];
    uint8_t HL[2];

    uint16_t SP;
    uint16_t PC;
} CPU_registers;


#endif
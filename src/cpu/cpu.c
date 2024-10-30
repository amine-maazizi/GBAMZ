#include "cpu.h"

void initialize_CPU(CPU_registers* cpu) {
    cpu->PC = 0x0100;
    cpu->SP = 0xFFFE;
    cpu->AF[1] = 0x00;

}

uint8_t fetch_next_byte(CPU_registers* cpu, uint8_t* memory)
{
    uint8_t next_byte = memory[cpu->PC];
    cpu->PC++;
    return next_byte;
};

void decode_execute_opcode(CPU_registers* cpu, uint8_t opcode, uint8_t* memory) {
    switch (opcode) {
        case JP:
            uint16_t address = fetch_next_byte(cpu, memory);
            address |= fetch_next_byte(cpu, memory) << 8;
            cpu->PC = bytes;
            break;
        case NOP:
            break;
    }
}
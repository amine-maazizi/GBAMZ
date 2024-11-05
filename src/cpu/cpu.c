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
    // For opcodes with no operand BLOCKS 0, 1, 2
    switch (opcode) {
        case NOP:
            break;
        case LD_IMM16_SP: {
            uint16_t imm16 = fetch_next_byte(cpu, memory);
            imm16 |= fetch_next_byte(cpu, memory) << 8;
            cpu->SP = imm16;
            break;
        }
        case RLCA: {
            uint8_t carry_out = (cpu->AF[0] & 0x80) ? 1 : 0;

            cpu->AF[0] = (cpu->AF[0] << 1) | carry_out;

            if (carry_out) {
                cpu->AF[1] |= FLAG_C;  
            } else {
                cpu->AF[1] &= ~FLAG_C; 
            }

            cpu->AF[1] &= ~(FLAG_Z | FLAG_N | FLAG_H);
            break;
        }
        case RRCA: {
            uint8_t carry_out = (cpu->AF[0] & 0x01) ? 0x80 : 0x00;

            cpu->AF[0] = (cpu->AF[0] >> 1) | carry_out;

            if (carry_out) {
                cpu->AF[1] |= FLAG_C;  
            } else {
                cpu->AF[1] &= ~FLAG_C; 
            }

            cpu->AF[1] &= ~(FLAG_Z | FLAG_N | FLAG_H);
            break;
        }
        case RLA: {
            uint8_t old_carry = (cpu->AF[1] & FLAG_C) ? 1 : 0; 
            uint8_t new_carry = (cpu->AF[0] & 0x80) ? 1 : 0;    

            cpu->AF[0] = (cpu->AF[0] << 1) | old_carry;        
            // Update carry flag
            if (new_carry) {
                cpu->AF[1] |= FLAG_C;
            } else {
                cpu->AF[1] &= ~FLAG_C;
            }

            cpu->AF[1] &= ~(FLAG_Z | FLAG_N | FLAG_H);        
            break;
        }
        case RRA: {
            uint8_t old_carry = (cpu->AF[1] & FLAG_C) ? 0x80 : 0; 
            uint8_t new_carry = (cpu->AF[0] & 0x01) ? 1 : 0;      

            cpu->AF[0] = (cpu->AF[0] >> 1) | old_carry;           

            if (new_carry) {
                cpu->AF[1] |= FLAG_C;
            } else {
                cpu->AF[1] &= ~FLAG_C;
            }

            cpu->AF[1] &= ~(FLAG_Z | FLAG_N | FLAG_H);             
            break;
        }
        case DAA: {
            if ((cpu->AF[0] & 0x0F) > 0x09 || (cpu->AF[1] & FLAG_H)) {
                cpu->AF[0] += 0x06;
            }

            if ((cpu->AF[0] & 0xF0) > 0x90 || (cpu->AF[1] & FLAG_C)) {
                cpu->AF[0] += 0x60;
                cpu->AF[1] |= FLAG_C;  
            }

            cpu->AF[1] &= ~FLAG_H;

            if (cpu->AF[0] == 0) {
                cpu->AF[1] |= FLAG_Z;
            } else {
                cpu->AF[1] &= ~FLAG_Z;
            }            
            break;
        }
        case CPL: {
            cpu->AF[0] ^= 0xFF;
            cpu->AF[1] |= (FLAG_N | FLAG_H);
            break;
        }
        case SCF: {
            cpu->AF[1] |= FLAG_C;
            cpu->AF[1] &= ~(FLAG_N | FLAG_H);
            break;
        }
        case CCF: {
            cpu->AF[1] = (cpu->AF[1] & FLAG_C) ? (cpu->AF[1] & ~FLAG_C) : (cpu->AF[1] | FLAG_C);
            cpu->AF[1] &= ~(FLAG_N | FLAG_H);
            break;
        }
        case JR_IMM8: {
            int8_t imm8 = (int8_t)fetch_next_byte(cpu, memory); // Signed since it's relative
            cpu->PC = imm8;
            break;
        }
        case STOP: {
            // To be fully implemented once I set up the input system.
            sleep(1000);
            break;
        }
    }

    // For opcodes with no operands BLOCK 3
    switch (opcode) {
        case ADD_A_IMM8:
            uint8_t imm8 = fetch_next_byte(cpu, memory);
            cpu->AF[1] = 0;  // Clear flags
            if (((cpu->AF[0] & 0x0F) + (imm8 & 0x0F)) > 0x0F) cpu->AF[1] |= FLAG_H; // Half-Carry
            if ((cpu->AF[0] + imm8) > 0xFF) cpu->AF[1] |= FLAG_C; // Carry
            cpu->AF[0] += imm8;
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case ADC_A_IMM8:
            uint8_t imm8 = fetch_next_byte(cpu, memory);
            uint8_t carry = (cpu->AF[1] & FLAG_C) ? 1 : 0;
            cpu->AF[1] = 0;  // Clear flags
            if (((cpu->AF[0] & 0x0F) + (imm8 & 0x0F) + carry) > 0x0F) cpu->AF[1] |= FLAG_H; // Half-Carry
            if ((cpu->AF[0] + imm8 + carry) > 0xFF) cpu->AF[1] |= FLAG_C; // Carry
            cpu->AF[0] += imm8 + carry;
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case SUB_A_IMM8:
            uint8_t imm8 = fetch_next_byte(cpu, memory);
            cpu->AF[1] = FLAG_N;  // Set subtract flag
            if ((cpu->AF[0] & 0x0F) < (imm8 & 0x0F)) cpu->AF[1] |= FLAG_H; // Half-Carry
            if (cpu->AF[0] < imm8) cpu->AF[1] |= FLAG_C; // Carry
            cpu->AF[0] -= imm8;
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case SBC_A_IMM8:
            uint8_t imm8 = fetch_next_byte(cpu, memory);
            carry = (cpu->AF[1] & FLAG_C) ? 1 : 0;
            cpu->AF[1] = FLAG_N;  // Set subtract flag
            if ((cpu->AF[0] & 0x0F) < ((imm8 & 0x0F) + carry)) cpu->AF[1] |= FLAG_H; // Half-Carry
            if (cpu->AF[0] < (imm8 + carry)) cpu->AF[1] |= FLAG_C; // Carry
            if (cpu->AF[0] - imm8 - carry == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            cpu->AF[0] -= (imm8 + carry);
            break;

        case AND_A_IMM8:
            uint8_t imm8 = fetch_next_byte(cpu, memory);
            cpu->AF[0] &= imm8;
            cpu->AF[1] = FLAG_H;  // Clear all flags except Half-Carry (bitwise AND always sets Half-Carry)
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case XOR_A_IMM8:
            uint8_t imm8 = fetch_next_byte(cpu, memory);
            cpu->AF[0] ^= imm8;
            cpu->AF[1] = 0;  // Clear all flags
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case OR_A_IMM8:
            uint8_t imm8 = fetch_next_byte(cpu, memory);
            cpu->AF[0] |= imm8;
            cpu->AF[1] = 0;  // Clear all flags
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case CP_A_IMM8:
            uint8_t imm8 = fetch_next_byte(cpu, memory);
            cpu->AF[1] = FLAG_N;  // Set subtract flag
            if ((cpu->AF[0] & 0x0F) < (imm8 & 0x0F)) cpu->AF[1] |= FLAG_H; // Half-Carry
            if (cpu->AF[0] < imm8) cpu->AF[1] |= FLAG_C; // Carry
            if ((cpu->AF[0] - imm8) == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;
    
        case RET: {
            cpu->PC = memory[cpu->SP] | (memory[cpu->SP + 1] << 8);
            cpu->SP += 2;
            break;
        }

        case RETI: {
            // To be completed once the interupt system is completed
            cpu->PC = memory[cpu->SP] | (memory[cpu->SP + 1] << 8);
            cpu->SP += 2;
            break;
        }

        case JP_IMM16: {
            uint16_t address = fetch_next_byte(cpu, memory);
            address |= fetch_next_byte(cpu, memory) << 8;
            cpu->PC = address;
            break;
        }

        case JP_HL: {
            cpu->PC = (cpu->HL[0] << 8)| cpu->HL[1];
            break;
        }
       
        case CALL_IMM16: {
            // Stack grows downward so here we're creating space 
            cpu->SP -= 2;  // Decrement stack pointer by 2 to make space for PC
            memory[cpu->SP] = cpu->PC & 0xFF;         // Store lower byte of PC
            memory[cpu->SP + 1] = (cpu->PC >> 8) & 0xFF;      // Store upper byte of PC


            uint16_t address = fetch_next_byte(cpu, memory);
            address |= fetch_next_byte(cpu, memory) << 8;
            cpu->PC = address; 
            break;
        }
    
        case LDH_C_A: {
            memory[0xFF << 8 | cpu->BC[1]] = cpu->AF[0]; break;
        }
        case LDH_IMM8_A: {
            uint8_t address = fetch_next_byte(cpu, memory);
            memory[0xFF << 8 | address] = cpu->AF[0]; break;
        }
        case LD_IMM16_A: {
            uint16_t address = fetch_next_byte(cpu, memory) | fetch_next_byte(cpu, memory) << 8;
            memory[address] = cpu->AF[0]; break;
        }
        case LDH_A_C: {
            cpu->AF[0] = memory[0xFF << 8 | cpu->BC[1]]; break;
        }
        case LDH_A_IMM8: {
            uint8_t address = fetch_next_byte(cpu, memory);
            cpu->AF[0] = memory[0xFF << 8 | address]; break;
        }
        case LDH_A_IMM16: {
            uint16_t address = fetch_next_byte(cpu, memory) | fetch_next_byte(cpu, memory) << 8;
            cpu->AF[0] = memory[address]; break;
        }

    }

    // Another block for opcodes with no operands BLOCK 3
    switch (opcode) {
        case ADD_SP_IMM8: {
            int8_t address = (int8_t)fetch_next_byte(cpun memory); // [-128, 127] 
            cpu->SP += address; break;
        }
        case LD_HL_SP_P_IMM8: {
            int8_t address = (int8_t)fetch_next_byte(cpun memory); // [-128, 127] 
            cpu->HL = cpu->SP + address; break;
        }
        case LD_SP_HL: {
            cpu->SP = cpu->HL; break;
        }
        case DI: {
            // TBI ONCE THE INTERUPTS ARE CODED IN.
            break;
        }
        case EI: {
            // SAME
            break;
        }
    }

    // For opcodes with an operand in bits 4 and 5
    switch (opcode & 0xCF) {

        case LD_R16_IMM16: {
            uint8_t dest = (opcode & MASK_BIT_45) >> 4;
            uint16_t imm16 = fetch_next_byte(cpu, memory);
            imm16 |= fetch_next_byte(cpu, memory) << 8;
            set_16bit_register(cpu, dest, imm16);
            break;
        }
        case LD_R16MEM_A: {
            uint8_t dest = (opcode & MASK_BIT_45) >> 4;
            store_accumulator(cpu, memory, dest);
            break;
        }
        case LD_A_R16MEM: {
            uint8_t src = (opcode & MASK_BIT_45) >> 4;
            load_accumulator(cpu, memory, src);
            break;
        }
        case INC_R16: {
            uint8_t operand = (opcode & MASK_BIT_45) >> 4;
            inc_16bit_register(cpu, operand);
            break;
        }
        case DEC_R16: {
            uint8_t operand = (opcode & MASK_BIT_45) >> 4;
            dec_16bit_register(cpu, operand);
            break;
        }
        case ADD_HL_R16: {
            uint8_t operand = (opcode & MASK_BIT_45) >> 4;
            uint16_t r16_value = get_16bit_address(cpu, operand);
            uint16_t hl_value = (cpu->HL[1] << 8) | cpu->HL[0];
            uint32_t result = hl_value + r16_value;  // Use 32-bit to detect overflow

            cpu->HL[0] = result & 0xFF;
            cpu->HL[1] = (result >> 8) & 0xFF;

            // Update flags
            cpu->AF[1] &= ~(FLAG_N);  // Clear N flag (since it's an addition)
            cpu->AF[1] = (result > 0xFFFF) ? (cpu->AF[1] | FLAG_C) : (cpu->AF[1] & ~FLAG_C);  // Set or clear Carry
            cpu->AF[1] = ((hl_value & 0xFFF) + (r16_value & 0xFFF) > 0xFFF) ? (cpu->AF[1] | FLAG_H) : (cpu->AF[1] & ~FLAG_H);  // Set or clear Half Carry
            break;
        }


        case POP_R16STK: { 
            uint8_t operand = (opcode & MASK_BIT_45) >> 4;  
            switch (operand) {
                case 0x00:  // POP BC
                    cpu->BC = cpu->memory[cpu->SP] | (cpu->memory[cpu->SP + 1] << 8);
                    break;
                case 0x01:  // POP DE
                    cpu->DE = cpu->memory[cpu->SP] | (cpu->memory[cpu->SP + 1] << 8);
                    break;
                case 0x02:  // POP HL
                    cpu->HL = cpu->memory[cpu->SP] | (cpu->memory[cpu->SP + 1] << 8);
                    break;
                case 0x03:  // POP AF
                    cpu->AF = cpu->memory[cpu->SP] | (cpu->memory[cpu->SP + 1] << 8);
                    break;
            }
            cpu->SP += 2;  // Move stack pointer up by 2
            break;
        }

        case PUSH_R16STK: {
            uint8_t operand = (opcode & MASK_BIT_45) >> 4;  
            cpu->SP -= 2;  // Decrement stack pointer by 2 to make space
            switch (operand) {
                case 0x00:  // PUSH BC
                    cpu->memory[cpu->SP] = cpu->BC & 0xFF;
                    cpu->memory[cpu->SP + 1] = (cpu->BC >> 8) & 0xFF;
                    break;
                case 0x01:  // PUSH DE
                    cpu->memory[cpu->SP] = cpu->DE & 0xFF;
                    cpu->memory[cpu->SP + 1] = (cpu->DE >> 8) & 0xFF;
                    break;
                case 0x02:  // PUSH HL
                    cpu->memory[cpu->SP] = cpu->HL & 0xFF;
                    cpu->memory[cpu->SP + 1] = (cpu->HL >> 8) & 0xFF;
                    break;
                case 0x03:  // PUSH AF
                    cpu->memory[cpu->SP] = cpu->AF & 0xFF;
                    cpu->memory[cpu->SP + 1] = (cpu->AF >> 8) & 0xFF;
                    break;
            }
            break;
        }
    }


    // For opcodes with an operand in bits 3, 4 and 5
    switch (opcode & 0xC7) {
        case INC_R8: {
            uint8_t reg_code = (opcode & MASK_BIT_345) >> 3;
            uint8_t* reg = get_8bit_register(cpu, reg_code);
            (*reg)++;
            // Update flags (Zero, Subtraction, Half Carry)
            cpu->AF[1] = (*reg == 0) ? (cpu->AF[1] | FLAG_Z) : (cpu->AF[1] & ~FLAG_Z);
            cpu->AF[1] &= ~FLAG_N;
            cpu->AF[1] = ((*reg & 0x0F) == 0) ? (cpu->AF[1] | FLAG_H) : (cpu->AF[1] & ~FLAG_H);
            break;
        }
        case DEC_R8: {
            uint8_t reg_code = (opcode & MASK_BIT_345) >> 3;
            uint8_t* reg = get_8bit_register(cpu, reg_code);
            (*reg)--;
            // Update flags (Zero, Subtraction, Half Carry)
            cpu->AF[1] = (*reg == 0) ? (cpu->AF[1] | FLAG_Z) : (cpu->AF[1] & ~FLAG_Z);
            cpu->AF[1] |= FLAG_N;
            cpu->AF[1] = ((*reg & 0x0F) == 0x0F) ? (cpu->AF[1] | FLAG_H) : (cpu->AF[1] & ~FLAG_H);
            break;
        }
        case LD_R8_IMM8: {
            uint8_t reg_code = (opcode & MASK_BIT_345) >> 3;
            uint8_t* reg = get_8bit_register(cpu, reg_code);
            *reg = fetch_next_byte(cpu, memory);
            break;
        }

        case RST_TGT3: {
            uint8_t operand = (opcode & MASK_BIT_345) >> 3;

            switch (operand) {
                case 0x00: cpu->PC = 0x00; break;
                case 0x01: cpu->PC = 0x08; break;
                case 0x02: cpu->PC = 0x10; break;
                case 0x03: cpu->PC = 0x18; break;
                case 0x04: cpu->PC = 0x20; break;
                case 0x05: cpu->PC = 0x28; break;
                case 0x06: cpu->PC = 0x30; break;
                case 0x07: cpu->PC = 0x38; break;
            }
        }

    }

// For opcodes with an operand in bits 3 and 4
    switch (opcode & 0xE7) {
        case JR_COND_IMM8: {
            uint8_t cond = opcode & MASK_BIT_34;  
            int8_t imm8 = (int8_t)fetch_next_byte(cpu, memory);  
            bool should_jump = false;

            switch (cond) {
                case 0x00:  // Not Zero (NZ)
                    should_jump = !(cpu->AF[1] & FLAG_Z);
                    break;
                case 0x08:  // Zero (Z)
                    should_jump = cpu->AF[1] & FLAG_Z;
                    break;
                case 0x10:  // No Carry (NC)
                    should_jump = !(cpu->AF[1] & FLAG_C);
                    break;
                case 0x18:  // Carry (C)
                    should_jump = cpu->AF[1] & FLAG_C;
                    break;
            }

            if (should_jump) {
                cpu->PC += imm8;
            }
            break;
        }

        case RET_COND: {
            uint8_t cond = opcode & MASK_BIT_34;  
            bool should_return = false;

            switch (cond) {
                case 0x00:  // Not Zero (NZ)
                    should_return = !(cpu->AF[1] & FLAG_Z);
                    break;
                case 0x08:  // Zero (Z)
                    should_return = cpu->AF[1] & FLAG_Z;
                    break;
                case 0x10:  // No Carry (NC)
                    should_return = !(cpu->AF[1] & FLAG_C);
                    break;
                case 0x18:  // Carry (C)
                    should_return = cpu->AF[1] & FLAG_C;
                    break;
            }

            if (should_return) {
                cpu->PC = memory[cpu->SP] | (memory[cpu->SP + 1] << 8);
                cpu->SP += 2;
            }
            break;
        }

        case JP_COND_IMM16: {
            uint8_t cond = opcode & MASK_BIT_34;  
            bool should_jump = false;

            switch (cond) {
                case 0x00:  // Not Zero (NZ)
                    should_jump = !(cpu->AF[1] & FLAG_Z);
                    break;
                case 0x08:  // Zero (Z)
                    should_jump = cpu->AF[1] & FLAG_Z;
                    break;
                case 0x10:  // No Carry (NC)
                    should_jump = !(cpu->AF[1] & FLAG_C);
                    break;
                case 0x18:  // Carry (C)
                    should_jump = cpu->AF[1] & FLAG_C;
                    break;
            }

            if (should_jump) {
                uint16_t address = fetch_next_byte(cpu, memory);
                address |= fetch_next_byte(cpu, memory) << 8;
                cpu->PC = address;
            }
            break;
        }

        case CALL_COND_IMM16: {
            uint8_t cond = opcode & MASK_BIT_34;  
            bool should_call = false;

            switch (cond) {
                case 0x00:  // Not Zero (NZ)
                    should_call = !(cpu->AF[1] & FLAG_Z);
                    break;
                case 0x08:  // Zero (Z)
                    should_call = cpu->AF[1] & FLAG_Z;
                    break;
                case 0x10:  // No Carry (NC)
                    should_call = !(cpu->AF[1] & FLAG_C);
                    break;
                case 0x18:  // Carry (C)
                    should_call = cpu->AF[1] & FLAG_C;
                    break;
            }

            if (should_call) {
                cpu->SP -= 2;  
                memory[cpu->SP] = cpu->PC & 0xFF;         // Store lower byte of PC
                memory[cpu->SP + 1] = (cpu->PC >> 8) & 0xFF;  // Store upper byte of PC

                uint16_t address = fetch_next_byte(cpu, memory);
                address |= fetch_next_byte(cpu, memory) << 8;
                cpu->PC = address;
            }
            break;
        }
    }


    // For opcodes with an operand in bits 3, 4, 5 and another one in bits 0, 1, 2
    switch (opcode & 0xC0) {
        case 0x40: {  // LD_R8_R8 instruction block
            uint8_t src = opcode & MASK_BIT_012;
            uint8_t dest = (opcode & MASK_BIT_345) >> 3;
            if (opcode == HALT) {
                sleep(1000);
            } else {
                set_8bit_register(cpu, dest, get_8bit_register(cpu, src));
            }
            break;
        }
    }

    // For opcodes with an operand in bits 0, 1, 2
    switch (opcode & 0xF8) {
        case ADD_A_R8:
            uint8_t operand = get_8bit_register(cpu, opcode & MASK_BIT_012);
            cpu->AF[1] = 0;  // Clear flags
            if (((cpu->AF[0] & 0x0F) + (operand & 0x0F)) > 0x0F) cpu->AF[1] |= FLAG_H; // Half-Carry
            if ((cpu->AF[0] + operand) > 0xFF) cpu->AF[1] |= FLAG_C; // Carry
            cpu->AF[0] += operand;
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case ADC_A_R8:
            uint8_t operand = get_8bit_register(cpu, opcode & MASK_BIT_012);
            uint8_t carry = (cpu->AF[1] & FLAG_C) ? 1 : 0;
            cpu->AF[1] = 0;  // Clear flags
            if (((cpu->AF[0] & 0x0F) + (operand & 0x0F) + carry) > 0x0F) cpu->AF[1] |= FLAG_H; // Half-Carry
            if ((cpu->AF[0] + operand + carry) > 0xFF) cpu->AF[1] |= FLAG_C; // Carry
            cpu->AF[0] += operand + carry;
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case SUB_A_R8:
            uint8_t operand = get_8bit_register(cpu, opcode & MASK_BIT_012);
            cpu->AF[1] = FLAG_N;  // Set subtract flag
            if ((cpu->AF[0] & 0x0F) < (operand & 0x0F)) cpu->AF[1] |= FLAG_H; // Half-Carry
            if (cpu->AF[0] < operand) cpu->AF[1] |= FLAG_C; // Carry
            cpu->AF[0] -= operand;
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case SBC_A_R8:
            uint8_t operand = get_8bit_register(cpu, opcode & MASK_BIT_012);
            carry = (cpu->AF[1] & FLAG_C) ? 1 : 0;
            cpu->AF[1] = FLAG_N;  // Set subtract flag
            if ((cpu->AF[0] & 0x0F) < ((operand & 0x0F) + carry)) cpu->AF[1] |= FLAG_H; // Half-Carry
            if (cpu->AF[0] < (operand + carry)) cpu->AF[1] |= FLAG_C; // Carry
            if (cpu->AF[0] - operand - carry == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            cpu->AF[0] -= (operand + carry);
            break;

        case AND_A_R8:
            uint8_t operand = get_8bit_register(cpu, opcode & MASK_BIT_012);
            cpu->AF[0] &= operand;
            cpu->AF[1] = FLAG_H;  // Clear all flags except Half-Carry (bitwise AND always sets Half-Carry)
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case XOR_A_R8:
            uint8_t operand = get_8bit_register(cpu, opcode & MASK_BIT_012);
            cpu->AF[0] ^= operand;
            cpu->AF[1] = 0;  // Clear all flags
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case OR_A_R8:
            uint8_t operand = get_8bit_register(cpu, opcode & MASK_BIT_012);
            cpu->AF[0] |= operand;
            cpu->AF[1] = 0;  // Clear all flags
            if (cpu->AF[0] == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;

        case CP_A_R8:
            uint8_t operand = get_8bit_register(cpu, opcode & MASK_BIT_012);
            cpu->AF[1] = FLAG_N;  // Set subtract flag
            if ((cpu->AF[0] & 0x0F) < (operand & 0x0F)) cpu->AF[1] |= FLAG_H; // Half-Carry
            if (cpu->AF[0] < operand) cpu->AF[1] |= FLAG_C; // Carry
            if ((cpu->AF[0] - operand) == 0) cpu->AF[1] |= FLAG_Z; // Zero flag
            break;
    }
}


void set_16bit_register(CPU_registers* cpu, uint8_t reg_code, uint16_t value) {
    switch (reg_code) {
        case 0x00: cpu->BC[0] = value & 0xFF; cpu->BC[1] = (value >> 8) & 0xFF; break;
        case 0x01: cpu->DE[0] = value & 0xFF; cpu->DE[1] = (value >> 8) & 0xFF; break;
        case 0x02: cpu->HL[0] = value & 0xFF; cpu->HL[1] = (value >> 8) & 0xFF; break;
        case 0x03: cpu->SP = value; break;
    }
}

uint16_t get_16bit_address(CPU_registers* cpu, uint8_t reg_code) {
    switch (reg_code) {
        case 0x00: return (cpu->BC[1] << 8) | cpu->BC[0];
        case 0x01: return (cpu->DE[1] << 8) | cpu->DE[0];
        case 0x02: return (cpu->HL[1] << 8) | cpu->HL[0];
        default: return 0;
    }
}

void inc_16bit_register(CPU_registers* cpu, uint8_t reg_code) {
    switch (reg_code) {
        case 0x00: // BC
            cpu->BC[0]++;
            if (cpu->BC[0] == 0) cpu->BC[1]++; // Handle carry for low byte overflow
            break;
        case 0x01: // DE
            cpu->DE[0]++;
            if (cpu->DE[0] == 0) cpu->DE[1]++;
            break;
        case 0x02: // HL
            cpu->HL[0]++;
            if (cpu->HL[0] == 0) cpu->HL[1]++;
            break;
        case 0x03: // SP
            cpu->SP++;
            break;
    }
}

void dec_16bit_register(CPU_registers* cpu, uint8_t reg_code) {
    switch (reg_code) {
        case 0x00: // BC
            cpu->BC[0]--;
            if (cpu->BC[0] == 0xFF) cpu->BC[1]--; // Handle carry for low byte overflow
            break;
        case 0x01: // DE
            cpu->DE[0]--;
            if (cpu->DE[0] == 0xFF) cpu->DE[1]--;
            break;
        case 0x02: // HL
            cpu->HL[0]--;
            if (cpu->HL[0] == 0xFF) cpu->HL[1]--;
            break;
        case 0x03: // SP
            cpu->SP--;
            break;
    }
}

// Function to set the value of an 8-bit register
void set_8bit_register(CPU_registers* cpu, uint8_t reg_code, uint8_t value) {
    switch (reg_code) {
        case 0x00: cpu->B = value; break;
        case 0x01: cpu->C = value; break;
        case 0x02: cpu->D = value; break;
        case 0x03: cpu->E = value; break;
        case 0x04: cpu->H = value; break;
        case 0x05: cpu->L = value; break;
        case 0x06: cpu->memory[(cpu->HL[1] << 8) | cpu->HL[0]] = value; break; // [HL] memory location
        case 0x07: cpu->A = value; break;
        default: break; // Undefined register code
    }
}

// Function to get the value of an 8-bit register
uint8_t get_8bit_register(CPU_registers* cpu, uint8_t reg_code) {
    switch (reg_code) {
        case 0x00: return cpu->B;
        case 0x01: return cpu->C;
        case 0x02: return cpu->D;
        case 0x03: return cpu->E;
        case 0x04: return cpu->H;
        case 0x05: return cpu->L;
        case 0x06: return cpu->memory[(cpu->HL[1] << 8) | cpu->HL[0]]; // [HL] memory location
        case 0x07: return cpu->A;
        default: return 0; // Undefined register code
    }
}


void store_accumulator(CPU_registers* cpu, uint8_t* memory, uint8_t dest_code) {
    uint16_t addr = get_16bit_address(cpu, dest_code);
    memory[addr] = cpu->AF[0];
    if (dest_code == 0x02) {  // HL+ (HLI)
        cpu->HL[0]++;
        if (cpu->HL[0] == 0) cpu->HL[1]++;
    } else if (dest_code == 0x03) {  // HL- (HLD)
        cpu->HL[0]--;
        if (cpu->HL[0] == 0xFF) cpu->HL[1]--;
    }
}

void load_accumulator(CPU_registers* cpu, uint8_t* memory, uint8_t src_code) {
    uint16_t addr = get_16bit_address(cpu, src_code);
    cpu->AF[0] = memory[addr];
    if (src_code == 0x02) {  // HL+ (HLI)
        cpu->HL[0]++;
        if (cpu->HL[0] == 0) cpu->HL[1]++;
    } else if (src_code == 0x03) {  // HL- (HLD)
        cpu->HL[0]--;
        if (cpu->HL[0] == 0xFF) cpu->HL[1]--;
    }
}

uint8_t* get_8bit_register(CPU_registers* cpu, uint8_t reg_code) {
    switch (reg_code) {
        case 0x00: return &cpu->BC[1];  // B
        case 0x01: return &cpu->BC[0];  // C
        case 0x02: return &cpu->DE[1];  // D
        case 0x03: return &cpu->DE[0];  // E
        case 0x04: return &cpu->HL[1];  // H
        case 0x05: return &cpu->HL[0];  // L
        case 0x07: return &cpu->AF[0];  // A
        default: return NULL;
    }
}



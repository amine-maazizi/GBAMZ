#ifndef OPCODE_H
#define OPCODE_H

// Masks
#define MASK_BIT_45 0x30
#define MASK_BIT_345 0x38  
#define MASK_BIT_34 0x18

//// BLOCK 0 instructions ////
#define NOP 0x00

#define LD_R16_IMM16 0x01
#define LD_R16MEM_A 0x02
#define LD_A_R16MEM 0x0A
#define LD_IMM16_SP 0x08

#define INC_R16 0x03
#define DEC_R16 0x0B
#define ADD_HL_R16 0x09

#define INC_R8 0x04
#define DEC_R8 0x05
#define LD_R8_IMM8 0x06

#define RLCA 0x07
#define RRCA 0x0F
#define RLA  0x17
#define RRA  0x1F
#define DAA  0x27
#define CPL  0x2F
#define SCF  0x37
#define CCF  0X3F

#define JR_IMM8 0x18
#define JR_COND_IMM8 0x20

#define STOP 0x10

#define JP 0xC3

#endif 
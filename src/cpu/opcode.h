#ifndef OPCODE_H
#define OPCODE_H

// Masks
#define MASK_BIT_45 0x30
#define MASK_BIT_345 0x38  
#define MASK_BIT_34 0x18
#define MASK_BIT_012 0x07

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

///// Block 1: 8-bit register-to-register loads ////
#define LD_R8_R8 0x40
#define HALT 0x75

//// Block 2: 8-bit arithmetics ////
#define ADD_A_R8 0x80
#define ADC_A_R8 0x88
#define SUB_A_R8 0x90
#define SBC_A_R8 0x98
#define AND_A_R8 0xA0
#define XOR_A_R8 0xA8
#define OR_A_R8 0xB0
#define CP_A_R8 0xB8

// Block 3
#define ADD_A_IMM8 0xC6
#define ADC_A_IMM8 0xCE
#define SUB_A_IMM8 0xD6
#define SBC_A_IMM8 0xDE
#define AND_A_IMM8 0xE6
#define XOR_A_IMM8 0xEE
#define OR_A_IMM8 0xF6
#define CP_A_IMM8 0xFE

#define RET_COND 0xC0
#define RET 0xC9
#define RETI 0xD9
#define JP_COND_IMM16  0xC2
#define JP_IMM16 0xC3
#define JP_HL 0xE9
#define CALL_COND_IMM16 0xC4
#define CALL_IMM16 0xCD
#define RST_TGT3 0xC7

#define POP_R16STK 0xC1
#define PUSH_R16STK 0xC6

#define LDH_C_A 0xE2
#define LDH_IMM8_A 0xE0
#define LD_IMM16_A 0xEA
#define LDH_A_C 0xF2
#define LDH_A_IMM8 0xF0
#define LD_A_IMM16 0xFA

#define ADD_SP_IMM8 0xE8
#define LD_HL_SP_P_IMM8 0xF8
#define LD_SP_HL 0xF9

#define DI 0xF3
#define EI 0xFB

// CB prefix instructions
#define CB_EXTENDED_OPCODES 0xCB

#define RLC_R8 0x00
#define RRC_R8 0x08
#define RC_R8 0x10
#define RR_R8 0x18
#define SLA_R8 0x20
#define SRA_R8 0x28
#define SWAP_R8 0x30
#define SRL_R8 0x38

#define BIT_B3_R8 0x40
#define RES_B3_R8 0x80
#define SET_B3_R8 0xC0

#endif 
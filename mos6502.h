#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdint.h>
#include <map>
using namespace std;

#define MEMORY_SIZE 0x10000
#define RESET_LOW 0xFFFC // RESET vector low byte
#define RESET_HIGH 0xFFFD // RESET vector high byte
#define SP_START 0x1FD // Stack Pointer start address

#define LOW_BYTE 0xFF
#define HIGH_BYTE 0xFF
#define NIBBLE 0x0F
#define BYTE_HIGH_BIT 0x80
#define BYTE_LOW_BIT 0x01
#define CARRY_BIT 0x100

class MOS6502
{
private:
    typedef void (*op_ptr)(uint16_t);

    uint8_t memory[MEMORY_SIZE];
    uint8_t reg_A; // Accumulator
    uint8_t reg_X;
    uint8_t reg_Y;
    uint8_t reg_SP; // Stack Pointer
    uint16_t reg_PC; // Program Counter
    
    struct StatusRegister {
        bool N; // Negative
        bool V; // Overflow
        bool _;
        bool B; // Break
        bool D; // Decimal
        bool I; // Interrupt Disable
        bool Z; // Zero
        bool C; // Carry
    } reg_status;

    enum Mode {
        ACC, // Accumulator mode
        IMM, // Immediate mode
        ABS, // Absolute mode
        ZPG, // Zero Page mode
        ZPX, // Zero Page Indexed with X
        ZPY, // Zero Page Indexed with Y
        AIX, // Absolute Indexed with X
        AIY, // Absolute Indexed with Y
        IMP, // Implied mode
        REL, // Relative mode
        IIX, // Zero Page Indirect Indexed with X
        IIY, // Zero Page Indirect Indexed with Y
        IND  // Indirect mode
    };

    struct Instruction {
        op_ptr op_func;
        Mode addr_mode;
    };

    void initialize();

    uint8_t fetch();
    Instruction decode(uint8_t byte);
    void execute(instruction instr);

    uint8_t * operand_from_mode(Mode m);

    map<uint8_t, Instruction> decoder = 
    {{0x00, Instruction{op_BRK, IMP}},
     {0x01, Instruction{op_ORA, IIX}},
     {0x02, Instruction{op_ILLEGAL, IMP}},
     {0x03, Instruction{op_ILLEGAL, IMP}},
     {0x04, Instruction{op_ILLEGAL, IMP}},
     {0x05, Instruction{op_ORA, ZPG}},
     {0x06, Instruction{op_ASL, ZPG}},
     {0x07, Instruction{op_ILLEGAL, IMP}},
     {0x08, Instruction{op_PHP, IMP}},
     {0x09, Instruction{op_ORA, IMM}},
     {0x0A, Instruction{op_ASL, ACC}},
     {0x0B, Instruction{op_ILLEGAL, IMP}},
     {0x0C, Instruction{op_ILLEGAL, IMP}},
     {0x0D, Instruction{op_ORA, ABS}},
     {0x0E, Instruction{op_ASL, ABS}},
     {0x0F, Instruction{op_ILLEGAL, IMP}},
     {0x10, Instruction{op_BPL, REL}},
     {0x11, Instruction{op_ORA, IIY}},
     {0x12, Instruction{op_ILLEGAL, IMP}},
     {0x13, Instruction{op_ILLEGAL, IMP}},
     {0x14, Instruction{op_ILLEGAL, IMP}},
     {0x15, Instruction{op_ORA, ZPX}},
     {0x16, Instruction{op_ASL, ZPX}},
     {0x17, Instruction{op_ILLEGAL, IMP}},
     {0x18, Instruction{op_CLC, IMP}},
     {0x19, Instruction{op_ORA, AIY}},
     {0x1A, Instruction{op_ILLEGAL, IMP}},
     {0x1B, Instruction{op_ILLEGAL, IMP}},
     {0x1C, Instruction{op_ILLEGAL, IMP}},
     {0x1D, Instruction{op_ORA, AIX}},
     {0x1E, Instruction{op_ASL, AIX}},
     {0x1F, Instruction{op_ILLEGAL, IMP}},
     {0x20, Instruction{op_JSR, ABS}},
     {0x21, Instruction{op_AND, IIX}},
     {0x22, Instruction{op_ILLEGAL, IMP}},
     {0x23, Instruction{op_ILLEGAL, IMP}},
     {0x24, Instruction{op_BIT, ZPG}},
     {0x25, Instruction{op_AND, ZPG}},
     {0x26, Instruction{op_ROL, ZPG}},
     {0x27, Instruction{op_ILLEGAL, IMP}},
     {0x28, Instruction{op_PLP, IMP}},
     {0x29, Instruction{op_AND, IMM}},
     {0x2A, Instruction{op_ROL, ACC}},
     {0x2B, Instruction{op_ILLEGAL, IMP}},
     {0x2C, Instruction{op_BIT, ABS}},
     {0x2D, Instruction{op_AND, ABS}},
     {0x2E, Instruction{op_ROL, ABS}},
     {0x2F, Instruction{op_ILLEGAL, IMP}},
     {0x30, Instruction{op_BMI, REL}},
     {0x31, Instruction{op_AND, IIY}},
     {0x32, Instruction{op_ILLEGAL, IMP}},
     {0x33, Instruction{op_ILLEGAL, IMP}},
     {0x34, Instruction{op_ILLEGAL, IMP}},
     {0x35, Instruction{op_AND, ZPX}},
     {0x36, Instruction{op_ROL, ZPX}},
     {0x37, Instruction{op_ILLEGAL, IMP}},
     {0x38, Instruction{op_SEC, IMP}},
     {0x39, Instruction{op_AND, AIY}},
     {0x3A, Instruction{op_ILLEGAL, IMP}},
     {0x3B, Instruction{op_ILLEGAL, IMP}},
     {0x3C, Instruction{op_ILLEGAL, IMP}},
     {0x3D, Instruction{op_AND, AIX}},
     {0x3E, Instruction{op_ROL, AIX}},
     {0x3F, Instruction{op_ILLEGAL, IMP}},
     {0x40, Instruction{op_RTI, IMP}},
     {0x41, Instruction{op_EOR, IIX}},
     {0x42, Instruction{op_ILLEGAL, IMP}},
     {0x43, Instruction{op_ILLEGAL, IMP}},
     {0x44, Instruction{op_ILLEGAL, IMP}},
     {0x45, Instruction{op_EOR, ZPG}},
     {0x46, Instruction{op_LSR, ZPG}},
     {0x47, Instruction{op_ILLEGAL, IMP}},
     {0x48, Instruction{op_PHA, IMP}},
     {0x49, Instruction{op_EOR, IMM}},
     {0x4A, Instruction{op_LSR, ACC}},
     {0x4B, Instruction{op_ILLEGAL, IMP}},
     {0x4C, Instruction{op_JMP, ABS}},
     {0x4D, Instruction{op_EOR, ABS}},
     {0x4E, Instruction{op_LSR, ABS}},
     {0x4F, Instruction{op_ILLEGAL, IMP}},
     {0x50, Instruction{op_BVC, REL}},
     {0x51, Instruction{op_EOR, IIY}},
     {0x52, Instruction{op_ILLEGAL, IMP}},
     {0x53, Instruction{op_ILLEGAL, IMP}},
     {0x54, Instruction{op_ILLEGAL, IMP}},
     {0x55, Instruction{op_EOR, ZPX}},
     {0x56, Instruction{op_LSR, ZPX}},
     {0x57, Instruction{op_ILLEGAL, IMP}},
     {0x58, Instruction{op_CLI, IMP}},
     {0x59, Instruction{op_EOR, AIY}},
     {0x5A, Instruction{op_ILLEGAL, IMP}},
     {0x5B, Instruction{op_ILLEGAL, IMP}},
     {0x5C, Instruction{op_ILLEGAL, IMP}},
     {0x5D, Instruction{op_EOR, AIX}},
     {0x5E, Instruction{op_LSR, AIX}},
     {0x5F, Instruction{op_ILLEGAL, IMP}},
     {0x60, Instruction{op_RTS, IMP}},
     {0x61, Instruction{op_ADC, IIX}},
     {0x62, Instruction{op_ILLEGAL, IMP}},
     {0x63, Instruction{op_ILLEGAL, IMP}},
     {0x64, Instruction{op_ILLEGAL, IMP}},
     {0x65, Instruction{op_ADC, ZPG}},
     {0x66, Instruction{op_ROR, ZPG}},
     {0x67, Instruction{op_ILLEGAL, IMP}},
     {0x68, Instruction{op_PLA, IMP}},
     {0x69, Instruction{op_ADC, IMM}},
     {0x6A, Instruction{op_ROR, ACC}},
     {0x6B, Instruction{op_ILLEGAL, IMP}},
     {0x6C, Instruction{op_JMP, IND}},
     {0x6D, Instruction{op_ADC, ABS}},
     {0x6E, Instruction{op_ROR, ABS}},
     {0x6F, Instruction{op_ILLEGAL, IMP}},
     {0x70, Instruction{op_BVS, REL}},
     {0x71, Instruction{op_ADC, IIY}},
     {0x72, Instruction{op_ILLEGAL, IMP}},
     {0x73, Instruction{op_ILLEGAL, IMP}},
     {0x74, Instruction{op_ILLEGAL, IMP}},
     {0x75, Instruction{op_ADC, ZPX}},
     {0x76, Instruction{op_ROR, ZPX}},
     {0x77, Instruction{op_ILLEGAL, IMP}},
     {0x78, Instruction{op_SEI, IMP}},
     {0x79, Instruction{op_ADC, AIY}},
     {0x7A, Instruction{op_ILLEGAL, IMP}},
     {0x7B, Instruction{op_ILLEGAL, IMP}},
     {0x7C, Instruction{op_ILLEGAL, IMP}},
     {0x7D, Instruction{op_ADC, AIX}},
     {0x7E, Instruction{op_ROR, AIX}},
     {0x7F, Instruction{op_ILLEGAL, IMP}},
     {0x80, Instruction{op_ILLEGAL, IMP}},
     {0x81, Instruction{op_STA, IIX}},
     {0x82, Instruction{op_ILLEGAL, IMP}},
     {0x83, Instruction{op_ILLEGAL, IMP}},
     {0x84, Instruction{op_STY, ZPG}},
     {0x85, Instruction{op_STA, ZPG}},
     {0x86, Instruction{op_STX, ZPG}},
     {0x87, Instruction{op_ILLEGAL, IMP}},
     {0x88, Instruction{op_DEY, IMP}},
     {0x89, Instruction{op_ILLEGAL, IMP}},
     {0x8A, Instruction{op_TXA, IMP}},
     {0x8B, Instruction{op_ILLEGAL, IMP}},
     {0x8C, Instruction{op_STY, ABS}},
     {0x8D, Instruction{op_STA, ABS}},
     {0x8E, Instruction{op_STX, ABS}},
     {0x8F, Instruction{op_ILLEGAL, IMP}},
     {0x90, Instruction{op_BCC, REL}},
     {0x91, Instruction{op_STA, IIY}},
     {0x92, Instruction{op_ILLEGAL, IMP}},
     {0x93, Instruction{op_ILLEGAL, IMP}},
     {0x94, Instruction{op_STY, ZPX}},
     {0x95, Instruction{op_STA, ZPX}},
     {0x96, Instruction{op_STX, ZPY}},
     {0x97, Instruction{op_ILLEGAL, IMP}},
     {0x98, Instruction{op_TYA, IMP}},
     {0x99, Instruction{op_STA, AIY}},
     {0x9A, Instruction{op_TXS, IMP}},
     {0x9B, Instruction{op_ILLEGAL, IMP}},
     {0x9C, Instruction{op_ILLEGAL, IMP}},
     {0x9D, Instruction{op_STA, AIX}},
     {0x9E, Instruction{op_ILLEGAL, IMP}},
     {0x9F, Instruction{op_ILLEGAL, IMP}},
     {0xA0, Instruction{op_LDY, IMM}},
     {0xA1, Instruction{op_LDA, IIX}},
     {0xA2, Instruction{op_LDX, IMM}},
     {0xA3, Instruction{op_ILLEGAL, IMP}},
     {0xA4, Instruction{op_LDY, ZPG}},
     {0xA5, Instruction{op_LDA, ZPG}},
     {0xA6, Instruction{op_LDX, ZPG}},
     {0xA7, Instruction{op_ILLEGAL, IMP}},
     {0xA8, Instruction{op_TAY, IMP}},
     {0xA9, Instruction{op_LDA, IMM}},
     {0xAA, Instruction{op_TAX, IMP}},
     {0xAB, Instruction{op_ILLEGAL, IMP}},
     {0xAC, Instruction{op_LDY, ABS}},
     {0xAD, Instruction{op_LDA, ABS}},
     {0xAE, Instruction{op_LDX, ABS}},
     {0xAF, Instruction{op_ILLEGAL, IMP}},
     {0xB0, Instruction{op_BCS, REL}},
     {0xB1, Instruction{op_LDA, IIY}},
     {0xB2, Instruction{op_ILLEGAL, IMP}},
     {0xB3, Instruction{op_ILLEGAL, IMP}},
     {0xB4, Instruction{op_LDY, ZPX}},
     {0xB5, Instruction{op_LDA, ZPX}},
     {0xB6, Instruction{op_LDX, ZPY}},
     {0xB7, Instruction{op_ILLEGAL, IMP}},
     {0xB8, Instruction{op_CLV, IMP}},
     {0xB9, Instruction{op_LDA, AIY}},
     {0xBA, Instruction{op_TSX, IMP}},
     {0xBB, Instruction{op_ILLEGAL, IMP}},
     {0xBC, Instruction{op_LDY, AIX}},
     {0xBD, Instruction{op_LDA, AIX}},
     {0xBE, Instruction{op_LDX, AIY}},
     {0xBF, Instruction{op_ILLEGAL, IMP}},
     {0xC0, Instruction{op_CPY, IMM}},
     {0xC1, Instruction{op_CMP, IIX}},
     {0xC2, Instruction{op_ILLEGAL, IMP}},
     {0xC3, Instruction{op_ILLEGAL, IMP}},
     {0xC4, Instruction{op_CPY, ZPG}},
     {0xC5, Instruction{op_CMP, ZPG}},
     {0xC6, Instruction{op_DEC, ZPG}},
     {0xC7, Instruction{op_ILLEGAL, IMP}},
     {0xC8, Instruction{op_INY, IMP}},
     {0xC9, Instruction{op_CMP, IMM}},
     {0xCA, Instruction{op_DEX, IMP}},
     {0xCB, Instruction{op_ILLEGAL, IMP}},
     {0xCC, Instruction{op_CPY, ABS}},
     {0xCD, Instruction{op_CMP, ABS}},
     {0xCE, Instruction{op_DEC, ABS}},
     {0xCF, Instruction{op_ILLEGAL, IMP}},
     {0xD0, Instruction{op_BNE, REL}},
     {0xD1, Instruction{op_CMP, IIY}},
     {0xD2, Instruction{op_ILLEGAL, IMP}},
     {0xD3, Instruction{op_ILLEGAL, IMP}},
     {0xD4, Instruction{op_ILLEGAL, IMP}},
     {0xD5, Instruction{op_CMP, ZPX}},
     {0xD6, Instruction{op_DEC, ZPX}},
     {0xD7, Instruction{op_ILLEGAL, IMP}},
     {0xD8, Instruction{op_CLD, IMP}},
     {0xD9, Instruction{op_CMP, AIY}},
     {0xDA, Instruction{op_ILLEGAL, IMP}},
     {0xDB, Instruction{op_ILLEGAL, IMP}},
     {0xDC, Instruction{op_ILLEGAL, IMP}},
     {0xDD, Instruction{op_CMP, AIX}},
     {0xDE, Instruction{op_DEC, AIX}},
     {0xDF, Instruction{op_ILLEGAL, IMP}},
     {0xE0, Instruction{op_CPX, IMM}},
     {0xE1, Instruction{op_SBC, IIX}},
     {0xE2, Instruction{op_ILLEGAL, IMP}},
     {0xE3, Instruction{op_ILLEGAL, IMP}},
     {0xE4, Instruction{op_CPX, ZPG}},
     {0xE5, Instruction{op_SBC, ZPG}},
     {0xE6, Instruction{op_INC, ZPG}},
     {0xE7, Instruction{op_ILLEGAL, IMP}},
     {0xE8, Instruction{op_INX, IMP}},
     {0xE9, Instruction{op_SBC, IMM}},
     {0xEA, Instruction{op_NOP, IMP}},
     {0xEB, Instruction{op_ILLEGAL, IMP}},
     {0xEC, Instruction{op_CPX, ABS}},
     {0xED, Instruction{op_SBC, ABS}},
     {0xEE, Instruction{op_INC, ABS}},
     {0xEF, Instruction{op_ILLEGAL, IMP}},
     {0xF0, Instruction{op_BEQ, REL}},
     {0xF1, Instruction{op_SBC, IIY}},
     {0xF2, Instruction{op_ILLEGAL, IMP}},
     {0xF3, Instruction{op_ILLEGAL, IMP}},
     {0xF4, Instruction{op_ILLEGAL, IMP}},
     {0xF5, Instruction{op_SBC, ZPX}},
     {0xF6, Instruction{op_INC, ZPX}},
     {0xF7, Instruction{op_ILLEGAL, IMP}},
     {0xF8, Instruction{op_SED, IMP}},
     {0xF9, Instruction{op_SBC, AIY}},
     {0xFA, Instruction{op_ILLEGAL, IMP}},
     {0xFB, Instruction{op_ILLEGAL, IMP}},
     {0xFC, Instruction{op_ILLEGAL, IMP}},
     {0xFD, Instruction{op_SBC, AIX}},
     {0xFE, Instruction{op_INC, AIX}},
     {0xFF, Instruction{op_ILLEGAL, IMP}}};

    void op_ADC(uint8_t *operand);
    void op_AND(uint8_t *operand);
    void op_ASL(uint8_t *operand);
    void op_BCC(uint8_t *operand);
    void op_BCS(uint8_t *operand);
    void op_BEQ(uint8_t *operand);
    void op_BIT(uint8_t *operand);
    void op_BMI(uint8_t *operand);
    void op_BNE(uint8_t *operand);
    void op_BPL(uint8_t *operand);
    void op_BRK(uint8_t *operand);
    void op_BVC(uint8_t *operand);
    void op_BVS(uint8_t *operand);
    void op_CLC(uint8_t *operand);
    void op_CLD(uint8_t *operand);
    void op_CLI(uint8_t *operand);
    void op_CLV(uint8_t *operand);
    void op_CMP(uint8_t *operand);
    void op_CPX(uint8_t *operand);
    void op_CPY(uint8_t *operand);
    void op_DEC(uint8_t *operand);
    void op_DEX(uint8_t *operand);
    void op_DEY(uint8_t *operand);
    void op_EOR(uint8_t *operand);
    void op_INC(uint8_t *operand);
    void op_INX(uint8_t *operand);
    void op_INY(uint8_t *operand);
    void op_JMP(uint8_t *operand);
    void op_JSR(uint8_t *operand);
    void op_LDA(uint8_t *operand);
    void op_LDX(uint8_t *operand);
    void op_LDY(uint8_t *operand);
    void op_LSR(uint8_t *operand);
    void op_NOP(uint8_t *operand);
    void op_ORA(uint8_t *operand);
    void op_PHA(uint8_t *operand);
    void op_PHP(uint8_t *operand);
    void op_PLA(uint8_t *operand);
    void op_PLP(uint8_t *operand);
    void op_ROL(uint8_t *operand);
    void op_ROR(uint8_t *operand);
    void op_RTI(uint8_t *operand);
    void op_RTS(uint8_t *operand);
    void op_SBC(uint8_t *operand);
    void op_SEC(uint8_t *operand);
    void op_SED(uint8_t *operand);
    void op_SEI(uint8_t *operand);
    void op_STA(uint8_t *operand);
    void op_STX(uint8_t *operand);
    void op_STY(uint8_t *operand);
    void op_TAX(uint8_t *operand);
    void op_TAY(uint8_t *operand);
    void op_TSX(uint8_t *operand);
    void op_TXA(uint8_t *operand);
    void op_TXS(uint8_t *operand);
    void op_TYA(uint8_t *operand);

    void op_ILLEGAL(uint8_t *operand); // illegal opcode

public:
    MOS6502();
    MOS6502(string filename, uint16_t location = reg_PC);

    bool load(string filename, uint16_t location);
    void reset();
    void run(uint16_t steps);
    void step();

    uint8_t get_memory(uint16_t location = reg_PC);
    uint8_t get_A();
    uint8_t get_X();
    uint8_t get_Y();
    uint8_t get_SP();
    uint16_t get_PC();
    uint8_t get_status();

    void MOS6502::set_memory(uint16_t location, uint8_t memory_val);
    void MOS6502::set_A(uint8_t val);
    void MOS6502::set_X(uint8_t val);
    void MOS6502::set_Y(uint8_t val);
    void MOS6502::set_SP(uint8_t val);
    void MOS6502::set_PC(uint16_t val);
    void MOS6502::set_status(uint8_t status_byte);

};
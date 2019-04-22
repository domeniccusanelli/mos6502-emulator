#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdint.h>
#include <map>
using namespace std;

#define MEMORY_SIZE 0x10000
#define RESET_LOW 0xFFFC     // RESET vector low byte
#define RESET_HIGH 0xFFFD    // RESET vector high byte
#define IRQ_LOW 0xFFFE       // IRQ vector low byte
#define IRQ_HIGH 0xFFFF      // IRQ vector high byte
#define SP_START 0x1FD       // Stack Pointer start address

#define LOW_BYTE 0xFF
#define HIGH_BYTE 0xFF
#define NIBBLE 0x0F
#define BYTE_HIGH_BIT 0x80
#define BYTE_LOW_BIT 0x01
#define CARRY_BIT 0x100

class MOS6502
{
private:
    typedef void (MOS6502::*op_ptr)(uint8_t*);

    uint8_t memory[MEMORY_SIZE];
    uint8_t reg_A; // Accumulator
    uint8_t reg_X;
    uint8_t reg_Y;
    uint16_t reg_SP; // Stack Pointer
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
    void execute(Instruction instr);

    uint8_t * operand_from_mode(Mode m);

    map<uint8_t, Instruction> decoder = 
    {{0x00, {&MOS6502::op_BRK, IMP}},
     {0x01, {&MOS6502::op_ORA, IIX}},
     {0x02, {&MOS6502::op_ILLEGAL, IMP}},
     {0x03, {&MOS6502::op_ILLEGAL, IMP}},
     {0x04, {&MOS6502::op_ILLEGAL, IMP}},
     {0x05, {&MOS6502::op_ORA, ZPG}},
     {0x06, {&MOS6502::op_ASL, ZPG}},
     {0x07, {&MOS6502::op_ILLEGAL, IMP}},
     {0x08, {&MOS6502::op_PHP, IMP}},
     {0x09, {&MOS6502::op_ORA, IMM}},
     {0x0A, {&MOS6502::op_ASL, ACC}},
     {0x0B, {&MOS6502::op_ILLEGAL, IMP}},
     {0x0C, {&MOS6502::op_ILLEGAL, IMP}},
     {0x0D, {&MOS6502::op_ORA, ABS}},
     {0x0E, {&MOS6502::op_ASL, ABS}},
     {0x0F, {&MOS6502::op_ILLEGAL, IMP}},
     {0x10, {&MOS6502::op_BPL, REL}},
     {0x11, {&MOS6502::op_ORA, IIY}},
     {0x12, {&MOS6502::op_ILLEGAL, IMP}},
     {0x13, {&MOS6502::op_ILLEGAL, IMP}},
     {0x14, {&MOS6502::op_ILLEGAL, IMP}},
     {0x15, {&MOS6502::op_ORA, ZPX}},
     {0x16, {&MOS6502::op_ASL, ZPX}},
     {0x17, {&MOS6502::op_ILLEGAL, IMP}},
     {0x18, {&MOS6502::op_CLC, IMP}},
     {0x19, {&MOS6502::op_ORA, AIY}},
     {0x1A, {&MOS6502::op_ILLEGAL, IMP}},
     {0x1B, {&MOS6502::op_ILLEGAL, IMP}},
     {0x1C, {&MOS6502::op_ILLEGAL, IMP}},
     {0x1D, {&MOS6502::op_ORA, AIX}},
     {0x1E, {&MOS6502::op_ASL, AIX}},
     {0x1F, {&MOS6502::op_ILLEGAL, IMP}},
     {0x20, {&MOS6502::op_JSR, ABS}},
     {0x21, {&MOS6502::op_AND, IIX}},
     {0x22, {&MOS6502::op_ILLEGAL, IMP}},
     {0x23, {&MOS6502::op_ILLEGAL, IMP}},
     {0x24, {&MOS6502::op_BIT, ZPG}},
     {0x25, {&MOS6502::op_AND, ZPG}},
     {0x26, {&MOS6502::op_ROL, ZPG}},
     {0x27, {&MOS6502::op_ILLEGAL, IMP}},
     {0x28, {&MOS6502::op_PLP, IMP}},
     {0x29, {&MOS6502::op_AND, IMM}},
     {0x2A, {&MOS6502::op_ROL, ACC}},
     {0x2B, {&MOS6502::op_ILLEGAL, IMP}},
     {0x2C, {&MOS6502::op_BIT, ABS}},
     {0x2D, {&MOS6502::op_AND, ABS}},
     {0x2E, {&MOS6502::op_ROL, ABS}},
     {0x2F, {&MOS6502::op_ILLEGAL, IMP}},
     {0x30, {&MOS6502::op_BMI, REL}},
     {0x31, {&MOS6502::op_AND, IIY}},
     {0x32, {&MOS6502::op_ILLEGAL, IMP}},
     {0x33, {&MOS6502::op_ILLEGAL, IMP}},
     {0x34, {&MOS6502::op_ILLEGAL, IMP}},
     {0x35, {&MOS6502::op_AND, ZPX}},
     {0x36, {&MOS6502::op_ROL, ZPX}},
     {0x37, {&MOS6502::op_ILLEGAL, IMP}},
     {0x38, {&MOS6502::op_SEC, IMP}},
     {0x39, {&MOS6502::op_AND, AIY}},
     {0x3A, {&MOS6502::op_ILLEGAL, IMP}},
     {0x3B, {&MOS6502::op_ILLEGAL, IMP}},
     {0x3C, {&MOS6502::op_ILLEGAL, IMP}},
     {0x3D, {&MOS6502::op_AND, AIX}},
     {0x3E, {&MOS6502::op_ROL, AIX}},
     {0x3F, {&MOS6502::op_ILLEGAL, IMP}},
     {0x40, {&MOS6502::op_RTI, IMP}},
     {0x41, {&MOS6502::op_EOR, IIX}},
     {0x42, {&MOS6502::op_ILLEGAL, IMP}},
     {0x43, {&MOS6502::op_ILLEGAL, IMP}},
     {0x44, {&MOS6502::op_ILLEGAL, IMP}},
     {0x45, {&MOS6502::op_EOR, ZPG}},
     {0x46, {&MOS6502::op_LSR, ZPG}},
     {0x47, {&MOS6502::op_ILLEGAL, IMP}},
     {0x48, {&MOS6502::op_PHA, IMP}},
     {0x49, {&MOS6502::op_EOR, IMM}},
     {0x4A, {&MOS6502::op_LSR, ACC}},
     {0x4B, {&MOS6502::op_ILLEGAL, IMP}},
     {0x4C, {&MOS6502::op_JMP, ABS}},
     {0x4D, {&MOS6502::op_EOR, ABS}},
     {0x4E, {&MOS6502::op_LSR, ABS}},
     {0x4F, {&MOS6502::op_ILLEGAL, IMP}},
     {0x50, {&MOS6502::op_BVC, REL}},
     {0x51, {&MOS6502::op_EOR, IIY}},
     {0x52, {&MOS6502::op_ILLEGAL, IMP}},
     {0x53, {&MOS6502::op_ILLEGAL, IMP}},
     {0x54, {&MOS6502::op_ILLEGAL, IMP}},
     {0x55, {&MOS6502::op_EOR, ZPX}},
     {0x56, {&MOS6502::op_LSR, ZPX}},
     {0x57, {&MOS6502::op_ILLEGAL, IMP}},
     {0x58, {&MOS6502::op_CLI, IMP}},
     {0x59, {&MOS6502::op_EOR, AIY}},
     {0x5A, {&MOS6502::op_ILLEGAL, IMP}},
     {0x5B, {&MOS6502::op_ILLEGAL, IMP}},
     {0x5C, {&MOS6502::op_ILLEGAL, IMP}},
     {0x5D, {&MOS6502::op_EOR, AIX}},
     {0x5E, {&MOS6502::op_LSR, AIX}},
     {0x5F, {&MOS6502::op_ILLEGAL, IMP}},
     {0x60, {&MOS6502::op_RTS, IMP}},
     {0x61, {&MOS6502::op_ADC, IIX}},
     {0x62, {&MOS6502::op_ILLEGAL, IMP}},
     {0x63, {&MOS6502::op_ILLEGAL, IMP}},
     {0x64, {&MOS6502::op_ILLEGAL, IMP}},
     {0x65, {&MOS6502::op_ADC, ZPG}},
     {0x66, {&MOS6502::op_ROR, ZPG}},
     {0x67, {&MOS6502::op_ILLEGAL, IMP}},
     {0x68, {&MOS6502::op_PLA, IMP}},
     {0x69, {&MOS6502::op_ADC, IMM}},
     {0x6A, {&MOS6502::op_ROR, ACC}},
     {0x6B, {&MOS6502::op_ILLEGAL, IMP}},
     {0x6C, {&MOS6502::op_JMP, IND}},
     {0x6D, {&MOS6502::op_ADC, ABS}},
     {0x6E, {&MOS6502::op_ROR, ABS}},
     {0x6F, {&MOS6502::op_ILLEGAL, IMP}},
     {0x70, {&MOS6502::op_BVS, REL}},
     {0x71, {&MOS6502::op_ADC, IIY}},
     {0x72, {&MOS6502::op_ILLEGAL, IMP}},
     {0x73, {&MOS6502::op_ILLEGAL, IMP}},
     {0x74, {&MOS6502::op_ILLEGAL, IMP}},
     {0x75, {&MOS6502::op_ADC, ZPX}},
     {0x76, {&MOS6502::op_ROR, ZPX}},
     {0x77, {&MOS6502::op_ILLEGAL, IMP}},
     {0x78, {&MOS6502::op_SEI, IMP}},
     {0x79, {&MOS6502::op_ADC, AIY}},
     {0x7A, {&MOS6502::op_ILLEGAL, IMP}},
     {0x7B, {&MOS6502::op_ILLEGAL, IMP}},
     {0x7C, {&MOS6502::op_ILLEGAL, IMP}},
     {0x7D, {&MOS6502::op_ADC, AIX}},
     {0x7E, {&MOS6502::op_ROR, AIX}},
     {0x7F, {&MOS6502::op_ILLEGAL, IMP}},
     {0x80, {&MOS6502::op_ILLEGAL, IMP}},
     {0x81, {&MOS6502::op_STA, IIX}},
     {0x82, {&MOS6502::op_ILLEGAL, IMP}},
     {0x83, {&MOS6502::op_ILLEGAL, IMP}},
     {0x84, {&MOS6502::op_STY, ZPG}},
     {0x85, {&MOS6502::op_STA, ZPG}},
     {0x86, {&MOS6502::op_STX, ZPG}},
     {0x87, {&MOS6502::op_ILLEGAL, IMP}},
     {0x88, {&MOS6502::op_DEY, IMP}},
     {0x89, {&MOS6502::op_ILLEGAL, IMP}},
     {0x8A, {&MOS6502::op_TXA, IMP}},
     {0x8B, {&MOS6502::op_ILLEGAL, IMP}},
     {0x8C, {&MOS6502::op_STY, ABS}},
     {0x8D, {&MOS6502::op_STA, ABS}},
     {0x8E, {&MOS6502::op_STX, ABS}},
     {0x8F, {&MOS6502::op_ILLEGAL, IMP}},
     {0x90, {&MOS6502::op_BCC, REL}},
     {0x91, {&MOS6502::op_STA, IIY}},
     {0x92, {&MOS6502::op_ILLEGAL, IMP}},
     {0x93, {&MOS6502::op_ILLEGAL, IMP}},
     {0x94, {&MOS6502::op_STY, ZPX}},
     {0x95, {&MOS6502::op_STA, ZPX}},
     {0x96, {&MOS6502::op_STX, ZPY}},
     {0x97, {&MOS6502::op_ILLEGAL, IMP}},
     {0x98, {&MOS6502::op_TYA, IMP}},
     {0x99, {&MOS6502::op_STA, AIY}},
     {0x9A, {&MOS6502::op_TXS, IMP}},
     {0x9B, {&MOS6502::op_ILLEGAL, IMP}},
     {0x9C, {&MOS6502::op_ILLEGAL, IMP}},
     {0x9D, {&MOS6502::op_STA, AIX}},
     {0x9E, {&MOS6502::op_ILLEGAL, IMP}},
     {0x9F, {&MOS6502::op_ILLEGAL, IMP}},
     {0xA0, {&MOS6502::op_LDY, IMM}},
     {0xA1, {&MOS6502::op_LDA, IIX}},
     {0xA2, {&MOS6502::op_LDX, IMM}},
     {0xA3, {&MOS6502::op_ILLEGAL, IMP}},
     {0xA4, {&MOS6502::op_LDY, ZPG}},
     {0xA5, {&MOS6502::op_LDA, ZPG}},
     {0xA6, {&MOS6502::op_LDX, ZPG}},
     {0xA7, {&MOS6502::op_ILLEGAL, IMP}},
     {0xA8, {&MOS6502::op_TAY, IMP}},
     {0xA9, {&MOS6502::op_LDA, IMM}},
     {0xAA, {&MOS6502::op_TAX, IMP}},
     {0xAB, {&MOS6502::op_ILLEGAL, IMP}},
     {0xAC, {&MOS6502::op_LDY, ABS}},
     {0xAD, {&MOS6502::op_LDA, ABS}},
     {0xAE, {&MOS6502::op_LDX, ABS}},
     {0xAF, {&MOS6502::op_ILLEGAL, IMP}},
     {0xB0, {&MOS6502::op_BCS, REL}},
     {0xB1, {&MOS6502::op_LDA, IIY}},
     {0xB2, {&MOS6502::op_ILLEGAL, IMP}},
     {0xB3, {&MOS6502::op_ILLEGAL, IMP}},
     {0xB4, {&MOS6502::op_LDY, ZPX}},
     {0xB5, {&MOS6502::op_LDA, ZPX}},
     {0xB6, {&MOS6502::op_LDX, ZPY}},
     {0xB7, {&MOS6502::op_ILLEGAL, IMP}},
     {0xB8, {&MOS6502::op_CLV, IMP}},
     {0xB9, {&MOS6502::op_LDA, AIY}},
     {0xBA, {&MOS6502::op_TSX, IMP}},
     {0xBB, {&MOS6502::op_ILLEGAL, IMP}},
     {0xBC, {&MOS6502::op_LDY, AIX}},
     {0xBD, {&MOS6502::op_LDA, AIX}},
     {0xBE, {&MOS6502::op_LDX, AIY}},
     {0xBF, {&MOS6502::op_ILLEGAL, IMP}},
     {0xC0, {&MOS6502::op_CPY, IMM}},
     {0xC1, {&MOS6502::op_CMP, IIX}},
     {0xC2, {&MOS6502::op_ILLEGAL, IMP}},
     {0xC3, {&MOS6502::op_ILLEGAL, IMP}},
     {0xC4, {&MOS6502::op_CPY, ZPG}},
     {0xC5, {&MOS6502::op_CMP, ZPG}},
     {0xC6, {&MOS6502::op_DEC, ZPG}},
     {0xC7, {&MOS6502::op_ILLEGAL, IMP}},
     {0xC8, {&MOS6502::op_INY, IMP}},
     {0xC9, {&MOS6502::op_CMP, IMM}},
     {0xCA, {&MOS6502::op_DEX, IMP}},
     {0xCB, {&MOS6502::op_ILLEGAL, IMP}},
     {0xCC, {&MOS6502::op_CPY, ABS}},
     {0xCD, {&MOS6502::op_CMP, ABS}},
     {0xCE, {&MOS6502::op_DEC, ABS}},
     {0xCF, {&MOS6502::op_ILLEGAL, IMP}},
     {0xD0, {&MOS6502::op_BNE, REL}},
     {0xD1, {&MOS6502::op_CMP, IIY}},
     {0xD2, {&MOS6502::op_ILLEGAL, IMP}},
     {0xD3, {&MOS6502::op_ILLEGAL, IMP}},
     {0xD4, {&MOS6502::op_ILLEGAL, IMP}},
     {0xD5, {&MOS6502::op_CMP, ZPX}},
     {0xD6, {&MOS6502::op_DEC, ZPX}},
     {0xD7, {&MOS6502::op_ILLEGAL, IMP}},
     {0xD8, {&MOS6502::op_CLD, IMP}},
     {0xD9, {&MOS6502::op_CMP, AIY}},
     {0xDA, {&MOS6502::op_ILLEGAL, IMP}},
     {0xDB, {&MOS6502::op_ILLEGAL, IMP}},
     {0xDC, {&MOS6502::op_ILLEGAL, IMP}},
     {0xDD, {&MOS6502::op_CMP, AIX}},
     {0xDE, {&MOS6502::op_DEC, AIX}},
     {0xDF, {&MOS6502::op_ILLEGAL, IMP}},
     {0xE0, {&MOS6502::op_CPX, IMM}},
     {0xE1, {&MOS6502::op_SBC, IIX}},
     {0xE2, {&MOS6502::op_ILLEGAL, IMP}},
     {0xE3, {&MOS6502::op_ILLEGAL, IMP}},
     {0xE4, {&MOS6502::op_CPX, ZPG}},
     {0xE5, {&MOS6502::op_SBC, ZPG}},
     {0xE6, {&MOS6502::op_INC, ZPG}},
     {0xE7, {&MOS6502::op_ILLEGAL, IMP}},
     {0xE8, {&MOS6502::op_INX, IMP}},
     {0xE9, {&MOS6502::op_SBC, IMM}},
     {0xEA, {&MOS6502::op_NOP, IMP}},
     {0xEB, {&MOS6502::op_ILLEGAL, IMP}},
     {0xEC, {&MOS6502::op_CPX, ABS}},
     {0xED, {&MOS6502::op_SBC, ABS}},
     {0xEE, {&MOS6502::op_INC, ABS}},
     {0xEF, {&MOS6502::op_ILLEGAL, IMP}},
     {0xF0, {&MOS6502::op_BEQ, REL}},
     {0xF1, {&MOS6502::op_SBC, IIY}},
     {0xF2, {&MOS6502::op_ILLEGAL, IMP}},
     {0xF3, {&MOS6502::op_ILLEGAL, IMP}},
     {0xF4, {&MOS6502::op_ILLEGAL, IMP}},
     {0xF5, {&MOS6502::op_SBC, ZPX}},
     {0xF6, {&MOS6502::op_INC, ZPX}},
     {0xF7, {&MOS6502::op_ILLEGAL, IMP}},
     {0xF8, {&MOS6502::op_SED, IMP}},
     {0xF9, {&MOS6502::op_SBC, AIY}},
     {0xFA, {&MOS6502::op_ILLEGAL, IMP}},
     {0xFB, {&MOS6502::op_ILLEGAL, IMP}},
     {0xFC, {&MOS6502::op_ILLEGAL, IMP}},
     {0xFD, {&MOS6502::op_SBC, AIX}},
     {0xFE, {&MOS6502::op_INC, AIX}},
     {0xFF, {&MOS6502::op_ILLEGAL, IMP}}};

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
    MOS6502(string filename);
    MOS6502(string filename, uint16_t location);

    bool load(string filename);
    bool load(string filename, uint16_t location);
    void reset();
    void run(uint16_t steps);
    void step();

    uint8_t get_memory(uint16_t location);
    uint8_t get_A();
    uint8_t get_X();
    uint8_t get_Y();
    uint8_t get_SP();
    uint16_t get_PC();
    uint8_t get_status();

    void set_memory(uint16_t location, uint8_t memory_val);
    void set_A(uint8_t val);
    void set_X(uint8_t val);
    void set_Y(uint8_t val);
    void set_SP(uint8_t val);
    void set_PC(uint16_t val);
    void set_status(uint8_t status_byte);

};
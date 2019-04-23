#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
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
        string op_name;
    };

    string last_instruction;
    string last_operand;

    void initialize();

    uint8_t fetch();
    Instruction decode(uint8_t byte);
    void execute(Instruction instr);

    uint8_t * operand_from_mode(Mode m);

    string to_hex_string(uint16_t num);

    map<uint8_t, Instruction> decoder = 
    {{0x00, {&MOS6502::op_BRK, IMP, "BRK"}},
     {0x01, {&MOS6502::op_ORA, IIX, "ORA"}},
     {0x02, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x03, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x04, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x05, {&MOS6502::op_ORA, ZPG, "ORA"}},
     {0x06, {&MOS6502::op_ASL, ZPG, "ASL"}},
     {0x07, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x08, {&MOS6502::op_PHP, IMP, "PHP"}},
     {0x09, {&MOS6502::op_ORA, IMM, "ORA"}},
     {0x0A, {&MOS6502::op_ASL, ACC, "ASL"}},
     {0x0B, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x0C, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x0D, {&MOS6502::op_ORA, ABS, "ORA"}},
     {0x0E, {&MOS6502::op_ASL, ABS, "ASL"}},
     {0x0F, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x10, {&MOS6502::op_BPL, REL, "BPL"}},
     {0x11, {&MOS6502::op_ORA, IIY, "ORA"}},
     {0x12, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x13, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x14, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x15, {&MOS6502::op_ORA, ZPX, "ORA"}},
     {0x16, {&MOS6502::op_ASL, ZPX, "ASL"}},
     {0x17, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x18, {&MOS6502::op_CLC, IMP, "CLC"}},
     {0x19, {&MOS6502::op_ORA, AIY, "ORA"}},
     {0x1A, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x1B, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x1C, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x1D, {&MOS6502::op_ORA, AIX, "ORA"}},
     {0x1E, {&MOS6502::op_ASL, AIX, "ASL"}},
     {0x1F, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x20, {&MOS6502::op_JSR, ABS, "JSR"}},
     {0x21, {&MOS6502::op_AND, IIX, "AND"}},
     {0x22, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x23, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x24, {&MOS6502::op_BIT, ZPG, "BIT"}},
     {0x25, {&MOS6502::op_AND, ZPG, "AND"}},
     {0x26, {&MOS6502::op_ROL, ZPG, "ROL"}},
     {0x27, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x28, {&MOS6502::op_PLP, IMP, "PLP"}},
     {0x29, {&MOS6502::op_AND, IMM, "AND"}},
     {0x2A, {&MOS6502::op_ROL, ACC, "ROL"}},
     {0x2B, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x2C, {&MOS6502::op_BIT, ABS, "BIT"}},
     {0x2D, {&MOS6502::op_AND, ABS, "AND"}},
     {0x2E, {&MOS6502::op_ROL, ABS, "ROL"}},
     {0x2F, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x30, {&MOS6502::op_BMI, REL, "BMI"}},
     {0x31, {&MOS6502::op_AND, IIY, "AND"}},
     {0x32, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x33, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x34, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x35, {&MOS6502::op_AND, ZPX, "AND"}},
     {0x36, {&MOS6502::op_ROL, ZPX, "ROL"}},
     {0x37, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x38, {&MOS6502::op_SEC, IMP, "SEC"}},
     {0x39, {&MOS6502::op_AND, AIY, "AND"}},
     {0x3A, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x3B, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x3C, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x3D, {&MOS6502::op_AND, AIX, "AND"}},
     {0x3E, {&MOS6502::op_ROL, AIX, "ROL"}},
     {0x3F, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x40, {&MOS6502::op_RTI, IMP, "RTI"}},
     {0x41, {&MOS6502::op_EOR, IIX, "EOR"}},
     {0x42, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x43, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x44, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x45, {&MOS6502::op_EOR, ZPG, "EOR"}},
     {0x46, {&MOS6502::op_LSR, ZPG, "LSR"}},
     {0x47, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x48, {&MOS6502::op_PHA, IMP, "PHA"}},
     {0x49, {&MOS6502::op_EOR, IMM, "EOR"}},
     {0x4A, {&MOS6502::op_LSR, ACC, "LSR"}},
     {0x4B, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x4C, {&MOS6502::op_JMP, ABS, "JMP"}},
     {0x4D, {&MOS6502::op_EOR, ABS, "EOR"}},
     {0x4E, {&MOS6502::op_LSR, ABS, "LSR"}},
     {0x4F, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x50, {&MOS6502::op_BVC, REL, "BVC"}},
     {0x51, {&MOS6502::op_EOR, IIY, "EOR"}},
     {0x52, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x53, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x54, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x55, {&MOS6502::op_EOR, ZPX, "EOR"}},
     {0x56, {&MOS6502::op_LSR, ZPX, "LSR"}},
     {0x57, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x58, {&MOS6502::op_CLI, IMP, "CLI"}},
     {0x59, {&MOS6502::op_EOR, AIY, "EOR"}},
     {0x5A, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x5B, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x5C, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x5D, {&MOS6502::op_EOR, AIX, "EOR"}},
     {0x5E, {&MOS6502::op_LSR, AIX, "LSR"}},
     {0x5F, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x60, {&MOS6502::op_RTS, IMP, "RTS"}},
     {0x61, {&MOS6502::op_ADC, IIX, "ADC"}},
     {0x62, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x63, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x64, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x65, {&MOS6502::op_ADC, ZPG, "ADC"}},
     {0x66, {&MOS6502::op_ROR, ZPG, "ROR"}},
     {0x67, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x68, {&MOS6502::op_PLA, IMP, "PLA"}},
     {0x69, {&MOS6502::op_ADC, IMM, "ADC"}},
     {0x6A, {&MOS6502::op_ROR, ACC, "ROR"}},
     {0x6B, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x6C, {&MOS6502::op_JMP, IND, "JMP"}},
     {0x6D, {&MOS6502::op_ADC, ABS, "ADC"}},
     {0x6E, {&MOS6502::op_ROR, ABS, "ROR"}},
     {0x6F, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x70, {&MOS6502::op_BVS, REL, "BVS"}},
     {0x71, {&MOS6502::op_ADC, IIY, "ADC"}},
     {0x72, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x73, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x74, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x75, {&MOS6502::op_ADC, ZPX, "ADC"}},
     {0x76, {&MOS6502::op_ROR, ZPX, "ROR"}},
     {0x77, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x78, {&MOS6502::op_SEI, IMP, "SEI"}},
     {0x79, {&MOS6502::op_ADC, AIY, "ADC"}},
     {0x7A, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x7B, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x7C, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x7D, {&MOS6502::op_ADC, AIX, "ADC"}},
     {0x7E, {&MOS6502::op_ROR, AIX, "ROR"}},
     {0x7F, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x80, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x81, {&MOS6502::op_STA, IIX, "STA"}},
     {0x82, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x83, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x84, {&MOS6502::op_STY, ZPG, "STY"}},
     {0x85, {&MOS6502::op_STA, ZPG, "STA"}},
     {0x86, {&MOS6502::op_STX, ZPG, "STX"}},
     {0x87, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x88, {&MOS6502::op_DEY, IMP, "DEY"}},
     {0x89, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x8A, {&MOS6502::op_TXA, IMP, "TXA"}},
     {0x8B, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x8C, {&MOS6502::op_STY, ABS, "STY"}},
     {0x8D, {&MOS6502::op_STA, ABS, "STA"}},
     {0x8E, {&MOS6502::op_STX, ABS, "STX"}},
     {0x8F, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x90, {&MOS6502::op_BCC, REL, "BCC"}},
     {0x91, {&MOS6502::op_STA, IIY, "STA"}},
     {0x92, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x93, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x94, {&MOS6502::op_STY, ZPX, "STY"}},
     {0x95, {&MOS6502::op_STA, ZPX, "STA"}},
     {0x96, {&MOS6502::op_STX, ZPY, "STX"}},
     {0x97, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x98, {&MOS6502::op_TYA, IMP, "TYA"}},
     {0x99, {&MOS6502::op_STA, AIY, "STA"}},
     {0x9A, {&MOS6502::op_TXS, IMP, "TXS"}},
     {0x9B, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x9C, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x9D, {&MOS6502::op_STA, AIX, "STA"}},
     {0x9E, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0x9F, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xA0, {&MOS6502::op_LDY, IMM, "LDY"}},
     {0xA1, {&MOS6502::op_LDA, IIX, "LDA"}},
     {0xA2, {&MOS6502::op_LDX, IMM, "LDX"}},
     {0xA3, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xA4, {&MOS6502::op_LDY, ZPG, "LDY"}},
     {0xA5, {&MOS6502::op_LDA, ZPG, "LDA"}},
     {0xA6, {&MOS6502::op_LDX, ZPG, "LDX"}},
     {0xA7, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xA8, {&MOS6502::op_TAY, IMP, "TAY"}},
     {0xA9, {&MOS6502::op_LDA, IMM, "LDA"}},
     {0xAA, {&MOS6502::op_TAX, IMP, "TAX"}},
     {0xAB, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xAC, {&MOS6502::op_LDY, ABS, "LDY"}},
     {0xAD, {&MOS6502::op_LDA, ABS, "LDA"}},
     {0xAE, {&MOS6502::op_LDX, ABS, "LDX"}},
     {0xAF, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xB0, {&MOS6502::op_BCS, REL, "BCS"}},
     {0xB1, {&MOS6502::op_LDA, IIY, "LDA"}},
     {0xB2, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xB3, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xB4, {&MOS6502::op_LDY, ZPX, "LDY"}},
     {0xB5, {&MOS6502::op_LDA, ZPX, "LDA"}},
     {0xB6, {&MOS6502::op_LDX, ZPY, "LDX"}},
     {0xB7, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xB8, {&MOS6502::op_CLV, IMP, "CLV"}},
     {0xB9, {&MOS6502::op_LDA, AIY, "LDA"}},
     {0xBA, {&MOS6502::op_TSX, IMP, "TSX"}},
     {0xBB, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xBC, {&MOS6502::op_LDY, AIX, "LDY"}},
     {0xBD, {&MOS6502::op_LDA, AIX, "LDA"}},
     {0xBE, {&MOS6502::op_LDX, AIY, "LDX"}},
     {0xBF, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xC0, {&MOS6502::op_CPY, IMM, "CPY"}},
     {0xC1, {&MOS6502::op_CMP, IIX, "CMP"}},
     {0xC2, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xC3, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xC4, {&MOS6502::op_CPY, ZPG, "CPY"}},
     {0xC5, {&MOS6502::op_CMP, ZPG, "CMP"}},
     {0xC6, {&MOS6502::op_DEC, ZPG, "DEC"}},
     {0xC7, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xC8, {&MOS6502::op_INY, IMP, "INY"}},
     {0xC9, {&MOS6502::op_CMP, IMM, "CMP"}},
     {0xCA, {&MOS6502::op_DEX, IMP, "DEX"}},
     {0xCB, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xCC, {&MOS6502::op_CPY, ABS, "CPY"}},
     {0xCD, {&MOS6502::op_CMP, ABS, "CMP"}},
     {0xCE, {&MOS6502::op_DEC, ABS, "DEC"}},
     {0xCF, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xD0, {&MOS6502::op_BNE, REL, "BNE"}},
     {0xD1, {&MOS6502::op_CMP, IIY, "CMP"}},
     {0xD2, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xD3, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xD4, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xD5, {&MOS6502::op_CMP, ZPX, "CMP"}},
     {0xD6, {&MOS6502::op_DEC, ZPX, "DEC"}},
     {0xD7, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xD8, {&MOS6502::op_CLD, IMP, "CLD"}},
     {0xD9, {&MOS6502::op_CMP, AIY, "CMP"}},
     {0xDA, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xDB, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xDC, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xDD, {&MOS6502::op_CMP, AIX, "CMP"}},
     {0xDE, {&MOS6502::op_DEC, AIX, "DEC"}},
     {0xDF, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xE0, {&MOS6502::op_CPX, IMM, "CPX"}},
     {0xE1, {&MOS6502::op_SBC, IIX, "SBC"}},
     {0xE2, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xE3, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xE4, {&MOS6502::op_CPX, ZPG, "CPX"}},
     {0xE5, {&MOS6502::op_SBC, ZPG, "SBC"}},
     {0xE6, {&MOS6502::op_INC, ZPG, "INC"}},
     {0xE7, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xE8, {&MOS6502::op_INX, IMP, "INX"}},
     {0xE9, {&MOS6502::op_SBC, IMM, "SBC"}},
     {0xEA, {&MOS6502::op_NOP, IMP, "NOP"}},
     {0xEB, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xEC, {&MOS6502::op_CPX, ABS, "CPX"}},
     {0xED, {&MOS6502::op_SBC, ABS, "SBC"}},
     {0xEE, {&MOS6502::op_INC, ABS, "INC"}},
     {0xEF, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xF0, {&MOS6502::op_BEQ, REL, "BEQ"}},
     {0xF1, {&MOS6502::op_SBC, IIY, "SBC"}},
     {0xF2, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xF3, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xF4, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xF5, {&MOS6502::op_SBC, ZPX, "SBC"}},
     {0xF6, {&MOS6502::op_INC, ZPX, "INC"}},
     {0xF7, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xF8, {&MOS6502::op_SED, IMP, "SED"}},
     {0xF9, {&MOS6502::op_SBC, AIY, "SBC"}},
     {0xFA, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xFB, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xFC, {&MOS6502::op_ILLEGAL, IMP, "ILL"}},
     {0xFD, {&MOS6502::op_SBC, AIX, "SBC"}},
     {0xFE, {&MOS6502::op_INC, AIX, "INC"}},
     {0xFF, {&MOS6502::op_ILLEGAL, IMP, "ILL"}}};

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

    uint8_t get_memory();
    uint8_t get_memory(uint16_t location);
    uint8_t get_A();
    uint8_t get_X();
    uint8_t get_Y();
    uint8_t get_SP();
    uint16_t get_PC();
    uint8_t get_status();
    string get_last_instr();

    void set_memory(uint16_t location, uint8_t memory_val);
    void set_A(uint8_t val);
    void set_X(uint8_t val);
    void set_Y(uint8_t val);
    void set_SP(uint8_t val);
    void set_PC(uint16_t val);
    void set_status(uint8_t status_byte);

};
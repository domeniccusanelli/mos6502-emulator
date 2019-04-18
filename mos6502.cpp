#include "mos6502.h"

//TODO: interrupts, BRK, decimal mode, document

MOS6502::MOS6502()
{
    initialize();
}

MOS6502::MOS6502(string filename, uint16_t location)
{
    initialize();
    load(filename, location);
}

void MOS6502::initialize()
{
    // set RESET vector to first address after stack
    memory[RESET_LOW] = 0x00;
    memory[RESET_HIGH] = 0x02;
    reset();
}

void MOS6502::load(string filename, uint16_t location)
{
    FILE * rom = fopen(filename, "r");
    fread(&memory[location], 1, (size_t)(MEMORY_SIZE - location), rom);
    fclose(rom);
}

void MOS6502::reset()
{
    reg_A = 0x00;
    reg_X = 0x00;
    reg_Y = 0x00;
    reg_SP = SP_START;
    reg_PC = (memory[RESET_HIGH] << 8) | memory[RESET_LOW]; // set program counter to RESET vector
    reg_status = {0, 0, 1, 0, 0, 1, 0, 0};
}

void MOS6502::run(uint16_t steps)
{
    for (int i = 0; i < steps; i++)
    {
        step();
    }
}

void MOS6502::step()
{
    uint8_t current_instr_byte = fetch();
    Instruction decoded_instr = decode(current_instr_byte);
    execute(decoded_instr);
}

uint8_t MOS6502::fetch()
{
    return memory[reg_PC++];
}

Instruction MOS6502::decode(uint8_t byte)
{
    return decoder[byte];
}

void MOS6502::execute(Instruction instr)
{
    uint16_t = operand = operand_from_mode(instr.addr_mode);
    instr.op_func(operand);
}

uint8_t * MOS6502::operand_from_mode(Mode m)
{
    uint8_t * operand;

    switch(m)
    {
        case ACC:
            operand = &reg_A;
            break;

        case IMM:
            operand = &memory[reg_PC++];
            break;

        case ABS:
            uint16_t addr = (memory[reg_PC++] | memory[reg_PC++] << 8);
            operand = &memory[addr];
            break;

        case ZPG:
            uint16_t addr = memory[reg_PC++];
            operand = &memory[addr];
            break;

        case ZPX:
            uint16_t addr = memory[reg_PC++] + reg_X;
            operand = &memory[addr];
            break;

        case ZPY:
            uint16_t addr = memory[reg_PC++] + reg_Y;
            operand = &memory[addr];
            break;

        case AIX:
            uint16_t addr = (memory[reg_PC++] | memory[reg_PC++] << 8) + reg_X;
            operand = &memory[addr];
            break;

        case AIY:
            uint16_t addr = (memory[reg_PC++] | memory[reg_PC++] << 8) + reg_Y;
            operand = &memory[addr];
            break;

        case IMP:
            operand = &memory[reg_PC]; // unused
            break;

        case REL:
            operand = &memory[reg_PC++];
            break;

        case IIX:
            uint16_t addr_location = memory[reg_PC++] + reg_X;
            uint16_t addr = (memory[addr_location] | memory[addr_location + 1] << 8);
            operand = &memory[addr];
            break;

        case IIY:
            uint16_t addr_location = memory[pc++];
            uint16_t addr = (memory[addr_location] | memory[addr_location + 1] << 8) + reg_Y;
            operand = &memory[addr];
            break;

        case IND:
            uint16_t addr_location = (memory[reg_PC++] | memory[reg_PC++] << 8);
            uint16_t addr = (memory[addr_location] | memory[addr_location + 1] << 8)
            operand = &memory[addr]
            break;
    }

    return operand;
}

uint8_t MOS6502::get_memory(uint16_t location)
{
    return memory[location];
}

uint8_t MOS6502::get_A()
{
    return reg_A;
}

uint8_t MOS6502::get_X()
{
    return reg_X;
}

uint8_t MOS6502::get_Y()
{
    return reg_Y;
}

uint8_t MOS6502::get_SP()
{
    return reg_SP;
}

uint16_t MOS6502::get_PC()
{
    return reg_PC;
}

uint8_t MOS6502::get_status()
{
    uint8_t status_byte = (reg_status.N << 7 |
                           reg_status.V << 6 |
                           reg_status._ << 5 |
                           reg_status.B << 4 |
                           reg_status.D << 3 |
                           reg_status.I << 2 |
                           reg_status.Z << 1 |
                           reg_status.C);
    return status_byte;
}

void MOS6502::set_memory(uint16_t location, uint8_t val)
{
    memory[location] = val;
}

void MOS6502::set_A(uint8_t val)
{
    reg_A = val;
}

void MOS6502::set_X(uint8_t val)
{
    reg_X = val;
}

void MOS6502::set_Y(uint8_t val)
{
    reg_Y = val;
}

void MOS6502::set_SP(uint8_t val)
{
    reg_SP = val;
}

void MOS6502::set_PC(uint16_t val)
{
    reg_PC = val;
}

void MOS6502::set_status(uint8_t status_byte)
{
    reg_status.N = status_byte & 0x80;
    reg_status.V = status_byte & 0x40;
    reg_status._ = status_byte & 0x20;
    reg_status.B = status_byte & 0x10;
    reg_status.D = status_byte & 0x8;
    reg_status.I = status_byte & 0x4;
    reg_status.Z = status_byte & 0x2;
    reg_status.C = status_byte & 0x1;
}

void MOS6502::op_ADC(uint16_t *operand)
{
    uint8_t memory_val = *operand;
    uint16_t result = reg_A + memory_val + reg_status.C;

    if(reg_status.D)
    {
        uint16_t high_nibble = (reg_A  >> 4) + (memory_val >> 4);
        uint8_t low_nibble = (reg_A & NIBBLE) + (memory_val & NIBBLE) + reg_status.C;

        if(low_nibble > 0x09)
        {
            low_nibble += 0x6;
            low_nibble &= NIBBLE;
            high_nibble += 0x10;
        }
        if(high_nibble > 0x9)
        {
            high_nibble += 0x6;
        }

        result = (high_nibble << 4) | low_nibble;
    }

    reg_status.C = result & CARRY_BIT;
    result &= LOW_BYTE; // fit result to one byte
    reg_status.N = result & BYTE_HIGH_BIT;
    bool operand_test = !((reg_A ^ memory_val) & BYTE_HIGH_BIT);
    bool result_test = ((reg_A ^ result) & BYTE_HIGH_BIT);
    reg_status.V = operand_test && result_test;
    reg_status.Z = (result == 0);

    reg_A = result;
}

void MOS6502::op_AND(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    reg_A = reg_A & memory_val;
    reg_status.N = reg_A & BYTE_HIGH_BIT;
    reg_status.Z = (reg_A == 0);
}

void MOS6502::op_ASL(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    uint16_t result = memory_val << 1;

    reg_status.C = result & CARRY_BIT;
    result &= LOW_BYTE; // fit result to one byte
    reg_status.N = result & BYTE_HIGH_BIT;
    reg_status.Z = (result == 0);

    *operand = result;
}

void MOS6502::op_BCC(uint8_t *operand)
{
    int8_t memory_val = *operand;
    if(!reg_status.C)
    {
        reg_PC += memory_val;
    }
}

void MOS6502::op_BCS(uint8_t *operand)
{
    int8_t memory_val = *operand;
    if(reg_status.C)
    {
        reg_PC += memory_val;
    }
}

void MOS6502::op_BEQ(uint8_t *operand)
{
    int8_t memory_val = *operand;
    if(reg_status.Z)
    {
        reg_PC += memory_val;
    }
}

void MOS6502::op_BIT(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    uint16_t result = reg_A & memory_val;
    reg_status.N = memory_val & BYTE_HIGH_BIT;
    reg_status.V = memory_val & 0x40;
    reg_status.Z = (result == 0);
}

void MOS6502::op_BMI(uint8_t *operand)
{
    int8_t memory_val = *operand;
    if(reg_status.N)
    {
        reg_PC += memory_val;
    }
}

void MOS6502::op_BNE(uint8_t *operand)
{
    int8_t memory_val = *operand;
    if(!reg_status.Z)
    {
        reg_PC += memory_val;
    }
}

void MOS6502::op_BPL(uint8_t *operand)
{
    int8_t memory_val = *operand;
    if(!reg_status.N)
    {
        reg_PC += memory_val;
    }
}

void MOS6502::op_BRK(uint8_t *operand)
{

}

void MOS6502::op_BVC(uint8_t *operand)
{
    int8_t memory_val = *operand;
    if(!reg_status.V)
    {
        reg_PC += memory_val;
    }
}

void MOS6502::op_BVS(uint8_t *operand)
{
    int8_t memory_val = *operand;
    if(reg_status.V)
    {
        reg_PC += memory_val;
    }
}

void MOS6502::op_CLC(uint8_t *operand)
{
    reg_status.C = 0;
}

void MOS6502::op_CLD(uint8_t *operand)
{
    reg_status.D = 0;
}

void MOS6502::op_CLI(uint8_t *operand)
{
    reg_status.I = 0;
}

void MOS6502::op_CLV(uint8_t *operand)
{
    reg_status.V = 0;
}

void MOS6502::op_CMP(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    uint16_t result;

    result = reg_A - memory_val;

    reg_status.C = !(result & CARRY_BIT);
    result &= LOW_BYTE; // fit result to one byte
    reg_status.N = result & BYTE_HIGH_BIT;
    bool operand_test = !((reg_A ^ memory_val) & BYTE_HIGH_BIT);
    bool result_test = ((reg_A ^ result) & BYTE_HIGH_BIT);
    reg_status.V = operand_test && result_test;
}

void MOS6502::op_CPX(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    uint16_t result;

    result = reg_X - memory_val;

    reg_status.C = !(result & CARRY_BIT);
    result &= LOW_BYTE; // fit result to one byte
    reg_status.N = result & BYTE_HIGH_BIT;
    bool operand_test = !((reg_A ^ memory_val) & BYTE_HIGH_BIT);
    bool result_test = ((reg_A ^ result) & BYTE_HIGH_BIT);
    reg_status.V = operand_test && result_test;
}

void MOS6502::op_CPY(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    uint16_t result;

    result = reg_Y - memory_val;

    reg_status.C = !(result & CARRY_BIT);
    result &= LOW_BYTE; // fit result to one byte
    reg_status.N = result & BYTE_HIGH_BIT;
    bool operand_test = !((reg_A ^ memory_val) & BYTE_HIGH_BIT);
    bool result_test = ((reg_A ^ result) & BYTE_HIGH_BIT);
    reg_status.V = operand_test && result_test;
}

void MOS6502::op_DEC(uint8_t *operand)
{
    uint8_t memory_val = *operand--;
    reg_status.N = memory_val & BYTE_HIGH_BIT;
    reg_status.Z = (memory_val == 0);
}

void MOS6502::op_DEX(uint8_t *operand)
{
    reg_X--;
    reg_status.N = reg_X & BYTE_HIGH_BIT;
    reg_status.Z = (reg_X == 0);
}

void MOS6502::op_DEY(uint8_t *operand)
{
    reg_Y--;
    reg_status.N = reg_Y & BYTE_HIGH_BIT;
    reg_status.Z = (reg_Y == 0);
}

void MOS6502::op_EOR(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    reg_A = reg_A ^ memory_val;
    reg_status.N = reg_A & BYTE_HIGH_BIT;
    reg_status.Z = (reg_A == 0);
}

void MOS6502::op_INC(uint8_t *operand)
{
    memory_val = *operand++;
    reg_status.N = memory_val & BYTE_HIGH_BIT;
    reg_status.Z = (memory_val == 0);
}

void MOS6502::op_INX(uint8_t *operand)
{
    reg_X++;
    reg_status.N = reg_X & BYTE_HIGH_BIT;
    reg_status.Z = (reg_X == 0);
}

void MOS6502::op_INY(uint8_t *operand)
{
    reg_Y++;
    reg_status.N = reg_Y & BYTE_HIGH_BIT;
    reg_status.Z = (reg_Y == 0);
}

void MOS6502::op_JMP(uint8_t *operand)
{
    //TODO: insert JMP bug
    reg_PC = *operand;
}

void MOS6502::op_JSR(uint8_t *operand)
{
    reg_PC -= 1;
    memory[reg_SP--] = reg_PC & HIGH_BYTE;
    memory[reg_SP--] = reg_PC & LOW_BYTE;
    reg_PC = *operand;
}

void MOS6502::op_LDA(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    reg_A = memory_val;
    reg_status.N = reg_A & BYTE_HIGH_BIT;
    reg_status.Z = (reg_A == 0);
}

void MOS6502::op_LDX(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    reg_X = memory_val;
    reg_status.N = reg_X & BYTE_HIGH_BIT;
    reg_status.Z = (reg_X == 0);
}

void MOS6502::op_LDY(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    reg_Y = memory_val;
    reg_status.N = reg_Y & BYTE_HIGH_BIT;
    reg_status.Z = (reg_Y == 0);
}

void MOS6502::op_LSR(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    uint16_t result;
    reg_status.C = result & BYTE_LOW_BIT;

    result = memory_val >> 1;
    result &= LOW_BYTE; // fit result to one byte
    reg_status.N = result & BYTE_HIGH_BIT;
    reg_status.Z = (result == 0);

    *operand = result;
}

void MOS6502::op_NOP(uint8_t *operand)
{
    ;
}

void MOS6502::op_ORA(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    reg_A = reg_A | memory_val;
    reg_status.N = reg_A & BYTE_HIGH_BIT;
    reg_status.Z = (reg_A == 0);
}

void MOS6502::op_PHA(uint8_t *operand)
{
    memory[reg_SP--] = reg_A;
}

void MOS6502::op_PHP(uint8_t *operand)
{
    uint8_t status_byte = get_status();
    memory[reg_SP--] = status_byte;
}

void MOS6502::op_PLA(uint8_t *operand)
{
    reg_A = memory[++reg_SP];
}

void MOS6502::op_PLP(uint8_t *operand)
{
    set_status(memory[++reg_SP]);
}

void MOS6502::op_ROL(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    uint16_t result = memory_val << 1;
    result |= reg_status.C;

    reg_status.C = result & CARRY_BIT;
    result &= LOW_BYTE; // fit result to one byte
    reg_status.N = result & BYTE_HIGH_BIT;
    reg_status.Z = (result == 0);

    *operand = result;
}

void MOS6502::op_ROR(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    uint16_t result = memory_val >> 1;;
    result |= reg_status.C << 7;

    reg_status.C = memory_val & BYTE_LOW_BIT;
    result &= LOW_BYTE; // fit result to one byte
    reg_status.N = result & BYTE_HIGH_BIT;
    reg_status.Z = (result == 0);

    *operand = result;
}

void MOS6502::op_RTI(uint8_t *operand)
{
    set_status(memory[++reg_SP]);
    reg_PC = memory[++reg_SP] & (memory[++reg_SP] << 8);
}

void MOS6502::op_RTS(uint8_t *operand)
{
    reg_PC = (memory[++reg_SP] & (memory[++reg_SP] << 8)) + 1;
}

void MOS6502::op_SBC(uint8_t *operand)
{
    uint8_t memory_val = *operand;
    uint16_t result = reg_A - memory_val - (1 - reg_status.C);

    bool operand_test = !((reg_A ^ memory_val) & BYTE_HIGH_BIT);
    bool result_test = ((reg_A ^ result) & BYTE_HIGH_BIT);
    reg_status.V = operand_test && result_test;

    if(reg_status.D)
    {
        uint16_t high_nibble = (reg_A >> 4) - (memory_val >> 4);
        uint8_t low_nibble = (reg_A & NIBBLE) - (memory_val & NIBBLE)  - (1 - reg_status.C);

        if(low_nibble > 0x9)
        {
            low_nibble -= 0x6;
            low_nibble &= NIBBLE;
            high_nibble -= 0x10;
        }
        if(high_nibble > 0x9)
        {
            high_nibble -= 0x6;
        }

        result = (high_nibble << 4) | low_nibble;
    }

    reg_status.C = !(result & CARRY_BIT);
    result &= LOW_BYTE; // fit result to one byte
    reg_status.N = result & BYTE_HIGH_BIT;
    reg_status.Z = (result == 0);

    reg_A = result;
}

void MOS6502::op_SEC(uint8_t *operand)
{
    reg_status.C = 1;
}

void MOS6502::op_SED(uint8_t *operand)
{
    reg_status.D = 1;
}

void MOS6502::op_SEI(uint8_t *operand)
{
    reg_status.I = 1;
}

void MOS6502::op_STA(uint8_t *operand)
{
    *operand = reg_A;
}

void MOS6502::op_STX(uint8_t *operand)
{
    *operand = reg_X;
}

void MOS6502::op_STY(uint8_t *operand)
{
    *operand = reg_Y;
}

void MOS6502::op_TAX(uint8_t *operand)
{
    reg_X = reg_A;
    reg_status.N = reg_X & BYTE_HIGH_BIT;
    reg_status.Z = (reg_X == 0);
}

void MOS6502::op_TAY(uint8_t *operand)
{
    reg_Y = reg_A;
    reg_status.N = reg_Y & BYTE_HIGH_BIT;
    reg_status.Z = (reg_Y == 0);
}

void MOS6502::op_TSX(uint8_t *operand)
{
    reg_X = reg_SP;
    reg_status.N = reg_X & BYTE_HIGH_BIT;
    reg_status.Z = (reg_X == 0);
}

void MOS6502::op_TXA(uint8_t *operand)
{
    reg_A = reg_X;
    reg_status.N = reg_A & BYTE_HIGH_BIT;
    reg_status.Z = (reg_A == 0);
}

void MOS6502::op_TXS(uint8_t *operand)
{
    reg_SP = reg_X;
}

void MOS6502::op_TYA(uint8_t *operand)
{
    reg_A = reg_Y;
    reg_status.N = reg_A & BYTE_HIGH_BIT;
    reg_status.Z = (reg_A == 0);
}

void MOS6502::op_ILLEGAL(uint8_t *operand)
{
    cerr << "Undefined instruction at memory location 0x" << hex << (reg_PC - 1) << endl;
}
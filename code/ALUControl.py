
ADD = 0b0010
SUB = 0b0110
AND = 0b0000
OR = 0b0001


def ALU_control(opcode, funct7, funct3, ALUop):
    if ALUop == 0b00:
        return ADD
    elif ALUop == 0b01:
        return SUB
    elif ALUop == 0b10:
        if opcode == '1101111':
            return ADD
        elif funct7 == '0100000':
            return SUB
        elif funct3 == '000':
            return ADD
        elif funct3 == '111':
            return AND
        elif funct3 == '110' or funct3 == '100':
            return OR
        else:
            return ADD
    else:
        return ADD



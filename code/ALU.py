from convertor import *
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


def ALU(ALU_control, ins, input1_raw, input2_raw):

    if (type(input1_raw) == int):
        input1 = input1_raw
    else: 
        input1 = bitstring_to_int(input1_raw)

    if (type(input2_raw) == int):
        input2 = input2_raw
    else: 
        input2 = bitstring_to_int(input2_raw)
    # input2 = bitstring_to_int(input2_raw)
    # input1 = input1_raw
    # input2 = input2_raw
    if ALU_control == ADD:
        output = do_add(input1, input2)
    elif ALU_control == SUB:
        output = do_sub(input1, input2)
    elif ALU_control == AND:
        output = do_and(input1, input2)
    elif ALU_control == OR:
        if ins == 'OR' or ins == 'ORI':
            output = do_or(input1, input2)
        else:
            output = do_xor(input1, input2)

    return output


def do_add(input1, input2):
    return input1 + input2


def do_sub(input1, input2):
    return input1 - input2


def do_and(input1, input2):
    return input1 & input2


def do_or(input1, input2):
    return input1 | input2


def do_xor(input1, input2):
    return input1 ^ input2
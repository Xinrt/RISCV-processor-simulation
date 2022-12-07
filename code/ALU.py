from convertor import *

ADD = 0b0010
SUB = 0b0110
AND = 0b0000
OR = 0b0001


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
        output = input1 + input2
    elif ALU_control == SUB:
        output = input1 - input2
    elif ALU_control == AND:
        output = input1 & input2
    elif ALU_control == OR:
        if ins == 'OR' or ins == 'ORI':
            output = input1 | input2
        else:
            output = input1 ^ input2

    return output


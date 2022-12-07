from convertor import *


class Parser(object):
    def __init__(self, instr) -> None:
        assert (len(instr) == 32)

        self.instr = instr
        self.funct7 = self.instr[:7]
        self.funct3 = self.instr[17:20]
        self.opcode = self.instr[25:]

    def parse(self):

        type = self.type_decode()
        rs1 = self.instr[12:17]
        rs2 = self.instr[7:12]
        rd = self.instr[20:25]

        instr = ''
        if type == 'R':
            instr = self.R_decode()

        elif type == 'I':
            instr = self.I_decode()

        elif type == 'S':
            instr = self.S_decode()

        elif type == 'B':
            instr = self.B_decode()

        elif type == 'J':
            instr = self.J_decode()
        else:
            instr = 'HALT'

        return type, instr, rs2, rs1, rd


    def type_decode(self):
        opcode = self.opcode

        if opcode == '0110011':
            return 'R'
        elif opcode == '0010011' or opcode == '0000011':
            return 'I'
        elif opcode == '1101111':
            return 'J'
        elif opcode == '1100011':
            return 'B'
        elif opcode == '0100011':
            return 'S'
        elif opcode == '1111111':  
            return 'H'
        else:
            raise Exception('Wrong instruction type')

    def R_decode(self):
        funct3 = self.funct3
        funct7 = self.funct7

        if funct3 == '000':
            if funct7 == '0000000':
                return 'ADD'
            elif funct7 == '0100000':
                return 'SUB'
            else:
                raise Exception('Wrong R Type instruction.')
        elif funct3 == '100':
            return 'XOR'
        elif funct3 == '110':
            return 'OR'
        elif funct3 == '111':
            return 'AND'
        else:
            raise Exception('Wrong R Type instruction.')

    def B_decode(self):
        funct3 = self.funct3

        if funct3 == '000':
            return 'BEQ'
        elif funct3 == '001':
            return 'BNE'
        else:
            raise Exception('Wrong B Type instruction.')

    def S_decode(self):
        return 'SW'

    def J_decode(self):
        return 'JAL'

    def I_decode(self):
        funct3 = self.funct3
        opcode = self.opcode

        if funct3 == '000':
            if opcode == '0010011':
                return 'ADDI'
            elif opcode == '0000011':
                return 'LW'
            else:
                raise Exception('Wrong I Type instruction.')
        elif funct3 == '100':
            return 'XORI'
        elif funct3 == '110':
            return 'ORI'
        elif funct3 == '111':
            return 'ANDI'
        else:
            raise Exception('Wrong I Type instruction.')


def getImm(instr, type):
    imm= '0'
    if type == 'I':
        imm = instr[:12]

    elif type == 'S':
        imm = instr[:7] + instr[20:25]

    elif type == 'B':
        imm = instr[0] + instr[-8] + instr[1:7] + instr[20:24] + '0'



    elif type == 'J':
        imm = instr[0] + instr[12:20] + instr[1:11] + '0'

    imm = bitstring_to_int(imm)
    imm = int_to_bitstring(imm)
    return imm

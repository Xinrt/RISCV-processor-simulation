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
        rs1_raw = self.instr[12:17]
        rs2_raw = self.instr[7:12]
        rd_raw = self.instr[20:25]

        # rs1 = int(rs1, 2)
        # rs2 = int(rs2, 2)
        # rd = int(rd, 2)
        ins = ''

        if type == 'R':
            ins = self.R_instr_decode()

        elif type == 'I':
            ins = self.I_instr_decode()

        elif type == 'S':
            ins = self.S_instr_decode()

        elif type == 'B':
            ins = self.B_instr_decode()

        elif type == 'J':
            ins = self.J_instr_decode()
        else:
            ins = 'HALT'

        return type, ins, rs2_raw, rs1_raw, rd_raw

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
        elif opcode == '1111111':  # HALT
            return 'H'
        else:
            raise Exception('Wrong instruction type')

    def R_instr_decode(self):
        funct3 = self.funct3
        funct7 = self.funct7

        if funct3 == '000':
            if funct7 == '0000000':
                return 'ADD'
            elif funct7 == '0100000':
                return 'SUB'
            else:
                raise Exception('Wrong instruction. In R type instruction decoder')
        elif funct3 == '100':
            return 'XOR'
        elif funct3 == '110':
            return 'OR'
        elif funct3 == '111':
            return 'AND'
        else:
            raise Exception('Wrong instruction. In R type instruction decoder')

    def B_instr_decode(self):
        funct3 = self.funct3

        if funct3 == '000':
            return 'BEQ'
        elif funct3 == '001':
            return 'BNE'
        else:
            raise Exception('Wrong instruction. In B type instruction decoder')

    def S_instr_decode(self):
        return 'SW'

    def J_instr_decode(self):
        return 'JAL'

    def I_instr_decode(self):
        funct3 = self.funct3
        opcode = self.opcode

        if funct3 == '000':
            if opcode == '0010011':
                return 'ADDI'
            elif opcode == '0000011':
                return 'LW'
            else:
                raise Exception('Wrong instruction. In I type instruction decoder')
        elif funct3 == '100':
            return 'XORI'
        elif funct3 == '110':
            return 'ORI'
        elif funct3 == '111':
            return 'ANDI'
        else:
            raise Exception('Wrong instruction. In I type instruction decoder')


def ImmGen(instr, type):
    imm_raw = '0'
    if type == 'I':
        imm_raw = instr[:12]

    elif type == 'S':
        imm_raw = instr[:7] + instr[20:25]

    elif type == 'B':
        imm_raw = instr[0] + instr[-8] + instr[1:7] + instr[20:24] + '0'

    elif type == 'J':
        imm_raw = instr[0] + instr[12:20] + instr[1:11] + '0'

    imm_raw = bitstring_to_int(imm_raw)
    imm_raw = int_to_bitstring(imm_raw)
    return imm_raw

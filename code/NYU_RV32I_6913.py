import os
import argparse

from ALU import ALU
from ALUControl import ALU_control
from ControlUnit import ControlUnit
from convertor import *
from decoder import Parser, getImm

MemSize = 1000  # memory size, in reality, the memory size should be 2^32, but for this lab, for the space resaon, we keep it as this large number, but the memory is still 32-bit addressable.


class InsMem(object):
    def __init__(self, name, ioDir):
        self.id = name
        self.IMem = ['00000000' for i in range(0,MemSize)]
        with open(ioDir + "/6913_ProjA_TC/TC1/imem.txt") as im:
            self.IMem = [data.replace("\n", "") for data in im.readlines()]
        # for i in range(0, len(buf)):
        #     self.IMem[i] = buf[i]

    def readInstr(self, ReadAddress):
        # read instruction memory
        # return 32 bit hex val
        instruction = ''
        # i = 0, 1, 2, 3 (4 lines in the imem.txt file makes one instruction)
        print("Address: ", ReadAddress)
        for i in range(4):
            instruction += self.IMem[ReadAddress + i]
        # instruction = ''.join(reversed(instruction))

        return instruction
        # return self.IMem[ReadAddress]


class DataMem(object):
    def __init__(self, name, ioDir):
        self.id = name
        self.ioDir = ioDir
        self.DMem = ['00000000' for i in range(0,MemSize)]
        with open(ioDir + "/6913_ProjA_TC/TC1/dmem.txt") as dm:
                buf = [data.replace("\n", "") for data in dm.readlines()]
        for i in range(0, len(buf)):
            self.DMem[i] = buf[i]


    def readDataMem(self, ReadAddress):
        # read data memory
        # return 32 bit hex val
        if isinstance(ReadAddress, str):
            ReadAddress = bitstring_to_int(ReadAddress)

        data = ''
        for i in range(4):
            data += self.DMem[ReadAddress + i]
        return data

    def writeDataMem(self, Address, WriteData):
        # write data into byte addressable memory
        if isinstance(WriteData, int):
            WriteData = int_to_bitstring(WriteData)
        if isinstance(Address, str):
            Address = bitstring_to_int(Address)

        parsedData = [WriteData[:8], WriteData[8:16], WriteData[16:24], WriteData[24:]]
        print('Before write: {}', format(self.DMem[Address:Address + 10]))
        print(Address)
        for i in range(4):
            self.DMem[Address + i] = parsedData[i]
        print('After write: {}', format(self.DMem[Address:Address + 10]))

    def outputDataMem(self):
        resPath = self.ioDir + "/" + self.id + "_DMEMResult.txt"
        with open(resPath, "w") as rp:
            rp.writelines([str(data) + "\n" for data in self.DMem])


class RegisterFile(object):
    def __init__(self, ioDir):
        self.outputFile = ioDir + "RFResult.txt"
        self.Registers = [0x0 for i in range(32)]

    def readRF(self, Reg_addr):
        # Fill in
        print("Reg_addr:", Reg_addr)
        return self.Registers[int(str(Reg_addr),2)]

    def writeRF(self, Reg_addr, Wrt_reg_data):
        # Fill in
        print("write_data:", Wrt_reg_data)
        self.Registers[int(str(Reg_addr),2)] = Wrt_reg_data

    def outputRF(self, cycle):
        op = ["-" * 70 + "\n", "State of RF after executing cycle:" + str(cycle) + "\n"]
        op.extend([str(val) + "\n" for val in self.Registers])
        if (cycle == 0):
            perm = "w"
        else:
            perm = "a"
        with open(self.outputFile, perm) as file:
            file.writelines(op)


class State(object):
    def __init__(self):
        self.IF = {"nop": False, "PC": 0, 'PCSrc': 0, 'Flush': False}
        self.ID = {"nop": False, 'PC': 0, "Instr": '0', "Rs1": 0, "Rs2": 0, "Rd": 0, }
        self.EX = {"nop": False, 'Ins': '', "Read_data1": 0, "Read_data2": 0, "Imm": 0, "Rs1": 0, "Rs2": 0, "Rd": 0,
                   'funct7': '', 'funct3': '', 'opcode': '', "Branch": 0, "MemRead": 0, "MemtoReg": 0, "ALUOp": 0,
                   'MemWrite': 0, 'ALUSrc': 0, 'RegWrite': 0}
        self.MEM = {"nop": False, "ALUoutput": 0, 'Read_data2': 0, 'Load_data': 0, "Rs1": 0, "Rs2": 0, 'Rd': 0,
                    "MemtoReg": 0, "MemRead": 0, "MemWrite": 0, "RegWrite": 0}
        self.WB = {"nop": False, "ALUoutput": 0, 'Load_data': 0, "Rs1": 0, "Rs2": 0, "Rd": 0, "RegWrite": 0,
                   'MemtoReg': 0}


class Core(object):
    def __init__(self, ioDir, imem, dmem):
        self.myRF = RegisterFile(ioDir)
        self.cycle = 0
        self.halted = False
        self.ioDir = ioDir
        self.state = State()
        self.nextState = State()
        self.ext_imem = imem
        self.ext_dmem = dmem

    def assign_value(self, ioDir, imem, dmem):
        self.ioDir = ioDir
        self.ext_imem = imem
        self.ext_dmem = dmem


class SingleStageCore(Core):
    def __init__(self, ioDir, imem, dmem):
        super(SingleStageCore, self).__init__(ioDir + "/SS_", imem, dmem)
        self.opFilePath = ioDir + "/StateResult_SS.txt"

    def step(self):
        # Your implementation start here
        # --------------------- IF stage ---------------------
        PC = self.state.IF['PC']
        instr = self.ext_imem.readInstr(PC)

        # --------------------- ID stage ---------------------
        parser = Parser(instr)
        funct7 = parser.funct7
        funct3 = parser.funct3
        opcode = parser.opcode
        type, ins, rs2, rs1, rd = parser.parse()
        imm = getImm(instr, type)

        print("cycle ", self.cycle)
        print("ins: ", ins)
        print("rd: ", rd)
        print("rs1: ", rs1)
        print("rs2: ", rs2)
        print("imm: ", imm)

        # if HALT
        if type == 'H':
            self.state.IF['nop'] = True
            self.state.ID['nop'] = True
            self.state.EX['nop'] = True
            self.state.MEM['nop'] = True
            self.state.WB['nop'] = True

        self.state.ID['Instr'] = instr

        # --------------------- EX stage ---------------------
        rs1_data = self.myRF.readRF(rs1)
        rs2_data = self.myRF.readRF(rs2)

        main_con = ControlUnit(type, ins)
        alu_con = ALU_control(opcode, funct7, funct3, main_con.ALUOp)
        input2 = self.EX_MUX(rs2_data, imm, main_con.ALUSrc)
        aluRes = ALU(alu_con, ins, rs1_data, input2)

        # Branch
        if ins == 'BEQ':
            aluRes = aluRes == 0
        elif ins == 'BNE':
            aluRes = aluRes != 0

        branchControl = main_con.Branch & aluRes
        
        
        self.nextState.IF['PC'] = self.branch_MUX(PC + 4, PC + bitstring_to_int(str(imm)), branchControl)

        self.state.EX = {"nop": False, "Read_data1": rs1_data, "Read_data2": rs2_data, "Imm": imm, "Rs1": rs1,
                         "Rs2": rs2,
                         "Rd": rd, "Branch": main_con.Branch,
                         "MemtoReg": main_con.MemtoReg,
                         "is_I_type": False, "MemRead": main_con.MemRead, "MemWrite": main_con.MemWrite,
                         "ALUOp": main_con.ALUOp, "ALUSrc": main_con.ALUSrc, "RegWrite": main_con.RegWrite}

        # --------------------- MEM stage --------------------
        lw_value = 0
        if main_con.MemWrite:
            self.do_store(rs2, aluRes)
        elif main_con.MemRead:
            lw_value = self.do_load(aluRes)

        wb_value = self.WB_MUX(aluRes, lw_value, main_con.MemtoReg)

        self.state.MEM = {"ALUoutput": aluRes, "Store_data": rs2, "Rs1": rs1, "Rs2": rs2, "Rd": rd,
                          "MemtoReg": main_con.MemtoReg, "MemRead": main_con.MemRead, "MemWrite": main_con.MemWrite, "RegWrite": main_con.RegWrite}

        # --------------------- WB stage ---------------------
        if main_con.RegWrite:
            self.do_write_back(rd, wb_value)

        self.state.WB = {"Write_data": wb_value, "Rs1": rs1, "Rs2": rs2, "Rd": rd,
                         "RegWrite": main_con.RegWrite}

        # Our implementation end here
        if self.state.IF["nop"]:
            self.halted = True

        self.myRF.outputRF(self.cycle)  
        self.printState(self.nextState, self.cycle)  

        self.state = self.nextState
        self.cycle += 1

    def printState(self, state, cycle):
        printstate = ["-" * 70 + "\n", "State after executing cycle: " + str(cycle) + "\n"]
        printstate.append("IF.PC: " + str(state.IF["PC"]) + "\n")
        printstate.append("IF.nop: " + str(state.IF["nop"]) + "\n")

        if (cycle == 0):
            perm = "w"
        else:
            perm = "a"
        with open(self.opFilePath, perm) as wf:
            wf.writelines(printstate)

    def EX_MUX(self, rs2, imm, ALUSrc):
        if ALUSrc:
            return imm
        return rs2

    def branch_MUX(self, next, branch, logic_bit):
        if logic_bit:
            return branch
        return next

    def do_store(self, rs2, ALU_output):
        self.ext_dmem.writeDataMem(ALU_output, rs2)

    def do_load(self, ALU_output):
        return self.ext_dmem.readDataMem(ALU_output)

    def WB_MUX(self, ALU_output, lw_value, MemtoReg):
        if MemtoReg:
            return lw_value
        return ALU_output

    def do_write_back(self, rd, wb_value):
        self.myRF.writeRF(rd, wb_value)


class FiveStageCore(Core):
    def __init__(self, ioDir, imem, dmem):
        super(FiveStageCore, self).__init__(ioDir + "\\FS_", imem, dmem)
        self.opFilePath = ioDir + "\\StateResult_FS.txt"

    def step(self):
        # Your implementation
        # --------------------- WB stage ---------------------

        # --------------------- MEM stage --------------------

        # --------------------- EX stage ---------------------

        # --------------------- ID stage ---------------------

        # --------------------- IF stage ---------------------






        # --------------------- IF stage ---------------------
        if not self.state.IF['nop']:
            print('in if')
            self.IF()

        # --------------------- ID stage ---------------------
        if not self.state.ID['nop']:
            print('in id')
            # Control unit
            instr = self.nextState.ID['Instr']
            # print("instr: ", instr)
            parser = Parser(instr)
            type, ins, _, _, _ = parser.parse()
            if type == 'H':
                self.nextState.ID['nop'] = True
                self.nextState.IF['nop'] = True

            main_con = ControlUnit(type, ins)
            PCWrite = self.hazard_detection_unit()

            rs1_data_raw, rs2_data_raw, imm_raw = self.ID()

            # Branch
            PC = self.state.ID['PC']
            jump = 1
            if ins == 'BEQ':
                jump = rs1_data_raw == rs2_data_raw
            elif ins == 'BNE':
                jump = rs1_data_raw != rs2_data_raw
            self.state.IF['PCSrc'] = main_con.Branch and jump
            self.nextState.IF['PC'] = self.branch_MUX(PC, imm_raw, PCWrite)

            self.nextState.EX['Branch'] = main_con.Branch
            self.nextState.EX['MemRead'] = main_con.MemRead
            self.nextState.EX['MemtoReg'] = main_con.MemtoReg
            self.nextState.EX['ALUOp'] = main_con.ALUOp
            self.nextState.EX['MemWrite'] = main_con.MemWrite
            self.nextState.EX['ALUSrc'] = main_con.ALUSrc
            self.nextState.EX['RegWrite'] = main_con.RegWrite
        else:
            self.nextState.EX['nop'] = True

        # --------------------- EX stage ---------------------
        if not self.state.EX['nop']:
            print('in ex')
            forwarding = self.forwarding_unit()
            self.EX(forwarding)
        else:
            self.nextState.MEM['nop'] = True

        # --------------------- MEM stage --------------------
        if not self.state.MEM['nop']:
            print('in mem')
            self.MEM()
        else:
            self.nextState.WB['nop'] = True

        # --------------------- WB stage ---------------------
        if not self.state.WB['nop']:
            print('in wb')
            self.WB()

        # self.halted = True
        if self.state.IF["nop"] and self.state.ID["nop"] and self.state.EX["nop"] and self.state.MEM["nop"] and \
                self.state.WB["nop"]:
            self.halted = True

        self.myRF.outputRF(self.cycle)  # dump RF
        self.printState(self.nextState, self.cycle)  # print states after executing cycle 0, cycle 1, cycle 2 ...

        self.state = self.nextState  # The end of the cycle and updates the current state with the values calculated in this cycle
        self.cycle += 1

    def printState(self, state, cycle):
        printstate = ["-" * 70 + "\n", "State after executing cycle: " + str(cycle) + "\n"]
        printstate.extend(["IF." + key + ": " + str(val) + "\n" for key, val in state.IF.items()])
        printstate.extend(["ID." + key + ": " + str(val) + "\n" for key, val in state.ID.items()])
        printstate.extend(["EX." + key + ": " + str(val) + "\n" for key, val in state.EX.items()])
        printstate.extend(["MEM." + key + ": " + str(val) + "\n" for key, val in state.MEM.items()])
        printstate.extend(["WB." + key + ": " + str(val) + "\n" for key, val in state.WB.items()])

        if (cycle == 0):
            perm = "w"
        else:
            perm = "a"
        with open(self.opFilePath, perm) as wf:
            wf.writelines(printstate)

    def WB(self):
        ALU_output_raw = self.state.WB['ALUoutput']
        lw_value = self.state.WB['Load_data']
        rd = self.state.WB['Rd']
        RegWrite = self.state.WB['RegWrite']
        MemtoReg = self.state.WB['MemtoReg']

        wb_value = self.WB_MUX(ALU_output_raw, lw_value, MemtoReg)
        if RegWrite:
            self.do_write_back(rd, wb_value)

    def MEM(self):
        rs2_data_raw = self.state.MEM['Read_data2']
        MemWrite = self.state.MEM['MemWrite']
        MemRead = self.state.MEM['MemRead']
        ALU_output_raw = self.state.MEM['ALUoutput']
        lw_value = 0

        if MemWrite:
            self.do_store(rs2_data_raw, ALU_output_raw)
        elif MemRead:
            lw_value = self.do_load(ALU_output_raw)

        self.nextState.WB['nop'] = False
        self.nextState.WB['RegWrite'] = self.state.MEM['RegWrite']
        self.nextState.WB['MemtoReg'] = self.state.MEM['MemtoReg']
        self.nextState.WB['Load_data'] = lw_value

    def EX(self, forwarding):
        forwardA, forwardB = forwarding

        ins = self.state.EX['Ins']
        ALUOp = self.state.EX['ALUOp']
        rs1_data_raw = self.state.EX['Read_data1']
        rs2_data_raw = self.state.EX['Read_data2']
        imm_raw = self.state.EX['Imm']
        funct7 = self.state.EX['funct7']
        funct3 = self.state.EX['funct3']
        opcode = self.state.EX['opcode']

        ALU_con = ALU_control(opcode, funct7, funct3, ALUOp)
        input1_raw = self.EX_MUX_A(rs1_data_raw, forwardA)
        inputB_raw = self.EX_MUX_B(rs2_data_raw, forwardB)
        input2_raw = self.EX_MUX_2(inputB_raw, imm_raw)

        # print("self.state.MEM['ALUoutput']: ", self.state.MEM['ALUoutput'])
        ALU_output_raw = ALU(ALU_con, ins, input1_raw, input2_raw)

        self.nextState.MEM['nop'] = False
        self.nextState.MEM['ALUoutput'] = ALU_output_raw
        self.nextState.MEM['Read_data2'] = rs2_data_raw
        self.nextState.MEM['Rs1'] = self.state.EX['Rs1']
        self.nextState.MEM['Rs2'] = self.state.EX['Rs2']
        self.nextState.MEM['Rd'] = self.state.EX['Rd']
        self.nextState.MEM['MemRead'] = self.state.EX['MemRead']
        self.nextState.MEM['MemWrite'] = self.state.EX['MemWrite']
        self.nextState.MEM['RegWrite'] = self.state.EX['RegWrite']
        self.nextState.MEM['MemtoReg'] = self.state.EX['MemtoReg']

    def ID(self):
        instr = self.nextState.ID['Instr']
        PC = self.state.ID['PC']
        parser = Parser(instr)
        funct7 = parser.funct7
        funct3 = parser.funct3
        opcode = parser.opcode
        type, ins, rs2, rs1, rd = parser.parse()
        imm_raw = getImm(instr, type)
        # rs2 = int(rs2_raw, 2)
        # rs1 = int(rs1_raw, 2)
        # rd = int(rd_raw, 2)
        rs1_data_raw = self.myRF.readRF(rs1)
        rs2_data_raw = self.myRF.readRF(rs2)
        if type == 'J':
            rs1_data_raw = int_to_bitstring(PC)
            rs2_data_raw = int_to_bitstring(4)

        print('{}\t{}\tx{}\tx{}\tx{}\t{}'.format(self.cycle, ins, rd, rs1, rs2, bitstring_to_int(imm_raw)))

        self.nextState.ID['Rs1'] = rs1
        self.nextState.ID['Rs2'] = rs2
        self.nextState.ID['Rd'] = rd

        self.nextState.EX['nop'] = False
        self.nextState.EX['Ins'] = ins
        self.nextState.EX['Read_data1'] = rs1_data_raw
        self.nextState.EX['Read_data2'] = rs2_data_raw
        self.nextState.EX['Imm'] = imm_raw
        self.nextState.EX['Rs1'] = rs1
        self.nextState.EX['Rs2'] = rs2
        self.nextState.EX['Rd'] = rd
        self.nextState.EX['funct3'] = funct3
        self.nextState.EX['funct7'] = funct7
        self.nextState.EX['opcode'] = opcode

        return (rs1_data_raw, rs2_data_raw, imm_raw)

    def IF(self):
        PC = self.state.IF['PC']
        instr = self.ext_imem.readInstr(PC)

        self.state.IF['PC'] = self.nextState.IF['PC']
        self.state.IF['Flush'] = self.state.IF['PCSrc']

        self.nextState.ID['PC'] = PC
        if self.state.ID['nop'] == True:
            self.nextState.IF['PC'] = PC + 4
        self.nextState.ID['Instr'] = instr
        # print("self.nextState.ID['Instr']", self.nextState.ID['Instr'])

    def WB_MUX(self, ALU_output_raw, lw_value, MemtoReg):
        if MemtoReg:
            return lw_value
        return ALU_output_raw

    def EX_MUX_A(self, rs1, forwardA):
        if forwardA == 0:
            return rs1
        elif forwardA == 10:
            return self.state.MEM['ALUoutput']
        elif forwardA == 1:
            return self.state.WB['Write_data']

    def EX_MUX_B(self, rs2, forwardB):
        # print("forwardB: ", forwardB)
        if forwardB == 0:
            return rs2
        elif forwardB == 10:
            # print("aaaaaaaaaaaaaaa")
            return self.state.MEM['ALUoutput']
        elif forwardB == 1:
            return self.state.WB['Write_data']

    def EX_MUX_2(self, inputB, imm_raw):
        if self.state.EX['ALUSrc']:
            return imm_raw
        return inputB

    def control_MUX(self, main_con, hazard_con):
        PCWrite = True
        IF_IDWrite = True
        if hazard_con:
            main_con.stall()
            PCWrite = False
            IF_IDWrite = False
        return (PCWrite, IF_IDWrite)

    def branch_MUX(self, PC, imm_raw, PCWrite):
        if PCWrite:
            imm = bitstring_to_int(imm_raw)
            if self.state.IF['PCSrc']:
                return PC + imm
            else:
                return PC + 4
        return PC

    def do_store(self, rs2_data_raw, ALU_output_raw):
        self.ext_dmem.writeDataMem(ALU_output_raw, rs2_data_raw)

    def do_load(self, ALU_output_raw):
        return self.ext_dmem.readDataMem(ALU_output_raw)

    def do_write_back(self, rd, wb_value):
        self.myRF.writeRF(rd, wb_value)

    def hazard_detection_unit(self):
        ID_EX = self.state.EX
        IF_ID = self.state.ID
        PCWrite = True
        if ID_EX['MemRead'] and ((ID_EX['Rd'] == IF_ID['Rs1']) or (ID_EX['Rd'] == IF_ID['Rs2'])):
            self.state.EX['nop'] = True
            self.state.MEM['nop'] = True
            self.state.WB['nop'] = True
            PCWrite = False
        return PCWrite

    def forwarding_unit(self):
        forwardA = 0
        forwardB = 0
        EX_MEM = self.state.MEM
        MEM_WB = self.state.WB
        ID_EX = self.state.EX

        if (EX_MEM['RegWrite'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs1'])):
            forwardA = 10

        if (EX_MEM['RegWrite'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs2'])):
            forwardB = 10

        if MEM_WB['RegWrite'] and (MEM_WB['Rd'] != 0) and not (
                EX_MEM['RegWrite'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs1'])) and (
                MEM_WB['Rd'] == ID_EX['Rs1']):
            forwardA = 0b01

        if (MEM_WB['RegWrite'] and (MEM_WB['Rd'] != 0) and not (
                EX_MEM['RegWrite'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs2'])) and (
                MEM_WB['Rd'] == ID_EX['Rs2'])):
            forwardB = 0b01

        return (forwardA, forwardB)


if __name__ == "__main__":

    # parse arguments for input file location
    parser = argparse.ArgumentParser(description='RV32I processor')
    parser.add_argument('--iodir', default="", type=str, help='Directory containing the input files.')
    args = parser.parse_args()

    ioDir = os.path.abspath(args.iodir)
    print("IO Directory:", ioDir)

    imem = InsMem("Imem", ioDir)
    dmem_ss = DataMem("SS", ioDir)
    dmem_fs = DataMem("FS", ioDir)

    ssCore = SingleStageCore(ioDir, imem, dmem_ss)
    fsCore = FiveStageCore(ioDir, imem, dmem_fs)

    while (True):
        if not ssCore.halted:
            ssCore.step()

        if not fsCore.halted:
            fsCore.step()

        if ssCore.halted and fsCore.halted:
            break

    # dump SS and FS data mem.
    dmem_ss.outputDataMem()
    dmem_fs.outputDataMem()

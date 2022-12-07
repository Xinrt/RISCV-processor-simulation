import os
import argparse

from ALU import ALU_control, ALU
from ControlUnit import ControlUnit
from convertor import *
from decoder import Parser, ImmGen

MemSize = 1000  # memory size, in reality, the memory size should be 2^32, but for this lab, for the space resaon, we keep it as this large number, but the memory is still 32-bit addressable.


class InsMem(object):
    def __init__(self, name, ioDir):
        self.id = name
        self.IMem = ['00000000' for i in range(0, MemSize)]
        with open(ioDir + "/6913_ProjA_TC/TC4/imem.txt") as im:
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
        self.DMem = ['00000000' for i in range(0, MemSize)]
        with open(ioDir + "/6913_ProjA_TC/TC4/dmem.txt") as dm:
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
        return self.Registers[int(str(Reg_addr), 2)]

    def writeRF(self, Reg_addr, Wrt_reg_data):
        # Fill in
        print("write_data:", Wrt_reg_data)
        self.Registers[int(str(Reg_addr), 2)] = Wrt_reg_data

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
        self.IF = {"nop": False, "PC": 0}
        self.ID = {"nop": False, "Instr": 0}
        self.EX = {"nop": False, "Read_data1": 0, "Read_data2": 0, "Imm": 0, "Rs1": 0, "Rs2": 0, "Rd": 0, "Branch": 0, "Wrt_reg_addr": 0,
                   "is_I_type": False, "rd_mem": 0,
                   "wrt_mem": 0, "alu_op": 0, "alu_src": 0, "wrt_enable": 0}
        self.MEM = {"nop": False, "ALUresult": 0, "Store_data": 0, "Rs1": 0, "Rs2": 0, "Rd": 0, "Wrt_reg_addr": 0, "rd_mem": 0,
                    "wrt_mem": 0, "wrt_enable": 0}
        self.WB = {"nop": False, "Wrt_data": 0, "Rs1": 0, "Rs2": 0, "Rd": 0, "Wrt_reg_addr": 0, "wrt_enable": 0}


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
        instruction = self.ext_imem.readInstr(PC)

        # --------------------- ID stage ---------------------
        parser = Parser(instruction)
        funct7 = parser.funct7
        funct3 = parser.funct3
        opcode = parser.opcode
        type, ins, rs2, rs1, rd = parser.parse()
        imm = ImmGen(instruction, type)

        print('{}\t{}\tx{}\tx{}\tx{}\t{}'.format(self.cycle, ins, rd, rs1, rs2, imm))

        # if HALT
        if type == 'H':
            self.state.IF['nop'] = True
            self.state.ID['nop'] = True
            self.state.EX['nop'] = True
            self.state.MEM['nop'] = True
            self.state.WB['nop'] = True

        self.state.ID['Instr'] = instruction

        # --------------------- EX stage ---------------------
        rs1_value = self.myRF.readRF(rs1)
        rs2_value = self.myRF.readRF(rs2)

        main_con = ControlUnit(type, ins)
        alu_con = ALU_control(opcode, funct7, funct3, main_con.ALUOp)
        print("alu_con: ", alu_con)
        input2 = self.EX_MUX(rs2_value, imm, main_con.ALUSrc)
        print("ins: ", ins)
        print("rs1_value: ", rs1_value)
        print("input2: ", input2)
        alu_output = ALU(alu_con, ins, rs1_value, input2)

        # Branch
        if ins == 'BEQ':
            alu_output = alu_output == 0
        elif ins == 'BNE':
            alu_output = alu_output != 0

        branch_logic = main_con.Branch & alu_output

        self.nextState.IF['PC'] = self.branch_MUX(PC + 4, PC + bitstring_to_int(str(imm)), branch_logic)

        # EX stage
        self.state.EX = {"nop": False, "Read_data1": rs1_value, "Read_data2": rs2_value, "Imm": imm, "Rs1": rs1, "Rs2": rs2,
                         "Rd": rd, "Branch": main_con.Branch,
                         "Wrt_reg_addr": main_con.MemtoReg,
                         "is_I_type": False, "rd_mem": main_con.MemRead, "wrt_mem": main_con.MemWrite,
                         "alu_op": main_con.ALUOp, "alu_src": main_con.ALUSrc, "wrt_enable": main_con.RegWrite}

        # --------------------- MEM stage --------------------
        lw_value = 0
        if main_con.MemWrite:
            self.do_store(rs2, alu_output)
        elif main_con.MemRead:
            lw_value = self.do_load(alu_output)

        wb_value = self.WB_MUX(alu_output, lw_value, main_con.MemtoReg)

        self.state.MEM = {"ALUresult": alu_output, "Store_data": alu_output, "Rs1": rs1, "Rs2": rs2, "Rd": rd,
                          "Wrt_reg_addr": main_con.MemtoReg, "rd_mem": main_con.MemRead}

        # --------------------- WB stage ---------------------
        if main_con.RegWrite:
            self.do_write_back(rd, wb_value)

        self.state.WB = {"Wrt_data": wb_value, "Rs1": rs1, "Rs2": rs2, "Rd": rd, "Wrt_reg_addr": main_con.MemtoReg,
                         "wrt_enable": main_con.RegWrite}
        # Our implementation end here

        # self.halted = True
        if self.state.IF["nop"]:
            self.halted = True

        self.myRF.outputRF(self.cycle)  # dump RF
        self.printState(self.nextState, self.cycle)  # print states after executing cycle 0, cycle 1, cycle 2 ...

        self.state = self.nextState  # The end of the cycle and updates the current state with the values calculated in this cycle
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
        PC = self.state.IF['PC']
        instruction = self.ext_imem.readInstr(PC)

        # --------------------- ID stage ---------------------
        parser = Parser(instruction)
        funct7 = parser.funct7
        funct3 = parser.funct3
        opcode = parser.opcode
        type, ins, rs2, rs1, rd = parser.parse()
        imm_raw = ImmGen(instruction, type)
        # rs2 = int(rs2_raw, 2)
        # rs1 = int(rs1_raw, 2)
        # rd = int(rd_raw, 2)
        rs1_data_raw = self.myRF.readRF(rs1)
        rs2_data_raw = self.myRF.readRF(rs2)
        if type == 'J':
            rs1_data_raw = int_to_bitstring(PC)
            rs2_data_raw = int_to_bitstring(4)
        print('{}\t{}\tx{}\tx{}\tx{}\t{}'.format(self.cycle, ins, rd, rs1, rs2, bitstring_to_int(imm_raw)))

        if type == 'H':
            self.state.ID['nop'] = True

        self.state.ID['Instr'] = instruction
        ID_EX = (PC, rs1_data_raw, rs2_data_raw, imm_raw, rd)

        main_con = ControlUnit(type, ins)
        EX_con = (main_con.ALUSrc, main_con.ALUOp)
        MEM_con = (main_con.Branch, main_con.MemRead, main_con.MemWrite)
        WB_con = (main_con.RegWrite, main_con.MemtoReg)

        IF_ID = (PC, instruction)
        # PC, rs1_data_raw, rs2_data_raw, imm_raw, rd = self.ID(IF_ID)
        self.state.EX = {"nop": False, "Read_data1": rs1_data_raw, "Read_data2": rs2_data_raw, "Imm": imm_raw, "Rs1": rs1, "Rs2": rs2,
                         "Rd": rd, "Branch": main_con.Branch,
                         "Wrt_reg_addr": main_con.MemtoReg,
                         "is_I_type": False, "rd_mem": main_con.MemRead, "wrt_mem": main_con.MemWrite,
                         "alu_op": main_con.ALUOp, "alu_src": main_con.ALUSrc, "wrt_enable": main_con.RegWrite}

        # --------------------- EX stage ---------------------
        rs1 = self.state.EX['Rs1']
        rs2 = self.state.EX['Rs2']
        # rd_EX = self.state.MEM['Wrt_']
        forwarding = self.forwarding_unit()
        # PC_imm, ALU_zero, ALU_output_raw, rs2_data_raw, rd = self.EX(ID_EX, EX_con, opcode, funct7, funct3, forwarding)

        forwardA, forwardB = forwarding

        ALUSrc, ALUOp = EX_con
        PC, rs1_data_raw, rs2_data_raw, imm_raw, rd = ID_EX

        ALU_con = ALU_control(opcode, funct7, funct3, ALUOp)
        input1_raw = self.EX_MUX_A(rs1_data_raw, forwardA)
        inputB_raw = self.EX_MUX_B(rs2_data_raw, forwardB)
        input2_raw = self.EX_MUX_2(inputB_raw, imm_raw)

        ALU_output_raw = ALU(ALU_con, ins, input1_raw, input2_raw)

        imm = bitstring_to_int(imm_raw)
        PC_imm = PC + imm
        # Branch
        ALU_zero = 0
        if ins == 'BEQ':
            ALU_zero = ALU_output_raw == 0
        elif ins == 'BNE':
            ALU_zero = ALU_output_raw != 0

        EX_MEM = (PC_imm, ALU_zero, ALU_output_raw, rs2_data_raw, rd)

        self.state.MEM = {"ALUresult": ALU_output_raw, "Store_data": rs2, "Rs1": rs1, "Rs2": rs2, "Rd": rd,
                          "Wrt_reg_addr": main_con.MemtoReg, "rd_mem": main_con.MemRead, "wrt_mem": main_con.MemWrite, "wrt_enable": main_con.RegWrite}


        # --------------------- MEM stage --------------------
        # PC_imm, PCsrc, MEM_WB = self.MEM(EX_MEM, MEM_con)
        PC_imm, ALU_zero, ALU_output_raw, rs2_data_raw, rd = EX_MEM
        Branch, MemRead, MemWrite = MEM_con
        ALU_output_raw = int_to_bitstring(ALU_output_raw)
        PCsrc = Branch and ALU_zero

        lw_value = 0
        if MemWrite:
            self.do_store(rs2_data_raw, ALU_output_raw)
        elif MemRead:
            lw_value = self.do_load(ALU_output_raw)

        wb_value = self.WB_MUX(ALU_output_raw, lw_value, main_con.MemtoReg)
        MEM_WB = (wb_value, ALU_output_raw, rd)

        # --------------------- WB stage --------------------
        wb_value, ALU_output_raw, rd = MEM_WB
        RegWrite, MemtoReg = WB_con
        wb_value = self.WB_MUX(ALU_output_raw, lw_value, MemtoReg)
        if RegWrite:
            self.do_write_back(rd, wb_value)

        self.state.WB = {"Wrt_data": wb_value, "Rs1": rs1, "Rs2": rs2, "Rd": rd, "Wrt_reg_addr": main_con.MemtoReg,
                         "wrt_enable": main_con.RegWrite}

        # end here

        self.halted = True
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

    def hazard_detection_unit(self):
        ID_EX = self.state.EX
        IF_ID = self.state.ID
        if ID_EX['MemRead'] and ((ID_EX['rd'] == IF_ID['rs1']) or (ID_EX['rd'] == IF_ID['rs2'])):
            self.state.ID['nop'] = True
            return True
        return False

    def forwarding_unit(self):
        forwardA = 0
        forwardB = 0
        EX_MEM = self.state.MEM
        MEM_WB = self.state.WB
        ID_EX = self.state.EX

        if (EX_MEM['wrt_enable'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs1'])):
            forwardA = 10

        if (EX_MEM['wrt_enable'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs2'])):
            forwardB = 10

        if MEM_WB['wrt_enable'] and (MEM_WB['Rd'] != 0) and not (
                EX_MEM['wrt_enable'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs1'])) and (
                MEM_WB['Rd'] == ID_EX['Rs1']):
            forwardA = 0b01

        if (MEM_WB['wrt_enable'] and (MEM_WB['Rd'] != 0) and not (
                EX_MEM['wrt_enable'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs2'])) and (
                MEM_WB['Rd'] == ID_EX['Rs2'])):
            forwardB = 0b01

        return (forwardA, forwardB)

    def EX_MUX_A(self, rs1, forwardA):
        if forwardA == 0b00:
            return rs1
        elif forwardA == 0b10:
            return self.state.MEM['ALUresult']
        elif forwardA == 0b01:
            return self.state.WB['Write_data']

    def EX_MUX_B(self, rs2, forwardB):
        if forwardB == 0b00:
            return rs2
        elif forwardB == 0b10:
            return self.state.MEM['ALUresult']
        elif forwardB == 0b01:
            return self.state.WB['Write_data']

    def EX_MUX_2(self, inputB, imm_raw):
        if self.state.EX['alu_src']:
            return imm_raw
        return inputB

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

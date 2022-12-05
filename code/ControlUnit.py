class ControlUnit(object):
    def __init__(self, type, ins) -> None:
        self.Branch = 0
        self.MemRead = 0
        self.MemtoReg = 0
        self.ALUOp = 0
        self.MemWrite = 0
        self.ALUSrc = 0
        self.RegWrite = 0
        self.control(type, ins)

    def control(self, type, ins):
        if type == 'R':
            self.ALUSrc = 0
            self.MemtoReg = 0
            self.RegWrite = 1
            self.MemRead = 0
            self.MemWrite = 0
            self.Branch = 0
            self.ALUOp = 0b10


        elif type == 'I':
            self.ALUSrc = 1
            self.MemtoReg = 0
            self.RegWrite = 1
            self.MemRead = 0
            self.MemWrite = 0
            self.Branch = 0
            self.ALUOp = 0b10
            if ins == 'LW':
                self.MemRead = 1
                self.MemtoReg = 1

        elif type == 'S':
            self.ALUSrc = 1
            self.RegWrite = 0
            self.MemRead = 0
            self.MemWrite = 1
            self.Branch = 0
            self.ALUOp = 0b00

        elif type == 'B':
            self.ALUSrc = 0
            self.RegWrite = 0
            self.MemRead = 0
            self.MemWrite = 0
            self.Branch = 1
            self.ALUOp = 0b01

        elif type == 'J':
            self.ALUSrc = 0
            self.MemtoReg = 0
            self.RegWrite = 1
            self.MemRead = 0
            self.MemWrite = 0
            self.Branch = 1
            self.ALUOp = 0b10
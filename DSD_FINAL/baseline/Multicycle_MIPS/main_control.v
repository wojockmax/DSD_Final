module main_control (
    Op,
    funct,
    RegDst,
    ALUSrc,
    MemToReg,
    RegWrite,
    MemRead,
    MemWrite,
    Branch, 
    Jump,  
    RegDst_ra, 
    JumptoReg,
    ALUctrl
);

    input wire [5:0] Op;
    input wire [5:0] funct;
    output wire RegDst;
    output wire ALUSrc;
    output wire MemToReg;
    output wire RegWrite;
    output wire MemRead;
    output wire MemWrite;
    output wire Branch;
    output wire Jump;     
    output wire RegDst_ra; 
    output wire JumptoReg;
    output wire [3:0] ALUctrl;

    wire [1:0] ALUOp;
    
    control controller(
    .Op(Op),
    .funct5(funct[5]),
    .funct3(funct[3]),
    .funct0(funct[0]),
    .ALUOp(ALUOp), 
    .RegDst(RegDst),
    .ALUSrc(ALUSrc),
    .MemToReg(MemToReg),
    .RegWrite(RegWrite),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
    .Branch(Branch), 
    .Jump(Jump),  
    .RegDst_ra(RegDst_ra), 
    .JumptoReg(JumptoReg)
    );

    ALU_control alc(
    .Op(Op),
    .funct(funct),
    .ALUOp(ALUOp),
    .ctrl(ALUctrl)
    );



endmodule

`include "Multicycle_MIPS/control.v"
`include "Multicycle_MIPS/ALU_control.v"

module control(
    Op,
    funct5,
    funct3,
    funct0,
    ALUOp, 
    RegDst,
    ALUSrc,
    MemToReg,
    RegWrite,
    MemRead,
    MemWrite,
    Branch, 
    Jump,  
    RegDst_ra, 
    JumptoReg
);
    input [5:0] Op;
    input funct5;
    input funct3;
    input funct0;
    output[1:0] ALUOp;
    output RegDst;
    output ALUSrc;
    output MemToReg;
    output RegWrite;
    output MemRead;
    output MemWrite;
    output Branch;
    output Jump;     
    output RegDst_ra; 
    output JumptoReg;
    
    wire [5:0] Op;
    wire RegDst, ALUSrc, MemToReg, RegWrite, Branch, Jump, RegDst_ra, JumptoReg, Jalr;
    wire [1:0] ALUOp;
    
    assign ALUOp[1] = (~Op[3]) & (~Op[2]) & (~Op[1]);
    assign ALUOp[0] = (~Op[3]) & Op[2];
    assign RegDst = (~Op[3]) & (~Op[1]);
    assign ALUSrc = Op[3] | Op[0];
    assign MemToReg = Op[5];
    assign RegWrite = (((~Op[2]) & (~Op[1])) | ((~Op[3]) & Op[0]) | ((~Op[5]) & Op[3])) & ((~JumptoReg) | Jalr);
    assign MemRead = Op[5] & (~Op[3]);
    assign MemWrite = Op[5] & Op[3];
    assign Branch = (~Op[3]) & Op[2];
    assign Jump = ((~Op[5]) & (~Op[3]) & Op[1]);
    assign RegDst_ra = (~Op[5] & ~Op[3] & Op[0]) | Jalr;
    assign JumptoReg = (~Op[1]) & (~Op[2]) & (~Op[3]) & (~funct5) & funct3; 

    assign Jalr = JumptoReg & funct0;


endmodule

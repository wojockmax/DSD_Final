`include "Multicycle_MIPS/main_control.v"
`include "Multicycle_MIPS/register_file.v"
`include "Multicycle_MIPS/ALU.v"
// Single Cycle MIPS
//=========================================================
// Input/Output Signals:
// positive-edge triggered         clk
// active low asynchronous reset   rst_n
// instruction memory interface    IR_addr, IR
// output for testing purposes     RF_writedata  
//=========================================================
// Wire/Reg Specifications:
// control signals             MemToReg, MemRead, MemWrite, 
//                             RegDST, RegWrite, Branch, 
//                             Jump, ALUSrc, ALUOp
// ALU control signals         ALUctrl
// ALU input signals           ALUin1, ALUin2
// ALU output signals          ALUresult, ALUzero
// instruction specifications  r, j, jal, jr, lw, sw, beq
// sign-extended signal        SignExtend
// MUX output signals          MUX_RegDST, MUX_MemToReg, 
//                             MUX_Src, MUX_Branch, MUX_Jump
// registers input signals     Reg_R1, Reg_R2, Reg_W, WriteData 
// registers                   Register
// registers output signals    ReadData1, ReadData2
// data memory contral signals CEN, OEN, WEN
// data memory output signals  ReadDataMem
// program counter/address     PCin, PCnext, JumpAddr, BranchAddr
//=========================================================

module MIPS_Pipeline(
    // control interface
    clk, 
    rst_n,
//----------I cache interface-------        
    ICACHE_ren,
    ICACHE_wen,
    ICACHE_addr,
    ICACHE_wdata,
    ICACHE_stall,
    ICACHE_rdata,
//----------D cache interface-------
    DCACHE_ren,
    DCACHE_wen,
    DCACHE_addr,
    DCACHE_wdata,
    DCACHE_stall,
    DCACHE_rdata,
);

//==== in/out declaration =================================
    // control interface
    input wire clk; 
    input wire rst_n;
//----------I cache interface-------        
    output wire ICACHE_ren;
    output wire ICACHE_wen;
    output wire [29:0] ICACHE_addr;
    output wire [31:0] ICACHE_wdata;
    input wire ICACHE_stall;
    input wire [31:0] ICACHE_rdata;
//----------D cache interface-------
    output wire DCACHE_ren;
    output wire DCACHE_wen;
    output wire [29:0] DCACHE_addr;
    output wire [31:0] DCACHE_wdata;
    input wire DCACHE_stall;
    input wire [31:0] DCACHE_rdata;

//==== reg/wire declaration =============================== 
    //---- Modules ----------------------------------------
    //---- Main control -----------------------------------
    wire [5:0] Op;
    wire RegDst, ALUSrc, MemToReg, RegWrite, Branch, Jump, RegDst_ra, Jumptoreg, MemWrite, MemRead;
    //---- ALU control ------------------------------------
    wire [5:0] funct;
    wire [3:0] ALU_ctrl;
    //---- Other wires and regs ---------------------------
    wire [31:0] IR_sign_extended, reg_WriteData, ReadRegData_X, ReadRegData_Y;
    wire [4:0] ReadRegister_X, ReadRegister_Y, WriteRegister;
    wire zero;
    wire [31:0] x, y, ALUresult;
    wire [29:0] A;
    wire [31:0] mem_addr;
    wire Branchresult;
    //---- IR_addr ----------------------------------------
    reg [31:0] PC;
    wire [31:0] PC_next, PC_add4, JumpAddr, BranchAddr, ReadDataMem, IR;
//==== combinational part =================================
    //---- Modules ----------------------------------------

    assign ICACHE_wen = 0;
    assign ICACHE_ren = 1;
    assign ICACHE_addr = PC[31:2];
    assign ICACHE_wdata = 0;
    assign IR = ICACHE_rdata;
    assign DCACHE_wen = MemWrite;
    assign DCACHE_ren = MemRead;
    assign DCACHE_addr = A;
    assign DCACHE_wdata = ReadRegData_Y;
    assign ReadDataMem = DCACHE_rdata;
    assign stall =ICACHE_stall | DCACHE_stall;

    main_control mc(
        .Op(Op),
        .funct(funct),
        .RegDst(RegDst),
        .ALUSrc(ALUSrc),
        .MemToReg(MemToReg),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .Branch(Branch), 
        .Jump(Jump),  
        .RegDst_ra(RegDst_ra), 
        .JumptoReg(Jumptoreg),
        .ALUctrl(ALU_ctrl)
    );
    
    register_file rf(
        clk,
        rst_n,
        RegWrite,
        WriteRegister,
        reg_WriteData,
        ReadRegister_X,
        ReadRegister_Y,
        ReadRegData_X,
        ReadRegData_Y
    );

    alu alu1(
        ALU_ctrl,
        x,
        y,
        ALUresult,
        zero
    );

    //---- Main control -----------------------------------
    assign Op[5:0] = IR[31:26];


    //---- ALU control ------------------------------------
    assign funct[5:0] = IR[5:0];

    //---- Other wires and regs ---------------------------

    assign A = ALUresult[31:2];

    assign ReadRegister_X = IR[25:21];
    assign ReadRegister_Y = IR[20:16];
    assign WriteRegister = ({5{RegDst_ra}} | ((RegDst) ? IR[15:11] :  IR[20:16]));
    assign reg_WriteData = (MemToReg) ? ReadDataMem : ((RegDst_ra) ? PC_add4 : ALUresult);
    assign IR_sign_extended = {{16{IR[15]}}, IR[15:0]};

    assign x = (ALU_ctrl[3]) ? IR_sign_extended : ReadRegData_X;
    assign y = (ALUSrc) ? IR_sign_extended : ReadRegData_Y;

    //---- IR_addr ----------------------------------------
    assign Branchresult = !(ReadRegData_X ^ ReadRegData_Y) & Branch;
    assign PC_add4[31:2] = PC[31:2] + 30'd1;
    assign PC_add4[1:0] = PC[1:0];
    assign JumpAddr = {PC_add4[31:28], IR[25:0], 2'b0};
    assign BranchAddr[31:2] = PC_add4[31:2] + (IR_sign_extended[29:0] & {29{Branchresult}});
    assign BranchAddr[1:0] = PC_add4[1:0];
    assign PC_next = (stall) ? PC : ((Jumptoreg) ? ReadRegData_X : ((Jump) ? JumpAddr : BranchAddr));

//==== sequential part ====================================
always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
        PC <= 0;
    end 
    else begin
        PC <= PC_next;
    end
end

//=========================================================
endmodule

//==== Other modules ======================================
module alu(
    ctrl,
    x,
    y,
    out,
    zero
);
    
    input  [3:0] ctrl;
    input  [31:0] x;
    input  [31:0] y;
    output [31:0] out;
    wire [4:0] shamt;
    output zero;

    wire [3:0] ctrl;
    assign shamt = x[10:6];
    reg [31:0] out;

    assign zero = !(x ^ y);
    always@(*) begin
        case(ctrl)
            4'b0000: out = x + y;
            4'b0001: out = x - y;
            4'b0010: out = x & y;
            4'b0011: out = x | y;
            4'b0100: out = x ^ y;
            4'b0101: out = ~(x | y);
            4'b0110: out = (x < y) ? 1 : 0;
            4'b1000: out = y << shamt;
            4'b1001: out = y >> shamt;
            4'b1011: out = {{y[31]},y >> shamt};
            default: out = 0;
        endcase
    end

endmodule

module register_file(
    Clk  ,
    rst  ,
    WEN  ,
    RW   ,
    busW ,
    RX   ,
    RY   ,
    busX ,
    busY
);
    input        Clk, WEN, rst;
    input  [4:0] RW, RX, RY;
    input  [31:0] busW;
    output [31:0] busX, busY;  
    // write your design here
    reg [31:0] r0_w, r1_w, r2_w, r3_w, r4_w, r5_w, r6_w, r7_w, r8_w, r9_w;
    reg [31:0] r10_w, r11_w, r12_w, r13_w, r14_w, r15_w, r16_w, r17_w, r18_w, r19_w;
    reg [31:0] r20_w, r21_w, r22_w, r23_w, r24_w, r25_w, r26_w, r27_w, r28_w, r29_w;
    reg [31:0] r30_w, r31_w;
    reg [31:0] r0_r, r1_r, r2_r, r3_r, r4_r, r5_r, r6_r, r7_r, r8_r, r9_r;
    reg [31:0] r10_r, r11_r, r12_r, r13_r, r14_r, r15_r, r16_r, r17_r, r18_r, r19_r;
    reg [31:0] r20_r, r21_r, r22_r, r23_r, r24_r, r25_r, r26_r, r27_r, r28_r, r29_r;
    reg [31:0] r30_r, r31_r;
    reg [31:0] r_busX, r_busY; 
 	wire [4:0] Writeaddr; 

    assign busX = r_busX;
    assign busY = r_busY;
    assign Writeaddr = ({5{WEN}} & RW);
        
always@(*) begin
    case(Writeaddr)
        5'd0: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd1: begin
            r0_w = 32'd0;
            r1_w = busW;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd2: begin
            r0_w = 32'd00;
            r1_w = r1_r;
            r2_w = busW;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd3: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = busW;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd4: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = busW;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd5: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = busW;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd6: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = busW;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd7: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = busW;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd8: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = busW;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd9: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = busW;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd10: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = busW;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd11: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = busW;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd12: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = busW;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd13: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = busW;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd14: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = busW;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd15: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = busW;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd16: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = busW;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd17: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = busW;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd18: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = busW;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd19: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = busW;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd20: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = busW;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd21: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = busW;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd22: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = busW;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd23: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = busW;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd24: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = busW;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd25: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = busW;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd26: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = busW;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd27: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = busW;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd28: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = busW;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd29: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = busW;
            r30_w = r30_r;
            r31_w = r31_r;
        end
        5'd30: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = busW;
            r31_w = r31_r;
        end
        5'd31: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = busW;
        end
        /*default: begin
            r0_w = 32'd0;
            r1_w = r1_r;
            r2_w = r2_r;
            r3_w = r3_r;
            r4_w = r4_r;
            r5_w = r5_r;
            r6_w = r6_r;
            r7_w = r7_r;
            r8_w = r8_r;
            r9_w = r9_r;
            r10_w = r10_r;
            r11_w = r11_r;
            r12_w = r12_r;
            r13_w = r13_r;
            r14_w = r14_r;
            r15_w = r15_r;
            r16_w = r16_r;
            r17_w = r17_r;
            r18_w = r18_r;
            r19_w = r19_r;
            r20_w = r20_r;
            r21_w = r21_r;
            r22_w = r22_r;
            r23_w = r23_r;
            r24_w = r24_r;
            r25_w = r25_r;
            r26_w = r26_r;
            r27_w = r27_r;
            r28_w = r28_r;
            r29_w = r29_r;
            r30_w = r30_r;
            r31_w = r31_r;
        end*/
    endcase
end

always @(*) begin 
    case(RX)
        5'd0: r_busX = 32'd0;
        5'd1: r_busX = r1_r;
        5'd2: r_busX = r2_r;
        5'd3: r_busX = r3_r;
        5'd4: r_busX = r4_r;
        5'd5: r_busX = r5_r;
        5'd6: r_busX = r6_r;
        5'd7: r_busX = r7_r;
        5'd8: r_busX = r8_r;
        5'd9: r_busX = r9_r;
        5'd10: r_busX = r10_r;
        5'd11: r_busX = r11_r;
        5'd12: r_busX = r12_r;
        5'd13: r_busX = r13_r;
        5'd14: r_busX = r14_r;
        5'd15: r_busX = r15_r;
        5'd16: r_busX = r16_r;
        5'd17: r_busX = r17_r;
        5'd18: r_busX = r18_r;
        5'd19: r_busX = r19_r;
        5'd20: r_busX = r20_r;
        5'd21: r_busX = r21_r;
        5'd22: r_busX = r22_r;
        5'd23: r_busX = r23_r;
        5'd24: r_busX = r24_r;
        5'd25: r_busX = r25_r;
        5'd26: r_busX = r26_r;
        5'd27: r_busX = r27_r;
        5'd28: r_busX = r28_r;
        5'd29: r_busX = r29_r;
        5'd30: r_busX = r30_r;
        5'd31: r_busX = r31_r;
        //default: r_busX = 32'd0;
    endcase // RX
    case(RY)
        5'd0: r_busY = 32'd0;
        5'd1: r_busY = r1_r;
        5'd2: r_busY = r2_r;
        5'd3: r_busY = r3_r;
        5'd4: r_busY = r4_r;
        5'd5: r_busY = r5_r;
        5'd6: r_busY = r6_r;
        5'd7: r_busY = r7_r;
        5'd8: r_busY = r8_r;
        5'd9: r_busY = r9_r;
        5'd10: r_busY = r10_r;
        5'd11: r_busY = r11_r;
        5'd12: r_busY = r12_r;
        5'd13: r_busY = r13_r;
        5'd14: r_busY = r14_r;
        5'd15: r_busY = r15_r;
        5'd16: r_busY = r16_r;
        5'd17: r_busY = r17_r;
        5'd18: r_busY = r18_r;
        5'd19: r_busY = r19_r;
        5'd20: r_busY = r20_r;
        5'd21: r_busY = r21_r;
        5'd22: r_busY = r22_r;
        5'd23: r_busY = r23_r;
        5'd24: r_busY = r24_r;
        5'd25: r_busY = r25_r;
        5'd26: r_busY = r26_r;
        5'd27: r_busY = r27_r;
        5'd28: r_busY = r28_r;
        5'd29: r_busY = r29_r;
        5'd30: r_busY = r30_r;
        5'd31: r_busY = r31_r;
        //default: r_busY = 32'd0;
    endcase // RY
end 
always@(posedge Clk, negedge rst) begin
    if(~rst) begin
        r0_r <= 32'd0;
        r1_r <= 32'd0;
        r2_r <= 32'd0;
        r3_r <= 32'd0;
        r4_r <= 32'd0;
        r5_r <= 32'd0;
        r6_r <= 32'd0;
        r7_r <= 32'd0;
        r8_r <= 32'd0;
        r9_r <= 32'd0;
        r10_r <= 32'd0;
        r11_r <= 32'd0;
        r12_r <= 32'd0;
        r13_r <= 32'd0;
        r14_r <= 32'd0;
        r15_r <= 32'd0;
        r16_r <= 32'd0;
        r17_r <= 32'd0;
        r18_r <= 32'd0;
        r19_r <= 32'd0;
        r20_r <= 32'd0;
        r21_r <= 32'd0;
        r22_r <= 32'd0;
        r23_r <= 32'd0;
        r24_r <= 32'd0;
        r25_r <= 32'd0;
        r26_r <= 32'd0;
        r27_r <= 32'd0;
        r28_r <= 32'd0;
        r29_r <= 32'd0;
        r30_r <= 32'd0;
        r31_r <= 32'd0;
    end
    else begin
        r0_r <= 32'd0;
        r1_r <= r1_w;
        r2_r <= r2_w;
        r3_r <= r3_w;
        r4_r <= r4_w;
        r5_r <= r5_w;
        r6_r <= r6_w;
        r7_r <= r7_w;
        r8_r <= r8_w;
        r9_r <= r9_w;
        r10_r <= r10_w;
        r11_r <= r11_w;
        r12_r <= r12_w;
        r13_r <= r13_w;
        r14_r <= r14_w;
        r15_r <= r15_w;
        r16_r <= r16_w;
        r17_r <= r17_w;
        r18_r <= r18_w;
        r19_r <= r19_w;
        r20_r <= r20_w;
        r21_r <= r21_w;
        r22_r <= r22_w;
        r23_r <= r23_w;
        r24_r <= r24_w;
        r25_r <= r25_w;
        r26_r <= r26_w;
        r27_r <= r27_w;
        r28_r <= r28_w;
        r29_r <= r29_w;
        r30_r <= r30_w;
        r31_r <= r31_w;
    end
end     

endmodule

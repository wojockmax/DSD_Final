`include "Multicycle_MIPS/ALU.v"
module Exec(
    // clock, stall
    Clk,
    rst_n,
    Stall,
    // control input
    ALUctrl_EX,
    RegDst_EX,
	ALUSrc_EX,
	MemToReg_EX,
	MemWrite_EX,
	MemRead_EX,
    RegWrite_EX,
    RegDst_ra_EX,
    // PC reg (for jalr and jal)
    StorePC_EX,
    // register file reg
    RegAddrY_EX,
    RegDataX_EX,
    RegDataY_EX,
    Sign_extended_EX,
    //Output
    JumpReg_addr_EX,
    MemToReg_MEM,
    MemWrite_MEM,
    MemRead_MEM,
    RegWrite_MEM,
    ALU_result_MEM,
    WriteReg_Addr_MEM,
    WriteMem_Data_MEM,
    // output for hazard
    RegAddr_EX,
    RegData_EX
);

//==== input/output declaration =============================== 
    // input & output 
    // clock, stall
    input wire Clk;
    input wire rst_n;
    input wire Stall;
    // control input
    input wire [3:0] ALUctrl_EX;
    input wire RegDst_EX;
    input wire ALUSrc_EX;
    input wire MemToReg_EX;
    input wire MemWrite_EX;
    input wire MemRead_EX;
    input wire RegWrite_EX;
    input wire RegDst_ra_EX;
    // PC reg (for jalr and jal)
    input wire [31:0] StorePC_EX;
    // register file reg
    input wire [4:0] RegAddrY_EX;
    input wire [31:0] RegDataX_EX;
    input wire [31:0] RegDataY_EX;
    input wire [31:0] Sign_extended_EX;
    // Output
    output wire [31:0] JumpReg_addr_EX;
    output reg MemToReg_MEM;
    output reg MemWrite_MEM;
    output reg MemRead_MEM;
    output reg RegWrite_MEM;
    output reg [31:0] ALU_result_MEM, WriteMem_Data_MEM;
    //output reg Hi_MEM, Lo_MEM;
    output reg [4:0] WriteReg_Addr_MEM;
    // output for hazard
    output wire [4:0] RegAddr_EX;
    output wire [31:0] RegData_EX;

   
 
//==== reg/wire declaration ================================ 
    // ALU input
    wire [31:0] ALU_inputX, ALU_inputY;
    wire [31:0] ALU_out;
    //Output
    reg [31:0] ALU_result_MEM_n, WriteMem_Data_MEM_n, Hi_MEM_n, Lo_MEM_n;
    reg [4:0] WriteReg_Addr_MEM_n;
    reg MemToReg_MEM_n, MemWrite_MEM_n, MemRead_MEM_n, RegWrite_MEM_n;
    //tmp
    wire [4:0] addr_tmp;

//==== combinational part =================================
    // submodule
    ALU a1(
    .ctrl(ALUctrl_EX),
    .x(ALU_inputX),
    .y(ALU_inputY),
    .out(ALU_out)
    );

    // assignment
    assign ALU_inputX = (ALUctrl_EX[3]) ? Sign_extended_EX : RegDataX_EX;
    assign ALU_inputY = (ALUSrc_EX) ? Sign_extended_EX : RegDataY_EX;
    assign RegData_EX = (RegDst_ra_EX) ? StorePC_EX : ALU_out;
    assign addr_tmp = (RegDst_EX) ? Sign_extended_EX[15:11] : RegAddrY_EX;
    assign RegAddr_EX = {5{RegDst_ra_EX}} | addr_tmp;

    // combinational
    always @(*) begin
        if(Stall) begin
            MemToReg_MEM_n = MemToReg_MEM;
            MemWrite_MEM_n = MemWrite_MEM;
            MemRead_MEM_n = MemRead_MEM;
            RegWrite_MEM_n = RegWrite_MEM;
            ALU_result_MEM_n = ALU_result_MEM; 
            WriteMem_Data_MEM_n = WriteMem_Data_MEM; 
            WriteReg_Addr_MEM_n = WriteReg_Addr_MEM;
            //Hi_MEM_n = Hi_MEM; 
            //Lo_MEM_n = Lo_MEM;       
        end
        else begin
            MemToReg_MEM_n = MemToReg_EX;
            MemWrite_MEM_n = MemWrite_EX;
            MemRead_MEM_n = MemRead_EX;
            RegWrite_MEM_n = RegWrite_EX;
            ALU_result_MEM_n = RegData_EX; 
            WriteMem_Data_MEM_n = RegDataY_EX; 
            WriteReg_Addr_MEM_n = RegAddr_EX;
            //Hi_MEM_n = Hi_MEM; 
            //Lo_MEM_n = Lo_MEM;
        end
    end

//==== sequential part ===================================
    always @(posedge Clk or negedge rst_n) begin 
        if(~rst_n) begin
            MemToReg_MEM <= 0;
            MemWrite_MEM <= 0;
            MemRead_MEM <= 0;
            RegWrite_MEM <= 0;
            ALU_result_MEM <= 0;
            WriteMem_Data_MEM <= 0;
            WriteReg_Addr_MEM <= 0;
            //Hi_MEM <= 0;
            //Lo_MEM <= 0;
        end else begin
            MemToReg_MEM <= MemToReg_MEM_n;
            MemWrite_MEM <= MemWrite_MEM_n;
            MemRead_MEM <= MemRead_MEM_n;
            RegWrite_MEM <= RegWrite_MEM_n;
            ALU_result_MEM <= ALU_result_MEM_n; 
            WriteMem_Data_MEM <= WriteMem_Data_MEM_n; 
            WriteReg_Addr_MEM <= WriteReg_Addr_MEM_n;
            //Hi_MEM <= Hi_MEM_n; 
            //Lo_MEM <= Lo_MEM_n;
        end
    end

endmodule



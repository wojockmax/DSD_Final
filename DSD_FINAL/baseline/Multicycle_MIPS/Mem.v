module Mem(
    // clock, stall
    Clk,
    rst_n,
    Stall,
    //Input
    MemToReg_MEM,
    RegWrite_MEM,
    ALU_result_MEM,
    WriteReg_Addr_MEM,
    Mem_Data_MEM,
    //Output
    RegData_MEM,
    RegWrite_WB,
    WriteReg_Addr_WB,
    WriteReg_Data_WB
);

//==== input/output declaration =============================== 
    // input & output 
    // clock, stall
    input wire Clk;
    input wire rst_n;
    input wire Stall;
    // input
    input wire MemToReg_MEM;
    input wire RegWrite_MEM;
    input wire [31:0] ALU_result_MEM;
    input wire [4:0] WriteReg_Addr_MEM;
    input wire [31:0] Mem_Data_MEM;
    // output
    output wire [31:0] RegData_MEM;
    output reg RegWrite_WB;
    output reg [4:0] WriteReg_Addr_WB;
    output reg [31:0] WriteReg_Data_WB;

//==== reg/wire declaration ==============================
    reg RegWrite_WB_n;
    reg [4:0] WriteReg_Addr_WB_n;
    reg [31:0] WriteReg_Data_WB_n;
//==== combinational part ================================ 
	assign RegData_MEM = (MemToReg_MEM) ? Mem_Data_MEM : ALU_result_MEM;

    always @(*) begin
        if(Stall) begin
            RegWrite_WB_n = RegWrite_WB;
            WriteReg_Data_WB_n = WriteReg_Data_WB;
            WriteReg_Addr_WB_n = WriteReg_Addr_WB;
        end
        else begin
            RegWrite_WB_n = RegWrite_MEM;
            WriteReg_Data_WB_n = RegData_MEM;
            WriteReg_Addr_WB_n = WriteReg_Addr_MEM;
        end
    end

//==== sequential part ===================================


    always @(posedge Clk or negedge rst_n) begin 
        if(~rst_n) begin
            RegWrite_WB <= 0;
            WriteReg_Data_WB <= 0;
            WriteReg_Addr_WB <= 0;
        end else begin
            RegWrite_WB <= RegWrite_WB_n;
            WriteReg_Data_WB <= WriteReg_Data_WB_n;
            WriteReg_Addr_WB <= WriteReg_Addr_WB_n;
        end
    end

endmodule



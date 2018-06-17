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
    input wire clk; 
    input wire rst_n;
//----------I cache interface-------        
    output wire ICACHE_ren;
    output wire ICACHE_wen;
    output wire [29:0] ICACHE_addr;
    output wire [31:0] ICACHE_wdata;
    input  wire ICACHE_stall;
    input  wire [31:0] ICACHE_rdata;
//----------D cache interface-------
    output wire DCACHE_ren;
    output wire DCACHE_wen;
    output wire [29:0] DCACHE_addr;
    output wire [31:0] DCACHE_wdata;
    input  wire DCACHE_stall;
    input  wire [31:0] DCACHE_rdata;

//==== reg/wire declaration ===============================
    // PCpath
    wire [31:0] PC;
    // IF
    wire [31:0] IR_IF;
    wire [31:0] PC_IF;
    // ID
    wire [31:0] IR_ID;
    wire [31:0] PC_ID;
    wire Jump_ID;
    wire Branch_result_ID;
    wire [29:0] PC_Sign_extended_ID;
    wire [31:0] PCtoReg_ID;
    wire [31:0] JumpReg_addr_ID;
    wire MemRead_ID;
    wire JumptoReg_ID;
    // EX
    wire [3:0] ALUctrl_EX;
    wire RegDst_EX;
    wire ALUSrc_EX;
    wire MemToReg_EX;
    wire MemWrite_EX;
    wire MemRead_EX;
    wire RegWrite_EX;
    wire RegDst_ra_EX;
    wire [4:0] RegAddrY_EX;
    wire [31:0] StorePC_EX;
    wire [31:0] RegDataX_EX;
    wire [31:0] RegDataY_EX;
    wire [31:0] Sign_extended_EX;
    // MEM
    wire MemToReg_MEM;
    wire MemWrite_MEM;
    wire MemRead_MEM;
    wire RegWrite_MEM;
    wire [31:0] ALU_result_MEM;
    wire [4:0] WriteReg_Addr_MEM;
    wire [31:0] WriteMem_Data_MEM;
    wire [31:0] Mem_Data_MEM;
    //WB
    wire MemToReg_WB;
    wire RegWrite_WB;
    wire [31:0] Mem_Data_WB;
    wire [31:0] ALU_result_WB;
    wire [4:0] WriteReg_Addr_WB;
    wire [31:0] WriteReg_Data_WB;
    // register file
    wire WEN;
    wire [4:0] RW;
    wire [31:0] busW;
    wire [4:0] RX;
    wire [4:0] RY;
    wire [31:0] busX;
    wire [31:0] busY;
    // forwarding
    wire [4:0] RegAddr_EX;
    wire [4:0] RegAddr_MEM;
    wire [4:0] RegAddr_WB;
    wire [31:0] RegData_EX;
    wire [31:0] RegData_MEM;
    wire [31:0] RegData_WB;
    wire [4:0] RegAddrX_ID;
    wire [4:0] RegAddrY_ID;
    wire [31:0] Data_X_hazard_in;
    wire [31:0] Data_X_hazard_out;
    wire [31:0] Data_Y_hazard_in;
    wire [31:0] Data_Y_hazard_out;
    // stall
    wire Stall_PC;
    wire Stall_Ifetch;
    wire Stall_RegDec;
    wire Stall_Exec;
    wire Stall_Mem;
    wire Flush_Ifetch;

//==== assignment =========================================
    // output 
    assign ICACHE_ren = 1;
    assign ICACHE_wen = 0;
    assign ICACHE_addr = PC[31:2];
    assign ICACHE_wdata = 0;
    assign DCACHE_ren = MemRead_MEM;
    assign DCACHE_wen = MemWrite_MEM;
    assign DCACHE_addr = ALU_result_MEM[31:2];
    assign DCACHE_wdata = WriteMem_Data_MEM;
    // IR
    assign IR_IF = ICACHE_rdata;
    // RegDec
    assign PCtoReg_ID = PC_ID;
    // Wr
    assign Mem_Data_MEM = DCACHE_rdata;
    // forwarding
    assign RegAddr_MEM = WriteReg_Addr_MEM;
    assign RegAddr_WB = WriteReg_Addr_WB;
    assign RegData_WB = WriteReg_Data_WB;
//==== submodule part (pipeline) ==========================

    PCpath pc_path(
        // clock, stall
        .Clk(clk),
        .rst_n(rst_n),
        .Stall(Stall_PC),
        //Input
        .PC_ID(PC_ID),
        .Branch_result_ID(Branch_result_ID),
        .Jump_ID(Jump_ID),
        .JumptoReg_ID(JumptoReg_ID),
        .IR_ID(IR_ID[25:0]),
        .PC_Sign_extended_ID(PC_Sign_extended_ID),
        .JumpReg_addr_ID(JumpReg_addr_ID),
        //output
        .PC(PC),
        .PC_IF(PC_IF)
    );
	
    Ifetch ife(
        // clock, stall, and flush
        .Clk(clk),
        .rst_n(rst_n),
        .Stall(Stall_Ifetch),
        .Flush(Flush_Ifetch),
        //Input
        .IR_IF(IR_IF),
        .PC_IF(PC_IF),
        //Output
        .IR_ID(IR_ID),
        .PC_ID(PC_ID)
    );

    RegDec rd(
        // clock, stall
        .Clk(clk),
        .rst_n(rst_n),
        .Stall(Stall_RegDec),
        // IR
        .IR(IR_ID),
        // register file
        .RegWrite_WB(RegWrite_WB),
        .WriteReg_Addr_WB(WriteReg_Addr_WB),
        .WriteReg_Data_WB(WriteReg_Data_WB),
        // control reg
        .ALUctrl_EX(ALUctrl_EX),
        .RegDst_EX(RegDst_EX),
        .ALUSrc_EX(ALUSrc_EX),
        .MemToReg_EX(MemToReg_EX),
        .MemWrite_EX(MemWrite_EX),
        .MemRead_EX(MemRead_EX),
        .RegWrite_EX(RegWrite_EX),
        .RegDst_ra_EX(RegDst_ra_EX),
        // PC reg (for jalr and jal)
        .StorePC_EX(StorePC_EX),
        // for PC
        .PC_Sign_extended_ID(PC_Sign_extended_ID),
        .Branch_result_ID(Branch_result_ID),
        .Jump_ID(Jump_ID),  
        .PCtoReg_ID(PCtoReg_ID),
        // register file reg
        .RegAddrY_EX(RegAddrY_EX),
        .RegDataX_EX(RegDataX_EX),
        .RegDataY_EX(RegDataY_EX),
        .Sign_extended_EX(Sign_extended_EX),
        // for load hazard
        .MemRead_ID(MemRead_ID),
        // for JumptoReg
        .JumptoReg_ID(JumptoReg_ID),
        .JumpReg_addr_ID(JumpReg_addr_ID),
        //hazard check
        .RegAddrX_ID(RegAddrX_ID),
        .RegAddrY_ID(RegAddrY_ID),
        .Data_X_hazard_in(Data_X_hazard_in),
        .Data_Y_hazard_in(Data_Y_hazard_in),
        .Data_X_hazard_out(Data_X_hazard_out),
        .Data_Y_hazard_out(Data_Y_hazard_out)
    );

    Exec ex(
        // clock, stall
        .Clk(clk),
        .rst_n(rst_n),
        .Stall(Stall_Exec),
        // control input
        .ALUctrl_EX(ALUctrl_EX),
        .RegDst_EX(RegDst_EX),
        .ALUSrc_EX(ALUSrc_EX),
        .MemToReg_EX(MemToReg_EX),
        .MemWrite_EX(MemWrite_EX),
        .MemRead_EX(MemRead_EX),
        .RegWrite_EX(RegWrite_EX),
        .RegDst_ra_EX(RegDst_ra_EX),
        // PC reg (for jalr and jal)
        .StorePC_EX(StorePC_EX),
        // register file reg
        .RegAddrY_EX(RegAddrY_EX),
        .RegDataX_EX(RegDataX_EX),
        .RegDataY_EX(RegDataY_EX),
        .Sign_extended_EX(Sign_extended_EX),
        //Output
        .MemToReg_MEM(MemToReg_MEM),
        .MemWrite_MEM(MemWrite_MEM),
        .MemRead_MEM(MemRead_MEM),
        .RegWrite_MEM(RegWrite_MEM),
        .ALU_result_MEM(ALU_result_MEM),
        .WriteReg_Addr_MEM(WriteReg_Addr_MEM),
        .WriteMem_Data_MEM(WriteMem_Data_MEM),
        // output for hazard
    	.RegAddr_EX(RegAddr_EX),
    	.RegData_EX(RegData_EX)
    );

    Mem memo(
        // clock, stall
        .Clk(clk),
        .rst_n(rst_n),
        .Stall(Stall_Mem),
        //Input
        .MemToReg_MEM(MemToReg_MEM),
        .RegWrite_MEM(RegWrite_MEM),
        .ALU_result_MEM(ALU_result_MEM),
        .WriteReg_Addr_MEM(WriteReg_Addr_MEM),
        .Mem_Data_MEM(Mem_Data_MEM),
        //Output
        .RegData_MEM(RegData_MEM),
        .RegWrite_WB(RegWrite_WB),
        .WriteReg_Addr_WB(WriteReg_Addr_WB),
        .WriteReg_Data_WB(WriteReg_Data_WB)
    );

    /*Wr w_reg(
        //Input
        .MemToReg_WB(MemToReg_WB),
        .ALU_result_WB(ALU_result_WB),
        .Mem_Data_WB(Mem_Data_WB),
        //Output
        .WriteReg_Data_WB(WriteReg_Data_WB)
    );*/
    
//==== submodule part (hazard) ============================
    Forwarding fw(
        // input (addr)
        .RegAddrX_ID(RegAddrX_ID),
        .RegAddrY_ID(RegAddrY_ID),
        .RegAddr_EX(RegAddr_EX),
        .RegAddr_MEM(RegAddr_MEM),
        .RegAddr_WB(RegAddr_WB),
        // input (control)
        .RegWrite_EX(RegWrite_EX),
        .RegWrite_MEM(RegWrite_MEM),
        .RegWrite_WB(RegWrite_WB),
        // input (data)
        .RegData_EX(RegData_EX),
        .RegData_MEM(RegData_MEM),
        .RegData_WB(RegData_WB),
        // input (data)
        .Data_X_hazard_in(Data_X_hazard_in),
        .Data_Y_hazard_in(Data_Y_hazard_in),
        // output(data)
        .Data_X_hazard_out(Data_X_hazard_out),
        .Data_Y_hazard_out(Data_Y_hazard_out)
    );

    StallFlush sf(
        // input (Stall)
        .Icache_Stall(ICACHE_stall),
        .Dcache_Stall(DCACHE_stall),
        // input (Load)
        .MemRead_ID(MemRead_ID),
        // input (Jump)
        .Jump_ID(Jump_ID),
        .JumptoReg_ID(JumptoReg_ID),
        // Branch
        .Branch_result_ID(Branch_result_ID),
        // output (Stall)
        .Stall_PC(Stall_PC),
        .Stall_Ifetch(Stall_Ifetch),
        .Stall_RegDec(Stall_RegDec),
        .Stall_Exec(Stall_Exec),
        .Stall_Mem(Stall_Mem),
        // output (Flush)
        .Flush_Ifetch(Flush_Ifetch)
    );


//=========================================================

endmodule

`include "Multicycle_MIPS/PCpath.v"
`include "Multicycle_MIPS/Ifetch.v"
`include "Multicycle_MIPS/RegDec.v"
`include "Multicycle_MIPS/Exec.v"
`include "Multicycle_MIPS/Mem.v"
//`include "Multicycle_MIPS/Wr.v"
`include "Multicycle_MIPS/Forwarding.v"
`include "Multicycle_MIPS/StallFlush.v"

`include "Multicycle_MIPS/main_control.v"
`include "Multicycle_MIPS/register_file.v"
module RegDec(
    // clock, stall
    Clk,
    rst_n,
    Stall,
    // IR
	IR,
    // register file
    RegWrite_WB,
    WriteReg_Addr_WB,
    WriteReg_Data_WB,
    // control reg
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
    // for PC
    PC_Sign_extended_ID,
    Branch_result_ID,
    Jump_ID,  
    PCtoReg_ID,
    // register file reg
    RegAddrY_EX,
    RegDataX_EX,
    RegDataY_EX,
    Sign_extended_EX,
    // for load hazard
    MemRead_ID,
    // for JumptoReg
    JumptoReg_ID,
    JumpReg_addr_ID,
    //hazard check
    RegAddrX_ID,
    RegAddrY_ID,
    Data_X_hazard_in,
    Data_Y_hazard_in,
    Data_X_hazard_out,
    Data_Y_hazard_out
);

//==== input/output declaration =============================== 
 
    // clock, stall
    input wire Clk;
    input wire rst_n;
    input wire Stall;
    // IR
    input wire [31:0] IR;
    //write register
    input wire RegWrite_WB;
    input wire [4:0] WriteReg_Addr_WB;
    input wire [31:0] WriteReg_Data_WB;
    // control reg 
    output reg [3:0] ALUctrl_EX;
    output reg RegDst_EX;
    output reg ALUSrc_EX;
    output reg MemToReg_EX;
    output reg MemWrite_EX;
    output reg MemRead_EX;
    output reg RegWrite_EX;
    output reg RegDst_ra_EX;
    // information of register file (send to next stage)
    output reg [4:0] RegAddrY_EX;
    output reg [31:0] RegDataX_EX, RegDataY_EX, Sign_extended_EX;
    // PC reg (for jalr and jal)
    output reg [31:0] StorePC_EX;
    // for PC 
    output wire [29:0] PC_Sign_extended_ID;
    input wire [31:0] PCtoReg_ID;
    output wire Branch_result_ID;
    output wire Jump_ID;
    // for load hazard
    output wire MemRead_ID;
    // for JumptoReg
    output wire JumptoReg_ID;
    output wire [31:0] JumpReg_addr_ID;
    //hazard check
    output wire [4:0] RegAddrX_ID;
    output wire [4:0] RegAddrY_ID;
    output wire [31:0] Data_X_hazard_in;
    output wire [31:0] Data_Y_hazard_in;
    output wire [31:0] Data_X_hazard_out;
    output wire [31:0] Data_Y_hazard_out;
 
 //==== reg/wire declaration =============================== 
    // control (next)
    reg [3:0] ALUctrl_EX_n;
    reg RegDst_EX_n;
    reg ALUSrc_EX_n;
    reg MemToReg_EX_n;
    reg RegWrite_EX_n;
    reg MemRead_EX_n;
    reg MemWrite_EX_n;    
    reg RegDst_ra_EX_n;
    wire Branch; 
    // connected from main control
    wire [3:0] ALUctrl_ctrl;
    wire RegDst_ctrl;
    wire ALUSrc_ctrl;
    wire MemToReg_ctrl;
    wire RegWrite_ctrl;
    wire MemRead_ctrl;
    wire MemWrite_ctrl;    
    wire RegDst_ra_ctrl;
    wire JumptoReg_ctrl; 
    // register file
    wire [4:0] addrx;
    wire [4:0] addry;
    wire [31:0] busX;
    wire [31:0] busY;
    wire RW;
    // IR
    wire [5:0] Op; 
    wire [4:0] rs, rt;
    wire [15:0] imm;
    wire [5:0] funct;
    reg [31:0] Sign_extended_EX_n;

    // PC reg (for jalr and jal)
    reg [31:0] StorePC_EX_n;

    // register file (send to next stage)
    reg [4:0] RegAddrY_EX_n;
    reg [31:0] RegDataX_EX_n, RegDataY_EX_n;

//==== combinational part =================================
    // submodule
    main_control mc(
    .Op(Op),
    .funct(funct),
    .RegDst(RegDst_ctrl),
    .ALUSrc(ALUSrc_ctrl),
    .MemToReg(MemToReg_ctrl),
    .RegWrite(RegWrite_ctrl),
    .MemRead(MemRead_ctrl),
    .MemWrite(MemWrite_ctrl),
    .Branch(Branch), 
    .Jump(Jump_ID),  
    .RegDst_ra(RegDst_ra_ctrl), 
    .JumptoReg(JumptoReg_ctrl),
    .ALUctrl(ALUctrl_ctrl)
    );

    register_file rf(
        .Clk(Clk),
        .rst_n(rst_n),
        .WEN(RegWrite_WB),
        .RW(WriteReg_Addr_WB),
        .busW(WriteReg_Data_WB),
        .RX(addrx),
        .RY(addry),
        .busX(busX),
        .busY(busY)
    );

    // assignment for output
    assign Branch_result_ID = Branch & (!(Data_X_hazard_out ^ Data_Y_hazard_out));

    // assignment for control
    assign Op = IR[31:26];
    assign rs = IR[25:21];
    assign rt = IR[20:16];
    assign imm = IR[15:0];
    assign funct = IR[5:0];

    // send to register (asynchrous)
    assign addrx = {5{RegDst_ra_ctrl}} | rs;
    assign addry = rt;

    // PC
    assign PC_Sign_extended_ID = {{14{imm[15]}}, imm};

    // for load hazard
    assign MemRead_ID = MemRead_ctrl;
    
    // for JumptoReg
    assign JumptoReg_ID = JumptoReg_ctrl;
    assign JumpReg_addr_ID = Data_X_hazard_out;

    // for hazard
    assign RegAddrX_ID = addrx;
    assign RegAddrY_ID = addry;
    assign Data_X_hazard_in = busX;
    assign Data_Y_hazard_in = busY;

    // combinational
    always @(*) begin
        if(Stall) begin
            ALUctrl_EX_n = ALUctrl_EX;
            RegDst_EX_n = RegDst_EX;
            ALUSrc_EX_n = ALUSrc_EX;
            MemToReg_EX_n = MemToReg_EX;
            RegWrite_EX_n = RegWrite_EX;
            MemRead_EX_n = MemRead_EX;
            MemWrite_EX_n = MemWrite_EX;    
            RegDst_ra_EX_n = RegDst_ra_EX;
            Sign_extended_EX_n = Sign_extended_EX;
            StorePC_EX_n =  StorePC_EX;
            RegAddrY_EX_n = RegAddrY_EX;
            RegDataX_EX_n = RegDataX_EX;
            RegDataY_EX_n = RegDataY_EX;
        end
        else begin
            ALUctrl_EX_n = ALUctrl_ctrl;
            RegDst_EX_n = RegDst_ctrl;
            ALUSrc_EX_n = ALUSrc_ctrl;
            MemToReg_EX_n = MemToReg_ctrl;
            RegWrite_EX_n = RegWrite_ctrl;
            MemRead_EX_n = MemRead_ctrl;
            MemWrite_EX_n = MemWrite_ctrl;    
            RegDst_ra_EX_n = RegDst_ra_ctrl;
            Sign_extended_EX_n = {{2{imm[15]}}, PC_Sign_extended_ID};
            StorePC_EX_n = PCtoReg_ID;
            RegAddrY_EX_n = addry;
            RegDataX_EX_n = Data_X_hazard_out;
            RegDataY_EX_n = Data_Y_hazard_out;
        end
    end
    
//==== sequential part ===================================
    always @(posedge Clk or negedge rst_n) begin 
        if(~rst_n) begin
            ALUctrl_EX <= 0;
            RegDst_EX <= 0;
            ALUSrc_EX <= 0;
            MemToReg_EX <= 0;
            RegWrite_EX <= 0;
            MemRead_EX <= 0;
            MemWrite_EX <= 0;    
            RegDst_ra_EX <= 0;
            Sign_extended_EX <= 0;
            StorePC_EX <= 0;
            RegAddrY_EX <= 0;
            RegDataX_EX <= 0;
            RegDataY_EX <= 0;
        end else begin
            ALUctrl_EX <= ALUctrl_EX_n;
            RegDst_EX <= RegDst_EX_n;
            ALUSrc_EX <= ALUSrc_EX_n;
            MemToReg_EX <= MemToReg_EX_n;
            RegWrite_EX <= RegWrite_EX_n;
            MemRead_EX <= MemRead_EX_n;
            MemWrite_EX <= MemWrite_EX_n;    
            RegDst_ra_EX <= RegDst_ra_EX_n;
            Sign_extended_EX <= Sign_extended_EX_n;
            StorePC_EX <=  StorePC_EX_n;
            RegAddrY_EX <= RegAddrY_EX_n;
            RegDataX_EX <= RegDataX_EX_n;
            RegDataY_EX <= RegDataY_EX_n;
        end
    end

endmodule



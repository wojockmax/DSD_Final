module PCpath(
    // clock, stall
    Clk,
    rst_n,
    Stall,
    //Input
    PC_ID,
    Branch_result_ID,
    Jump_ID,
    JumptoReg_ID,
    IR_ID,
    PC_Sign_extended_ID,
    JumpReg_addr_ID,
    //output
    PC,
    PC_IF
);

//==== input/output declaration =============================== 
    // input & output 
    // clock, stall
    input wire Clk;
    input wire rst_n;
    input wire Stall;
    // input
    input wire [31:0] PC_ID;
    input wire Branch_result_ID;
    input wire Jump_ID;
    input wire JumptoReg_ID;
    input wire [25:0] IR_ID;
    input wire [29:0] PC_Sign_extended_ID;
    input wire [31:0] JumpReg_addr_ID;
    // output
    output reg [31:0] PC;
    output wire [31:0] PC_IF;
//==== reg/wire declaration =============================== 
    wire [31:0] PCadd, Jump_addr, PC_tmp, Branch_addr;
    reg [31:0] PC_n;
//==== combinational part ================================ 
	
	assign PCadd = {PC[31:2] + 1, PC[1:0]};
    assign PC_IF = PCadd;
    assign Jump_addr = {PC_ID[31:28], IR_ID, 2'd0};
    assign Branch_addr = {PC_ID[31:2] + PC_Sign_extended_ID, PC_ID[1:0]};
    assign PC_tmp = (JumptoReg_ID) ? JumpReg_addr_ID : ((Jump_ID) ? Jump_addr : ((Branch_result_ID) ? Branch_addr : PCadd));

    always @(*) begin
        if(Stall) begin
            PC_n = PC;
        end
        else begin
            PC_n = PC_tmp;
        end
    end

//==== sequential part ===================================
    always @(posedge Clk or negedge rst_n) begin
        if(~rst_n) begin
            PC <= 0;
        end else begin
            PC <= PC_n;
        end
    end
endmodule



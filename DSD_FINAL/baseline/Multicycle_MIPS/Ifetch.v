module Ifetch(
    // clock, stall, and flush
    Clk,
    rst_n,
    Stall,
    Flush,
    //Input
    IR_IF,
    PC_IF,
    //Output
    IR_ID,
    PC_ID
);

//==== input/output declaration =============================== 
    // input & output 
    // clock, stall, and flush
    input wire Clk;
    input wire rst_n;
    input wire Stall;
    input wire Flush;
    // input
    input wire [31:0] IR_IF;
    input wire [31:0] PC_IF;
    // output
    output reg [31:0] IR_ID;
    output reg [31:0] PC_ID;
//==== reg/wire declaration =============================== 
    reg [31:0] IR_ID_n, PC_ID_n;
//==== combinational part ================================ 
	
	always @(*) begin
		if (Stall) begin
			IR_ID_n = IR_ID;
			PC_ID_n = PC_ID;
		end
		else if(Flush) begin
			IR_ID_n = 0;
			PC_ID_n = 0;
		end
		else begin
			IR_ID_n = IR_IF;
			PC_ID_n = PC_IF;
		end	
	end
//==== sequential part ===================================
	
	always @(posedge Clk or negedge rst_n) begin
		if(~rst_n) begin
			IR_ID <= 0;
			PC_ID <= 0;
		end else begin
			IR_ID <= IR_ID_n;
			PC_ID <= PC_ID_n;
		end
	end



endmodule



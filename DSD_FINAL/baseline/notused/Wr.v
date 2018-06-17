module Wr(
    //Input
    MemToReg_WB,
    ALU_result_WB,
    Mem_Data_WB,
    //Output
    WriteReg_Data_WB,
);

//==== input/output declaration =============================== 
    // input & output 
    // input
    input wire MemToReg_WB;
    input wire [31:0] ALU_result_WB;
    input wire [31:0] Mem_Data_WB;
    // output
    output wire [31:0] WriteReg_Data_WB;

//==== combinational part ================================ 
	
	assign WriteReg_Data_WB = (MemToReg_WB) ? Mem_Data_WB : ALU_result_WB;

endmodule



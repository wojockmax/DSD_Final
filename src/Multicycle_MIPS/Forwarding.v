module Forwarding(
    RegAddrX_ID,
    RegAddrY_ID,
    RegAddr_EX,
    RegAddr_MEM,
    RegAddr_WB,
    // input (control)
    RegWrite_EX,
    RegWrite_MEM,
    RegWrite_WB,
    // input (data)
    RegData_EX,
    RegData_MEM,
    RegData_WB,
    // input (data)
    Data_X_hazard_in,
    Data_Y_hazard_in,
    // output(data)
    Data_X_hazard_out,
    Data_Y_hazard_out
);

//==== input/output declaration =============================== 
    input wire [4:0] RegAddrX_ID;
    input wire [4:0] RegAddrY_ID;
    input wire [4:0] RegAddr_EX;
    input wire [4:0] RegAddr_MEM;
    input wire [4:0] RegAddr_WB;
    // input (control)
    input wire RegWrite_EX;
    input wire RegWrite_MEM;
    input wire RegWrite_WB;
    // input (data)
    input wire [31:0] RegData_EX;
    input wire [31:0] RegData_MEM;
    input wire [31:0] RegData_WB;
    // input (data)
    input wire [31:0] Data_X_hazard_in;
    input wire [31:0] Data_Y_hazard_in;
    // output(data)
    output wire [31:0] Data_X_hazard_out;
    output wire [31:0] Data_Y_hazard_out;

//==== reg/wire declaration ================================ 
    wire X_EX, X_MEM, X_WB, Y_EX, Y_MEM, Y_WB;
//==== combinational part ================================== 
    assign X_EX = (!(RegAddrX_ID ^ RegAddr_EX) & RegWrite_EX & |(RegAddrX_ID));
    assign X_MEM = (!(RegAddrX_ID ^ RegAddr_MEM) & RegWrite_MEM & |(RegAddrX_ID));
    assign X_WB = (!(RegAddrX_ID ^ RegAddr_WB) & RegWrite_WB & |(RegAddrX_ID));
    assign Y_EX = (!(RegAddrY_ID ^ RegAddr_EX) & RegWrite_EX & |(RegAddrY_ID));
    assign Y_MEM = (!(RegAddrY_ID ^ RegAddr_MEM) & RegWrite_MEM & |(RegAddrY_ID));
    assign Y_WB = (!(RegAddrY_ID ^ RegAddr_WB) & RegWrite_WB & |(RegAddrY_ID));

    assign Data_X_hazard_out = (X_EX) ? RegData_EX : ((X_MEM) ? RegData_MEM : ((X_WB) ? RegData_WB : Data_X_hazard_in));
    assign Data_Y_hazard_out = (Y_EX) ? RegData_EX : ((Y_MEM) ? RegData_MEM : ((Y_WB) ? RegData_WB : Data_Y_hazard_in));

endmodule



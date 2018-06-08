module ALU_control(
	Op,
    funct,
    ALUOp,
    ctrl
);
	input [5:0] Op;
    input [5:0] funct;
    input [1:0] ALUOp;
    output [3:0] ctrl;

    wire [5:0] Op;
    wire [5:0] funct;
    wire [3:0] ctrl;
    wire [1:0] ALUOp;
    wire [3:0] r_ctrl, i_ctrl;

    R_control rctrl(.funct(funct), .ctrl(r_ctrl));
    I_control ictrl(.Op(Op), .ctrl(i_ctrl));

    assign ctrl = (ALUOp[1]) ? r_ctrl : i_ctrl;

endmodule

module R_control(
    funct,
    ctrl
);
    input [5:0] funct;
    output [3:0] ctrl;

    wire [5:0] funct;
    wire [3:0] ctrl;

    assign ctrl[3] = (~funct[3]) & (~funct[5]);
    assign ctrl[2] = (funct[3] & funct[1]) | (funct[2] & funct[1]);
    assign ctrl[1] = (funct[2] & (~funct[1])) | (funct[3] & funct[1]);
    assign ctrl[0] = ((~funct[3]) & funct[0]) | ((~funct[3]) & (~funct[2]) & funct[1]);


endmodule

module I_control(
	Op,
    ctrl
);
	input [5:0] Op;
    output [3:0] ctrl;

    wire [5:0] Op;
    wire [3:0] ctrl;

    assign ctrl[3] = 0;
    assign ctrl[2] = (~Op[5]) & Op[1];
    assign ctrl[1] = (Op[2] & (~Op[1])) | ((~Op[5]) & (~Op[2]) & Op[1]);
    assign ctrl[0] = (~Op[5]) & Op[0];


endmodule

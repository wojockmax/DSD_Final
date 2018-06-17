module ALU(
    ctrl,
    x,
    y,
    out 
);
    
    input  [3:0] ctrl;
    input  [31:0] x;
    input  [31:0] y;
    //output       zero;
    output [31:0] out;

    wire [3:0] ctrl;
    wire [4:0] shamt;
    //reg carry;
    reg [31:0] out;

    //assign zero = !(x ^ y);
    assign shamt = x[10:6];

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

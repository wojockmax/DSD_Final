module StallFlush(
    // input (Stall)
    Icache_Stall,
    Dcache_Stall,
    // input (Load)
    MemRead_ID,
    // input (Jump)
    Jump_ID,
    JumptoReg_ID,
    // Branch
    Branch_result_ID,
    // output (Stall)
    Stall_PC,
    Stall_Ifetch,
    Stall_RegDec,
    Stall_Exec,
    Stall_Mem,
    // output (Flush)
    Flush_Ifetch
);

//==== input/output declaration =============================== 
    // input (Stall)
    input wire Icache_Stall;
    input wire Dcache_Stall;
    // input (Load)
    input wire MemRead_ID;
    // input (Jump)
    input wire Jump_ID;
    input wire JumptoReg_ID;
    // Branch
    input wire Branch_result_ID;
    // output (Stall)
    output wire Stall_PC;
    output wire Stall_Ifetch;
    output wire Stall_RegDec;
    output wire Stall_Exec;
    output wire Stall_Mem;
    // output (Flush)
    output wire Flush_Ifetch;

//==== combinational part ================================ 

    assign Stall_PC = Icache_Stall | Dcache_Stall | MemRead_ID;
    assign Stall_Ifetch = Icache_Stall | Dcache_Stall;
    assign Stall_RegDec = Icache_Stall | Dcache_Stall;
    assign Stall_Exec = Icache_Stall | Dcache_Stall;
    assign Stall_Mem = Icache_Stall | Dcache_Stall;
    assign Flush_Ifetch = Branch_result_ID | MemRead_ID | Jump_ID | JumptoReg_ID;	

endmodule



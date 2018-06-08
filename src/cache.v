module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_wdata,
    proc_stall,
    proc_rdata,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input             proc_reset;
    input             proc_read, proc_write;
    input      [29:0] proc_addr;
    input      [31:0] proc_wdata;
    output reg        proc_stall; //asynchronous output
    output reg [31:0] proc_rdata; //asynchronous output
    // memory interface
    input      [127:0] mem_rdata;
    input              mem_ready;
    output reg         mem_read, mem_write;
    output reg [27:0]  mem_addr;
    output reg [127:0] mem_wdata;
    
//==== wire/reg definition ================================
    
    // parameter
    localparam cache_length = 8;
    //localparam n_way        =  2;
    localparam blocknum     =  4;
    localparam tagnum       = 26; 
    // blocknum = cache_length / n_way
    // tagnum   = addr_length - log2(blocknum) -2  
    integer i; 
    reg [127:0] word [0:cache_length-1];
    reg [127:0] word_n [0:cache_length-1];
    reg [tagnum-1:0] tag_val [0:cache_length-1];
    reg [tagnum-1:0] tag_val_n [0:cache_length-1];
    reg [7:0] v_bit, v_bit_n;
    reg [7:0] d_bit, d_bit_n;

    //memory output register
    reg mem_read_n, mem_write_n;
    reg [27:0] mem_addr_n;
    reg [127:0] mem_wdata_n;
    

    //help
    wire hit;
    wire [2:0] hitpos;
    wire [1:0] tag_eq, valid, hit_block;
    wire [25:0] tag;
    wire [1:0] block;
    wire [1:0] offset;

    //temp store
    reg [127:0] selected_block;
    reg [27:0] mem_addr_tmp, mem_addr_tmp_n;
    reg [127:0] mem_wdata_tmp, mem_wdata_tmp_n;

    //dirty
    reg dirty, dirty_n;
    reg [3:0] two_way, two_way_n;

    wire[2:0] RU_block, NRU_block;

    //state
    reg [1:0] state, state_n;
    localparam [1:0] IDLE = 2'd0;
    localparam [1:0] EVALUATE = 2'd1;
    localparam [1:0] UPDATE_MEM = 2'd2;
    localparam [1:0] FETCH_MEM = 2'd3;

//==== combinational circuit ==============================

    assign tag = proc_addr[29:4];
    assign block = proc_addr[3:2];
    assign offset = proc_addr[1:0];
    assign NRU_block = {block, ~two_way[block]};
    assign RU_block = {block, two_way[block]};
    assign valid[0] = v_bit[{block, 1'b0}];
    assign valid[1] = v_bit[{block, 1'b1}];
    assign tag_eq[0] = (tag == tag_val[{block, 1'b0}]) ? 1 : 0;
    assign tag_eq[1] = (tag == tag_val[{block, 1'b1}]) ? 1 : 0;
    assign hit_block[0] = valid[0] & tag_eq[0];
    assign hit_block[1] = valid[1] & tag_eq[1];
    assign hit = hit_block[0] | hit_block[1];
    assign hitpos = {block, hit_block[1]};

    always@(*) begin
        for(i = 0; i < cache_length; i = i + 1) begin
            tag_val_n[i] = tag_val[i];
            word_n[i] = word[i];
        end
        mem_read_n = mem_read;
        mem_write_n = mem_write;
        mem_addr_n = mem_addr;
        mem_wdata_n = mem_wdata;
        v_bit_n = v_bit;
        d_bit_n = d_bit;
        dirty_n = dirty;
        mem_addr_tmp_n = mem_addr_tmp;
        mem_wdata_tmp_n = mem_wdata_tmp;
        two_way_n = two_way;
        case(state)

            IDLE: begin
                if(~proc_read & ~proc_write) begin
                    state_n = IDLE;
                end
                else begin
                    state_n = EVALUATE;
                end
                proc_stall = 0;
                proc_rdata = 0;
                mem_read_n = 0;
                mem_write_n = 0;
                mem_addr_n = 0;
                mem_wdata_n = 0;
                v_bit_n[RU_block] = 0;
                selected_block = 0;
            end

            EVALUATE: begin
                if(~proc_read && ~proc_write) begin
                    state_n = state;
                    proc_stall = 0;
                    proc_rdata = 0;
                    mem_read_n = 0;
                    mem_write_n = 0;
                    mem_addr_n = 0;
                    mem_wdata_n = 0;
                end
                else begin
                    if(hit) begin   
                        state_n = EVALUATE;
                        two_way_n[block] = hit_block[1];
                        if(proc_read) begin //read hit 
                            selected_block = word[hitpos];
                            case(offset)
                                2'd0: proc_rdata = selected_block[31:0];
                                2'd1: proc_rdata = selected_block[63:32];
                                2'd2: proc_rdata = selected_block[95:64];
                                2'd3: proc_rdata = selected_block[127:96];
                            endcase
                        end
                        else if(proc_write) begin //write hit   
                            case(offset)
                                2'd0: word_n[hitpos] = {word[hitpos][127:32], proc_wdata};
                                2'd1: word_n[hitpos] = {word[hitpos][127:64], proc_wdata, word[hitpos][31:0]};
                                2'd2: word_n[hitpos] = {word[hitpos][127:96], proc_wdata, word[hitpos][63:0]};
                                2'd3: word_n[hitpos] = {proc_wdata, word[hitpos][95:0]};
                            endcase
                            d_bit_n[hitpos] = 1'b1;
                            selected_block = 0;
                            proc_rdata = 0;
                        end
                        else begin //default
                            selected_block = 0;
                            proc_rdata = 0;
                        end 

                        proc_stall = 0;
                        mem_read_n = 0;
                        mem_write_n = 0;
                        mem_addr_n = 0;
                        mem_wdata_n = 0;
                    end
                    

                    else begin //miss
                        selected_block = 0;
                        if(d_bit[NRU_block]) begin //miss with dirty data address       
                            //read miss & write miss do the same thing first: write data back to memory   
                            dirty_n = 1;
                            mem_addr_tmp_n = {tag_val[NRU_block], block};
                            mem_wdata_tmp_n = word[NRU_block];          
                        end
                        else begin
                            dirty_n = 0;
                            mem_addr_tmp_n = 0;
                            mem_wdata_tmp_n = 0;                  
                        end
                        state_n = FETCH_MEM;
                        proc_stall = 1;
                        proc_rdata = 0;
                        mem_read_n = 1;
                        mem_write_n = 0;
                        mem_addr_n = proc_addr[29:2];
                        mem_wdata_n = 0;
                    end
                end
            end

            UPDATE_MEM: begin
                if(~proc_read && ~proc_write) begin
                    if(mem_ready) begin
                        state_n = EVALUATE;
                        mem_read_n = 0;
                        mem_write_n = 0;
                        mem_addr_n = 0;
                        mem_wdata_n = 0;
                        mem_addr_tmp_n = 0;
                        mem_wdata_tmp_n = 0;
                        dirty_n = 0;
                    end
                    else begin
                        state_n = UPDATE_MEM;
                    end
                    selected_block = 0;
                    proc_stall = 0;
                    proc_rdata = 0;
                end
                else begin
                    if(hit) begin   
                        if(mem_ready) begin
                            state_n = EVALUATE;
                            mem_read_n = 0;
                            mem_write_n = 0;
                            mem_addr_n = 0;
                            mem_wdata_n = 0;
                            mem_addr_tmp_n = 0;
                            mem_wdata_tmp_n = 0;
                            dirty_n = 0;
                        end
                        else begin
                            state_n = UPDATE_MEM;
                        end
                        if(proc_read) begin //read hit 
                            selected_block = word[hitpos];
                            case(offset)
                                2'd0: proc_rdata = selected_block[31:0];
                                2'd1: proc_rdata = selected_block[63:32];
                                2'd2: proc_rdata = selected_block[95:64];
                                2'd3: proc_rdata = selected_block[127:96];
                            endcase
                        end
                        else if(proc_write) begin //write hit   
                            case(offset)
                                2'd0: word_n[hitpos] = {word[hitpos][127:32], proc_wdata};
                                2'd1: word_n[hitpos] = {word[hitpos][127:64], proc_wdata, word[hitpos][31:0]};
                                2'd2: word_n[hitpos] = {word[hitpos][127:96], proc_wdata, word[hitpos][63:0]};
                                2'd3: word_n[hitpos] = {proc_wdata, word[hitpos][95:0]};
                            endcase
                            d_bit_n[hitpos] = 1'b1;
                            selected_block = 0;
                            proc_rdata = 0;
                        end
                        else begin //default
                            selected_block = 0;
                            proc_rdata = 0;
                        end 
                        proc_stall = 0;
                        two_way_n[block] = hit_block[1];
                    end
                    

                    else begin //miss
                        if(mem_ready) begin
                            selected_block = 0;
                            if(d_bit[NRU_block]) begin //miss with dirty data address       
                                //read miss & write miss do the same thing first: write data back to memory   
                                dirty_n = 1;
                                mem_addr_tmp_n = {tag_val[NRU_block], block};
                                mem_wdata_tmp_n = word[NRU_block];
                            end
                            else begin
                                dirty_n = 0;
                                mem_addr_tmp_n = 0;
                                mem_wdata_tmp_n = 0;
                            end
                            state_n = FETCH_MEM;
                            mem_read_n = 1;
                            mem_write_n = 0;
                            mem_addr_n = proc_addr[29:2];
                            mem_wdata_n = 0;
                        end
                        else begin
                            selected_block = 0;
                            dirty_n = 0;
                            mem_addr_tmp_n = 0;
                            mem_wdata_tmp_n = 0;
                            state_n = UPDATE_MEM;
                        end
                        proc_stall = 1;
                        proc_rdata = 0;
                    end
                end           
                
            end

            FETCH_MEM: begin

                selected_block = 0;


                if(mem_ready) begin
                    if(dirty) begin
                        state_n = UPDATE_MEM; 
                        mem_read_n = 0;
                        mem_write_n = 1;
                        mem_addr_n = mem_addr_tmp;
                        mem_wdata_n = mem_wdata_tmp;

                    end
                    else begin
                        state_n = EVALUATE; 
                        mem_read_n = 0;
                        mem_write_n = 0;
                        mem_addr_n = 0;
                        mem_wdata_n = 0;
                    end
                    v_bit_n[NRU_block] = 1'b1;
                    d_bit_n[NRU_block] = 1'b0;
                    tag_val_n[NRU_block] = proc_addr[29:4];
                    word_n[NRU_block] = mem_rdata;
                    proc_stall = 1;
                    proc_rdata = 0;
                end

                else begin //wait for memory
                    state_n = FETCH_MEM;
                    proc_stall = 1;
                    proc_rdata = 0;
                    mem_read_n = 1;
                    mem_write_n = 0;
                    mem_addr_n = mem_addr;
                    mem_wdata_n = 0;
                end
            end

        endcase
    end

//==== sequential circuit =================================
    always@( posedge clk or posedge proc_reset ) begin
        if( proc_reset ) begin
            state <= EVALUATE;
            for(i = 0; i < cache_length; i = i + 1) begin
                tag_val[i] = 25'd0;
                word[i] = 127'd0;
            end
            v_bit <= 7'd0;
            d_bit <= 7'd0;
            mem_read <= 0;
            mem_write <= 0;
            mem_addr <= 0;
            mem_wdata <= 0;
            mem_addr_tmp <= 0;
            mem_wdata_tmp <= 0;
            dirty <= 0;
            two_way <= 0;
        end
        else begin
            state <= state_n;
            for(i = 0; i < cache_length; i = i + 1) begin
                tag_val[i] = tag_val_n[i];
                word[i] = word_n[i];
            end
            v_bit <= v_bit_n;
            d_bit <= d_bit_n;
            mem_read <= mem_read_n;
            mem_write <= mem_write_n;
            mem_addr <= mem_addr_n;
            mem_wdata <= mem_wdata_n;
            mem_addr_tmp <= mem_addr_tmp_n;
            mem_wdata_tmp <= mem_wdata_tmp_n;
            dirty <= dirty_n;
            two_way <= two_way_n;
        end
    end

endmodule
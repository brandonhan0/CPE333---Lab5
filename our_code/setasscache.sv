`timescale 1ns / 1ps

// should be all done

module setasscache (
    input  logic         CLK,
    input  logic         RST,
    input  logic [31:0]  address, // 
    input  logic         read,
    input  logic         write,
    input  logic [31:0]  write_data,
    input  logic [1:0]   size,
    input  logic         sign, // sign extension for reads
    input  logic [31:0]  IO_IN, //io in
    input  logic         update,
    input  logic         writeback,
    input  logic [31:0]  w0, w1, w2, w3,
    output logic [31:0]  ow0, ow1, ow2, ow3,
    output logic [31:0]  out,    // output when loading
    output logic         hit,
    output logic         miss,
    output logic         IO_WR, // io
    output logic         weAddrValid,
    output logic         mem_read,    // signal to fetch block from memory
    output logic [31:0]  mem_rd_addr, // address of the block to fetch
    output logic         mem_write,   // from fsm to write dirty block to mem
    output logic [31:0]  mem_wr_addr, // address of block to write to
    output logic         vic_dirty,
    output logic         vic_valid
    );
    
    parameter NUM_SETS    = 4;
    parameter WAYS        = 4;
    parameter BLOCK_WORDS = 4;   // each block is 4 words (4 × 32 bits = 128 bits)


    // A single cache line holds:
    //  - valid, dirty flags
    //  - 26-bit tag (address[31:6])
    //  - 128-bit block (4 words × 32 bits)
    typedef struct packed {
        logic        valid;
        logic        dirty;
        logic [25:0] tag;
        logic [127:0] block;
    } cache_line_t;


    // The 4-way, 4-set array of cache lines:
    cache_line_t     cache    [NUM_SETS][WAYS];
    // For true LRU: a 2-bit counter per way per set (0 = most recently used, 3 = least recently used)
    logic [1:0]      lru      [NUM_SETS][WAYS];


    // Decode address into:
    //  - word_offset = address[3:2]  (which 32-bit word in the 16-byte block)
    //  - set_index   = address[5:4]  (which of the 4 sets)
    //  - tag         = address[31:6]
    logic [1:0]      word_offset = address[3:2];
    logic [1:0]      set_index   = address[5:4];
    logic [25:0]     tag         = address[31:6];
    logic [31:0]     read_data;
    logic  i, j;
    integer lru_way = 0;
    //hit logic
    logic line_hit; // 1 if tag was found in set
    logic [1:0] hit_way; // index of the way that was hit
    // miss logic
    logic [25:0] evict_tag; // tag of the exvict way
    logic [31:0] refill_addr; // address = {tag, set_index, 4'b0000}
    // basically saved data for a write miss
    logic missed_is_write;
    logic [1:0] missed_word_offset;
    logic [31:0] missed_write_data;
    // preadjusted out data if we wanna j like throw away the size shit we can j stick w this
    logic [31:0]  raw_read_word;

    //========== picking apart address =======================================

    assign word_offset = address[3:2];  // which block
    assign set_index   = address[5:4];  // which of the 4 sets
    assign tag      = address[31:6]; // top 26 bits
    assign refill_addr = { tag, set_index, 4'b0000 }; // addr to put into dmem for anything brycen was right we j had to do ts

    //========== determine if we hit/miss logic ==============================================


    always_comb begin
        line_hit = 1'b0;
        hit_way = 2'b00;
        valid = 1'b0;
        dirty = 1'b0;
        evict_tag = 26'd0;
        missed_is_write = 1'b0;
        missed_word_offset = 2'd0;
        missed_write_data = 32'd0;
        // 1) Check all WAYS in this set for a tag match + valid
        for (i = 0; i < WAYS; i++) begin
            if (cache[set_index][i].valid && (cache[set_index][i].tag == tag)) begin
                line_hit       <= 1'b1;
                hit_way   = i[1:0];
            end
        end
        // determine hit
        if ((read || write) && line_hit) begin
            hit = 1'b1; // IO hit signal
        end else begin
            hit = 1'b0;
        end

        // determine miss
        if ((read || write) && !line_hit) begin
            miss = 1'b1;
        end else begin
            miss = 1'b0;
        end
        
        // if miss determine LRU way
        if (miss) begin
            lru_way = 0;
            for (i = 0; i < WAYS; i++) begin
                if (lru[set_index][i] == (WAYS - 1)) begin
                    lru_way = i;
                    break;
                end
            end
            vic_valid = cache[set_index][lru_way].valid;
            vic_dirty = cache[set_index][lru_way].dirty;
            evict_tag = cache[set_index][lru_way].tag;
        end

        // if write miss we need to set data
        if (write && miss) begin
            missed_is_write    = 1'b1;
            missed_word_offset = word_offset;
            missed_write_data  = write_data;
        end
    end 


    //=========== tag updates and LRU on hit or updates ====================================
    //
    // On reset, clear all valid/dirty bits and initialize LRU counters
    //

    // ----------- hit ---------------------
    always_ff @(posedge CLK or posedge RST) begin
        if (RST) begin
            for (i = 0; i < NUM_SETS; i++) begin
                for (int j = 0; j < WAYS; j++) begin
                    cache[i][j].valid <= 1'b0;
                    cache[i][j].dirty <= 1'b0;
                    cache[i][j].tag   <= 26'd0;
                    cache[i][j].block <= 128'd0;
                    lru_cnt[i][j]     <= j[1:0];  // initialize so that way 0 is MRU=0, way 1=1, etc.
                end
            end
            raw_read_word <= 32'd0;
        end else begin
            if(hit)begin // on hit we read block
                raw_read_word <= cache[set_index][hit_way].block[word_offset * WORD_BITS +: WORD_BITS]; // extracting word thats unformatted by memfile stuff
                if(write)begin // if its a store hit we write into cache and set dirty to 0
                    cache[set_index][hit_way].block[word_offset * WORD_BITS +: WORD_BITS] <= write_data; // PUT IT IN EUGHH
                    cache[set_index][hit_way].dirty <= 1'b1;
                end
                // updates lru adding 1 to every lru in the set that wasnt hit and setting the lru of the way we hit to 0 ty bryce
                for (int k = 0; k < WAYS; k++) begin
                    if (lru_cnt[set_index][k] < lru_cnt[set_index][hit_way]) begin
                        lru_cnt[set_index][k] <= lru_cnt[set_index][k] + 1;
                    end
                end
                lru_cnt[set_index][hit_way] <= 2'd0;
            end
            // -------------- refill fsm case (new stuff not from bryce) ------------ 

            // this happens when we wanted to write smth into a block but 
            // the data in the current way was dirty and not yet put into 
            // emory so its like a queue

            else if(update)begin
                cache[set_index][lru_way].block <= { w0, w1, w2, w3 }; // update block (j a normal update for typical cases) 
                cache[set_index][lru_way].valid <= 1'b1;

                if (missed_is_write) begin  // if previos cc miss was store miss so replace the updated
                cache[set_index][lru_way].block[missed_word_offset * WORD_BITS +: WORD_BITS] <= missed_write_data;
                cache[set_index][lru_way].dirty <= 1'b1;
                end
                else begin
                    // load miss so this is unedited set dirty bit to 0
                    cache[set_index][lru_way].dirty <= 1'b0;
                end
                for (int m = 0; m < WAYS; m++) begin // update lru
                    if (lru_cnt[set_index][m] < lru_cnt[set_index][lru_way]) begin
                        lru_cnt[set_index][m] <= lru_cnt[set_index][m] + 1;
                    end
                end
            lru_cnt[set_index][lru_way] <= 2'd0;
            end
            // -------------- miss -----------------------
        
            else if(miss)begin // miss == 0 but update == 0
            // alr handled this at the top fsm will handle miss logic, next cc the fsm will either write back to ram or refill
            end
        end
    end




    //=========== dmem signal logic ============================================

    // mem_read, mem_write, mem_wr/rd_addr, ow0..ow3
    // whenever we update we want to fetch block = {tag, set_index, 4’b0000} from dmem

    assign mem_read = update;
    assign mem_rd_addr = refill_addr;

    // whenever writeback = 1 we want to write to line in dmem so it can write
    assign mem_write   = writeback;
    assign mem_wr_addr = {evict_tag, set_index, 4'b0000 };

    always_comb begin // if writeback then that means this block is dirty and has to go to main ram
        if (writeback) begin
            ow0 = cache[set_index][lru_way].block[31:0];
            ow1 = cache[set_index][lru_way].block[63:32];
            ow2 = cache[set_index][lru_way].block[95:64];
            ow3 = cache[set_index][lru_way].block[127:96];
        end else begin // else just set to 0 no changes made to dmem
            ow0 = 32'd0;
            ow1 = 32'd0;
            ow2 = 32'd0;
            ow3 = 32'd0;
        end
    end


    // ========== MEM IO, we can delete ts we stole this anyways =============================
    logic [1:0] byteOffset;     
    always_comb begin
      if(address >= 32'h00010000) begin  // external address range
        IO_WR = write;                 // IO Write
        out = IO_IN;            // IO read from buffer
        weAddrValid = 0;                 // address beyond memory range
      end
      else begin
        IO_WR = 0;                  // not MMIO
        out = read_data_sized;   // output sized and sign extended data
        weAddrValid = write;      // address in valid memory range
      end
    end

    // ========= STOLEN FROM MEM FILE =================================================================     

    always_comb begin
            case ({ sign, size, address[1:0] })
                // signed byte
                4'b0_00_00: read_data_sized = { {24{raw_read_word[ 7]}}, raw_read_word[ 7: 0] };
                4'b0_00_01: read_data_sized = { {24{raw_read_word[15]}}, raw_read_word[15: 8] };
                4'b0_00_10: read_data_sized = { {24{raw_read_word[23]}}, raw_read_word[23:16] };
                4'b0_00_11: read_data_sized = { {24{raw_read_word[31]}}, raw_read_word[31:24] };

                // signed halfword
                4'b0_01_00: read_data_sized = { {16{raw_read_word[15]}}, raw_read_word[15: 0] };
                4'b0_01_01: read_data_sized = { {16{raw_read_word[23]}}, raw_read_word[23: 8] };
                4'b0_01_10: read_data_sized = { {16{raw_read_word[31]}}, raw_read_word[31:16] };
                4'b0_01_11: read_data_sized = { {16{raw_read_word[15]}}, raw_read_word[15: 0] };

                // word (aligned or misaligned)
                4'b0_10_00: read_data_sized = raw_read_word;
                4'b0_10_01: read_data_sized = raw_read_word;
                4'b0_10_10: read_data_sized = raw_read_word;
                4'b0_10_11: read_data_sized = raw_read_word;

                // unsigned byte
                4'b1_00_00: read_data_sized = { 24'd0, raw_read_word[ 7: 0] };
                4'b1_00_01: read_data_sized = { 24'd0, raw_read_word[15: 8] };
                4'b1_00_10: read_data_sized = { 24'd0, raw_read_word[23:16] };
                4'b1_00_11: read_data_sized = { 24'd0, raw_read_word[31:24] };

                // unsigned halfword
                4'b1_01_00: read_data_sized = { 16'd0, raw_read_word[15: 0] };
                4'b1_01_01: read_data_sized = { 16'd0, raw_read_word[23: 8] };
                4'b1_01_10: read_data_sized = { 16'd0, raw_read_word[31:16] };
                4'b1_01_11: read_data_sized = { 16'd0, raw_read_word[15: 0] };

                default:   read_data_sized = 32'd0;
            endcase
        end
endmodule


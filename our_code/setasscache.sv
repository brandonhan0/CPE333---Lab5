`timescale 1ns / 1ps


// MISING "w0-w3 and ow0-ow3 logic". Missing writeback, "missing mem module size and sign stuff"


module setasscache (
    input  logic         CLK,
    input  logic         RST,
    input  logic [31:0]  address, // 
    input  logic         read,
    input  logic         write,
    input  logic [31:0]  write_data,
    input  logic [1:0]   size,
    input  logic         sign,     // sign extension for reads
    input  logic [31:0]  IO_IN,    // Memory Mapped IO Read
    input  logic         update,
    input  logic [31:0]  w0, w1, w2, w3,
    output logic [31:0]  ow0, ow1, ow2, ow3,
    output logic [31:0]  out,    // output when loading
    output logic         hit,
    output logic         miss,
    output logic         IO_WR, // Memory Mapped IO Write
    output logic         weAddrValid
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


    // On reset, clear all valid/dirty bits and initialize LRU counters
    always_ff @(posedge CLK or posedge RST) begin
        if(read)
            ioBuffer <= IO_IN;
        if (RST) begin
            for (i = 0; i < NUM_SETS; i++) begin
                for (j = 0; j < WAYS; j++) begin
                    cache[i][j].valid <= 1'b0;
                    cache[i][j].dirty <= 1'b0;
                    cache[i][j].tag   <= 26'd0;
                    cache[i][j].block <= 128'd0;
                    lru[i][j]         <= j[1:0];  // initialize so that way 0 is MRU=0, way 1=1, etc.
                end
            end
            hit <= 0;
            miss <= 0;
            read_data <= 32'd0;
            IO_WR <= 0;
            
        end else if (read || write) begin
            // Default: assume miss until proven otherwise
            hit         <= 1'b0;
            integer hit_way = -1;
            
            // 1) Check all WAYS in this set for a tag match + valid
            for (i = 0; i < WAYS; i++) begin
                if (cache[set_index][i].valid && (cache[set_index][i].tag == tag)) begin
                    hit       <= 1'b1;
                    hit_way   = i;
                    break;
                end
            end


            if (hit) begin
                // 2a) On a hit: extract the requested 32-bit word from the 128-bit block
                read_data <= cache[set_index][hit_way].block[word_offset * 32 +: 32];

                

                if (write) begin
                    // 2b) On store, overwrite just the 32 bit slice and set dirty
                    if (cache[set_index][lru_way].valid && cache[set_index][lru_way].dirty) begin
                        // *** WRITEBACK should occur here (outside this module) ***
                        ow0 <= cache[set_index][lru_way].block[31:0];
                        ow1 <= cache[set_index][lru_way].block[63:32];
                        ow2 <= cache[set_index][lru_way].block[95:64];
                        ow3 <= cache[set_index][lru_way].block[127:96];
                    end else begin
                        cache[set_index][hit_way].block[word_offset * 32 +: 32] <= write_data;

                        cache[set_index][hit_way].dirty                         <= 1'b1;
                    end
                end


                // 2c) Update LRU: all ways with LRU < this way get +1, this way becomes 0 (MRU)
                for (i = 0; i < WAYS; i++) begin
                    if (lru[set_index][i] < lru[set_index][hit_way])
                        lru[set_index][i] <= lru[set_index][i] + 1;
                end
                lru[set_index][hit_way] <= 2'd0;
            end
            else begin
                // 3) Miss: pick the way whose LRU counter == (WAYS-1) (i.e. least recently used)
                lru_way = 0;
                for (i = 0; i < WAYS; i++) begin
                    if (lru[set_index][i] == (WAYS - 1)) begin
                        lru_way = i;
                        break;
                    end
                end


                // 3a) If that line is valid & dirty, we'd need a writeback (handled externally)
                if (cache[set_index][lru_way].valid && cache[set_index][lru_way].dirty) begin
                    // *** WRITEBACK should occur here (outside this module) ***
                    ow0 <= cache[set_index][lru_way].block[31:0];
                    ow1 <= cache[set_index][lru_way].block[63:32];
                    ow2 <= cache[set_index][lru_way].block[95:64];
                    ow3 <= cache[set_index][lru_way].block[127:96];
                    
                end


                // 3b) Bring the new 128-bit block in from memory.
                //     Here we simulate it with a dummy constant; in real hardware you’d assert a memory-read.
                if(update) begin
                    cache[set_index][lru_way].block <= {w0, w1, w2, w3};
                    cache[set_index][lru_way].valid <= 1'b1;
                    cache[set_index][lru_way].tag   <= tag;
                end
                // If this was a store miss, we overwrite that 32-bit slice immediately:
                cache[set_index][lru_way].dirty <= (write ? 1'b1 : 1'b0);
                if (write) begin
                    cache[set_index][lru_way].block[word_offset * 32 +: 32] <= write_data;
                end


                // 3c) Return the requested word (either from the fetched block or the newly written slice)
                read_data <= cache[set_index][lru_way].block[word_offset * 32 +: 32];


                // 3d) Update LRU similarly: this way becomes MRU (0), increment all others that were < old LRU of lru_way
                for (i = 0; i < WAYS; i++) begin
                    if (lru[set_index][i] < lru[set_index][lru_way])
                        lru[set_index][i] <= lru[set_index][i] + 1;
                end
                lru[set_index][lru_way] <= 2'd0;
            end
        end
        else begin
            // If neither read nor write, do nothing to cache/memory
            hit <= 1'b0;
        end
    end


// ========= STOLEN FROM MEM FILE =================================================================


    logic [31:0] ioBuffer, memReadSized;
    logic [1:0] byteOffset;     
    assign byteOffset = address[1:0];     

    always_comb begin
      case({sign,size,byteOffset})
        5'b00011: memReadSized = {{24{read_data[31]}},read_data[31:24]};  // signed byte
        5'b00010: memReadSized = {{24{read_data[23]}},read_data[23:16]};
        5'b00001: memReadSized = {{24{read_data[15]}},read_data[15:8]};
        5'b00000: memReadSized = {{24{read_data[7]}},read_data[7:0]};
                                    
        5'b00110: memReadSized = {{16{read_data[31]}},read_data[31:16]};  // signed half
        5'b00101: memReadSized = {{16{read_data[23]}},read_data[23:8]};
        5'b00100: memReadSized = {{16{read_data[15]}},read_data[15:0]};
            
        5'b01000: memReadSized = read_data;                   // word
               
        5'b10011: memReadSized = {24'd0,read_data[31:24]};    // unsigned byte
        5'b10010: memReadSized = {24'd0,read_data[23:16]};
        5'b10001: memReadSized = {24'd0,read_data[15:8]};
        5'b10000: memReadSized = {24'd0,read_data[7:0]};
               
        5'b10110: memReadSized = {16'd0,read_data[31:16]};    // unsigned half
        5'b10101: memReadSized = {16'd0,read_data[23:8]};
        5'b10100: memReadSized = {16'd0,read_data[15:0]};
            
        default:  memReadSized = 32'b0;     // unsupported size, byte offset combination
      endcase
    end



    // Memory Mapped IO
    always_comb begin
      if(address >= 32'h00010000) begin  // external address range
        IO_WR = write;                 // IO Write
        out = ioBuffer;            // IO read from buffer
        weAddrValid = 0;                 // address beyond memory range
      end
      else begin
        IO_WR = 0;                  // not MMIO
        out = memReadSized;   // output sized and sign extended data
        weAddrValid = write;      // address in valid memory range
      end
    end
endmodule
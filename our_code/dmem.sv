`timescale 1ns / 1ps
                                                                                                                       
  module dmem (
    input MEM_CLK,
    input MEM_RDEN2,        // read enable data
    input MEM_WE2,          // write enable.
    input [31:0] mem_rd_addr, // Data Memory Addr
    input [31:0] mem_wr_addr,  // Data to save
    input logic [31:0] ow0,
    input logic [31:0] ow1,
    input logic [31:0] ow2,
    input logic [31:0] ow3,
    output logic [31:0] w0,
    output logic [31:0] w1,
    output logic [31:0] w2,
    output logic [31:0] w3
  );
    
    logic [13:0] wordAddr2;
    logic [31:0] memReadWord, ioBuffer, memReadSized;
    logic [1:0] byteOffset = mem_rd_addr[3:2];
       
    (* rom_style="{distributed | block}" *)
    (* ram_decomp = "power" *) logic [31:0] memory [0:16383];
    
    initial begin
        $readmemh("Test_All.mem", memory, 0, 16383);
    end
    
    // determines word address from the byte address making the block search logic obsolete
    logic [13:0] word_rd_index;
    logic [13:0] word_wr_index;
    always_comb begin
        word_rd_index = ( mem_rd_addr[15:2] ); 
        word_wr_index = ( mem_wr_addr[15:2] );
    end
            
    // BRAM requires all reads and writes to occur synchronously
    always_ff @(posedge MEM_CLK) begin
    
      // save data (WD) to memory (ADDR2)
      if (MEM_WE2 == 1) begin     // write enable and valid address space
        memory[word_wr_index + 0]  <= ow0;
        memory[word_wr_index + 1]  <= ow1;
        memory[word_wr_index + 2]  <= ow2;
        memory[word_wr_index + 3]  <= ow3;
      end
    
      if (MEM_RDEN2) begin     // dont need big ah case statement bc when we read we fetch the whole block basically so offset doesnt matter
                               // so we just make offset obsolete and use the address without offset
        w0 <= memory[word_rd_index + 0];
        w1 <= memory[word_rd_index + 1];
        w2 <= memory[word_rd_index + 2];
        w3 <= memory[word_rd_index + 3];
        
      end
    end
       

 endmodule

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
    
            
    // BRAM requires all reads and writes to occur synchronously
    always_ff @(posedge MEM_CLK) begin
    
      // save data (WD) to memory (ADDR2)
      if (MEM_WE2 == 1) begin     // write enable and valid address space
        memory[mem_wr_addr + 0]  <= ow0;
        memory[mem_wr_addr + 1]  <= ow1;
        memory[mem_wr_addr + 2]  <= ow2;
        memory[mem_wr_addr + 3]  <= ow3;
      end
    
      if (MEM_RDEN2) begin     // Read word from memory
      case(byteOffset)
      
      2'b00: w0 <= memory[mem_rd_addr + 0];
             w1 <= memory[mem_rd_addr + 1];
             w2 <= memory[mem_rd_addr + 2];
             w3 <= memory[mem_rd_addr + 3];

      2'b01: w0 <= memory[mem_rd_addr - 1];
             w1 <= memory[mem_rd_addr + 0];
             w2 <= memory[mem_rd_addr + 1];
             w3 <= memory[mem_rd_addr + 2];

      2'b10: w0 <= memory[mem_rd_addr - 2];
             w1 <= memory[mem_rd_addr - 1];
             w2 <= memory[mem_rd_addr + 0];
             w3 <= memory[mem_rd_addr + 1];

      2'b11: w0 <= memory[mem_rd_addr - 3];
             w1 <= memory[mem_rd_addr - 2];
             w2 <= memory[mem_rd_addr - 1];
             w3 <= memory[mem_rd_addr + 0];

      endcase
        
      end
    end
       

 endmodule

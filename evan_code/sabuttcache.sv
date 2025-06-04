`timescale 1ns / 1ps
module CacheAs(
    input clk,
    input rst,
    input [31:0] address,
    input [31:0] datain,
    input cache_write,
    input cache_read, 
    output logic [31:0] dataout, 
    input [1:0] MEM_SIZE, 
    input MEM_SIGN, 
    input update,
    output logic hit, 
    output logic miss, 
    output logic memory_read, 
    output logic memory_write, 
    output logic [31:0] mem_rd_addr,
    output logic [31:0] mem_wr_addr,
    input [31:0] w0, 
    input [31:0] w1, 
    input [31:0] w2, 
    input [31:0] w3, 
    output logic [31:0] ow0, 
    output logic [31:0] ow1, 
    output logic [31:0] ow2, 
    output logic [31:0] ow3,
    input [31:0] IO_IN,
    output  logic IO_WR 
);
// Cache Parameters
    parameter BLOCK_SIZE = 4;   // 4 words per block
    parameter NUM_BLOCKS = 16;  // 16 blocks in total
    parameter NUM_WAYS = 4;     // 4-way set associative
    parameter NUM_SETS =(NUM_BLOCKS / NUM_WAYS); // 4 sets

    // Address Breakdown
    logic [1:0] index ; // 2-bit index for selecting set
    logic [1:0] word_offset ; // 2-bit word offset inside block
    logic [1:0] byteoffset; // 2-bit byte offset
    logic [25:0] tag ;  // Remaining bits as tag
    // Cache Storage
    logic [25:0] tags [NUM_SETS-1:0][NUM_WAYS-1:0];  // Tags
    logic [25:0] tag_d1 ;
    logic [31:0] data [NUM_SETS-1:0][NUM_WAYS-1:0][BLOCK_SIZE-1:0]; // Cache Data (4 words per block)
    logic valid [NUM_SETS-1:0][NUM_WAYS-1:0];  // Valid bits
    logic dirty [NUM_SETS-1:0][NUM_WAYS-1:0];  // Dirty bits
    logic [3:0] lru_counter [NUM_SETS-1:0][NUM_WAYS-1:0]; // LRU Counters
    logic [31:0] read_data;
    logic [1:0] curr_way,curr_way_d1;
    logic [3:0] hit_way,empty_way ; // Stores way index for each set
    logic find_hit_way,find_empty_way,find_lru_way;

    logic write,read;

/////////////////////////////////////
// Start coding
/////////////////////////////////////
    assign index[1:0] = address[5:4];  // 2-bit index for selecting set
    assign word_offset[1:0] = address[3:2]; // 2-bit word offset inside block
    assign byteoffset[1:0] = address[1:0]; // 2-bit byte offset
    assign  tag[25:0] = address[31:6];  // Remaining bits as tag


/////////////////////////////////////
// Get hit, valid , full, evict
/////////////////////////////////////

    logic [NUM_SETS-1:0] cache_hit_vec,is_set_full, evict_cache_vec;
    logic [NUM_WAYS-1:0] tag_match [NUM_SETS-1:0];
    logic dirtybit,evict_cache,cache_hit;

    always_comb begin
        cache_hit_vec = 4'h0;
        is_set_full = 4'h0;
        evict_cache_vec = 4'h0;
        //  tag_match default zero
        for (int s = 0; s < NUM_SETS; s = s + 1) begin
            for (int w = 0; w < NUM_WAYS; w = w + 1) begin
                tag_match[s][w] = 1'b0; 
            end
        end
        
        //  Check for valid & tag  in all sets and ways
        for (int s = 0; s < NUM_SETS; s = s + 1) begin
            for (int w = 0; w < NUM_WAYS; w = w + 1) begin
                if (valid[s][w] && (tags[s][w] == tag)) begin
                    tag_match[s][w] = 1'b1;
                end
            end
        end 
       // check if each set of blocks are all full (the ways in same index)
           is_set_full[index] =  valid[index][0] & valid[index][1] & valid[index][2] & valid[index][3];
       // cache hit for each set ---- only right index with match  tag,and valid
           cache_hit_vec[index] =  (tag_match[index][0] | tag_match[index][1] | tag_match[index][2] | tag_match[index][3]) ; 

    // evict when no hit and full at the index set -- 4 ways  only check the blocks in same index set. 
           evict_cache_vec[index] = (~cache_hit_vec[index]) & is_set_full[index];

    end // end always

    assign  cache_hit = cache_hit_vec[index];
    assign  hit = cache_hit;
    assign  miss = ~cache_hit & (write | read); // create a pulse sent to CacheFsm
    assign evict_cache = (read | write ) & evict_cache_vec[index];
    assign  dirtybit =  dirty[index][curr_way]; // current dirty 

/////////////////////////////////////
// Find way 
/////////////////////////////////////
    
////// Find lru_way -oldest lru counter
    // lru_way when full & evict cache
    logic [1:0 ]  oldest,oldestL,oldestH, lru_way;
    always_comb 
    begin
      oldest=2'h0;
      oldestL=2'h0;
      oldestH=2'h0;
      lru_way=2'h0;
           if (evict_cache) begin
                    if (lru_counter[index][1] > lru_counter[index][0])
                        oldestL = 1; 
                    else 
                        oldestL = 0; 
                    if (lru_counter[index][3] > lru_counter[index][2])
                        oldestH = 3; 
                    else 
                        oldestH = 2; 
                    if (lru_counter[index][oldestH] > lru_counter[index][oldestL])
                        oldest = oldestH; 
                    else 
                        oldest = oldestL; 
            end // end if 
        lru_way= oldest;
     end// always


////// find  which ways match hit (4ways). only one way could match for each index
    // find way for hit 

   always_comb begin
    find_hit_way = 1'b0; // Default: No hit found
    hit_way = 4'h0; // Default: Way 0
        for (int w = 0; w < NUM_WAYS; w = w + 1) begin
            if ( tag_match[index][w] && (find_hit_way == 1'b0)) begin
                hit_way = {index[1:0],w[1:0]}; // Store the first detected hit way
                find_hit_way = 1'b1; // Mark that a hit was found
            end //if
        end // for w
    end // always_comb for find hit_way 


/////// find empty way (4 ways to find)  to store the new data
/////  only find the empty way match the index set
   always_comb begin
    find_empty_way = 1'b0; // Default: No hit found
    empty_way = 4'h0; // Default: Way 0
        for (int w = 0; w < NUM_WAYS; w = w + 1) begin
            if ( (valid[index][w]==1'b0) && (find_empty_way == 1'b0)) begin
                empty_way = {index[1:0],w[1:0]}; // Store the first detected clean way
                find_empty_way = 1'b1; // Mark that a hit was found
            end //if
        end // for w
    end // always_comb for find empty_way


// only use lru_way when cache evict
assign curr_way = (write | read) & cache_hit ? hit_way : 
                  (write | read) &  evict_cache ? lru_way :  empty_way;


/////////////////////////////////////
// LRU counter
/////////////////////////////////////
//////// Calculate LRU Counter
//  only increase the lru index counter when read,write 
//  set curent index  lru_way counter to zero if evict

   always_ff @(posedge clk)  begin
    if (rst) begin
        integer i, j;  // Declare loop variables inside always_ff
        for (i = 0; i < NUM_SETS; i = i + 1) begin
            for (j = 0; j < NUM_WAYS; j = j + 1) begin
                lru_counter[i][j] <= 4'h0;
            end // j
        end //i
    end // if 
    else if ( (read || write)) begin
            for (int way = 0; way < NUM_WAYS; way= way + 1) begin
             if ( evict_cache_vec[index] && (way==curr_way)) begin
                    // Update LRU (Most recently used gets highest counter)
                    lru_counter[index][way] <= 4'h0; // Set to max priority
              end // end if 
              else if ((lru_counter[index][way] <15)) begin
                    lru_counter[index][way] <= lru_counter[index][way] + 1;
              end // end else if
            end //  for way
     end //else  if read write  
  end // always_ff



///////////////////////////////////////////////////////
///   Memory Read Write 
///////////////////////////////////////////////////////

logic [31:0] wr_data,wr_data_d1;
logic [1:0] index_d1,word_offset_d1;

// Memory Interface  evict_cache is a pulse 
// memory write when miss & (write | read ) & full & dirtybit =1 ---- use lru way 
// memory read when miss. not full : empty_way, full: lru_way
assign memory_write = (write | read ) &  evict_cache & dirtybit & (~cache_hit) ;
assign memory_read = ~cache_hit & (read | write ); // same as miss

// write to memory only when evict and dirtybit =1
assign mem_wr_addr = {tags[index][curr_way], index, 4'b0000} ;

// always use current tag index for memory read
assign mem_rd_addr = {tag,index,4'h0};

assign {ow0, ow1, ow2, ow3} = {data[index][curr_way][+0], data[index][curr_way][1], data[index][curr_way][2], data[index][curr_way][3]};

logic [1:0] saved_mem_size, saved_byteoffset;
logic [31:0] saved_address;
logic saved_mem_sign,read_d1,write_d1;
    
    always_ff @(posedge clk) begin
      if (rst)
      begin
        read_d1 <= 1'b0;
        write_d1 <= 1'b0;
        curr_way_d1 <= 2'h0;
        index_d1 <= 2'h0;
        word_offset_d1 <= 2'h0;
        wr_data_d1 <= 32'h0;
        tag_d1 <=25'h0;
        saved_mem_size <= 2'h0;
        saved_mem_sign <= 1'b0;
        saved_byteoffset <=2'h0;
        saved_address <=32'h0;

      end
      else begin
// create one clock delay to wait for memory read 
        read_d1 <= read;
        write_d1 <= write;
        curr_way_d1 <= curr_way;
        index_d1 <= index;
        word_offset_d1 <= word_offset;
        wr_data_d1 <= wr_data;
        tag_d1 <=tag;
        saved_mem_size <= MEM_SIZE;
        saved_mem_sign <= MEM_SIGN;
        saved_byteoffset <=byteoffset;
        saved_address <=address;
     end
   end // clk 

/////////////////////////////////////////
// Cache write and read
/////////////////////////////////////////
//  miss need to read back from memory. write to  cache one clk later after read
// update cache

  always_ff @(posedge clk) begin
    if (rst) begin
        integer i, j;  // Declare loop variables inside always_ff
        for (i = 0; i < NUM_SETS; i = i + 1) begin
            for (j = 0; j < NUM_WAYS; j = j + 1) begin
                valid[i][j] <= 1'b0;
                dirty[i][j] <= 1'b0;
                tags[i][j] <= 26'h0;
            end // j
        end //i
    end // if 
  end // always

  always_ff @(posedge clk) begin
    // evict to memory and need to clear the dirty bit
          if (memory_write ) begin   
               dirty[index][curr_way] <= 1'b0 ;
          end
     ////// update valid and mark dirty after hit write
         if (write  & cache_hit) begin
               valid[index][curr_way] <= 1;
               dirty[index][curr_way] <= 1; 
          end // end if write

   //// memory read output is one clock later.
   ///// write to cache one clock later
   /// include write and read both
          if (update ) begin
               tags[index_d1][curr_way_d1] <= tag_d1;
               valid[index_d1][curr_way_d1] <= 1;
               dirty[index_d1][curr_way_d1] <= write_d1; // depend on write or read 
           end //end if update 
    end // end always

   ///////// write data to cache
  // write to cache if hit
    always_ff @(posedge clk) begin
         if (write & cache_hit) begin
               case (word_offset[1:0] )
                 2'h0 : data[index][curr_way][0] <= wr_data; // Load new block
                 2'h1 : data[index][curr_way][1] <= wr_data;
                 2'h2 : data[index][curr_way][2] <= wr_data;
                 2'h3 : data[index][curr_way][3] <= wr_data;
               endcase
          end // end if write
          // update cache if write miss. one clock later after  memory read, Read whole block before write
          if (update  & write_d1  ) begin
               case (word_offset_d1[1:0] )
                 2'h0 : {data[index_d1][curr_way_d1][0], data[index_d1][curr_way_d1][1], data[index_d1][curr_way_d1][2], data[index_d1][curr_way_d1][3]} <= {wr_data_d1, w1, w2, w3}; // Load new block
                 2'h1 : {data[index_d1][curr_way_d1][0], data[index_d1][curr_way_d1][1], data[index_d1][curr_way_d1][2], data[index_d1][curr_way_d1][3]} <= {w0, wr_data_d1, w2, w3}; // Load new block
                 2'h2 : {data[index_d1][curr_way_d1][0], data[index_d1][curr_way_d1][1], data[index_d1][curr_way_d1][2], data[index_d1][curr_way_d1][3]} <= {w0, w1, wr_data_d1, w3}; // Load new block
                 2'h3 : {data[index_d1][curr_way_d1][0], data[index_d1][curr_way_d1][1], data[index_d1][curr_way_d1][2], data[index_d1][curr_way_d1][3]} <= {w0, w1, w2, wr_data_d1}; // Load new block
                 default: {data[index_d1][curr_way_d1][0], data[index_d1][curr_way_d1][1], data[index_d1][curr_way_d1][2], data[index_d1][curr_way_d1][3]} <= {w0, w1, w2, w3}; // Load new block
               endcase 
             end // end if 

           // update cache after memory read  because of read miss
           // one clock after read memory
          if (update  & read_d1  ) begin
                  {data[index_d1][curr_way_d1][0], data[index_d1][curr_way_d1][1], data[index_d1][curr_way_d1][2], data[index_d1][curr_way_d1][3]} <= {w0, w1, w2, w3}; // Load new block
                
          end // end read_d1
    end //always_ff

// Memory read/ write
 always_ff @(posedge clk) begin
    if (rst) begin
       read_data <= 32'h0;
    end
    else if (cache_hit && read) begin
               read_data <= data[index][curr_way][word_offset]; // Fetch 32-bit word
         end
    else if (update  && read_d1) begin
               case (word_offset_d1[1:0] )
                 2'h0 : read_data <= w0; 
                 2'h1 : read_data <= w1;
                 2'h2 : read_data <= w2;
                 2'h3 : read_data <= w3;
               endcase
         end

  end // end always

////////////////////////////   
// Byte Offset 
///////////////////////////////////
 always_comb  begin
      wr_data=datain;
      if (write == 1) begin     // write enable and valid address space
      // 0-Byte, 1-Half, 2-Word 
        case({MEM_SIZE,byteoffset})
          4'b0000: wr_data[7:0] =  datain[7:0]; 
          4'b0001: wr_data[15:8] = datain[7:0];
          4'b0010: wr_data[23:16] = datain[7:0];
          4'b0011: wr_data[31:24] = datain[7:0];
          4'b0100: wr_data[15:0] =  datain[15:0]; 
          4'b0101: wr_data[23:8] =  datain[15:0]; 
          4'b0110: wr_data[31:16] =  datain[15:0]; 
          4'b1000: wr_data[31:0] =  datain[31:0]; 
          default : wr_data= datain;
      endcase
    end // if
 end // always

    logic [31:0] rd_data;
      // SIZE :0-Byte, 1-Half, 2-Word 
      // SIGN : 1 unsigned  0 signed
    always_comb begin
     rd_data =32'h0;
      case({saved_mem_sign,saved_mem_size,saved_byteoffset})
        5'b00011: rd_data = {{24{read_data[31]}},read_data[31:24]};  // signed byte
        5'b00010: rd_data = {{24{read_data[23]}},read_data[23:16]};
        5'b00001: rd_data = {{24{read_data[15]}},read_data[15:8]};
        5'b00000: rd_data = {{24{read_data[7]}},read_data[7:0]};
                                    
        5'b00110: rd_data = {{16{read_data[31]}},read_data[31:16]};  // signed half
        5'b00101: rd_data = {{16{read_data[23]}},read_data[23:8]};
        5'b00100: rd_data = {{16{read_data[15]}},read_data[15:0]};
            
        5'b01000: rd_data = read_data;                   // word
               
        5'b10011: rd_data = {24'd0,read_data[31:24]};    // unsigned byte
        5'b10010: rd_data = {24'd0,read_data[23:16]};
        5'b10001: rd_data = {24'd0,read_data[15:8]};
        5'b10000: rd_data = {24'd0,read_data[7:0]};
               
        5'b10110: rd_data = {16'd0,read_data[31:16]};    // unsigned half
        5'b10101: rd_data = {16'd0,read_data[23:8]};
        5'b10100: rd_data = {16'd0,read_data[15:0]};
            
        default:  rd_data = 32'b0;     // unsupported size, byte offset combination
      endcase
    end // alway_comb


////////////////////////////// 
/// IO BUFFER
////////////////////////////// 
assign read = cache_read;
logic [31:0] ioIn_buffer;
   always_ff @(posedge clk)
   begin
        if (rst) 
            ioIn_buffer <= 32'h0;
         else if (cache_read)
            ioIn_buffer<=IO_IN;       
   end
   
    always_comb
    begin
        IO_WR=0;
        if(saved_address >= 32'h11000000)
        begin
            write=0;
            dataout = ioIn_buffer;
            if(cache_write) IO_WR = 1;
        end
        else begin
            write=cache_write;
            dataout = rd_data;
        end   
    end

 endmodule

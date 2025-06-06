module Hazards_Pipeline (input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);    
       
typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        



typedef struct packed{ // if it has a "//" it means its assigned and used but we just need to be consistent and use it properly
    opcode_t opcode; // 
    logic [31:0] ir; //
    logic [4:0] rs1_addr; //
    logic [4:0] rs2_addr; //
    logic [4:0] rd_addr; //
    logic rs1_used; //
    logic rs2_used; //
    logic [3:0] alu_fun; //
    logic memWrite; //
    logic memRead2; //
    logic regWrite; //
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc; //
    logic [31:0] rs1; //
    logic [31:0] rs2; //
    logic [31:0] J_type; //
    logic [31:0] B_type; //
    logic [31:0] I_type; //
    logic [31:0] ALU_A; //
    logic [31:0] ALU_B; //
    logic [31:0] ALU_Result; //
} instr_t;


    wire reset, RF_WE, memRDEN2, memWE2, br_eq, br_lt, br_ltu;
    wire [1:0] srcA_SEL, PC_SEL;
    wire [2:0] srcB_SEL;
    wire [31:0] rd,mem_instruction,PC_DATA, ir, ALU_srcB, ALU_srcA, rs1, rs2, reg_data, dout2, result, WB_mux_out; // added wire for mem output
    wire [31:0] b_type, j_type, i_type, u_type, s_type, jal, branch, 
                jalr; // wires from imm_gen and bag to outputs
    wire [1:0] fsel1, fsel2;
    wire stall, flush, dec_mux_out;
    wire [31:0] PC;
    reg flush_2, memRDEN1, PC_WE;
    logic [31:0] pc_delay;

    wire [31:0] w0, w1, w2, w3, w4, w5, w6, w7;
    wire cache_stall, hit, miss, update;

    
//==== Instruction Fetch ===========================================
     logic [31:0] if_de_pc;

     instr_t de_inst;
    
     always_comb begin
     if (!(stall || cache_stall)) begin
        PC_WE = 1'b1;
        memRDEN1 = 1'b1;
     end else begin
        PC_WE = 1'b0;
        memRDEN1 = 1'b0;
     end
     end

    PC_MUX  #(.n(32)) PC_MUX  (
      .SEL   (PC_SEL), // this is determined by a future clock cycle so..
      .D0    (PC + 4), 
      .D1    (jalr), 
      .D2    (branch), 
      .D3    (jal),
      .D_OUT (PC_DATA));

   PC ProgramCounter (
      .data_in  (PC_DATA), 
      .ld       (PC_WE), 
      .clk      (CLK), 
      .clr      (RESET), 
      .data_out (PC));

     always_ff @(posedge CLK) begin //pipeline register
          if (!stall) begin
            pc_delay <= PC_DATA;
          end      
     end

    
//==== Instruction Decode ===========================================

    logic [31:0] de_ex_opA;
    logic [31:0] de_ex_opB;
    logic [31:0] de_ex_rs2;

                             
 //Memory OTTER_MEMORY (
 //     .MEM_CLK   (CLK),
 //     .MEM_RDEN1 (memRDEN1), 
 //     .MEM_RDEN2 (ex_mem_inst.memRead2), 
 //     .MEM_WE2   (ex_mem_inst.memWrite),
 //     .MEM_DIN2  (ex_mem_inst.rs2),
 //     .MEM_ADDR1 (PC[15:2]),
 //     .MEM_ADDR2 (ex_mem_inst.ALU_Result), 
 //     .MEM_SIZE  (ex_mem_inst.mem_type[1:0]),
 //     .MEM_SIGN  (ex_mem_inst.mem_type[2]),
 //     .IO_IN     (IOBUS_IN),
 //     .IO_WR     (IOBUS_WR),
 //     .MEM_DOUT1 (mem_instruction), // we dont get insturction memory from this anymore
 //     .MEM_DOUT2 (dout2)); //brandon smells funny ever since we stopped using data memory from this module
 
    instr_t de_ex_inst;
    
    always_comb begin
    if (hit)
        de_inst.ir = rd; //rd from cache
    else
        de_inst.ir = 32'h00000013; //NOP if miss
    end
    
    assign de_inst.pc = pc_delay;
    
    assign de_inst.opcode = opcode_t'(de_inst.ir[6:0]);

    assign de_inst.rs1_addr = de_inst.ir[19:15];

    assign de_inst.rs2_addr = de_inst.ir[24:20];

    assign de_inst.rd_addr = de_inst.ir[11:7];

    assign de_inst.mem_type = de_inst.ir[14:12];
   
    assign de_inst.rs1_used=    de_inst.rs1_addr != 0
                                && !(de_inst.opcode == LUI
                                || de_inst.opcode == AUIPC
                                || de_inst.opcode == JAL);

    assign de_inst.rs2_used=    de_inst.rs2_addr != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL
                                && de_inst.opcode != JALR
                                && de_inst.opcode != LOAD
                                && de_inst.opcode != OP_IMM;

    CU_DCDR DECODER  (
      .opcode    (de_inst.ir[6:0]), 
      .func3     (de_inst.ir[14:12]), 
      .func7     (de_inst.ir[30]), 
      .int_taken (1'b0),
      .br_eq     (br_eq),
      .br_lt     (br_lt),
      .br_ltu    (br_ltu),
      .ALU_FUN   (de_inst.alu_fun), 
      .srcA_SEL  (srcA_SEL),
      .srcB_SEL  (srcB_SEL),
      .PC_SEL    (),
      .RF_SEL    (de_inst.rf_wr_sel),
      .RF_WE     (de_inst.regWrite),
      .MEM_WE2   (de_inst.memWrite),
      .MEM_RDEN2 (de_inst.memRead2));
    
    mux_4t1_nb  #(.n(32)) REG_MUX  (
      .SEL   (mem_wb_inst.rf_wr_sel), 
      .D0    (mem_wb_inst.pc + 4), 
      .D2    (dout2), 
      .D3    (mem_wb_inst.ALU_Result),
      .D_OUT (reg_data));
      
      
    REG_FILE REG_FILE (
      .w_data (reg_data),
      .en     (mem_wb_inst.regWrite),
      .clk    (CLK), 
      .adr1   (de_inst.rs1_addr), 
      .adr2   (de_inst.rs2_addr), 
      .w_adr  (mem_wb_inst.rd_addr), 
      .rs1    (de_inst.rs1), 
      .rs2    (de_inst.rs2));
      
    // imm value generation
    assign de_inst.I_type = {{21{de_inst.ir[31]}}, de_inst.ir[30:25], de_inst.ir[24:20]}; 
    // create i-type imm from specified bits of OTTER output(ir)
    assign s_type = {{21{de_inst.ir[31]}}, de_inst.ir[30:25], de_inst.ir[11:7]};
    // create s-type imm from specified bits of OTTER output(ir)
    assign de_inst.B_type = {{20{de_inst.ir[31]}}, de_inst.ir[7], de_inst.ir[30:25], de_inst.ir[11:8], 1'b0};
    // create b-type imm from specified bits of OTTER output(ir)
    assign u_type = {de_inst.ir[31:12], 12'b0};
    // create u-type imm from specified bits of OTTER output(ir)
    assign de_inst.J_type = {{12{de_inst.ir[31]}}, de_inst.ir[19:12], de_inst.ir[20], de_inst.ir[30:21], 1'b0};
    // create j-type imm from specified bits of OTTER output(ir)


    mux_4t1_nb  #(.n(32)) srcA_MUX  (
      .SEL   (srcA_SEL), 
      .D0    (de_inst.rs1), 
      .D1    (u_type), 
      .D2    (~de_inst.rs1),
      .D_OUT (de_inst.ALU_A));


    mux_4t1_nb  #(.n(32)) srcB_MUX  (
      .SEL   (srcB_SEL), 
      .D0    (de_inst.rs2), 
      .D1    (de_inst.I_type), 
      .D2    (s_type), 
      .D3    (de_inst.pc),
      .D_OUT (de_inst.ALU_B));
    
   always_ff@(posedge CLK) begin //transfers stuff from pre register to post register
       if (!flush && !stall && !flush_2 && !cache_stall && !dm_stall) begin
          de_ex_inst  <= de_inst;
       end else begin
          de_ex_inst  <= 0;         
       end
       flush_2 <= flush;
    end
    
   //==== Execute/ALU ==================================================
    logic [31:0] branch1_fwd, branch2_fwd;
    logic [1:0] storeforward;

    HazardUnit HazardUnit (
        .ex_opcode(de_ex_inst.opcode),
        .de_adr1(de_inst.rs1_addr),
        .de_adr2(de_inst.rs2_addr),
        .ex_adr1(de_ex_inst.rs1_addr),
        .ex_adr2(de_ex_inst.rs2_addr),
        .ex_rd(de_ex_inst.rd_addr),
        .mem_rd(ex_mem_inst.rd_addr),
        .wb_rd(mem_wb_inst.rd_addr),
        .pc_source(PC_SEL),
        .mem_regWrite(ex_mem_inst.rf_wr_sel),
        .wb_regWrite(mem_wb_inst.rf_wr_sel),
        .de_rs1_used(de_inst.rs1_used),
        .de_rs2_used(de_inst.rs2_used),
        .ex_rs1_used(de_ex_inst.rs1_used),
        .ex_rs2_used(de_ex_inst.rs2_used),
        .fsel1(fsel1),
        .fsel2(fsel2),
        .load_use_haz(stall),
        .flush(flush),
        .storeforward(storeforward)
    );

     instr_t ex_mem_inst;
     
     
    // branch forwarding logic
    always_comb begin
        case(fsel1)
            0: branch1_fwd = de_ex_inst.rs1;
            1: branch1_fwd = ex_mem_inst.ALU_Result;
            2: branch1_fwd = mem_wb_inst.ALU_Result;
            3: branch1_fwd = dout2;
            default: branch1_fwd = de_ex_inst.rs1;
        endcase
    end
    
    always_comb begin
        case(fsel2)
            0: branch2_fwd = de_ex_inst.rs2;
            1: branch2_fwd = ex_mem_inst.ALU_Result;
            2: branch2_fwd = mem_wb_inst.ALU_Result;
            3: branch2_fwd = dout2;
            default: branch2_fwd = de_ex_inst.rs2;
        endcase
    end
        
     
    // branch address generation
    assign jal    = de_ex_inst.pc + (de_ex_inst.J_type); 
    // generate jal addr from j type imm
    assign jalr   = branch1_fwd + ((de_ex_inst.I_type)); 
    // generate jalr addr from i-type imm
    assign branch = de_ex_inst.pc + (de_ex_inst.B_type); 
    // generate branch addr from b-type imm
    
     // Modified BCG
     Branch_Cond_Gen BCG (
      .rs1     (branch1_fwd),
      .rs2     (branch2_fwd),
      .opcode  (de_ex_inst.opcode),
      .func3   (de_ex_inst.ir[14:12]),
      .br_eq   (br_eq),
      .br_lt   (br_lt),
      .br_ltu  (br_ltu),
      .PC_SEL  (PC_SEL));

//MUX1
    mux_4t1_nb  #(.n(32)) ALU_A_MUX  (
      .SEL   (fsel1), 
      .D0    (de_ex_inst.ALU_A), 
      .D1    (ex_mem_inst.ALU_Result),
      .D2    (mem_wb_inst.ALU_Result), 
      .D3    (dout2),                               // leave blank
      .D_OUT (ALU_srcA));
//MUX2
    mux_4t1_nb  #(.n(32)) ALU_B_MUX  (
      .SEL   (fsel2), 
      .D0    (de_ex_inst.ALU_B), 
      .D1    (ex_mem_inst.ALU_Result),
      .D2    (mem_wb_inst.ALU_Result), 
      .D3    (dout2),                              // leave blank
      .D_OUT (ALU_srcB));

      // This the ALU
    ALU ALU (
     .alu_fun(de_ex_inst.alu_fun),
     .op1(ALU_srcA),
     .op2(ALU_srcB),
     .result(de_ex_inst.ALU_Result)
    );
    
 always_ff@(posedge CLK) begin //transfers stuff from pre register to post register
        ex_mem_inst <= de_ex_inst;
        if (storeforward == 2'b01) begin
            ex_mem_inst.rs2 <= ex_mem_inst.ALU_Result;
        end
        if (storeforward == 2'b10) begin
            ex_mem_inst.rs2 <= mem_wb_inst.ALU_Result;
        end               
    end


//==== Memory ======================================================
     
    instr_t mem_wb_inst;
    
    always_ff@(posedge CLK) begin //transfers stuff from pre register to post register
        mem_wb_inst <= ex_mem_inst;
    end
    
    
//==== Write Back ==================================================
        


// o_O



//====== Instruction Memory Cache ====================================================



imem InstructionMemory (
     .a(PC),         // in (address wire)
     .w0(w0),      // out
     .w1(w1),
     .w2(w2),
     .w3(w3),
     .w4(w4),
     .w5(w5),
     .w6(w6),
     .w7(w7)
    );

CacheFSM Cache_FSM (
     .hit(hit),        // in
     .miss(miss),
     .CLK(CLK),
     .RST(RESET),
     .update(update),  // out
     .pc_stall(cache_stall)      // (stall)
    );

Cache Cache (
     .PC(PC),         // in
     .CLK(CLK),        
     .update(update),
     .w0(w0),        
     .w1(w1),
     .w2(w2),
     .w3(w3),
     .w4(w4),
     .w5(w5),
     .w6(w6),
     .w7(w7),
     .rd(rd),         // out (output of mem location)
     .hit(hit),
     .miss(miss)
    );

//====== Data Memory Cache ====================================================

logic [31:0] ow0, ow1, ow2, ow3, dw0, dw1, dw2, dw3, mem_wr_addr, mem_rd_addr;
logic mem_read, mem_write, dm_hit, dm_update, dm_stall, dm_writeback, dm_victim_valid, dm_victim_dirty, cache_stall;

setasscache setasscache(
    .CLK(CLK), //top
    .RST(RST), //top
    .address(rd), // main mem
    .read(ex_mem_inst.memRead2), 
    .write(ex_mem_inst.memWrite), 
    .write_data(ex_mem_inst.rs2), //check me on this
    .size(ex_mem_inst.mem_type[1:0]),  //mem input
    .sign(ex_mem_inst.mem_type[2]),  //mem input
    .update(dm_update), // fsm input
    .w0(dw0),  // block 1
    .w1(dw1),  // block 2
    .w2(dw2),  // block 3
    .w3(dw3),  // block 4
    .IO_IN(IOBUS_IN), //top
    .out(dout2),  
    .hit(dm_hit),  // hazard and fsm
    .miss(dm_miss),  // hazard and fsm
    .ow0(ow0), //read cache
    .ow1(ow1), 
    .ow2(ow2), 
    .ow3(ow3),
    .IO_WR(IOBUS_WR), //top
    .writeback(dm_writeback),
    .weAddrValid(dm_valid),
    .mem_read(mem_read),
    .mem_rd_addr(mem_rd_addr),
    .mem_write(mem_write),
    .mem_wr_addr(mem_wr_addr),
    .vic_dirty(dm_victim_dirty),
    .vic_valid(dm_victim_valid)
);


DM_FSM Cache_FSM ( // added dirty, vali, and writeback
     .hit(dm_hit),        // in
     .miss(dm_miss),
     .CLK(CLK),
     .RST(RESET),
     .valid(dm_victim_valid), // from cache
     .dirty(dm_victim_dirty), // from cache
     .update(dm_update),  // out
     .pc_stall(dm_stall),      // (stall)
     .writeback(dm_writeback) // to cache
    );

dmem DataMemory ( // changed implementation for writing and reading 
    .MEM_CLK (CLK),
    .MEM_RDEN2 (mem_read),        // read enable data
    .MEM_WE2 (mem_write),          // write enable.
    .mem_rd_addr (mem_rd_addr), // Data Memory Addr
    .mem_wr_addr (mem_wr_addr),  // Data to save
    .ow0 (ow0),
    .ow1 (ow1),
    .ow2 (ow2),
    .ow3 (ow3),
    .w0 (dw0),
    .w1 (dw1),
    .w2 (dw2),
    .w3 (dw3)
    );

endmodule

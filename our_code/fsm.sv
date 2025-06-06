`timescale 1ns / 1ps

module CacheFSM (
    input  hit,
    input  miss,
    input  CLK,
    input  RST,
    input dirty,
    input valid,
    output logic update,
    output logic pc_stall,
    output logic writeback
);

    typedef enum logic [0:0] {
        ST_READ_CACHE,
        ST_READ_MEM
    } state_type;

    state_type PS, NS;
    logic   writeback_pending; // basically a flag to idicate that we need to wb bc we had to store a dirty block before refilling the block

    always_ff @(posedge CLK) begin
        if (RST == 1)
            PS <= ST_READ_MEM;
            writeback_pending <= 1'b0;
        else
            PS <= NS;
    end

    always_comb begin
        update = 1'b1;
        pc_stall = 1'b0;
        writeback = 1'b0;
        case (PS)
// ========= READ CACHE STATE ==================
            ST_READ_CACHE: begin
                update       = 1'b0;
                do_writeback = 1'b0;
                pc_stall     = 1'b0;
                writeback_pending = 1'b0; 
                NS = ST_READ_CACHE;
                //if (hit) begin      DONT NEED THIS AS WE HANDLE HITS IN NEXT STATE
                //    NS = ST_READ_CACHE;
                //    end
                else if (miss) begin
                    pc_stall = 1'b1;
                    NS = ST_READ_MEM; // stall on a miss
                end
            end
// ========= READ MEM STATE ====================
            ST_READ_MEM: begin
                pc_stall = 1'b1;
                // this if stateement is to handle when we have a dirty block that needs to be written back to mem before we can refill
                if (writeback_pending == 1'b0 && valid && dirty) begin
                    writeback = 1'b1; // singnal to cache to MEM_WE2 = 1 so dmem can write back the dirty block to mem
                    update = 1'b0;    // we dont want to update the cache until we have written back the dirty block
                    writeback_pending = 1'b1;   // next cycle we’ll do the refill
                    NS = ST_READ_CACHE;
                end
                else begin // if theres a miss and we dont have to write back the dirty block we fill
                    writeback = 0;
                    update = 1;
                    writeback_pending = 1'b0; // reset the flag
                    NS = ST_READ_CACHE; // we can now refill the cache
                end
            end
            default: NS = ST_READ_CACHE; //defeualt j go in circles
        endcase
    end

endmodule
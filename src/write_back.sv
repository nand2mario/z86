// The write-back stage
//
// This stage handles register writes, flags updates and branching.
module write_back(
    input             clk,
    input             reset,

    // port 1 (register write)
    input             wb_reg_valid,
    input [2:0]       wb_reg,
    input [15:0]      wb_data,
    input             wb_width,       // 1: word, 0: byte

    // port 2 (2nd register write)
    input             wb_reg2_valid,
    input [2:0]       wb_reg2,
    input [15:0]      wb_data2,
    input             wb_width2,      // 1: word, 0: byte

    // port 3 for flags
    input             wb_flags_valid,
    input [15:0]      wb_flags,
    input [1:0]       wb_flags_update_mask,  // [0]=CF, [1]=OF

    // port 4 for segment register write
    input             wb_seg_valid,
    input [1:0]       wb_seg,
    input [15:0]      wb_seg_data,

    // branching
    input             br_taken,
    input [19:0]      br_target,
    input [15:0]      br_new_cs,
    input [15:0]      br_new_ip,

    // register file interface
    output reg [1:0]  reg1_we,
    output reg [2:0]  reg1_waddr,
    output reg [15:0] reg1_wdata,
    output reg [1:0]  reg2_we,
    output reg [2:0]  reg2_waddr,
    output reg [15:0] reg2_wdata,

    // other registers
    output reg [15:0] seg_ES /* verilator public */,
    output reg [15:0] seg_CS /* verilator public */,
    output reg [15:0] seg_SS /* verilator public */,
    output reg [15:0] seg_DS /* verilator public */,
    output reg [15:0] reg_ip /* verilator public */,

    // debug interface
    input             reg_wr,
    input    [3:0]    reg_addr,
    input   [15:0]    reg_din
);

import z86_package::*;

// Segment registers
logic [15:0] seg_ES_reg, seg_CS_reg, seg_SS_reg, seg_DS_reg;

// Forward register writes to regfile
always_comb begin
    reg1_we = 2'b00;
    reg2_we = 2'b00;
    reg1_waddr = 0;
    reg1_wdata = 0;
    reg2_waddr = 0;
    reg2_wdata = 0;

    // Register write
    if (wb_reg_valid) begin
        // if (wb_reg != 0 | wb_data != dbg_last_data) begin   // supress NOP AX=AX
        //     $display("WB reg=%x data=%x width=%x", wb_reg, wb_data, wb_width);
        //     dbg_last_data = wb_data;
        // end
        if (wb_width) begin                             // AX, CX, DX, BX, SP, BP, SI, DI
            reg1_we = 2'b11;
            reg1_waddr = wb_reg;
            reg1_wdata = wb_data;
        end else if (wb_reg[2]) begin
            reg1_we = 2'b10;
            reg1_waddr = wb_reg[1:0];
            reg1_wdata = {wb_data[7:0], wb_data[7:0]};    // AH, CH, DH, BH
        end else begin
            reg1_we = 2'b01;
            reg1_waddr = wb_reg[1:0];
            reg1_wdata = {wb_data[7:0], wb_data[7:0]};     // AL, CL, DL, BL
        end
    end

    // second register write
    if (wb_reg2_valid) begin
        // if (wb_reg2 != 0 | wb_data2 != dbg_last_data) begin   // supress NOP AX=AX
        //     $display("WB reg2=%x data2=%x width2=%x", wb_reg2, wb_data2, wb_width2);
        //     dbg_last_data = wb_data2;
        // end
        if (wb_width2) begin
            reg2_we = 2'b11;
            reg2_waddr = wb_reg2;
            reg2_wdata = wb_data2;
        end else if (wb_reg2[2]) begin
            reg2_we = 2'b10;
            reg2_waddr = wb_reg2[1:0];
            reg2_wdata = {wb_data2[7:0], wb_data2[7:0]};    // AH, CH, DH, BH
        end else begin
            reg2_we = 2'b01;
            reg2_waddr = wb_reg2[1:0];
            reg2_wdata = {wb_data2[7:0], wb_data2[7:0]};     // AL, CL, DL, BL
        end
    end

    if (reg_wr && reg_addr[3] == 1'b0) begin
        reg1_we = 2'b11;
        reg1_waddr = reg_addr[2:0];
        reg1_wdata = reg_din;
    end

end

// output of segment registers
always_comb begin
    seg_ES = seg_ES_reg;
    seg_CS = seg_CS_reg;
    seg_SS = seg_SS_reg;
    seg_DS = seg_DS_reg;

    // Forwarding of segment registers writes to output
    if (CONFIG_FORWARDING_REGFILE & wb_seg_valid) begin
        case (wb_seg[1:0])
            2'd0: seg_ES = wb_seg_data;
            2'd1: seg_CS = wb_seg_data;
            2'd2: seg_SS = wb_seg_data;
            2'd3: seg_DS = wb_seg_data;
        endcase
    end
end

always_ff @(posedge clk) begin

    // segment register write
    if (wb_seg_valid) begin
        unique case (wb_seg[1:0])
            2'd0: seg_ES_reg <= wb_seg_data;
            2'd1: seg_CS_reg <= wb_seg_data;
            2'd2: seg_SS_reg <= wb_seg_data;
            2'd3: seg_DS_reg <= wb_seg_data;
        endcase
    end

    // far transfer (JMP FAR or INT sequence finishing)
    if (br_taken) begin
        seg_CS_reg   <= br_new_cs;
        reg_ip   <= br_new_ip;
    end

    // debug interface
    if (reg_wr) begin
        casez (reg_addr)
        4'b0???: begin
            // reg1_we <= 2'b11;
            // reg1_waddr <= reg_addr[2:0];
            // reg1_wdata <= reg_din;
        end
        4'd8: seg_CS_reg <= reg_din;
        4'd9: seg_SS_reg <= reg_din;
        4'd10: seg_DS_reg <= reg_din;
        4'd11: seg_ES_reg <= reg_din;
        4'd12: reg_ip <= reg_din;
        // 4'd13: reg_f <= reg_din;    // now in execute
        default: ;
        endcase
    end
end

endmodule
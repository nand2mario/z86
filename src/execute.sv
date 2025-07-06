// The execution stage
//
// ex_ucode_valid decides whether microcode or hardwired logic is used:
// - Hardwired instruction. This may need to wait for memory read result (rd_ready) 
//   before execution. Once begun, it executes in a single cycle.
// - Microcode execution could take multiple cycles. This is used for complex 
//   instructions like JMP FAR (two memory reads).
//
// This also launches memory stores.
//
// Register/flags updates and branching are sent to the WRITE-BACK stage.
import z86_package::*;
module execute (
    input             clk,
    input             reset,
    output reg        halted /* verilator public */,

    // EXECUTE interface
    output reg        ex_ready,         // EXECUTE consumes an instruction
    input             ex_valid /* verilator public */,         
                                        // instruction output is valid
    input [15:0]      ex_ip_after /* verilator public */,
    input  [7:0]      ex_opcode /* verilator public */,
    input [16:0]      ex_iclass,        // one-hot hardwired instruction class
    input  [7:0]      ex_modrm,
    input [31:0]      ex_disp,
    input [15:0]      ex_imm,
    input             ex_mem_rd,
    input [15:0]      ex_e_addr_val,
    input [19:0]      ex_e_fulladdr,
    input [15:0]      ex_e_segment,
    input [15:0]      ex_g_val,

    input             ex_ucode_valid,         
    input  [8:0]      ex_ucode_entry_next,    

    input id_instruction ex_inst,

    // WRITE-BACK interface
    // port 1
    output reg        wb_reg_valid,         // register write
    output reg  [2:0] wb_reg,
    output reg [15:0] wb_data,
    output reg        wb_width,             // 1: word, 0: byte

    // port 2
    output reg        wb_reg2_valid,
    output reg [2:0]  wb_reg2,
    output reg [15:0] wb_data2,
    output reg        wb_width2,            // 1: word, 0: byte

    // port 3 for flags
    output reg        wb_flags_valid,
    output reg [15:0] wb_flags,
    output reg [1:0]  wb_flags_update_mask,  // [0]=CF, [1]=OF

    // port 4 for segment register write
    output reg        wb_seg_valid,
    output reg [1:0]  wb_seg,
    output reg [15:0] wb_seg_data,

    // memory interface
    output reg        rd,
    input             rd_ready,
    output reg [19:0] rd_addr,
    output reg        rd_word,
    input      [15:0] rd_data,
    output reg        rd_io,

    output reg        wr_valid,
    input             wr_ready,
    output reg [19:0] wr_addr,
    output reg [15:0] wr_data,
    output reg        wr_word,
    output reg        wr_io,

    // register file interface
    output reg        ex_reg,             // Microcode needs regfile
    output reg [2:0]  reg1_raddr,
    input      [15:0] reg1_rdata,
    output reg [2:0]  reg2_raddr,
    input      [15:0] reg2_rdata,
    output reg        reg1_we,
    output reg [2:0]  reg1_waddr,
    output reg [15:0] reg1_wdata,
    output reg        reg2_we,
    output reg [2:0]  reg2_waddr,
    output reg [15:0] reg2_wdata,

    // other registers
    output reg [15:0] reg_f /* verilator public */ = FLAGS_INIT,
    input      [15:0] seg_CS,        // from write-back
    input      [15:0] seg_SS,
    input      [15:0] seg_DS,
    input      [15:0] seg_ES,

    // FETCH and WRITE-BACK -  branch target
    output reg [19:0] br_target,
    output reg        br_taken,
    output reg [15:0] br_new_cs,
    output reg [15:0] br_new_ip,

    // debug interface
    input             reg_wr,
    input    [3:0]    reg_addr,
    input   [15:0]    reg_din
);

id_instruction ii;
assign ii = ex_inst;

// reg [15:0] reg_f /* verilator public */ = FLAGS_INIT ;
wire CF = reg_f[0], PF = reg_f[2], AF = reg_f[4], ZF = reg_f[6], SF = reg_f[7], TF = reg_f[8], IF = reg_f[9], DF = reg_f[10], OF = reg_f[11];

////////////////////////////////////////////////////////////////////////////
// Microcode execution
////////////////////////////////////////////////////////////////////////////

reg ex_first_cycle = 1'b1;     // there's no ongoing instruction in EXECUTE
always @(posedge clk) begin
    if (reset) ex_first_cycle <= 1'b1;
    else begin
        if (ex_valid) ex_first_cycle <= 1'b0;
        if (ex_ready) ex_first_cycle <= 1'b1;
    end
end

reg [15:0] iclass;

//--------------------------------------------------------------------
// Microcode driven instructions
//--------------------------------------------------------------------

typedef struct packed {
    logic [5:0] op;      // up to 64 micro-ops
    logic [3:0] arg;     // small immediate / register selector / flag bits
    logic       stop;    // assert on the final µ-step
} uinstr_t;

`define U(_op,_arg,_stop) '{op:_op, arg:_arg, stop:_stop}

`include "ucode_rom.svh"
`include "ucode_entry.svh"

uinstr_t mc_cur;         // current microcode instruction

// `ex_valid` and `ex_ready` logic:
// - Instruction is valid when `ex_valid==1`.
// - Instruction is also invalidated when `br_taken==1` - last instruction has taken a branch.
// - When `ex_valid==1 && ex_ready==1`, the instruction execution is done by EXECUTE.
// - For microcoded instructions, `ex_ready` is driven by the STOP bit of the microcode 
//   (last microcode). This way ex_* from DECODE2 stays constant until ex_ready is 1, 
//   simplifying microcode logic.
// - For hardwired instructions. `ex_ready` is 1 on the cycle the instruction is consumed by EXECUTE.
//   This allows back-to-back execution of hardwired instructions.
always_comb begin
    ex_ready = 1'b0;
    if (ex_valid) begin
        if (ex_ucode_valid) begin
            ex_ready = rd_ready & (~wr_valid | wr_ready) & ~div_busy & mc_cur.stop;
        end else begin
            ex_ready = (~ex_mem_rd | rd_ready) & (~wr_valid | wr_ready);
        end
    end
    if (br_taken) ex_ready = 1'b1;   // immediate finish current instruction is branch taken
end

// `mc_active` - micro-op is allowed to execute when write-back is ready and memory is ready.
wire        mc_active = ex_valid & rd_ready & (~wr_valid | wr_ready) & ~div_busy;
reg         mc_wait;                 // do not advance to next micro-op

// microcode output
reg         mc_valid;
reg         mc_reg_valid;
reg         mc_seg_valid;
reg [1:0]   mc_seg;
reg [15:0]  mc_seg_data;
reg [2:0]   mc_reg;
reg [15:0]  mc_data;
reg         mc_width;
reg         mc_reg2_valid;
reg [2:0]   mc_reg2;
reg [15:0]  mc_data2;
reg         mc_width2;

reg         mc_wr;
reg [19:0]  mc_wr_addr;
reg [15:0]  mc_wr_data;
reg         mc_wr_word;
reg         mc_wr_io;

reg         mc_cond, mc_cond_r;       // result of the last TEST*

reg         mc_branch_taken;
reg [8:0]   mc_branch_rel;
reg [4:0]   mc_div_cnt;

reg [8:0]   mc_pc;

// Microcode sequencer
always_ff @(posedge clk) begin
    logic [8:0] mc_pc_next;
    if (reset) begin
        mc_pc_next = 9'd0;
    end else if (mc_active) begin
        if (mc_wait)
            mc_pc_next = mc_pc;
        else if (mc_cur.stop)               // normal STOP
            mc_pc_next = 8'd0;
        else if (mc_branch_taken)                    // conditional branch
            mc_pc_next = mc_pc + 9'd1 + mc_branch_rel;
        else                                         // fall-through
            mc_pc_next = mc_pc + 9'd1;
    end
    if (ex_valid && ex_ready || ~ex_valid && ~ex_ready) begin // next cycle is new instruction
        mc_pc_next = ex_ucode_entry_next;
    end

    mc_pc <= mc_pc_next;
    mc_cur <= microcode_rom[mc_pc_next];
end

// ──────────────────────────────────────────────────────────────
// Multi-cycle computations for AAD, MUL, DIV
// ──────────────────────────────────────────────────────────────
// AAD computation : 3 cycles
logic aad_req;
logic [1:0] aad_state;     // 0: idle, 1: multiply, 2: add
logic aad_done;
logic [7:0] aad_res;
logic [7:0] aad_a, aad_b, aad_c;  // a*b+c

always @(posedge clk) begin
    if (reset) begin
        aad_done <= 1'b0;
        aad_state <= 2'b0;
    end else case (aad_state) 
        2'd0: begin
            aad_done <= 1'b0;
            if (aad_req) begin
                aad_a <= ex_e_addr_val[15:8];// regs[AX][15:8];
                aad_b <= ex_imm[7:0];
                aad_c <= ex_e_addr_val[7:0];// regs[AX][7:0];
                aad_state <= 2'd1;
            end
        end
        2'd1: begin
            aad_res <= aad_a * aad_b;
            aad_state <= 2'd2;
        end
        2'd2: begin
            aad_res <= aad_res + aad_c;
            aad_done <= 1'b1;
            aad_state <= 2'b0;
        end
        default: aad_state <= 2'b0;
    endcase
end

// MUL computation : 2 cycles
logic mul_req;
logic mul_state;     // 0: idle, 1: multiply
logic mul_done;
logic [15:0] mul_a, mul_a_reg;
logic [15:0] mul_b, mul_b_reg;
logic        mul_signed;
logic [31:0] mul_res;
always @(posedge clk) begin
    if (reset) begin
        mul_done <= 1'b0;
    end else if (mul_state==0) begin
        mul_done <= 1'b0;
        if (mul_req) begin
            mul_a_reg <= mul_a;
            mul_b_reg <= mul_b;
            mul_state <= 1'b1;
        end
    end else begin
        mul_done <= 1'b1;
        mul_state <= 1'b0;
        if (mul_signed) begin
            mul_res = signed'(mul_a_reg) * signed'(mul_b_reg);
        end else begin
            mul_res = mul_a_reg * mul_b_reg;
        end
    end
end

// Divide 
// - 16-bit divided by 8-bit (16 cycles), or 32-bit divided by 16-bit (32 cycles)
// - signed / unsigned
logic         div_start;
logic         div_width;     // 0 = byte, 1 = word
logic         div_signed;    // 0 = DIV, 1 = IDIV
logic [31:0]  div_dividend;
logic [15:0]  div_divisor;
logic         div_busy;
logic         div_done;
logic         div_overflow;  // or divide-by-zero
logic [15:0]  div_quot;
logic [15:0]  div_rem;

divider u_div (
    .clk        (clk),
    .rst_n      (~reset),
    .start      (div_start),
    .width      (div_width),
    .is_signed  (div_signed),
    .dividend   (div_dividend),
    .divisor    (div_divisor),
    .busy       (div_busy),
    .done       (div_done),
    .overflow   (div_overflow),
    .quotient   (div_quot),
    .remainder  (div_rem)
);

// BCD / ASCII Adjustment helper
logic adj_req;
logic [1:0] adj_state;
logic adj_done;
logic [15:0] adj_res;
logic [15:0] adj_flags;
logic        adj_cond;     // 1: divide-by-0 for AAM
always @(posedge clk) begin
    if (reset) begin
        adj_done <= 1'b0;
        adj_state <= 2'b0;
    end else case (adj_state)
        2'd0: begin
            adj_done <= 0;
            if (adj_req) begin
                adj_state <= 2'd1;
            end
        end
        2'd1: begin
            // current AX & flags
            logic [7:0] al  = ex_e_addr_val[7:0]; // regs[AX][7:0];
            logic [7:0] ah  = ex_e_addr_val[15:8];// regs[AX][15:8];
            logic [7:0] imm = ex_imm[7:0];
            logic       cf  = CF;
            logic       af  = AF;
            // working copies
            logic [7:0] al_n = al;
            logic [7:0] ah_n = ah;
            logic       cf_n = cf;
            logic       af_n = af;

            adj_state <= 2'd2;
            adj_cond <= 1'b0;

            unique case (mc_cur.arg)
            // ---------- DAA 27 ------------------------------------------------
            3'd0: begin
                if ( (al[3:0] > 4'd9) || af ) begin
                    {cf_n,al_n} = al + 8'h06;
                    af_n = 1'b1;
                end else af_n = 1'b0;

                if ( (al > 8'h99) || cf ) begin
                    al_n = al_n + 8'h60;
                    cf_n = 1'b1;
                end else cf_n = 1'b0;
            end
            // ---------- DAS 2F ------------------------------------------------
            3'd1: begin
                if ( (al[3:0] > 4'd9) || af ) begin
                    {cf_n,al_n} = {1'b0,al} - 9'd6;
                    af_n = 1'b1;
                end else af_n = 1'b0;

                if ( (al > 8'h99) || cf ) begin
                    al_n = al_n - 9'h60;
                    cf_n = 1'b1;
                end
            end
            // ---------- AAA 37 ------------------------------------------------
            3'd2: begin
                if ( (al[3:0] > 4'd9) || af ) begin
                    al_n = al + 8'h06;
                    ah_n = ah + 8'h01;
                    cf_n = 1'b1;  af_n = 1'b1;
                end else begin
                    cf_n = 1'b0;  af_n = 1'b0;
                end
                al_n &= 8'h0F;
                if (DEBUG) $display("AAA: al=%x, ah=%x, cf=%x, af=%x", al, ah, cf_n, af_n);
            end
            // ---------- AAS 3F ------------------------------------------------
            3'd3: begin
                if ( (al[3:0] > 4'd9) || af ) begin
                    al_n = al - 8'h06;
                    ah_n = ah - 8'h01;
                    cf_n = 1'b1;  af_n = 1'b1;
                end else begin
                    cf_n = 1'b0;  af_n = 1'b0;
                end
                al_n &= 8'h0F;
            end
            // ---------- AAM D4 ------------------------------------------------
            3'd4: begin
                ah_n =  al / imm;
                al_n =  al % imm;
                af_n = 1'b0;
                if (imm == 8'h00) begin
                    adj_cond <= 1'b1;
                    ah_n = ah;
                    al_n = al;
                end
                // CF/OF undefined – leave
                if (DEBUG) $display("AAM: al_n=%x, ah_n=%x, imm=%x, cf=%x, af=%x", al_n, ah_n, imm, cf_n, af_n);
            end
            // ---------- AAD D5 ------------------------------------------------
            // 3'd5: begin
            //     al_n =  (ah * imm) + al;
            //     ah_n =  8'h00;
            //     af_n = 1'b0;
            // end
            default: ;
            endcase            
            adj_res <= {ah_n, al_n};

            if (mc_cur.arg >= 4'd4)
                cf_n = reg_f[0];

            adj_flags <= {reg_f[15:5],
                          af_n,
                          reg_f[3:1],
                          cf_n};
        end

        2'd2: begin                // flags update in separate cycle to increase Fmax
            adj_state <= 0;
            adj_done <= 1'b1;

            adj_flags <= {adj_flags[15:8], 
                          adj_res[7],      // SF
                          (adj_res[7:0] == 8'h00), // ZF
                          adj_flags[5],
                          adj_flags[4],    // AF
                          adj_flags[3],
                          ~^adj_res[7:0],  // PF
                          adj_flags[1],    
                          adj_flags[0]};   // CF
            if (adj_cond)                  // no register modify on divide-by-0
                adj_flags <= reg_f;
        end

        default: adj_state <= 0;
    endcase
end

//--------------------------------------------------------------------
// Instruction and microcode decoding
//--------------------------------------------------------------------
reg mc_load_cs_ip;
reg [15:0] tmp_lo, tmp_hi, tmp_flags;
reg [15:0] tmp_lo_reg, tmp_hi_reg, tmp_flags_reg;
reg read_hi, read_lo, read_flags;    // reading into tmp_lo / tmp_hi
reg pop_reg, pop_next;               // pop memory read in progress

// 3 registers for microcode: tmp_lo / tmp_hi / tmp_flags
// tmp_lo defaults to E value / effective address, tmp_hi defaults to G value 
assign tmp_lo = ex_first_cycle ? ex_e_addr_val : 
                read_lo        ? rd_data :         
                                 tmp_lo_reg;
assign tmp_hi = ex_first_cycle ? ex_g_val :
                read_hi        ? rd_data : 
                                 tmp_hi_reg;
assign tmp_flags = read_flags ? rd_data : tmp_flags_reg;

reg [15:0] mc_new_ip, mc_new_cs, new_sp;
reg        mc_flags_valid;
reg [15:0] mc_flags;
reg [1:0]  mc_flags_update_mask;
// Shadowed SP for µ-code self-consistency
reg [15:0] sp_fw;   
reg        sp_fw_valid;
reg        halted_next;

// Register address generation for µ-op
always_comb begin
    ex_reg = 0;
    reg1_raddr = 0;
    reg2_raddr = 0;
    reg1_we = 0;
    reg1_waddr = 'x;
    reg1_wdata = 'x;
    reg2_we = 0;
    reg2_waddr = 'x;
    reg2_wdata = 'x;
    if (ex_ucode_valid) begin  
        ex_reg = ~mc_cur.stop;      // stop cycle does not use regfile
        reg2_raddr = R_SP;          // reg2 default to SP
        unique case (mc_cur.op)
        MC_PUSH: begin
            reg1_raddr = mc_cur.arg[3] ? mc_cur.arg[2:0]     // PUSH AX...
                                       : ex_modrm[2:0];      // PUSH RM
            // $display("MC_PUSH: reg1_raddr=%x", reg1_raddr);
        end
        MC_LOAD: begin
            reg1_raddr = mc_cur.arg[0] ? R_DI : R_SI;
        end
        MC_STORE: begin
            reg1_raddr = R_DI;
        end
        MC_IN,MC_OUT: begin
            reg1_raddr = R_DX;
        end
        MC_READ: begin
            reg1_raddr = mc_cur.arg[2:0];
        end
        MC_TESTZX, MC_DEC: begin
            reg1_raddr = R_CX;
        end
        MC_STR_CMP, MC_MULU, MC_MULS: begin
            reg1_raddr = R_AX;
        end
        MC_DIV: begin
            reg1_raddr = R_AX;
            reg2_raddr = R_DX;
        end
        default: ;
        endcase
    end    
end

logic [15:0] this_sp;

// Main µ-op execution
always_comb begin
    logic [15:0] val = 'x;

    mc_wait = 0;

    // defaults
    mc_data = 16'h0000;
    mc_valid = 0;
    mc_reg_valid = 0;
    mc_reg = 3'b0;
    mc_width = 1'b1;
    mc_reg2_valid = 0;
    mc_reg2 = 3'b0;
    mc_data2 = 16'h0000;
    mc_width2 = 1'b1;
    mc_seg_valid = 0;
    mc_seg = 'x;
    mc_seg_data = 'x;

    mc_load_cs_ip = 0;
    mc_new_ip = 16'h0000;
    mc_new_cs = 16'h0000;
    new_sp = 16'h0000;

    mc_wr = 0;
    mc_wr_addr = 20'h00000;
    mc_wr_data = 16'h0000;
    mc_wr_word = 1'b1;
    mc_wr_io = 0;

    mc_flags_valid = 0;
    mc_flags = reg_f;
    mc_flags_update_mask = 2'b00;

    mc_branch_taken = 0;
    mc_branch_rel = 8'd0;
    mc_cond = 'x;

    rd = 0;
    rd_io = 0;
    rd_addr = 'x;
    rd_word = 1'b1;

    div_start = 0;
    div_width = 0;
    div_signed = 0;
    div_dividend = 0;
    div_divisor = 0;

    aad_req = 1'b0;
    mul_req = 1'b0;
    mul_a = 'X;
    mul_b = 'X;
    mul_signed = 'X;
    adj_req = 1'b0;

    halted_next = halted;

    //------------------------------------------------------------------
    // µ-op execution 
    //------------------------------------------------------------------
    if (ex_ucode_valid) begin  
        // reg2_raddr = R_SP;
        this_sp = sp_fw_valid ? sp_fw : reg2_rdata;

        unique case (mc_cur.op)
        //==================================================================
        //  MC_PUSH  – push one word to the stack
        //     arg : CS       0  - CS
        //           IP_THIS  1  - this instruction's IP
        //           IP_AFTER 2  - CALL/INT return address
        //           RM       3  - regs[modrm[2:0]]
        //           FLAGS    4  - reg_f
        //           TMP_LO   5  - tmp_lo
        //           AX-DI    8-15 - regs[AX-DI]
        //==================================================================
        MC_PUSH: begin
            // -------- adjust SP + generate writes ----------------------------------
            new_sp = this_sp - 16'd2;       // pre-decrement

            // -------- pick value ---------------------------------------------------
            unique casez (mc_cur.arg)
                4'd0  : val = seg_CS;
                4'd1  : val = ii.ip_this;
                4'd2  : val = ex_ip_after;
                4'd3  : begin
                    val = reg1_rdata;
                    if (ex_modrm[2:0] == R_SP)
                        val = new_sp;
                end
                4'd4  : val = reg_f;
                4'd5  : val = tmp_lo;
                4'b1???: val = reg1_rdata;   // PUSH AX...
                default: ;
            endcase

            mc_valid        = 1'b1;

            // port-1 : memory write
            mc_wr = 1'b1;
            mc_wr_addr = {seg_SS,4'b0} + new_sp;
            mc_wr_data = val;

            // port-2 : SP ← new_sp
            mc_reg2_valid   = 1'b1;
            mc_reg2         = R_SP;
            mc_data2        = new_sp;
            // $display("MC_PUSH: val=%x, new_sp=%x, ex_modrm=%x", val, new_sp, ex_modrm);
        end

        //==================================================================
        //  MC_POP  – pop one word from the stack
        //            arg[3:0]     0: pop to tmp_lo
        //                         1: pop to tmp_hi
        //                         2: pop to tmp_flags
        //                         8-15: pop to regs[AX-DI]
        //==================================================================
        MC_POP: begin
            mc_wait = 1;
            pop_next = pop_reg;
            // We work in *two* micro-cycles:
            //  ▸ first cycle issues the read + writes SP+2
            //  ▸ data become available later and are latched by
            //    the tiny always_ff at the end of this file into tmp_lo,
            //    tmp_hi, or written back to registers. 
            if ((~wr_valid | wr_ready) &&rd_ready && ~pop_reg & ~br_taken) begin
                pop_next = 1;
                rd               = 1;                // launch read
                rd_addr          = {seg_SS,4'b0} + this_sp;

                new_sp           = this_sp + 16'd2;  // post-increment
                mc_valid         = 1'b1;
                mc_reg2_valid    = 1'b1;
                mc_reg2          = R_SP;
                mc_data2         = new_sp;
            end

            if (rd_ready && pop_reg) begin
                pop_next = 0;
                mc_wait = 0;        // continue to next µ-op
                if (mc_cur.arg[3]) begin           // Pop to register
                    mc_valid = 1'b1;
                    mc_reg_valid = 1'b1;
                    mc_reg = mc_cur.arg[2:0];
                    mc_data = rd_data;
                    mc_width = 1'b1;
                end
            end
        end

        //==================================================================
        //  MC_LOAD    arg[1:0]=0   RM ⇒ tmp_lo. RM is either register form (modrm[2:0]) or memory form (@EA)
        //             arg[1:0]=1   @(EA+2) ⇒ tmp_hi
        //             arg[1:0]=2   @(ex_e_segment:SI) ⇒ tmp_lo
        //             arg[1:0]=3   @(ES:DI) ⇒ tmp_hi
        //             arg[2]       1: inc/dec SI/DI (DF sets direction)
        //             arg[3]       1: 16-bit, 0: 8-bit
        //==================================================================
        MC_LOAD: begin
            logic [2:0] r = mc_cur.arg[0] ? R_DI : R_SI;
            logic [15:0] delta = mc_cur.arg[3] ? 16'd2 : 16'd1;
            if (DF) delta = -delta;             // honour Direction-Flag
            rd   = 1;
            // reg1_raddr = mc_cur.arg[0] ? R_DI : R_SI;
            case (mc_cur.arg[1:0]) 
            2'd0: rd_addr = { ex_e_segment, 4'd0 } + ex_e_addr_val;
            2'd1: rd_addr = { ex_e_segment, 4'd2 } + ex_e_addr_val;
            2'd2: rd_addr = { ex_e_segment, 4'd0 } + reg1_rdata;
            2'd3: rd_addr = { seg_ES, 4'd0 } + reg1_rdata;
            endcase

            // register form
            if (ex_modrm[7:6] == 2'b11 && mc_cur.arg[1:0] == 2'd0) begin
                rd = 0;          // cancel memory load, tmp_lo defaults to E value
            end

            // reg2_raddr = r;
            if (mc_cur.arg[2]) begin
                mc_reg2_valid = 1'b1;
                mc_reg2       = r;
                mc_data2      = reg1_rdata + delta;
                mc_valid      = 1'b1;
            end
            if (DEBUG) $display("MC_LOAD: arg=%x, addr=%x, mc_reg2_valid=%x, mc_reg2=%x, mc_data2=%x", mc_cur.arg, rd_addr, mc_reg2_valid, mc_reg2, mc_data2);
        end

        //==================================================================
        //  MC_STORE   arg[1:0]  1: tmp_lo → [EA], 
        //                       2: tmp_lo → [ES:DI], 
        //                       3: AL/AX → [ES:DI]
        //             arg[2]    1: inc/dec DI (DF sets direction)
        //             arg[3]    1: word, 0: byte
        //==================================================================
        MC_STORE: begin
            logic w      = mc_cur.arg[3];
            logic upDI   = mc_cur.arg[2];
            logic [15:0] delta = w ? 16'd2 : 16'd1;
            if (DF) delta = -delta;                 // honour Direction-Flag

            // ----------------------------------------------------------------
            // 1) memory WRITE
            // ----------------------------------------------------------------
            // reg1_raddr = R_DI;
            unique case (mc_cur.arg[1:0])
                2'd1: begin                 // tmp_lo → [EA]
                    mc_wr = 1'b1;
                    mc_wr_addr = ex_e_fulladdr; // {ex_e_segment,4'b0} + ex_e_addr_val;
                    mc_wr_data = tmp_lo;
                end
                2'd2: begin                 // tmp_lo → [ES:DI]
                    mc_wr = 1'b1;
                    mc_wr_addr = {seg_ES,4'b0} + reg1_rdata;
                    mc_wr_data = tmp_lo;
                end
                2'd3: begin                 // AL/AX → [ES:DI]
                    mc_wr = 1'b1;
                    mc_wr_addr = {seg_ES,4'b0} + reg1_rdata;
                    mc_wr_data = ex_e_addr_val;      // TODO: check if this gives us AX
                end
                default: ;   
            endcase
            mc_wr_word = w;
            // $display("MC_STORE: w=%x, addr=%x, data=%x", w, mc_addr, mc_data);

            // ----------------------------------------------------------------
            // 2) pointer maintenance   (port-1 & port-2 if we touch both)
            // ----------------------------------------------------------------
            if (upDI) begin
                mc_reg2_valid = 1'b1;
                mc_reg2       = R_DI;
                mc_data2      = reg1_rdata + delta;
            end
            mc_valid = 1'b1;                      // commit to WB stage
        end

        //==================================================================
        //  MC_IN  – read from I/O port
        //                  IO[DX] → tmp_lo
        //             arg[0]    1: word, 0: byte
        //==================================================================
        MC_IN: begin
            rd = 1;
            rd_addr = reg1_rdata;
            rd_io = 1;
            // rd_width = mc_cur.arg[0];
            // result captured to tmp_lo down below
        end

        //==================================================================
        //  MC_OUT  – write to I/O port
        //                  tmp_lo → IO[DX]
        //             arg[0]    1: word, 0: byte
        //==================================================================
        MC_OUT: begin
            mc_wr = 1;
            mc_wr_addr = reg1_rdata;
            mc_wr_data = tmp_lo;
            mc_wr_word = mc_cur.arg[0];
            mc_wr_io = 1;
            mc_valid = 1'b1;
        end

        //==================================================================
        //  MC_BR_REL16   IP ← IP_after + disp16   (segment unchanged)
        //==================================================================
        MC_BR_REL16: begin
            mc_valid     = 1'b1;
            mc_load_cs_ip   = 1'b1;
            mc_new_cs       = seg_CS;
            mc_new_ip       = ex_ip_after + ex_imm;        // disp already sign-extended
        end

        //==================================================================
        //  MC_BR_ABS16   IP ← tmp_lo    (segment unchanged)
        //==================================================================
        MC_BR_ABS16: begin
            mc_valid     = 1'b1;
            mc_load_cs_ip   = 1'b1;
            mc_new_cs       = seg_CS;
            mc_new_ip       = tmp_lo;                      // fetched earlier
            if (DEBUG) $display("MC_BR_ABS16: CS=%x IP=%x", mc_new_cs, mc_new_ip);
        end

        MC_BR_FAR: begin
            mc_valid     = 1'b1;
            mc_load_cs_ip   = 1'b1;
            if (mc_cur.arg[0]) begin              // arg = 1  ⇒  immediate 32-bit ptr
                mc_new_ip = ex_disp[15:0];           // low word  = offset
                mc_new_cs = ex_disp[31:16];          // high word = segment
            end else begin                        // arg = 0  ⇒  ptr already in tmp
                mc_new_ip = tmp_lo;
                mc_new_cs = tmp_hi;
            end    
            // $display("MC_BR_FAR: CS=%x IP=%x", mc_new_cs, mc_new_ip);
        end

        //------------------------------------------------------------------
        // ADJSP  (MC_ADJSP)   
        //             arg[0] = 0 : SP += Iw
        //             arg[0] = 1 : SP -= Iw
        //------------------------------------------------------------------
        MC_ADJSP: begin
            mc_valid      = 1;
            mc_reg2_valid = 1;
            mc_reg2       = R_SP;
            mc_data2      = mc_cur.arg[0] ? this_sp - ex_imm : this_sp + ex_imm;     
            if (DEBUG) $display("MC_ADJSP: SP=%x imm=%x, new=%x", this_sp, ex_imm, mc_data2);
        end

        //==================================================================
        //  MC_GETVEC  – read interrupt vector table entry, and also clears IF/TF
        //              arg = 15   ⇒ use imm8 from instruction (INT Ib)
        //              arg = 0-14 ⇒ literal vector number
        //  ▸ Read WORD from 0000:XXXX
        //    GETVEC_OFF) offset word → tmp_lo
        //    GETVEC_SEG) segment word → tmp_hi
        //==================================================================
        MC_GETVEC_OFF, MC_GETVEC_SEG: begin
            rd   = 1;                         // toggle
            rd_addr  = { 12'h000, 
                         mc_cur.arg == 4'd15 ? ex_imm[7:0] : mc_cur.arg, 
                         mc_cur.op == MC_GETVEC_SEG,
                         1'b0};

            mc_valid = 1'b1;
            mc_flags_valid = 1'b1;
            mc_flags[8] = 0;     // TF=0
            mc_flags[9] = 0;     // IF=0
                        
            if (mc_cur.op == MC_GETVEC_OFF) begin
                if (DEBUG) $display("MC_GETVEC_OFF: arg=%x, vec=%x, addr=%x", mc_cur.arg, mc_cur.arg ? mc_cur.arg : ex_imm[7:0], rd_addr);
            end else begin
                if (DEBUG) $display("MC_GETVEC_SEG: arg=%x, vec=%x, addr=%x", mc_cur.arg, mc_cur.arg ? mc_cur.arg : ex_imm[7:0], rd_addr);
            end
        end

        //------------------------------------------------------------------
        // WR_FLAGS  - FLAGS ← tmp_flags
        //------------------------------------------------------------------
        MC_WR_FLAGS: begin
            mc_valid     = 1'b1;
            mc_flags_valid = 1'b1;
            {mc_flags[11:6], mc_flags[4], mc_flags[2], mc_flags[0]} = {tmp_flags[11:6], tmp_flags[4], tmp_flags[2], tmp_flags[0]};
            mc_flags_update_mask = 2'b11;
            if (DEBUG) $display("MC_WR_FLAGS: flags=%x", mc_flags);
        end

        //==================================================================
        //  MC_READ
        //  args       0-7: regs[arg] → tmp_lo
        //               8: ENTER_IB  → tmp_hi
        //==================================================================
        MC_READ: begin
            // tmp_lo captured down below
        end

        // ------------------------------------------------------------------
        //  MC_WR_REG   args[2:0]
        //                0: tmp_lo -> AX
        //                4: tmp_lo -> SP
        //                5: tmp_lo -> BP
        //                8: tmp_lo -> AL
        //               10: tmp_lo -> regs[modrm[5:3]]
        //              W=arg[3] (0: byte, 1: word)
        // ------------------------------------------------------------------
        MC_WR_REG: begin
            mc_valid      = 1'b1;
            mc_reg_valid  = 1'b1;
            mc_reg        = mc_cur.arg[3:0] == 4'd10 ? ex_modrm[5:3] : mc_cur.arg[2:0];
            mc_data       = tmp_lo;
            mc_width      = mc_cur.arg != 4'd8;    
            if (DEBUG) $display("MC_WR_REG: reg=%x, data=%x, width=%x", mc_reg, mc_data, mc_width);
        end

        // ------------------------------------------------------------------
        //  MC_WR_SEG   tmp_hi -> segment[arg[2:0]]
        // ------------------------------------------------------------------
        MC_WR_SEG: begin
            mc_valid      = 1'b1;
            mc_seg_valid  = 1'b1;
            mc_seg        = mc_cur.arg[1:0];
            mc_seg_data   = tmp_hi;           // drives WB port-1 bus via mc_data
            if (DEBUG) $display("MC_WR_SEG: seg=%x, data=%x", mc_seg, mc_seg_data);
        end

        // ===============================================================
        // TESTF   – mc_cond = selected flag
        // ===============================================================
        MC_TESTF: begin
            unique case (mc_cur.arg[2:0])
                3'd0: mc_cond = CF;
                3'd1: mc_cond = PF;
                3'd2: mc_cond = AF;
                3'd3: mc_cond = ZF;
                3'd4: mc_cond = SF;
                3'd5: mc_cond = OF;
                default: mc_cond = 1'b0;
            endcase
        end

        // ===============================================================
        // TESTZX  –   arg[1:0]
        //             CX       (0) : mc_cond = (CX == 0)
        //             TMP_HI   (1) : mc_cond = (tmp_hi == 0)    (Ib==0 for ENTER)
        // ===============================================================
        MC_TESTZX:  begin
            // reg1_raddr = R_CX;
            if (mc_cur.arg[1:0] == 2'd1) begin
                mc_cond = (tmp_hi == 8'd0);
            end else begin
                mc_cond = (reg1_rdata == 16'd0);
            end
            // $display("MC_TESTZX: cond=%x, cx=%x", mc_cond, reg1_rdata);
        end

        // ===============================================================
        // IN_RANGE  – mc_cond = (tmp_lo <= Gv <= tmp_hi)
        // ===============================================================
        MC_IN_RANGE: begin
            mc_cond = (tmp_lo <= reg1_rdata && reg1_rdata <= tmp_hi);
        end

        // ===============================================================
        // J / JT / JF  – small relative microcode jump
        // ===============================================================
        MC_J, MC_JT, MC_JF: begin
            if ( (mc_cur.op==MC_J) || (mc_cur.op==MC_JT ? mc_cond_r : ~mc_cond_r) ) begin
                // ask the sequencer to jump; no wb / mem side-effects
                mc_valid = 1'b0;                // nothing for write-back
                mc_branch_taken = 1'b1;         // new wire (see below)
                mc_branch_rel   = { {5{mc_cur.arg[3]}}, mc_cur.arg[3:0] };  // sxt(4)
            end
        end


        // ------------------------------------------------------------------
        //  MC_STR_CMP   arg[0]=W   (0 = byte  1 = word)
        //
        //   For CMPS :  tmp_lo (DS:SI)  vs  tmp_hi (ES:DI)
        //   For SCAS :  AL/AX           vs  tmp_hi (ES:DI)
        //
        //   Computes Z,S,P,CF,OF,AF exactly like SUB but discards the result.
        // ------------------------------------------------------------------
        MC_STR_CMP: begin
            logic w = mc_cur.arg[0];
            logic [15:0] opA, opB;
            if (ex_opcode==8'hAE || ex_opcode==8'hAF) begin   // SCAS
                // reg1_raddr = R_AX;
                opA = w ? ex_e_addr_val : {8'h00,ex_e_addr_val[7:0]};
                opB = tmp_hi;
            end else begin                                    // CMPS
                opA = tmp_lo;
                opB = tmp_hi;
            end

            mc_valid              = 1'b1;
            mc_flags_valid = 1'b1;
            mc_flags = execute_sub(opA, opB,
                                    /*borrow=*/1'b0,
                                    /*width=*/w,
                                    /*e_is_op1=*/1'b1);

            mc_flags_update_mask   = 2'b11;    // CF+OF via mask, rest full
        end

        // ------------------------------------------------------------------
        //  MC_DEC  – 
        //             arg = 0 : CX--
        //             arg = 8 : tmp_hi--
        // ------------------------------------------------------------------
        MC_DEC: begin
            if (mc_cur.arg[3:0] == 4'd1) begin
                mc_valid = 1'b1;
                mc_reg_valid = 1'b1;
                mc_reg = R_CX;
                // reg1_raddr = R_CX;
                mc_data = reg1_rdata - 16'd1;

                mc_flags_valid = 1'b1;
                mc_flags = {mc_flags[15:1], 1'b0};
                if (DEBUG) $display("MC_DEC: reg=%x, new=%x", R_CX, mc_data);
            end
        end


        // ------------------------------------------------------------------
        // Unsigned / Signed multiply with a single DSP, 
        //       arg[1:0]  0 : AX    <= AL * r/m (8-bit)
        //                 1 : DX:AX <= AX * r/m (16-bit)
        //                 2 : G     <= imm8 * r/m
        //                 3 : G     <= imm16 * r/m
        // ------------------------------------------------------------------
        MC_MULU, MC_MULS: begin
            logic w = mc_cur.arg[0];
            logic signed_op = (mc_cur.op==MC_MULS);

            mc_wait = 1;
            if (~mul_done) begin
                if (mc_active) begin
                    // launch the multiplier
                    mul_req = 1'b1;
                    if (mc_cur.arg[1] == 1'b0) begin
                        // g_val is AX
                        mul_a = w         ? ex_g_val : 
                                signed_op ? { {8{ex_g_val[7]}}, ex_g_val[7:0] } :
                                            { 8'h00, ex_g_val[7:0] };
                        mul_b = w ? tmp_lo   : 
                                    signed_op ? { {8{tmp_lo[7]}}, tmp_lo[7:0] } :
                                                { 8'h00, tmp_lo[7:0]  };
                    end else begin
                        mul_a = w         ? ex_imm : 
                                signed_op ? { {8{ex_imm[7]}}, ex_imm[7:0] } :
                                            { 8'h00, ex_imm[7:0] };
                        mul_b = tmp_lo;
                    end 
                    mul_signed = signed_op;
                end
            end else begin
                // output the result
                mc_wait = 0;
                mc_valid         = 1'b1;
                mc_reg_valid     = 1'b1;
                mc_reg           = mc_cur.arg[1] ? ex_modrm[5:3] : R_AX;
                mc_data          = mul_res[15:0];      // low half always to AX

                if (~mc_cur.arg[1] & w) begin          // word → also write DX
                    mc_reg2_valid = 1'b1;
                    mc_reg2       = R_DX;
                    mc_data2      = mul_res[31:16];
                end

                // CF,OF = high part ≠ 0  (8086 rule)
                mc_flags_valid = 1'b1;
                mc_flags           = reg_f;
                mc_flags_update_mask = 2'b11;
                if (signed_op) begin
                    // CF=1 when the intermediate product differs from the sign extended product
                    mc_flags[0]    = (w | mc_cur.arg[1]) ? {16{mul_res[15]}} != mul_res[31:16] : {8{mul_res[7]}} != mul_res[15:8];  
                end else begin
                    // CF=1 when upper bits are not all zero
                    mc_flags[0]    = (w ? mul_res[31:16] != 0 : mul_res[15:8]) != 0;
                end
                mc_flags[11]       = mc_flags[0];
                mc_flags[7:4]      = 4'b0;
                mc_flags[2]        = ~^mul_res[7:0];    // PF, undefined
                mc_flags[6]        = mul_res[15:0] == 0;// ZF, undefined
                mc_flags[7]        = w ? mul_res[15] : mul_res[7];       // SF, undefined
                mc_flags_update_mask    = 2'b11;        // CF/OF only
                if (DEBUG) $display("MC_MULU: a=%x, b=%x, res=%x, flags=%x", mul_a, mul_b, mul_res[15:0], mc_flags);
            end
        end

        // ──────────────────────────────────────────────────────────────
        //  MC_DIV   – multi-cycle divide using divider module
        //            arg[0] = width  (0=8-bit, 1=16-bit)
        //            arg[1] = signed (0=DIV, 1=IDIV)
        // ──────────────────────────────────────────────────────────────
        MC_DIV: begin
            mc_wait = 1;
            // reg1_raddr = R_AX;
            // reg2_raddr = R_DX;
            // ----------- launch on first active cycle -----------------
            if (~div_busy && ~div_done) begin
                div_start    = 1'b1;
                div_width    = mc_cur.arg[0];
                div_signed   = mc_cur.arg[1];
                div_divisor  = div_width ? tmp_lo : 
                               div_signed ? { {8{tmp_lo[7]}}, tmp_lo[7:0] } :
                                            { 8'h00, tmp_lo[7:0] };
                div_dividend = div_width ? {reg2_rdata, reg1_rdata} :
                               div_signed ? { {16{reg1_rdata[15]}}, reg1_rdata} :
                                            {16'h0000, reg1_rdata};
            end
            // ----------- finished?  -----------------------------------
            if (div_done) begin
                // Set condition on divide-by-zero or overflow → INT0 by next microcode
                mc_cond = div_overflow;
                if (~div_overflow) begin
                    // normal completion → write quotient / remainder
                    if (mc_cur.arg[0]) begin    // DX = remainder, AX = quotient
                        mc_valid       = 1'b1;
                        mc_reg_valid   = 1'b1;
                        mc_reg         = R_AX;
                        mc_data        = div_quot;
                        mc_reg2_valid  = 1'b1;
                        mc_reg2        = R_DX;
                        mc_data2       = div_rem;
                    end else begin          // AH:AL = remainder:quotient
                        mc_valid       = 1'b1;
                        mc_reg_valid   = 1'b1;
                        mc_reg         = R_AX;
                        mc_data        = {div_rem[7:0], div_quot[7:0]};
                    end
                    // flags undefined – leave reg_f unchanged
                end
                mc_wait = 0;
            end
        end

        // ──────────────────────────────────────────────────────────────
        //  MC_ADJ   – BCD / ASCII adjust helper
        //             arg[2:0] selects instruction
        //                  0  DAA  (27)
        //                  1  DAS  (2F)
        //                  2  AAA  (37)
        //                  3  AAS  (3F)
        //                  4  AAM  (D4)  imm8 = e2_imm[7:0] (default 10)
        //                  5  AAD  (D5)  imm8 = e2_imm[7:0] (default 10)
        //  ─────────────────────────────────────────────────────────────
        MC_ADJ: begin
            mc_wait = 1;
            if (~adj_done) begin
                adj_req = 1'b1;
            end else begin
                mc_wait = 0;
                // ---------- write-back packet ----------------------------------
                mc_valid       = 1'b1;
                mc_reg_valid   = 1'b1;
                mc_reg         = R_AX;
                mc_data        = adj_res;
                mc_flags_valid = 1'b1;
                mc_flags   = adj_flags;
                mc_flags_update_mask = 2'b11;
                mc_cond = adj_cond;
            end
        end

        // Separate AAD µ-ops for performance
        MC_AAD: begin
            aad_req = 1'b0;
            mc_wait = 1;
            if (~aad_done) begin
                aad_req = 1'b1;
            end else begin
                mc_wait = 0;
                mc_valid = 1'b1;
                mc_reg_valid = 1'b1;
                mc_reg = R_AX;
                mc_data = {8'h00, aad_res};

                mc_flags_valid = 1'b1;
                mc_flags = reg_f;
                mc_flags[7] = aad_res[7];           // SF
                mc_flags[6] = (aad_res==8'h00);     // ZF
                mc_flags[2] = ~^aad_res;            // PF
                mc_flags[4] = 1'b0;
                mc_flags[0] = 1'b0;                 // CF, undefined
                mc_flags[11] = 1'b0;                // OF, undefined
                mc_flags_update_mask = 2'b11;
            end
        end

        MC_HALT: begin
            halted_next = 1'b1;
            mc_wait = 1'b1;          // stop mc_pc from advancing
        end

        default: ;

        endcase 

        // Gate wtth ex_valid
        if (~ex_valid) begin
            pop_next = 0;
        end

        // Gate with mc_active and br_taken
        if (~mc_active | br_taken) begin
            rd = 0;
            mc_valid = 0;
            div_start = 0;
            mc_flags_valid = 0;
            halted_next = 0;
        end
    end
end

always @(posedge clk) begin
    if (reset) begin
        halted <= 1'b0;
    end else begin
        halted <= halted_next;
    end
end


// Shadow SP for consecutive PUSH/POP µ-ops
always_ff @(posedge clk) begin
    if (reset) begin
        sp_fw_valid <= 1'b0;
    end else begin
        if (mc_active && ~br_taken && mc_reg2_valid && mc_reg2==R_SP) begin  // capture shadow SP written by µ-code
            sp_fw <= mc_data2;     
            sp_fw_valid <= 1'b1;
        end else if (ex_ucode_valid) begin
            if (reg2_raddr==R_SP & ~sp_fw_valid) begin          // enter microcode → capture current architectural SP
                sp_fw       <= reg2_rdata;
                sp_fw_valid <= 1'b1;
            end
        end else begin                 // leave microcode → drop the shadow    
            sp_fw_valid <= 1'b0;
        end
    end
end

// Latch rd_data for POP / RD_EA, mc_cond_r for TESTF
always_ff @(posedge clk) begin
    tmp_lo_reg <= tmp_lo;
    tmp_hi_reg <= tmp_hi;
    pop_reg <= pop_next;

    tmp_flags_reg <= tmp_flags;
    if (rd_ready) begin           // Data is available
        if (read_hi) tmp_hi_reg <= rd_data;  // +2 path
        if (read_lo) tmp_lo_reg <= rd_data;  // base EA / top-of-stack
        if (read_flags) tmp_flags_reg <= rd_data;  // flags
        read_lo <= 0; read_hi <= 0; read_flags <= 0;
    end
    if (mc_active &&
        ( mc_cur.op == MC_LOAD && ~mc_cur.arg[0] && (ex_modrm[7:6] != 2'b11 || mc_cur.arg[1:0] != 2'b00)  // not register form
        || mc_cur.op == MC_POP && mc_cur.arg[3:0] == 4'd0 && ~pop_reg
        || mc_cur.op == MC_GETVEC_OFF
        || mc_cur.op == MC_IN)) begin
            read_lo <= 1;
    end
    if (mc_active &&
        ( mc_cur.op == MC_LOAD && mc_cur.arg[0]
        || mc_cur.op == MC_POP && mc_cur.arg[3:0] == 4'd1 && ~pop_reg
        || mc_cur.op == MC_GETVEC_SEG)) begin
            read_hi <= 1;
    end
    if (mc_active && mc_cur.op == MC_POP && mc_cur.arg[3:0] == 4'd2) begin
        read_flags <= 1;
    end

    if (mc_active && mc_cur.op == MC_READ && ~mc_cur.arg[3]) begin
        tmp_lo_reg <= reg1_rdata;
    end

    // READ ENTER_IB -> tmp_hi
    if (mc_active && mc_cur.op == MC_READ && mc_cur.arg[3]) begin
        tmp_hi_reg <= ex_disp[23:16];
    end

    // DEC TMP_HI
    if (mc_active && mc_cur.op == MC_DEC && mc_cur.arg[3:0] == 4'd8) begin
        tmp_hi_reg <= tmp_hi_reg - 16'd1;
    end

    if (mc_active && (mc_cur.op == MC_TESTF || mc_cur.op == MC_TESTZX || 
                      mc_cur.op == MC_DIV || mc_cur.op == MC_ADJ || mc_cur.op == MC_IN_RANGE)) begin
        mc_cond_r <= mc_cond;
    end
end

// EXECUTE2 is now merged with EXECUTE1
//             if (ex_mem_rd) begin             // E is an address, so read memory
//                 e2_e_addr <= ex_e_addr_val;
//             end else begin                      // E is a register, so use the value from decode2
//                 e2_e_val_reg <= ex_e_addr_val;
//             end


////////////////////////////////////////////////////////////////////////////
// Hardwired execution
////////////////////////////////////////////////////////////////////////////

logic dest_is_reg;          // 1 = destination is a general register
logic [2:0] dest_reg;       // if dest_is_reg==1 : register index
logic dest_is_mem;          // 1 = destination is memory

logic [7:0] ex_arith_op;
logic ex_arith_carry;
logic ex_arith_is_logic;
logic ex_arith_e_is_opt1;

wire [15:0] ex_e_val = ex_mem_rd ? rd_data : ex_e_addr_val;

// decode write-back destination and others for I_ARITH and I_LOGIC
// TODO: move to decode
always_comb begin
    dest_is_reg = 1'b0;
    dest_reg = 3'b0;
    dest_is_mem = 1'b0;
    if (ex_opcode[7:6] == 2'b00) begin
        if (ex_opcode[2]) begin     // use_imm, always write back to acc
            dest_is_reg = 1'b1;
            dest_reg    = 3'b0;
            dest_is_mem = 1'b0;
        end else if (ex_opcode[1] == 1'b0) begin       // D = 0  → dest = E
            if (ex_modrm[7:6] == 2'b11) begin
                dest_is_reg = 1'b1;
                dest_reg    = ex_modrm[2:0];
            end else begin
                dest_is_mem = 1'b1;
            end
        end else begin                             // D = 1  → dest = G
            dest_is_reg = 1'b1;
            dest_reg    = ex_modrm[5:3];
        end
    end else begin
        if (ex_modrm[7:6] == 2'b11) begin
            dest_is_reg = 1'b1;
            dest_reg    = ex_modrm[2:0];
        end else begin
            dest_is_mem = 1'b1;
        end
    end

    // onehot decode of arithmetic operation
    if (ex_opcode[7])       
        ex_arith_op = 1 << ex_modrm[5:3];  // group 1
    else
        ex_arith_op = 1 << ex_opcode[5:3]; // < 40h
    ex_arith_carry = 1'b0;
    if (ex_opcode[5:4] == 2'd1 || ex_opcode[7] && ex_modrm[5:4] == 2'b01)
        ex_arith_carry = CF;               // carry for ADC/SBB
    ex_arith_is_logic = ex_opcode[7] & (ex_modrm[5:3] == 3'd1 || ex_modrm[5:3] == 3'd4 || ex_modrm[5:3] == 3'd6);  // OR=1, AND=4, XOR=6 for group 1
    ex_arith_e_is_opt1 = ex_opcode[7] | ~ex_opcode[1];
end

//--------------------------------------------------------------------
// Hardwired instructions operations
//--------------------------------------------------------------------
always @(posedge clk) begin
    logic [15:0] alu_result = ex_e_addr_val;
    logic [15:0] flags = reg_f;

    logic        load_cs_ip;  // EXECUTE2 → FETCH/SEG unit
    logic [15:0] new_cs;      // valid when load_cs_ip=1
    logic [15:0] new_ip;
    logic ex_w = ex_opcode[0];
    logic ex_d = ex_opcode[1];
    logic use_imm = ex_opcode[2];
    logic modrm_reg = ex_modrm[7:6] == 2'b11;

    logic is_cmp = 0;
    logic [15:0] res = 0;

    // defaults
    logic [15:0] hw_flags          = reg_f;
    logic        hw_flags_valid    = ex_valid & (~ex_mem_rd | rd_ready) & (~wr_valid | wr_ready);
    logic [1:0]  hw_flags_update_mask  = 2'b00;

    wb_reg_valid      <= 1'b0;
    wb_reg2_valid     <= 1'b0;
    wb_data           <= 16'h0000;

    br_taken          <= 1'b0;
    br_target         <= 'X;
    br_new_cs         <= 'X;
    br_new_ip         <= 'X;
    load_cs_ip        <= 1'b0;

    // we hold write request for multiple cycles
    // wr                <= 1'b0;
    // wr_addr           <= 'x;
    // wr_data           <= 'x;
    // wr_word           <= 1'b1;
    // wr_io             <= 1'b0;
    if (wr_ready) begin
        wr_valid      <= 1'b0;        // write request is done
        wr_io <= 0;
    end

    wb_reg            <= 3'b0;
    wb_reg2           <= 3'b0;
    wb_data2          <= 16'h0000;
    wb_width          <= 1'b1;       // default 16-bit
    wb_width2         <= 1'b1;       // default 16-bit
    wb_seg_valid      <= 1'b0;

    if (ex_valid & (~ex_mem_rd | rd_ready) & (~wr_valid | wr_ready) & ~br_taken)
    unique case (ex_iclass)

    // ------------------------------------------------------------------------
    // 1.  Arithmetic (including group 1 at 80h-83h)
    // ------------------------------------------------------------------------
    // This is critical path.
    I_ARITH: begin
        logic is_cmp = ex_arith_op[7];

        // 0: ADD, 1: OR, 2: ADC, 3: SBB, 4: AND, 5: SUB, 6: XOR, 7: CMP
        hw_flags_update_mask = 2'b11;   // default: update OF/CF
        unique case (ex_arith_op)
        8'h01: // add
            {alu_result,flags} = execute_add(ex_e_val, ex_g_val, 1'b0, ex_w);
        8'h02: // or
            alu_result = ex_e_val | ex_g_val;
        8'h04: // adc
            {alu_result,flags} = execute_add(ex_e_val, ex_g_val, ex_arith_carry, ex_w);
        8'h08: // sbb
            {alu_result,flags} = execute_sub(ex_e_val, ex_g_val, ex_arith_carry, ex_w, ex_arith_e_is_opt1);
        8'h10: // and
            alu_result = ex_e_val & ex_g_val;
        8'h20, // sub
        8'h80: // cmp
            {alu_result,flags} = execute_sub(ex_e_val, ex_g_val, 1'b0, ex_w, ex_arith_e_is_opt1);
        8'h40: // xor
            alu_result = ex_e_val ^ ex_g_val;
        default: begin
            alu_result = 'x;
        end
        endcase

        hw_flags = flags;
        if (ex_arith_is_logic) begin
            hw_flags[6] = (ex_w ? (alu_result==16'd0) : (alu_result[7:0]==8'd0));
            hw_flags[7] = ex_w ? alu_result[15] : alu_result[7];
            hw_flags[2] = ~^alu_result[7:0];
            hw_flags[0] = 1'b0;   // CF
            hw_flags[4] = 1'b0;   // AF
            hw_flags[11]= 1'b0;   // OF
        end
        // decide where to store the ALU result
        wb_reg_valid  <= ~is_cmp & dest_is_reg;
        wb_reg        <= dest_reg;
        wb_width      <= ex_w;
        wb_data       <= alu_result;

        wr_valid      <= ~is_cmp & dest_is_mem;
        wr_addr <= ex_e_fulladdr; // {ex_e_segment,4'b0} + ex_e_addr_val;
        wr_data <= alu_result;
        wr_word <= ex_w;

        if (DEBUG) $display("I_ARITH: alu_result: %h, ex_e_val: %h, ex_g_val: %h, ex_w: %b", alu_result, ex_e_val, ex_g_val, ex_w);
    end


    //--------------------------------------------------------------------
    // 2.  Logical OR/AND/XOR/NOT/NEG
    //--------------------------------------------------------------------
    I_LOGICAL: begin
        automatic logic [15:0] lsrc = logic_src();
        automatic logic [15:0] op1 = ex_e_val;
        automatic logic [15:0] op2 = ex_g_val;
        logic do_write_back = 1'b1;

        if (ex_opcode < 8'h40) begin
            // OR:08~0D, AND:20~25, XOR:30~35
            unique case (ex_opcode[5:4])       
            2'b00:  alu_result = ex_w ? (op1 | op2) :
                                        (op1[7:0] | op2[7:0]);
            2'b10:  alu_result = ex_w ? (op1 & op2) :
                                        (op1[7:0] & op2[7:0]);
            2'b11:  alu_result = ex_w ? (op1 ^ op2) :
                                        (op1[7:0] ^ op2[7:0]);
            default: alu_result = 16'hxxxx;
            endcase
            if (DEBUG) $display("alu_result: %h, op1: %h, op2: %h, ex_w: %b", alu_result, op1, op2, ex_w);
            hw_flags = calculate_flags(alu_result, 17'h0, ~ex_w, 1'b0, 1'b0);
            hw_flags[0] = 1'b0;        // CF cleared

        end else if (ex_opcode[7:6] == 2'b11 && ex_modrm[5:4] == 2'b01) begin
            // Handle NEG and NOT instructions (F6/F7.2/3)
            unique case (ex_modrm[5:3])
            3'b010: begin // NOT
                alu_result = ex_w ? ~ex_e_val : {8'h0, ~ex_e_val[7:0]};
                hw_flags = reg_f;  // NOT doesn't affect flags
            end
            3'b011: begin // NEG
                automatic logic [16:0] tmp = ex_w ? {1'b0, ex_e_val} : {9'b0, ex_e_val[7:0]};
                tmp = -tmp;
                alu_result = ex_w ? tmp[15:0] : {8'h0, tmp[7:0]};
                hw_flags  = calculate_flags(alu_result, tmp, ~ex_w, 1'b1, 1'b1);
                // manual: The CF flag set to 0 if the source operand is 0; otherwise it is set to 1. 
                // The OF, SF, ZF, AF, and PF flags are set according to the result.
                hw_flags[4] = ex_e_val[4] ^ alu_result[4];
                hw_flags[0] = tmp != 0;
                hw_flags[11] = ii.w ? ex_e_val == 16'h8000 : ex_e_val == 8'h80;  // OF
            end
            default: begin
                alu_result = 16'hxxxx;
                hw_flags   = 16'hxxxx;
            end
            endcase
        end else begin
            // TEST 84/85 (TEST E,G) / A8/A9 (TEST A,I) / F6.0/1 / F7.0/1 (TEST E,I)
            op1 = ex_e_val; 
            op2 = ex_opcode[5] ? ex_imm : ex_g_val; // A8/A9/F6.0/F7.0: IMM
            alu_result = ex_w ? (op1 & op2) : (op1[7:0] & op2[7:0]);
            if (DEBUG) $display("TEST: alu_result: %h, op1: %h, op2: %h, ex_w: %b", alu_result, op1, op2, ex_w);
            hw_flags = calculate_flags(alu_result, 17'h0, ~ex_w, 1'b0, 1'b0);
            hw_flags[0] = 1'b0;        // CF cleared
            hw_flags[4] = 1'b0;        // AF cleared
            hw_flags[11]= 1'b0;        // OF cleared
            do_write_back = 1'b0;       // TEST only affects flags
        end

        hw_flags_update_mask = 2'b11;       // Update OF and CF
        wb_reg_valid  <= dest_is_reg & do_write_back;
        wb_reg        <= dest_reg;
        wb_width      <= ex_w;
        wb_data       <= alu_result;

        wr_valid      <= dest_is_mem & do_write_back;
        wr_addr <= ex_e_fulladdr; // {ex_e_segment,4'b0} + ex_e_addr_val;
        wr_data <= alu_result;
        wr_word <= ex_w;

    end 
    //--------------------------------------------------------------------
    // 3.  Shifts / rotates  D0–D1 (by 1) , D2-D3 (by CL), C0–C1 (by imm8)
    //--------------------------------------------------------------------
    I_SHIFT: begin
        int bits   = ex_w ? 16 : 8;          // operand width in bits
        bit use_cl = ex_opcode[1];           // /2 variant
        bit use_imm = ~ex_opcode[4];         // C0-C1
        logic  [7:0] cnt_raw;
        logic        new_cf = reg_f[0];
        logic        new_of = reg_f[11];
        logic        new_af = reg_f[4];
        
        cnt_raw =   use_cl  ? ex_g_val[4:0] : 
                    use_imm ? ex_imm[4:0]   : 5'd1;

        // $display("I_SHIFT: cnt_raw: %d, ex_e_val: %h", cnt_raw, ex_e_val);

        if (ex_modrm[5]) begin   
            // SHIFTs
            // ──────────────────────────────────────────────────────────
            // SHL/SAL = 100 or 110   |  SHR = 101   |  SAR = 111
            // ──────────────────────────────────────────────────────────
            logic [4:0] cnt = shift_cnt(cnt_raw, bits);
            logic       saturated = cnt_raw > bits;
            logic [15:0] src  = ex_w ? ex_e_val : {8'h0, ex_e_val[7:0]};

            // $display("ex_modrm: %b, cnt_raw: %d, cnt: %d, bits: %d", ex_modrm, cnt_raw, cnt, bits);
            res = src;        // default “no op” when cnt==0
            if (cnt) begin
                unique case (ex_modrm[5:3])
                //--------------------------------------------------------
                3'b100,        // SHL / SAL   (identical)
                3'b110: begin
                    res      = src << cnt;
                    new_cf   = saturated ? 1'b0 : src[bits-cnt];
                    // OF defined only when cnt==1
                    new_of   = (cnt==1) ? (res[bits-1] ^ new_cf) : /*reg_f[11]*/1'b0;
                    new_af   = 0; // res[4]; AF for SHL/SHR is undefined
                end
                //--------------------------------------------------------
                3'b101: begin  // SHR (logical right)
                    res      = src >> cnt;
                    new_cf   = saturated ? 1'b0 : src[cnt-1];
                    new_of   = (cnt==1) ? src[bits-1] : /*reg_f[11]*/1'b0;
                    new_af   = 1'b0;
                end
                //--------------------------------------------------------
                3'b111: begin  // SAR (arithmetic right)
                    res      = ex_w ? $signed(src) >>> cnt : $signed(src[7:0]) >>> cnt;   // sign-extend
                    new_cf   = saturated ? (ex_w ? src[15] : src[7]) : src[cnt-1];
                    new_of   = 1'b0;                   // always cleared
                    new_af   = 1'b0;
                end
                default: ;
                endcase
            end

            // ---------------- write-back packet ------------------------
            wr_valid <= ii.e_is_m;
            wr_addr <= ex_e_fulladdr;
            wr_data <= res;
            wr_word <= ex_w;

            wb_reg_valid     <= ~ii.e_is_m;
            wb_data          <= res;
            wb_reg           <= ex_modrm[2:0];
            wb_width         <= ex_w;

            if (DEBUG) $display("I_SHIFT: res: %h, new_cf: %b, new_of: %b, new_af: %b", res, new_cf, new_of, new_af);

            // --- flag update (only if cnt != 0) ------------------------
            hw_flags_valid   = 1;
            hw_flags         = reg_f;
            if (cnt) begin
                hw_flags[0]  = new_cf;                              // CF
                hw_flags[11] = new_of;                              // OF
                // hw_flags[4]  = new_af;                              // AF
                hw_flags[7]  = ex_w ? res[15] : res[7];             // SF
                hw_flags[6]  = ex_w ? (res==16'h0000) : (res[7:0]==8'h00); // ZF
                hw_flags[2]  = ~^res[7:0];                          // PF
            end
            hw_flags_update_mask = 2'b11;   // CF & OF via mask; rest via full write
            // $display("res=%x,hw_flags: %x", res, hw_flags);

        end else begin
            // ROTATES
            logic [4:0] cnt = rotate_cnt(cnt_raw, bits, ex_modrm[4]);  // extra=1 for RCL/RCR
            if (DEBUG) $display("ROTATE: ex_e_val: %x, cnt_raw: %d, cnt: %d, bits: %d", ex_e_val, cnt_raw, cnt, bits);
            res = ex_e_val;
            if (cnt_raw) unique case (ex_modrm[5:3])
            3'b000: begin                       // ---------- ROL
                res    = rol(ex_e_val, cnt, ex_w);
                new_cf = res[0];
                new_of = (cnt==1) ? (res[bits-1] ^ new_cf) : reg_f[11];
            end
            3'b001: begin                       // ---------- ROR
                res    = ror(ex_e_val, cnt, ex_w);
                new_cf = res[bits-1];
                new_of = (cnt==1) ? (res[bits-1] ^ res[bits-2]) : reg_f[11];
            end
            3'b010: begin                       // ---------- RCL
                logic [16:0] rot = rcl_thru_cf(ex_e_val, CF, cnt, ex_w);
                res    = ex_w ? rot[15:0] : rot[7:0];
                new_cf = rot[bits];
                new_of = (cnt==1) ? (res[bits-1] ^ new_cf) : reg_f[11];
            end
            3'b011: begin                       // ---------- RCR
                logic [16:0] rot = rcr_thru_cf(ex_e_val, CF, cnt, ex_w);
                res    = ex_w ? rot[15:0] : rot[7:0];
                new_cf = rot[bits];
                new_of = (cnt==1) ? (res[bits-1] ^ res[bits-2]) : reg_f[11];
            end
            default: ;
            endcase

            // --- commit --------------------------------------------------
            wr_valid <= ii.e_is_m;
            wr_addr <= ex_e_fulladdr;
            wr_data <= res;
            wr_word <= ex_w;

            wb_reg_valid        <= ~ii.e_is_m;
            wb_reg              <= ex_modrm[2:0];
            wb_width            <= ex_w;
            wb_data             <= res;

            hw_flags_valid  = 1;
            hw_flags[0]     = new_cf;
            hw_flags[11]    = new_of;
            hw_flags_update_mask = 2'b11;
        end
    end


    //--------------------------------------------------------------------
    // 4.  INC / DEC 40-4F, FE.0/1, FF.0/1
    //--------------------------------------------------------------------
    I_INCDEC: begin
        logic is_dec = ex_opcode[7] ? ex_modrm[3] : ex_opcode[3];

        if (ii.w) begin
            alu_result = is_dec ? ex_e_val - 16'd1 : ex_e_val + 16'd1;
        end else begin
            alu_result[7:0] = is_dec ? ex_e_val[7:0] - 8'd1 : ex_e_val[7:0] + 8'd1;
            alu_result[15:8] = 0;
        end

        if (ii.e_is_m) begin
            wr_valid <= 1;
            wr_addr <= ex_e_fulladdr;
            wr_data <= alu_result;
            wr_word <= ex_w;
        end else begin
            wb_reg_valid <= 1;
            wb_reg       <= ii.e_is_op20 ? ex_opcode[2:0] : ex_modrm[2:0];
            wb_data      <= alu_result;
            wb_width     <= ii.w;
        end

        hw_flags_valid = 1;
        hw_flags    = calculate_flags(alu_result,alu_result,~ii.w,is_dec,1'b0);
        if (is_dec) begin   // dec: negative turning positive 
            hw_flags[11] = ii.w ? ~alu_result[15] & ex_e_val[15] : ~alu_result[7] & ex_e_val[7];  // OF
        end else begin      // inc: positive turning negative
            hw_flags[11] = ii.w ? alu_result[15] & ~ex_e_val[15] : alu_result[7] & ~ex_e_val[7];  // OF
        end
        hw_flags[4] = ex_e_val[4] ^ alu_result[4];    // AF
        hw_flags_update_mask = 2'b10;                // don't change CF
    end

    //--------------------------------------------------------------------
    // 5.  PUSH / POP (register & segment forms ‑ single stack access)
    //--------------------------------------------------------------------
    I_PUSHPOP: begin
        //----------------------------------------------------------------
        // 1) Identify the variant and gather operands
        //----------------------------------------------------------------
        logic is_pop      = ex_opcode[6] & ex_opcode[4] & ex_opcode[3] |   // 58 - 5F
                           ~ex_opcode[6] & ex_opcode[0] |   // 07, 17, 1F
                            ex_opcode[7];                   // 8F
        // stack pointer maths
        // logic is_seg      = ~ex_opcode[6];               // 06/0E/16/1E/07/17/1F
        logic [2:0] rsel  = ex_opcode[7] ? ex_modrm[2:0]    // 8F: POP r/m16
                                         : ex_opcode[2:0];  // 5?: e_is_op20
        logic [15:0] sp_before = ex_g_val; 
        logic [15:0] sp_after  = is_pop ? sp_before + 16'd2
                                        : sp_before - 16'd2;

        // ---- source value for PUSH;  destination for POP --------------
        logic [15:0] push_val;
        push_val = ex_e_val; // regs[rsel];
        if (rsel == R_SP) 
            push_val = sp_after;
        if (ex_opcode[7:4] == 4'h6) begin  // 68: PUSH Iw, 6A: PUSH Ib (sign-extended)
            push_val = ex_opcode[1] ? {{8{ex_imm[7]}}, ex_imm[7:0]} : ex_imm;
        end

        if (is_pop & ~ii.e_is_m & (rsel == R_SP))    // POP SP (8F C3 or 5C)
            sp_after = ex_e_val;

        //----------------------------------------------------------------
        // 2) Drive register and memory port
        //----------------------------------------------------------------
        if (is_pop) begin
            if (ii.e_is_m) begin    // POP m (8Fh)
                wr_valid <= 1;
                wr_addr <= ex_e_fulladdr;
                wr_data <= ex_e_val;
                wr_word <= 1'b1;
            end else begin
                // memory READ already issued by EXECUTE1 when ex_mem_rd=1
                // we are now in the cycle rd_busy==0, rd_data is valid
                if (ii.e_is_seg) begin
                    wb_seg_valid <= 1;
                    wb_seg       <= ex_opcode[4:3];
                    wb_seg_data  <= ex_e_val;
                end else begin
                    wb_reg_valid   <= 1;
                    wb_reg         <= rsel;
                    wb_data        <= ex_e_val;
                end
                if (DEBUG) $display("POP: wb_reg_valid=%b, wb_seg_valid=%b, wb_reg=%d, wb_data=%h", wb_reg_valid, wb_seg_valid, wb_reg, wb_data);
            end
        end else begin        // PUSH has memory write
            wr_valid <= 1;
            wr_addr <= {seg_SS,4'b0} + (is_pop ? sp_before : sp_after);
            wr_data <= push_val;
            wr_word <= 1'b1;              // word access
        end

        //----------------------------------------------------------------
        // 3) Second register port → update SP in both cases
        //----------------------------------------------------------------
        wb_reg2_valid      <= 1'b1;
        wb_reg2            <= R_SP;
        wb_data2           <= sp_after;

        if (DEBUG) $display("PUSH/POP: is_pop=%b, sp_before=%h, sp_after=%h, push_val=%h", is_pop, sp_before, sp_after, push_val);

        // flags unchanged
    end

    //--------------------------------------------------------------------
    // 6.  MOV family (many op-codes)
    //--------------------------------------------------------------------
    I_MOV: begin
        //--------------------------------------------------------
        // 1) select MOV source value
        //--------------------------------------------------------
        logic [15:0] mov_src;
        logic        src_is_seg = 1'b0;
        logic e_dest_is_reg = (ex_modrm[7:6] == 2'b11);
        logic e_dest_is_mem = ~e_dest_is_reg;

        logic is_mov_E_G    = (ex_opcode == 8'h88) || (ex_opcode == 8'h89);  // Gb -> Eb/Ev
        logic is_mov_E_Seg  = (ex_opcode == 8'h8C);                      // Seg -> Ev        

        unique case (ex_opcode)
        // reg/mem ↔ reg   (88-8B)
        8'h88,8'h89:             mov_src = ex_g_val;   // G → E
        8'h8A,8'h8B:             mov_src = ex_e_val;   // E → G
        // segment moves
        8'h8C:                   mov_src = ex_g_val;   // decode2 already put segement reg value in g_val
        // Imm → reg
        8'hB0,8'hB1,8'hB2,8'hB3,
        8'hB4,8'hB5,8'hB6,8'hB7: mov_src = {8'h00,  ex_imm[7:0]};
        8'hB8,8'hB9,8'hBA,8'hBB,
        8'hBC,8'hBD,8'hBE,8'hBF: mov_src = ex_imm;
        // Imm → mem  (C6/C7)    (imm already captured in ex_imm)
        8'hC6,8'hC7:             mov_src = ex_imm;
        // AL/AX ↔ [disp]
        8'hA0,8'hA1: begin      // memory → ACC handled via EXECUTE1
            mov_src = ex_e_val;   
            wb_width <= ex_opcode[0];
        end
        8'hA2,8'hA3: begin      // ACC → memory
            mov_src = ex_g_val;
            wb_width <= ex_opcode[0];
        end
        8'h8E: mov_src = ex_e_val;          // Ew → segment
        default: mov_src = 16'hXXXX;
        endcase

        if (DEBUG) $display("MOV: opcode=%h, modrm=%h, mov_src=%h", ex_opcode, ex_modrm, mov_src);

        //--------------------------------------------------------
        // 2) choose destination
        //--------------------------------------------------------
        unique case (ex_opcode)
        // --- Gb → Eb/Ev  (88/89), Ib/Iv → Eb/Ev (C6/C7) --------
        8'h88,8'h89,8'hC6,8'hC7: begin
            if (e_dest_is_reg) begin                 // …… register target
                wb_reg_valid <= 1'b1;
                wb_reg       <= ex_modrm[2:0];        // “E” field
                wb_width     <= ex_opcode[0];            // 0-byte, 1-word
            end else begin                           // …… memory target
                wr_valid <= 1'b1;
                wr_addr <= ex_e_fulladdr; // {ex_e_segment,4'b0} + ex_e_addr_val;
                wr_word <= ex_opcode[0];
                if (DEBUG) $display("MOV: wr_addr=%h, wr_word=%b, data=%h", wr_addr, wr_word, mov_src);
            end
        end
        // --- Gb ← Eb   or  ACC ← mem disp ----------------
        8'h8A,8'h8B: begin
            wb_reg_valid <= 1;
            wb_reg       <= ex_modrm[5:3];
            wb_width     <= ex_opcode[0];            // 0-byte, 1-word
        end
        // --- SegR → Ev   (8C /r) ----------------
        8'h8C: begin
            if (e_dest_is_reg) begin                 // …… register target
                wb_reg_valid <= 1'b1;
                wb_reg       <= ex_modrm[2:0];        // “E”
                wb_width     <= 1'b1;                // always word
            end else begin                           // …… memory target
                wr_valid <= 1'b1;
                wr_addr <= ex_e_fulladdr; // {ex_e_segment,4'b0} + ex_e_addr_val;
                wr_word <= 1'b1;
            end
        end
        // --- seg dest ---------------
        8'h8E: begin
            wb_seg_valid <= 1'b1;
            wb_seg       <= ex_modrm[4:3];   // 0=ES,1=CS,2=SS,3=DS
            wb_seg_data  <= ex_e_val;
        end
        // --- mem → ACC already handled ---------------
        8'hA0,8'hA1: begin         
            wb_reg_valid <= 1;
            wb_reg       <= R_AX;
        end
        8'hB0,8'hB1,8'hB2,8'hB3,
        8'hB4,8'hB5,8'hB6,8'hB7,
        8'hB8,8'hB9,8'hBA,8'hBB,
        8'hBC,8'hBD,8'hBE,8'hBF: begin
            wb_reg_valid <= 1;
            wb_reg       <= ex_opcode[2:0];
            wb_width     <= ex_opcode[3];
        end
        // --- mem dest ---------------
        8'hA2,8'hA3: begin
            wr_valid <= 1'b1;
            wr_addr <= ex_e_fulladdr; // {ex_e_segment, 4'b0} + ex_e_addr_val;   // ModR/M address from earlier
            wr_word <= ex_opcode[0];
            if (DEBUG) $display("mem dest: wr_addr=%x, wr_data=%x", {ex_e_segment, 4'b0} + ex_e_addr_val, mov_src);
        end
        default: ;
        endcase

        //--------------------------------------------------------
        // 3) finalise write-back packet
        //--------------------------------------------------------
        wb_data  <= mov_src;
        wr_data  <= mov_src;
        // flags unchanged -> do nothing with reg_f

    end

    //--------------------------------------------------------------------
    // 7.  Loop instructions E0-E2
    //--------------------------------------------------------------------
    I_LOOP: begin
        automatic logic cond;

        // Decrement CX and write it back
        alu_result     = ex_g_val - 16'd1;
        wb_reg_valid   <= 1;
        wb_reg         <= R_CX;
        wb_data        <= alu_result;

        // branch decision
        cond           = (alu_result != 16'd0);     // LOOP family needs CX≠0
        if (ex_opcode==8'hE1) cond &= ZF;           // LOOPE/Z
        if (ex_opcode==8'hE0) cond &= ~ZF;          // LOOPNE/NZ

        br_taken       <= cond;
        // signed 8-bit displacement in imm8 is already sign-extended by decoder
        br_new_cs      <= seg_CS;
        br_new_ip      <= ex_ip_after + {{12{ex_imm[7]}}, ex_imm[7:0]};   // TODO: Move to decode
        br_target      <= {seg_CS, 4'b0} + ex_ip_after + {{12{ex_imm[7]}}, ex_imm[7:0]};
    end


    //--------------------------------------------------------------------
    // 8.  Jump instructions
    //--------------------------------------------------------------------
    I_JUMP: begin
        automatic logic is_jmp_short   = (ex_opcode == 8'hEB);
        automatic logic is_jmp_near    = (ex_opcode == 8'hE9);
        automatic logic is_jmp_far     = (ex_opcode == 8'hEA);
        logic [15:0] target_off = 'X;
        logic [15:0] target_seg = 'X;

        if (ex_valid) begin
            // 1) Resolve new IP / CS
            if (is_jmp_short) begin
                target_off = ex_ip_after + {{8{ex_imm[7]}}, ex_imm[7:0]};
                target_seg = seg_CS;
            end else if (is_jmp_near) begin
                target_off = ex_ip_after + ex_imm;   // 16-bit signed add
                target_seg = seg_CS;
            end else if (is_jmp_far) begin
                target_off = ex_disp[15:0];          // low 16 bits already latched
                target_seg = ex_disp[31:16];         // upper word came in disp field
            end
            br_taken   <= 1'b1;
            br_target  <= {target_seg, 4'b0} + target_off;       // 24-bit addition
            br_new_cs  <= target_seg;
            br_new_ip  <= target_off;
        end
    end

    //--------------------------------------------------------------------
    // 9. I/O  (IN / OUT)
    //--------------------------------------------------------------------
    I_IO: begin
        logic       w    = ex_opcode[0];                 // 0 = byte, 1 = word
        logic       inop = (ex_opcode==8'hE4)||(ex_opcode==8'hE5)||
                           (ex_opcode==8'hEC)||(ex_opcode==8'hED);

        logic [15:0] port = ((ex_opcode==8'hE4)||(ex_opcode==8'hE5)||
                             (ex_opcode==8'hE6)||(ex_opcode==8'hE7))
                             ? {8'h00,ex_imm[7:0]}          // imm-port
                             : ex_g_val;                    // DX-port

        if (inop) begin        // ----------  IN ----------
            wb_reg_valid  <= 1'b1;
            wb_reg        <= R_AX;
            wb_width      <= w;
            wb_data       <= ex_e_val;                    // AX. TODO: handle w==0
        end else begin         // ----------  OUT ----------
            wr_valid      <= 1'b1;
            wr_addr       <= {4'h0, port};
            wr_word       <= w;
            wr_io         <= 1'b1;                           // <── mark I/O
            wr_data       <= w ? ex_e_val                    // AX / AL
                            : {8'h00, ex_e_val[7:0]};
            wr_word       <= w;
        end
    end

    //------------------------------------------------------------------
    // 10. Jcc  (70-7F, aliased 60-6F, 8-bit signed displacement)  +  JCXZ (E3)
    //------------------------------------------------------------------
    I_JCC: begin
        automatic logic is_jcc  = ~ex_opcode[7];     // 16 conditions, 8-bit disp
        automatic logic [3:0] cc = ex_opcode[3:0];   // condition selector 0-F
        // -------------------------------------------------  JCXZ  (E3  short, CX==0)
        automatic logic is_jcxz = ex_opcode[7];

        // -------------------------------- 1) Evaluate condition
        logic cond;
        logic [15:0] target_off;

        if (DEBUG) $display("JCC: %x", ex_opcode);

        if (is_jcxz) begin
            cond = (ex_g_val == 16'd0);   // CX
        end
        else begin
            unique case (cc)
            4'h0: cond =  OF;                                // JO
            4'h1: cond = ~OF;                                // JNO
            4'h2: cond =  CF;                                // JB/JNAE/JC
            4'h3: cond = ~CF;                                // JNB/JAE/JNC
            4'h4: cond =  ZF;                                // JE/JZ
            4'h5: cond = ~ZF;                                // JNE/JNZ
            4'h6: cond =  CF |  ZF;                          // JBE/JNA
            4'h7: cond = ~CF & ~ZF;                          // JA/JNBE
            4'h8: cond =  SF;                                // JS
            4'h9: cond = ~SF;                                // JNS
            4'hA: cond =  PF;                                // JP/JPE
            4'hB: cond = ~PF;                                // JNP/JPO
            4'hC: cond =  SF ^ OF;                           // JL/JNGE
            4'hD: cond = ~(SF ^ OF);                         // JGE/JNL
            4'hE: cond =  ZF | (SF ^ OF);                    // JLE/JNG
            4'hF: cond = ~ZF & ~(SF ^ OF);                   // JG/JNLE
            endcase
        end

        // -------------------------------- 2) Branch target
        target_off  = ex_ip_after + {{8{ex_imm[7]}}, ex_imm[7:0]}; // sign-extend disp

        br_taken    <= cond;
        br_target   <= {seg_CS, 4'b0} +  target_off ;   // still physical 20-bit because FETCH flattens
        br_new_cs   <= seg_CS;
        br_new_ip   <= target_off;
        /* No flag change, no register/memory write */
    end

    //------------------------------------------------------------------
    // 11. LEA r16, m16  (8D) 
    //------------------------------------------------------------------
    I_LEA: begin
        wb_reg_valid   <= 1'b1;
        wb_reg         <= ex_modrm[5:3];     // destination reg = "G"
        wb_data        <= ex_e_addr_val;     // address computed earlier
        // flags unchanged
    end

    //------------------------------------------------------------------
    // 12. XLAT  (D7)  
    //------------------------------------------------------------------
    I_XLAT: begin
        wb_reg_valid   <= 1'b1;
        wb_reg         <= R_AX;
        wb_data        <= {ex_g_val[15:8], ex_e_val[7:0]};  // G is AX, E is [BX+AL]
    end

    //------------------------------------------------------------------
    // 13. XCHG family  (86/87, 90–97) 
    //------------------------------------------------------------------
    I_XCHG: begin
        automatic logic       is_regreg = (ex_opcode[7:4]==4'h9) | (ex_modrm[7:6]==2'b11);
        automatic logic [2:0] reg_g     = (ex_opcode[7:4]==4'h9) ? R_AX           : ex_modrm[5:3];
        automatic logic [2:0] reg_e     = (ex_opcode[7:4]==4'h9) ? ex_opcode[2:0] : ex_modrm[2:0];
        logic w = ex_opcode[4] ? 1'b1 : ex_w;

        // --- port 1 --------------------------
        wb_width         <= w;
        if (is_regreg) begin
            wb_reg_valid <= 1;
            wb_reg       <= reg_e;
            wb_data      <= ex_g_val;     
        end else begin
            wr_valid     <= 1;
            wr_addr      <= ex_e_fulladdr; // {ex_e_segment, 4'b0} + ex_e_addr_val;
            wr_data      <= ex_g_val;
            wr_word      <= w;
        end

        // --- port 2 --------------------
        wb_reg2_valid    <= 1'b1;
        wb_reg2          <= reg_g;
        wb_data2         <= ex_e_val;
        wb_width2        <= w;

        if (ex_opcode != 8'h90) begin
            if (DEBUG) $display("XCHG: %x, is_regreg: %b, reg_g: %d, reg_e: %d, wr_addr: %h, wr_data: %h, wr_word: %b", ex_opcode, is_regreg, reg_g, reg_e, wr_addr, wr_data, wr_word);
            if (DEBUG) $display("ex_g_val: %h, ex_e_val: %h", ex_g_val, ex_e_val);
        end
    end

    //------------------------------------------------------------------
    // 14.  Sign-extension helpers  (CBW 98h,  CWD 99h)
    //------------------------------------------------------------------
    I_CXD: begin
        automatic logic is_cwd = (ex_opcode==8'h99);

        wb_reg_valid   <= 1'b1;

        if (is_cwd) begin            // CWD  -> DX ← sign(AX)
            wb_reg  <= R_DX;
            wb_data <= ex_e_val[15] ? 16'hFFFF : 16'h0000;
        end else begin               // CBW  -> AX ← sign(AL)
            wb_reg  <= R_AX;
            wb_data <= { {8{ex_e_val[7]}}, ex_e_val[7:0] };
        end
    end

    //------------------------------------------------------------------
    // 15.  Simple flag-flip instructions
    //      CLC F8 | STC F9 | CLI FA | STI FB | CLD FC | STD FD
    //------------------------------------------------------------------
    I_FLAGS: begin
        hw_flags_valid = 1'b1;
        hw_flags = reg_f;           // start from old
        hw_flags_update_mask = 2'b00;    // other flag bits unchanged
        unique case (ex_opcode)
            8'hF5: begin
                hw_flags[0]  = ~CF;  // CMC – CF ← ~CF
                hw_flags_update_mask = 2'b01;
            end
            8'hF8: begin
                hw_flags[0]  = 1'b0;  // CLC – CF ← 0
                hw_flags_update_mask = 2'b01;
            end
            8'hF9: begin
                hw_flags[0]  = 1'b1;  // STC – CF ← 1
                hw_flags_update_mask = 2'b01;
            end
            8'hFA: begin
                hw_flags[9]  = 1'b0;  // CLI – IF ← 0
            end
            8'hFB: begin
                hw_flags[9]  = 1'b1;  // STI – IF ← 1
            end
            8'hFC: begin
                hw_flags[10] = 1'b0;  // CLD – DF ← 0
            end
            8'hFD: begin
                hw_flags[10] = 1'b1;  // STD – DF ← 1
            end
            default: ;
        endcase
        // $display("FLAGS: %x, hw_flags: %h, hw_flags_update_mask: %b", ex_op, hw_flags, hw_flags_update_mask);
        /* no register/memory write */
    end

    //------------------------------------------------------------------
    // 16.  LAHF (9F)   and   SAHF (9E)
    //------------------------------------------------------------------
    I_LSAHF: begin
        if (ex_opcode[0]) begin              // -------- LAHF
            // AH ← SF:ZF:0:AF:0:PF:1:CF
            automatic logic [7:0] ah_new = { SF, ZF, 1'b0, AF, 1'b0, PF, 1'b1, CF };
            wb_reg_valid   <= 1'b1;
            wb_reg         <= R_AX;             // AX is register index 0
            wb_data        <= { ah_new , ex_e_val[7:0] };
        end
        else begin                           // -------- SAHF
            hw_flags_valid = 1'b1;
            hw_flags         = reg_f;
            hw_flags[7]      = ex_e_val[15]; // SF   (AH bit 7)
            hw_flags[6]      = ex_e_val[14]; // ZF   (bit 6)
            hw_flags[4]      = ex_e_val[12]; // AF   (bit 4)
            hw_flags[2]      = ex_e_val[10]; // PF   (bit 2)
            hw_flags[0]      = ex_e_val[8];  // CF   (bit 0)
            hw_flags_update_mask  = 2'b01;   // Affects CF but not OF
        end
    end

    // NOP and everything
    default: begin
        hw_flags_valid = 1'b0;
    end

    endcase

    if (hw_flags_valid) begin
        reg_f <= {hw_flags[15:12],
                  hw_flags_update_mask[1] ? hw_flags[11] : reg_f[11],
                  hw_flags[10:1],
                  hw_flags_update_mask[0] ? hw_flags[0] : reg_f[0]};
        if (DEBUG) $display("FLAGS: %b, hw_flags: %h, hw_flags_update_mask: %b", hw_flags_valid, hw_flags, hw_flags_update_mask);
    end

    //------------------------------------------------------------------
    // forward microcode-driven signals
    //------------------------------------------------------------------
    if (ex_ucode_valid) begin          // Microcode-specific initializations
        hw_flags_valid = 1'b0;
        // wr_valid <= 0;
    end
    if (mc_valid) begin
        wb_reg_valid   <= mc_reg_valid;
        wb_reg         <= mc_reg;
        wb_data        <= mc_data;
        wb_width       <= mc_width;

        wb_seg_valid   <= mc_seg_valid;
        wb_seg         <= mc_seg;
        wb_seg_data    <= mc_seg_data;

        wb_reg2_valid  <= mc_reg2_valid;
        wb_reg2        <= mc_reg2;
        wb_data2       <= mc_data2;
        wb_width2      <= mc_width2;

        // hw_flags_valid <= mc_flags_valid;
        // hw_flags       <= mc_flags;
        // hw_flags_update_mask <= mc_flags_update_mask;
        if (mc_flags_valid) begin
            reg_f <= {mc_flags[15:12],
                      mc_flags_update_mask[1] ? mc_flags[11] : reg_f[11],
                      mc_flags[10:1],
                      mc_flags_update_mask[0] ? mc_flags[0] : reg_f[0]};
        end

        wr_valid  <= mc_wr;         // turn pulse into hold request
        wr_addr <= mc_wr_addr;
        wr_data <= mc_wr_data;
        wr_word <= mc_wr_word;
        wr_io <= mc_wr_io;

        br_taken    <= mc_load_cs_ip;
        br_target   <= {mc_new_cs, 4'b0} + mc_new_ip;
        br_new_ip   <= mc_new_ip;
        br_new_cs   <= mc_new_cs;
        if (DEBUG) $display("mc_valid: %b, mc_reg_valid: %b, mc_reg: %d, mc_data: %h, mc_wr: %b, mc_wr_addr: %h, mc_wr_data: %h, mc_wr_word: %b, mc_reg2_valid: %b, mc_reg2: %d, mc_data2: %h, mc_flags: %h, mc_flags_update_mask: %b", mc_valid, mc_reg_valid, mc_reg, mc_data, mc_wr, mc_wr_addr, mc_wr_data, mc_wr_word, mc_reg2_valid, mc_reg2, mc_data2, mc_flags, mc_flags_update_mask);
        if (DEBUG) $display("load_cs_ip: %b, new_cs: %h, new_ip: %h", mc_load_cs_ip, mc_new_cs, mc_new_ip);
    end

    // debug interface
    if (reg_wr) begin
        if (reg_addr == 4'd13) begin
            reg_f <= reg_din;
        end
    end

    // reset flags
    // if (reset) begin
        // reg_f <= FLAGS_INIT;
    // end
end

reg wr_valid_r;
always @(posedge clk) begin
    if (wr_valid & ~wr_valid_r) begin
        if (DEBUG) $display("MEM_WR: wr_addr: %h, wr_data: %h, wr_word: %x", wr_addr, wr_data, wr_word);
    end
    wr_valid_r <= wr_valid;
end

//--------------------------------------------------------------------
// Helper functions
//--------------------------------------------------------------------
// Do ADD operation. Returns result and flags
function automatic [31:0] execute_add(
    input [15:0] e_val,
    input [15:0] g_val,
    input carry,
    input width
);
    reg [16:0] sum;
    reg [15:0] result;
    reg [15:0] flags = reg_f;
    reg aux_flag, overflow_flag;
    
    sum = width ? {1'b0, e_val} + {1'b0, g_val} + {16'b0, carry}: 
            {9'b0, e_val[7:0]} + {9'b0, g_val[7:0]} + {16'b0, carry};
    aux_flag = e_val[4] ^ g_val[4] ^ sum[4];
    overflow_flag = width ? (e_val[15] == g_val[15] && e_val[15] != sum[15]) :
                            (e_val[7] == g_val[7] && e_val[7] != sum[7]);
    
    result = width ? sum[15:0] : {8'h00, sum[7:0]};
    flags = calculate_flags(result, sum, ~width, 0, 1);
    flags[4] = aux_flag;
    flags[11] = overflow_flag;

    return {result, flags};
endfunction

// Do SUB operation and update flags
function automatic [31:0] execute_sub(
    input [15:0] e_val,
    input [15:0] g_val,
    input borrow,
    input width,
    input e_is_op1          // 1: e-g, 0: g-e
);
    reg [16:0] diff;
    reg [15:0] result;
    reg [15:0] flags = reg_f;
    reg aux_flag;
    reg overflow_flag;

    if (e_is_op1) begin // E - G
        diff = width ? {1'b0, e_val} - {1'b0, g_val} - {16'b0, borrow}: 
                {9'b0, e_val[7:0]} - {9'b0, g_val[7:0]} - {16'b0, borrow};
        aux_flag = e_val[4] ^ g_val[4] ^ diff[4];
        overflow_flag = width ? (e_val[15] != g_val[15] && e_val[15] != diff[15]) :
                                (e_val[7] != g_val[7] && e_val[7] != diff[7]);
        if (DEBUG) $display("e_is_op1: e_val: %h, g_val: %h, diff: %h, aux_flag: %b, overflow_flag: %b", e_val, g_val, diff, aux_flag, overflow_flag);
    end else begin     // G - E
        diff = width ? {1'b0, g_val} - {1'b0, e_val} - {16'b0, borrow}: 
                {9'b0, g_val[7:0]} - {9'b0, e_val[7:0]} - {16'b0, borrow};
        aux_flag = g_val[4] ^ e_val[4] ^ diff[4];
        overflow_flag = width ? (g_val[15] != e_val[15] && g_val[15] != diff[15]) :
                                (g_val[7] != e_val[7] && g_val[7] != diff[7]);
    end
    // $display("e_is_op1: %b", e_is_op1);
    
    result = width ? diff[15:0] : {8'h00, diff[7:0]};
    flags = calculate_flags(result, diff, ~width, 1, 1);
    flags[4] = aux_flag;
    flags[11] = overflow_flag;
    return {result, flags};
endfunction

// Flag calculation helper functions
// This clears aux and overflow flags
function automatic [15:0] calculate_flags(
    input [15:0] result,
    input [16:0] adder_result,
    input is_8bit,
    input is_subtract,
    input update_carry
);
    reg [7:0] byte_result = result[7:0];
    calculate_flags = reg_f;

    // Zero Flag
    calculate_flags[6] = (is_8bit ? (result[7:0] == 0) : (result == 0));
    
    // Sign Flag
    calculate_flags[7] = is_8bit ? result[7] : result[15];
    
    // Carry Flag (for addition/subtraction)
    if (update_carry) begin
        calculate_flags[0] = is_8bit ? adder_result[8] : adder_result[16];
        // if (is_subtract) // Borrow is inverse of carry
        // calculate_flags[0] = ~calculate_flags[0]; 
    end

    // Parity Flag (even parity of low 8 bits)
    calculate_flags[2] = ~^byte_result;

    // Clear aux and overflow flag by default
    calculate_flags[4] = 1'b0;
    calculate_flags[11] = 1'b0;
endfunction

function automatic [15:0] logic_src();
    logic_src = ex_modrm[5:3]==3'b000 ? ex_e_val :
                ex_modrm[5:3]==3'b001 ? ex_g_val :
                ex_modrm[5:3]==3'b010 ? ex_imm :
                16'hXXXX;
endfunction

// width  = 8 or 16
// extra  = 0 for ROL/ROR , 1 for RCL/RCR   (because of the carry bit)
function automatic [4:0] rotate_cnt
(
    input  [7:0] raw_cnt,
    input  int   width,
    input  bit   extra
);
    int mod = width + extra;      // 8/16  or  9/17
    rotate_cnt = raw_cnt ? (raw_cnt[4:0] % mod) : 5'd0;
endfunction

// ---------- plain ROL / ROR -----------------------------------------
function automatic [15:0] ror
(
    input [15:0] d,
    input [4:0]  n,       // 0-8, 0-16
    input        w
);
    if (w) begin
        logic [31:0] dup = {d,d};
        ror = dup[n +: 16];
    end else begin
        logic [15:0] dup = {d[7:0],d[7:0]};
        ror = dup[n +: 8];
    end
endfunction

function automatic [16:0] rol
(
    input [15:0] d,
    input [4:0]  n,  // 0-9 or 0-17
    input int    w
);
    rol = ror(d, w ? 16-n : 8-n, w);
endfunction

// ---------- through-carry versions ----------------------------------
// CF is bit width of the vector
function automatic [16:0] rcr_thru_cf
(
    input [15:0] d, 
    input        cf,
    input [4:0]  n,       // 0-9 or 0-17
    input int    w
);
    if (w) begin
        logic [33:0] dup = {cf,d,cf,d};
        rcr_thru_cf = dup[n +: 17];
    end else begin
        logic [17:0] dup = {cf,d[7:0],cf,d[7:0]};
        rcr_thru_cf = dup[n +: 9];
    end
endfunction

function automatic [17:0] rcl_thru_cf
(
    input [15:0] d,
    input        cf,
    input [4:0]  n,    // 0-9 or 0-17
    input int    w
);
    rcl_thru_cf = rcr_thru_cf(d, cf, w ? 17-n : 9-n, w);
endfunction

// Mask to 0-31, but saturate to width (8 or 16).  
// If raw_cnt == 0  ⇒  no flags change, no result change.
// If raw_cnt > width ⇒ treat as ‘width’.
function automatic [4:0] shift_cnt
(
    input [7:0] raw_cnt,
    input int   width
);
    if (raw_cnt == 0)        shift_cnt = 5'd0;
    else if (raw_cnt > width)shift_cnt = width[4:0];
    else                     shift_cnt = raw_cnt[4:0];
endfunction

`ifdef VERILATOR
// Count of instructions retired for CPI calculation
logic [31:0] retire_counter /* verilator public */;
always @(posedge clk) begin
    if (reset) 
        retire_counter <= 32'd0;
    else if (ex_valid & ex_ready) 
        retire_counter <= retire_counter + 1;
end
`endif

endmodule
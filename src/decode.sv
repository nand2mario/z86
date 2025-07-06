// Main (2nd-stage) instruction decoder
//
// This handles effective address calculation, microcode entry point selection, 
// hardwired instruction class decoding, register reads and memory load launches.
import z86_package::*;

module decode (
    input             clk,
    input             reset,

    // upstream (fetch) interface
    output            id_ready,         // instruct upstream (fetch) to send instructions
    input             id_valid,         // upstream (fetch) data is valid

    input      [3:0]  id_len,
    input      [15:0] id_ip_after,      // IP after this instruction
    input      [6:0]  id_prefix,        // the fetched instruction fields, PREFIX_* in z486_package.sv
    input      [7:0]  id_opcode,
    input             id_modrm_valid,
    input      [7:0]  id_modrm,
    input             id_disp_valid,
    input      [31:0] id_disp,          // 8 or 16-bit displacement, or 32-bit pointer
    input             id_imm_valid,
    input      [15:0] id_imm,           // 8 or 16-bit immediate

    input             id_disp8,
    input             id_disp16,
    input             id_disp32,        // pointer
    input             id_imm8,
    input             id_imm16,
    input             id_two_byte,

    input id_instruction id_inst,       // bus of control signals from fetch

    input             br_taken,

    // downstream (execute) interface
    input             ex_ready,         
    output reg        ex_valid,             // instruction output is valid
    output reg [15:0] ex_ip_after,
    output reg  [7:0] ex_opcode,
    output reg [16:0] ex_iclass,            // decoded hardwired instruction class
    output reg  [7:0] ex_modrm,
    output reg [15:0] ex_imm,
    output reg [31:0] ex_disp,
    output reg        ex_mem_rd,            // 1: memory read is ongoing for hardwired instruction
    output reg [15:0] ex_e_addr_val,        // E: effective address, or register value
    output reg [19:0] ex_e_fulladdr,        // E: full address
    output reg [15:0] ex_e_segment,
    output reg [15:0] ex_g_val,             // G: register value
    output reg        ex_ucode_valid,       // This instruction uses microcode
    output id_instruction ex_inst,

    output reg [8:0]  ex_ucode_entry_next,  // for next instruction

    // register file interface
    output reg [2:0]  reg1_raddr,
    input      [15:0] reg1_rdata,
    output reg [2:0]  reg2_raddr,
    input      [15:0] reg2_rdata,

    // other registers
    input      [15:0] seg_CS,
    input      [15:0] seg_SS,
    input      [15:0] seg_DS,
    input      [15:0] seg_ES,
    input      [15:0] reg_f,
    input      [15:0] reg_ip,

    // memory read interface
    output reg        rd,
    output reg [19:0] rd_addr,
    output reg        rd_io,
    output reg        rd_word,
    input      [15:0] rd_data,
    input             rd_ready,

    // CPU interrupt interface
    input             intr,             // maskable interrupt request (8259A INT)
    output            intra,            // INT-Acknowledge pulse to 8259A
    input             nmi,              // non-maskable interrupt (NMI) 
    input       [7:0] pic_vec,          // interrupt vector from PIC during INTA cycle

    // debug interface (only for verilator)
    input             reg_rd,
    input    [3:0]    reg_addr,
    output reg [15:0] reg_dout
);

id_instruction ii;  // just for shorter names
assign ii = id_inst;

reg [15:0] ex_g_val_next;
logic cycle_2nd;    // 2nd cycle of 2-cycle decode

// DECODE signals
wire [15:0] disp = {id_modrm[7] ? id_disp[15:8] : {8{id_disp[7]}}, id_disp[7:0]};

// mask of registers *read* this decode cycle
reg_mask_t rd_mask;
assign rd_mask = build_rd_mask();

// hazard if either scoreboard entry wants to write what we read
wire hazard = (sb0_valid & |(sb0_mask & rd_mask)) |
              (~CONFIG_FORWARDING_REGFILE & sb1_valid & |(sb1_mask & rd_mask));

assign id_ready = (~ex_valid | ex_ready) & (~id_valid |~ii.is_two_cycles | cycle_2nd) & ~hazard & (~id_mem_rd | rd_ready);

logic id_void_r;
wire id_void = (br_taken /*| int_pipeline_inject*/) | id_void_r;

// emit when signals from fetch are valid and we are ready
// when br_taken is 1, this instruction will be cleared
wire emit = id_ready & id_valid & ~id_void;

// inject nmi-based interrupt into the pipeline
reg nmi_latched, intr_latched;
reg start_int_service;
wire int_pipeline_inject = (nmi_latched | intr_latched) & emit;  // inject at an valid instruction boundary

// recognize simple POP (no ModR/M)
wire id_pop_r16 = (id_opcode >= 8'h58) && (id_opcode <= 8'h5F);
wire id_pop_seg = (id_opcode == 8'h07) || (id_opcode == 8'h17) || (id_opcode == 8'h1F);
wire id_pop_rm = (id_opcode == 8'h8F);

// ---- segment selection ------------------------------------
reg [15:0] id_e_segment;
// 1) recognise an explicit segment-override prefix
wire seg_ovr_cs = id_prefix[PREFIX_CS];
wire seg_ovr_ss = id_prefix[PREFIX_SS];
wire seg_ovr_ds = id_prefix[PREFIX_DS];
wire seg_ovr_es = id_prefix[PREFIX_ES];
wire seg_ovr    = |id_prefix[6:3];

// 2) does the 16-bit EA formula contain BP?
wire ea_uses_bp = id_modrm_valid &&
              (  (id_modrm[2:0] == 3'b010) |                 // BP+SI
                 (id_modrm[2:0] == 3'b011) |                 // BP+DI
                ((id_modrm[2:0] == 3'b110) &&
                 (id_modrm[7:6] != 2'b00)) );                // [BP] or BP+disp

reg id_mem_rd;
wire is_io_in = (id_opcode == 8'hE4 || id_opcode == 8'hE5 || id_opcode == 8'hEC || id_opcode == 8'hED);

// ─── 2-deep scoreboard ──────────────────────────────────────────────────────
reg             sb0_valid, sb1_valid;         // 1 ⇢ entry is occupied
reg_mask_t      sb0_mask , sb1_mask ;         // bit-mask of destination regs
wire            sb_shift = ex_ready;          // EXECUTE → WRITE-BACK
reg_mask_t      wr_mask;                      // mask built for *this* instr.
assign wr_mask = build_wr_mask();

//================================================================
// Microcode entry point selection
//================================================================

`include "ucode_entry.svh"

logic [8:0] id_ucode_entry;
logic id_ucode_valid;

assign ex_ucode_entry_next = id_ucode_entry;

// Entry points for microcoded instructions
always_comb begin
    id_ucode_valid = 1'b1;
    id_ucode_entry = UENTRY_NOP;
    casez ({id_two_byte, id_opcode})
        8'h27: id_ucode_entry = UENTRY_DAA;
        8'h2F: id_ucode_entry = UENTRY_DAS;
        8'h37: id_ucode_entry = UENTRY_AAA;
        8'h3F: id_ucode_entry = UENTRY_AAS;
        8'hD4: id_ucode_entry = UENTRY_AAM;
        8'hD5: id_ucode_entry = UENTRY_AAD;

        // 8'h8F: id_ucode_entry = id_modrm[7:6] == 2'b11 ? UENTRY_POP_R16 : UENTRY_POP_M16;

        8'h9A: id_ucode_entry = UENTRY_CALL_FAR;
        8'h9C: id_ucode_entry = UENTRY_PUSHF;
        8'h9D: id_ucode_entry = UENTRY_POPF;

        8'hA4: id_ucode_entry = UENTRY_MOVSB;               // String instructions
        8'hA5: id_ucode_entry = UENTRY_MOVSW;
        8'hA6: id_ucode_entry = UENTRY_CMPSB;
        8'hA7: id_ucode_entry = UENTRY_CMPSW;
        8'hAA: id_ucode_entry = UENTRY_STOSB;
        8'hAB: id_ucode_entry = UENTRY_STOSW;
        8'hAC: id_ucode_entry = UENTRY_LODSB;
        8'hAD: id_ucode_entry = UENTRY_LODSW;
        8'hAE: id_ucode_entry = UENTRY_SCASB;
        8'hAF: id_ucode_entry = UENTRY_SCASW;

        8'hC2: id_ucode_entry = UENTRY_RET_NEAR_IMM;        // RETN Iw
        8'hC3: id_ucode_entry = UENTRY_RET_NEAR;            // RETN
        8'hC4: id_ucode_entry = UENTRY_LES;
        8'hC5: id_ucode_entry = UENTRY_LDS;

        8'hCB: id_ucode_entry = UENTRY_RET_FAR;             // RETF
        8'hCA: id_ucode_entry = UENTRY_RET_FAR_IMM;         // RETF Iw
        8'hCC: id_ucode_entry = UENTRY_INT3;                 
        8'hCD: id_ucode_entry = UENTRY_INT_IMM;             // INT Ib
        8'hCE: id_ucode_entry = UENTRY_INTO;                 
        8'hCF: id_ucode_entry = UENTRY_IRET;                 

        8'hE8: id_ucode_entry = UENTRY_CALL_NEAR;

        8'hF1: id_ucode_entry = UENTRY_INT1;                 
        8'hF4: id_ucode_entry = UENTRY_HLT;
        8'hF6: case (id_modrm[5:3])                          // Group 3a
                3'b100: id_ucode_entry = UENTRY_MUL8;
                3'b101: id_ucode_entry = UENTRY_IMUL8;
                3'b110: id_ucode_entry = UENTRY_DIV8;
                3'b111: id_ucode_entry = UENTRY_IDIV8;
                default: id_ucode_valid = 1'b0;              // F6.0/1/2/3 (TEST, NOT, NEG) are hardwired
            endcase
        8'hF7: case (id_modrm[5:3])                          // Group 3b
                3'b100: id_ucode_entry = UENTRY_MUL16;
                3'b101: id_ucode_entry = UENTRY_IMUL16;
                3'b110: id_ucode_entry = UENTRY_DIV16;
                3'b111: id_ucode_entry = UENTRY_IDIV16;
                default: id_ucode_valid = 1'b0;              // F7.0/1/2/3 (TEST, NOT, NEG) are hardwired
            endcase

        8'hFF: case (id_modrm[5:3])                          // Group 5
            3'b010: id_ucode_entry = UENTRY_CALL_RM;         // CALL Ev
            3'b011: id_ucode_entry = UENTRY_CALL_EP;         
            3'b100: id_ucode_entry = UENTRY_JMP_RM;          // JMP Ev
            3'b101: id_ucode_entry = UENTRY_JMP_EP;
            3'b110,3'b111: id_ucode_entry = id_modrm[7:6] == 2'b11 ? UENTRY_PUSH_R16 : UENTRY_PUSH_M16;
            default: id_ucode_valid = 1'b0;                  // FF.0/1 (INC/DEC) are hardwired
        endcase

        // 80286 instructions
        8'h60: id_ucode_entry = UENTRY_PUSHA;                // PUSHA
        8'h61: id_ucode_entry = UENTRY_POPA;                 // POPA
        8'h62: id_ucode_entry = UENTRY_BOUND;                // BOUND Ev,Gv
        8'h63: id_ucode_entry = UENTRY_ARPL;                 // ARPL Wr,Ev
        8'h69: id_ucode_entry = UENTRY_IMUL_IW;              // IMUL Ev,Gv,Iw
        8'h6B: id_ucode_entry = UENTRY_IMUL_IB;              // IMUL Ev,Gv,Ib
        8'h6C: id_ucode_entry = UENTRY_INSB;                 // INSB
        8'h6D: id_ucode_entry = UENTRY_INSW;                 // INSW
        8'h6E: id_ucode_entry = UENTRY_OUTSB;                // OUTSB
        8'h6F: id_ucode_entry = UENTRY_OUTSW;                // OUTSW
        8'hC8: id_ucode_entry = UENTRY_ENTER;                // ENTER Iw,Ib
        8'hC9: id_ucode_entry = UENTRY_LEAVE;   

/*
        9'h100: case (id_modrm[5:3])                        
            3'd0: id_ucode_entry = UENTRY_SLDT;              // Store Local Descriptor Table
            3'd1: id_ucode_entry = UENTRY_STR;               // Store Task Register
            3'd2: id_ucode_entry = UENTRY_LLDT;              // Load Local Descriptor Table
            3'd3: id_ucode_entry = UENTRY_LTR;               // Load Task Register
            3'd4: id_ucode_entry = UENTRY_VERR;              // Verify Read
            3'd5: id_ucode_entry = UENTRY_VERW;              // Verify Write
            default: id_ucode_valid = 1'b0;
        endcase

        9'h101: case (id_modrm[5:3])
            3'd0: id_ucode_entry = UENTRY_SGDT;              // Store Global Descriptor Table
            3'd1: id_ucode_entry = UENTRY_SIDT;              // Store Interrupt Descriptor Table
            3'd2: id_ucode_entry = UENTRY_LGDT;              // Load Global Descriptor Table
            3'd3: id_ucode_entry = UENTRY_LIDT;              // Load Interrupt Descriptor Table
            3'd4: id_ucode_entry = UENTRY_SMSW;              // Store Machine Status Word
            3'd6: id_ucode_entry = UENTRY_LMSW;              // Load Machine Status Word
            default: id_ucode_valid = 1'b0;
        endcase

        9'h102: id_ucode_entry = UENTRY_LAR;                 // Load Access Rights
        9'h103: id_ucode_entry = UENTRY_LSL;                 // Load Segment Limit

        9'h106: id_ucode_entry = UENTRY_CTS;                 // Clear Task-Switched flag
*/
        default: begin
            id_ucode_entry = UENTRY_NOP;
            id_ucode_valid = 1'b0;                           // Other instructions hardwired in EXECUTE2
        end
    endcase

    if (id_prefix[PREFIX_REP_REPZ_REPE] | id_prefix[PREFIX_REPNE_REPNZ]) begin  // REP/REPE/REPZ/REPNE/REPNZ entries
        unique case (id_opcode)
            8'hA4: id_ucode_entry = UENTRY_REP_MOVSB;
            8'hA5: id_ucode_entry = UENTRY_REP_MOVSW;
            8'hAA: id_ucode_entry = UENTRY_REP_STOSB;
            8'hAB: id_ucode_entry = UENTRY_REP_STOSW;
            8'hAC: id_ucode_entry = UENTRY_REP_LODSB;
            8'hAD: id_ucode_entry = UENTRY_REP_LODSW;
            8'hA6: id_ucode_entry = id_prefix[PREFIX_REPNE_REPNZ] ? UENTRY_REPNE_CMPSB
                                                        : UENTRY_REPE_CMPSB;
            8'hA7: id_ucode_entry = id_prefix[PREFIX_REPNE_REPNZ] ? UENTRY_REPNE_CMPSW
                                                        : UENTRY_REPE_CMPSW;
            8'hAE: id_ucode_entry = id_prefix[PREFIX_REPNE_REPNZ] ? UENTRY_REPNE_SCASB
                                                        : UENTRY_REPE_SCASB;
            8'hAF: id_ucode_entry = id_prefix[PREFIX_REPNE_REPNZ] ? UENTRY_REPNE_SCASW
                                                        : UENTRY_REPE_SCASW;
            default:;
        endcase
    end

    if (int_pipeline_inject) begin
        id_ucode_entry = UENTRY_INT_IMM;
        id_ucode_valid = 1'b1;
    end

end

//================================================================
// Driving register file
//================================================================

wire CF = reg_f[0], PF = reg_f[2], AF = reg_f[4], ZF = reg_f[6], SF = reg_f[7], TF = reg_f[8], IF = reg_f[9], DF = reg_f[10], OF = reg_f[11];

always @(posedge clk) 
    if (reset) 
        cycle_2nd <= 0; 
    else if (id_valid & id_ready)
        cycle_2nd <= 0;
    else if (id_valid & ~id_ready & ii.is_two_cycles & ~hazard)   // for 2-cycle instructions, every cycle after the first is "2nd cycle"
        cycle_2nd <= 1;

// Addresses for register file
// Port 1 – “G” field   (opcode[5:3] or modRM[5:3])
// Port 2 – “E base”    (EA calculation or register form)
always_comb begin
    // G field
    logic g_active = 0;
    reg1_raddr = 'x;
    if (ii.g_is_reg) begin
        reg1_raddr = ii.w ? id_modrm[5:3] : {1'b0, id_modrm[4:3]};
        g_active = 1;
    end else if (ii.g_is_sp) begin
        reg1_raddr = R_SP;
        g_active = 1;
    end else if (ii.g_is_a) begin
        reg1_raddr = R_AX;
        g_active = 1;
    end else if (ii.g_is_c) begin
        reg1_raddr = R_CX;
        g_active = 1;
    end else if (ii.g_is_d) begin
        reg1_raddr = R_DX;
        g_active = 1;
    end else if (ii.g_is_seg) begin    // MOV Ew,Sw
        reg1_raddr = 'x;
        g_active = 1;
    end

    // E field
    reg2_raddr = R_BX;                 // default used by XLAT
    if (ii.e_is_m & ~cycle_2nd) begin  // critical path
        // Memory form E
        // Triple read:
        // We have 2 GPR register ports. In rare cases, we need to read 3 ports - taking two cycle.
        // When E is memory form and needs both base and index registers, and G also reads GPR
        // - on 1st cycle, E preempts G by setting reg1_raddr
        // - on 2nd cycle, E release reg1_addr, and uses buffered ea_base_r 
        //                 (see id_e_addr_val below)
        unique case (id_modrm[2:0])
            3'b000: begin reg1_raddr = R_BX; reg2_raddr = R_SI; end   // [BX+SI]
            3'b001: begin reg1_raddr = R_BX; reg2_raddr = R_DI; end   // [BX+DI]
            3'b010: begin reg1_raddr = R_BP; reg2_raddr = R_SI; end   // [BP+SI]
            3'b011: begin reg1_raddr = R_BP; reg2_raddr = R_DI; end   // [BP+DI]
            3'b100: reg2_raddr = R_SI;   // [SI]
            3'b101: reg2_raddr = R_DI;   // [DI]
            3'b110: reg2_raddr = R_BP;   // [BP]  (with disp)
            3'b111: reg2_raddr = R_BX;   // [BX]
        endcase
    end else if (ii.e_is_rm & ~cycle_2nd) begin
        // register form E
        reg2_raddr = ii.w ? id_modrm[2:0] : {1'b0, id_modrm[1:0]};
    end else if (ii.e_is_a) begin
        reg2_raddr = R_AX;
    end else if (ii.e_is_seg) begin
        reg2_raddr = 'x;
    end else if (ii.e_is_op20) begin
        reg2_raddr = id_opcode[2:0];
    end

    // debug interface
    reg_dout = 'x;
    if (reg_rd) begin
        casez (reg_addr)
        4'b0???: begin
            reg1_raddr = reg_addr[2:0];
            reg_dout = reg1_rdata;
        end
        4'd8: reg_dout = seg_CS;
        4'd9: reg_dout = seg_SS;
        4'd10: reg_dout = seg_DS;
        4'd11: reg_dout = seg_ES;
        4'd12: reg_dout = reg_ip;
        4'd13: reg_dout = reg_f;
        default: ;
        endcase
    end
end

logic [15:0] ea_base;
logic [15:0] ea_base_r;      // buffer EA for 2nd cycle of triple read
always @(posedge clk) ea_base_r <= ea_base;

logic [15:0] id_e_addr_val;   // e value for next instruction

// G value, E value, effective address calculation
always_comb begin : ea_calc
    // G-value (reg1_data) ------------------------------------------------------
    ex_g_val_next = 'x;

    if (ii.g_is_reg) begin
        ex_g_val_next = ii.w ? reg1_rdata :
                  (id_modrm[5] ? { 8'h00, reg1_rdata[15:8]}   // AH…BH
                                : { 8'h00, reg1_rdata[7:0]}); // AL…BL
    end else if (ii.g_is_sp | ii.g_is_a | ii.g_is_d) begin
        ex_g_val_next = reg1_rdata;
    end else if (ii.g_is_c) begin
        ex_g_val_next = ii.w ? reg1_rdata : { 8'h00, reg1_rdata[7:0]};
    end else if (ii.g_is_seg) begin
        case (id_modrm[4:3])
            2'b00: ex_g_val_next = seg_ES;
            2'b01: ex_g_val_next = seg_CS;
            2'b10: ex_g_val_next = seg_SS;
            2'b11: ex_g_val_next = seg_DS;
        endcase
        if (DEBUG) $display("G_seg [%b]=%x", id_modrm[4:3], ex_g_val_next);
    end

    if (ii.g_is_imm8_sign_ext)
        ex_g_val_next = {{8{id_imm[7]}}, id_imm[7:0]};
    else if (ii.g_is_imm)
        ex_g_val_next = ii.w ? id_imm : {8'h00, id_imm[7:0]};

    // E value (reg2_data) -----------------------------------------------------
    ea_base = ea_base_r;
    id_e_addr_val = 'x;
    if (ii.e_is_m) begin
        // Effective-address calculation --------------------------------------
        if (~cycle_2nd) begin              // 1st cycle or only cycle
            unique case (id_modrm[2:0])
            3'b000: ea_base = reg2_rdata + reg1_rdata;          // BX+SI
            3'b001: ea_base = reg2_rdata + reg1_rdata;          // BX+DI
            3'b010: ea_base = reg2_rdata + reg1_rdata;          // BP+SI
            3'b011: ea_base = reg2_rdata + reg1_rdata;          // BP+DI
            3'b100, 3'b101,
            3'b110, 3'b111: begin
                if (id_modrm[7:6] == 2'b00) begin
                    if (id_modrm[2:0] == 3'b110) begin
                        ea_base = id_disp;                  // [disp16]
                    end else begin
                        ea_base = reg2_rdata;               // SI / DI / BX
                    end
                end else
                    ea_base = reg2_rdata + disp;
            end
            endcase
            id_e_addr_val = ea_base;
        end else begin                    // 2nd cycle
            if (ii.e_is_base_index_disp) begin // [BX+SI+disp], add displacement
                id_e_addr_val = ea_base_r + disp;
            end else begin
                id_e_addr_val = ea_base_r;
            end
        end
    end else if (ii.e_is_imm_addr) begin
        id_e_addr_val = id_imm;
    end else if (ii.e_is_seg) begin
        case (id_opcode[4:3])
            2'b00: id_e_addr_val = seg_ES;
            2'b01: id_e_addr_val = seg_CS;
            2'b10: id_e_addr_val = seg_SS;
            2'b11: id_e_addr_val = seg_DS;
        endcase
        if (DEBUG) $display("E_seg [%b]=%x", id_opcode[5:4], id_e_addr_val);
    end else if (ii.is_xlat) begin
        id_e_addr_val = reg2_rdata + reg1_rdata[7:0];
    end else if (ii.w) begin   // RM register form + A + OP + O
        id_e_addr_val = reg2_rdata;
    end else begin             // byte-sized
        if (ii.e_is_op20) begin
            id_e_addr_val = {8'h00, id_opcode[2] ? reg2_rdata[15:8] : reg2_rdata[7:0]};
        end else if (ii.e_is_a) begin
            id_e_addr_val = {8'h00, reg2_rdata[7:0]};
        end else begin
            id_e_addr_val = {8'h00, id_modrm[2] ? reg2_rdata[15:8] : reg2_rdata[7:0]};
        end
    end
end

// Segment selection
logic [1:0] seg_sel;
always_comb begin
    if (seg_ovr) begin
        id_e_segment =
            seg_ovr_es ? seg_ES :
            seg_ovr_cs ? seg_CS :
            seg_ovr_ss ? seg_SS :
                         seg_DS;   // explicit DS override (3Eh)
        seg_sel = seg_ovr_es ? 2'd0 :
                  seg_ovr_cs ? 2'd1 :
                  seg_ovr_ss ? 2'd2 :
                               2'd3;
        if (DEBUG) $display("Segment override ES=%x, CS=%x, SS=%x", seg_ovr_es, seg_ovr_cs, seg_ovr_ss);
    end else if (ea_uses_bp) begin
        id_e_segment = seg_SS;
        seg_sel = 2'd2;
    end else begin
        id_e_segment = seg_DS;
        seg_sel = 2'd3;
        // $display("Segment = DS");
    end
end

// Setup memory read
always_comb begin
    id_mem_rd = ii.e_is_rm & ~ii.e_write_only & id_modrm[7:6] != 2'b11;

    // Special cases
    // MOV AL,Ob / MOV AX,Ov
    if (id_opcode == 8'hA0 || id_opcode == 8'hA1) 
        id_mem_rd = 1'b1;

    // XLAT forces   ex_e_addr_val = BX + AL   and marks it as "address"
    if (id_opcode == 8'hD7) 
        id_mem_rd  = 1'b1;

    rd_addr = {id_e_segment, 4'b0} + id_e_addr_val;     // critical path: id_e_addr_val
    rd_io = is_io_in;
    rd_word = 1'b1;

    // POP reg/seg is hardwired and we issue memory read here
    if (id_pop_r16 | id_pop_seg | id_pop_rm) begin
        id_mem_rd = 1;
        rd_addr = {seg_SS, 4'b0} + reg1_rdata;   // stack instructions are all Gsp
    end            

    // IN (E4/E5/EC/ED)
    if (is_io_in) begin
        id_mem_rd = 1'b1;
        rd_addr = id_opcode[3] ? ex_g_val_next : id_imm[7:0];
        rd_word = ii.w;
    end

    // if (rd)
    //     $display("DECODE: rd=1,rd_addr=%x", rd_addr);
end

assign rd = id_mem_rd & id_ready & id_valid & ~id_void & rd_ready;

// Send signals to EXECUTE
always @(posedge clk) begin
    logic w = id_opcode[0];
    if (reset) begin
        ex_valid <= 0;
        ex_mem_rd <= 0;
        ex_e_addr_val <= 0;
        ex_e_segment <= 0;
        ex_g_val <= 0;
        ex_ucode_valid <= 0;
    end else begin
        start_int_service <= 0;

        if (ex_ready)                   // result is consumed
            ex_valid <= 0; 
        if (emit) begin  // generate new result
            ex_valid <= 1;
            ex_ip_after <= id_ip_after;
            ex_imm <= id_imm;
            ex_disp <= id_disp;
            ex_opcode <= id_opcode;
            ex_modrm <= id_modrm;

            if (id_opcode == 8'h8E || id_opcode[7:4] == 4'h9) begin   // force 16-bit operations
                w = 1'b1;
            end

            ex_g_val <= ex_g_val_next;
            ex_e_addr_val <= id_e_addr_val;
            ex_e_fulladdr <= {id_e_segment, 4'b0} + id_e_addr_val;
            ex_mem_rd <= id_mem_rd;

            ex_e_segment <= id_e_segment;

            // Forward microcode entry
            // ex_ucode_entry <= id_ucode_entry;
            ex_ucode_valid <= id_ucode_valid;

            ex_inst <= id_inst;

            if (id_opcode != 8'h90)
                if (DEBUG) $display("DECODE: e_addr_val=%x, e_fulladdr=%x, g_val=%x, mem_rd=%x, rd_addr=%x", 
                    id_e_addr_val, {id_e_segment, 4'b0} + id_e_addr_val,
                    ex_g_val_next, id_mem_rd, rd_addr);
        end

        if (int_pipeline_inject) begin
            ex_valid      <= 1'b1;
            ex_opcode     <= 8'hCD;                         // INT Ib
            ex_imm        <= intr_latched ? pic_vec : 8'd2; // 2 = NMI vector
            start_int_service <= 1;
            ex_ucode_valid <= 1'b1;
            ex_ip_after <= id_inst.ip_this;  // return to this instruction after interrupt
//            is_ext_int       <= 1'b1;      // external interrupt flag
            int_is_nmi       <= nmi_latched;
        end

    end
end

// Decode hardwired instruction class
always @(posedge clk) begin
    if (reset) begin
        ex_iclass <= I_IDLE;
    end else if (emit) begin
        unique casez (id_opcode)
        8'h00,8'h01,8'h02,8'h03,8'h04,8'h05,
        8'h10,8'h11,8'h12,8'h13,8'h14,8'h15,
        8'h18,8'h19,8'h1A,8'h1B,8'h1C,8'h1D,
        8'h28,8'h29,8'h2A,8'h2B,8'h2C,8'h2D,
        8'h38,8'h39,8'h3A,8'h3B,8'h3C,8'h3D,
        8'h80,8'h81,8'h82,8'h83:            ex_iclass <= I_ARITH;
        8'h08,8'h09,8'h0A,8'h0B,8'h0C,8'h0D,
        8'h20,8'h21,8'h22,8'h23,8'h24,8'h25,
        8'h30,8'h31,8'h32,8'h33,8'h34,8'h35,
        8'h84,8'h85,8'hA8,8'hA9:  // TEST            
                                            ex_iclass <= I_LOGICAL;
        8'hF6,8'hF7:                        // F6/F7.0-3 is TEST/NOT/NEG, others are microcoded
                                            ex_iclass <= id_modrm[5:3] < 3'd4 ? I_LOGICAL : I_IDLE;
        8'hD0,8'hD1,8'hD2,8'hD3,8'hC0,8'hC1:ex_iclass <= I_SHIFT;     // Group 2 + 286 shift/rotate instructions
        8'h4?,8'hFE:                        ex_iclass <= I_INCDEC;
        8'hFF:                              // FF.0,FF.1 is INC/DEC, others are microcoded
                                            ex_iclass <= id_modrm[5:3] < 3'd2 ? I_INCDEC : I_IDLE;
        8'h88,8'h89,8'h8A,8'h8B,8'h8C,8'h8E,
        8'hA0,8'hA1,8'hA2,8'hA3,
        8'hB?,
        8'hC6,8'hC7:                        ex_iclass <= I_MOV;
        8'hE0,8'hE1,8'hE2:                  ex_iclass <= I_LOOP;
        8'hE9,8'hEA,8'hEB:                  ex_iclass <= I_JUMP;
        8'hE4,8'hE5,8'hE6,8'hE7,
        8'hEC,8'hED,8'hEE,8'hEF:            ex_iclass <= I_IO;
        8'hD7:                              ex_iclass <= I_XLAT;
        8'h7?,8'hE3:                        ex_iclass <= I_JCC;
        8'h8D:                              ex_iclass <= I_LEA;
        8'h86,8'h87, // XCHG r/m16, r16
        /*8'h90,*/8'h91,8'h92,8'h93,
        8'h94,8'h95,8'h96,8'h97:            ex_iclass <= I_XCHG;
        8'h98,8'h99:                        ex_iclass <= I_CXD;
        8'hF5,8'hF8,8'hF9,8'hFA,
        8'hFB,8'hFC,8'hFD:                  ex_iclass <= I_FLAGS;
        8'h9F,8'h9E:                        ex_iclass <= I_LSAHF;
        8'h5?,                        // PUSH/POP r16
        8'h68,8'h6A,                  // PUSH Iw/Ib
        8'h8F,                        // POP r/m16
        8'h06,8'h0E,8'h16,8'h1E,      // PUSH seg
        8'h07,8'h17,8'h1F:            // POP seg
                                            ex_iclass <= I_PUSHPOP;
        default:                            ex_iclass <= I_IDLE;
        endcase
        if (int_pipeline_inject) begin
            ex_iclass <= I_IDLE;      // executed by microcode
        end
    end
end

always @(posedge clk) begin
    if (reset) begin
        nmi_latched  <= 1'b0;
        intr_latched <= 1'b0;
    end
    else begin
        // latch rising edge
        nmi_latched  <= nmi_latched  | nmi;
        intr_latched <= intr_latched | (intr & IF & ~intra);

        // cleared when we *start* servicing
        if (start_int_service && ~br_taken && int_is_nmi) begin
            nmi_latched <= 1'b0;
            intra <= 1'b1;
        end
        if (start_int_service && ~br_taken && !int_is_nmi) begin
            intr_latched <= 1'b0;
            intra <= 1'b1;
        end

        if (~nmi && ~intr) begin
            intra <= 1'b0;
        end
    end
end

// ─── Hazard scoreboard maintenance ──────────────────────────────────────────────

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        sb0_valid <= 1'b0;  sb1_valid <= 1'b0;
        sb0_mask  <= '0;    sb1_mask  <= '0;
    end else begin
        // 1. pipeline advances → entry moves to older slot
        if (sb_shift) begin
            sb1_valid <= sb0_valid;
            sb1_mask  <= sb0_mask;
            sb1_mask.mem <= 1'b0;       // memory RAW only for one instruction
            sb1_mask.io <= 1'b0;        // IO RAW only for one instruction
            sb0_valid <= 1'b0;          // will be overwritten below if we emit
            sb0_mask  <= '0;
        end else begin
            sb1_valid <= 0;        // clear sb1 (WB) if no new instruction executes
            sb1_mask <= 0;
        end

        // 2. new instruction successfully *emitted* into EXECUTE
        if (emit) begin
            sb0_valid <= 1'b1;
            sb0_mask  <= wr_mask;
        end

        // 3. pipeline flush (taken branch / INT) -> scoreboard reset
        if (br_taken | int_pipeline_inject) begin
            sb0_valid <= 1'b0;  sb1_valid <= 1'b0;
            sb0_mask  <= '0;    sb1_mask  <= '0;
        end
    end
end

// build rd_mask
function automatic reg_mask_t build_rd_mask();
    reg_mask_t m = 0;

    if (~ii.g_write_only) begin
        if (ii.g_is_reg) m.gpr[ii.w ? id_modrm[5:3] : {1'b0, id_modrm[4:3]}] = 1'b1;
        if (ii.g_is_a)   m.gpr[R_AX] = 1'b1;
        if (ii.g_is_c)   m.gpr[R_CX] = 1'b1;
        if (ii.g_is_d)   m.gpr[R_DX] = 1'b1;
        if (ii.g_is_sp)  m.gpr[R_SP] = 1'b1;
    end

    if (ii.e_is_m) begin
        m.gpr |= BASE_INDEX[id_modrm[2:0]];
    end else if (~ii.e_write_only) begin
        if (ii.e_is_rm)   m.gpr[ii.w ? id_modrm[2:0] : {1'b0,id_modrm[1:0]}] = 1'b1;
        if (ii.e_is_a)    m.gpr[R_AX] = 1'b1;
        if (ii.e_is_op20) m.gpr[ii.w ? id_opcode[2:0] : {1'b0,id_opcode[1:0]}] = 1'b1;
    end

    if (ii.e_is_m & ~ii.e_write_only) m.mem = 1'b1;

    m.seg = 0;
    if (ii.g_is_seg & ~ii.g_write_only) m.seg[id_modrm[4:3]] = 1'b1;
    if (ii.e_is_seg & ~ii.e_write_only) m.seg[id_opcode[5:4]] = 1'b1;
    if (ii.is_mem_no_stack) m.seg[seg_sel] = 1'b1;
    if (id_opcode >= 8'hA4 && id_opcode <= 8'hA7) begin       // MOVS/CMPS, DS:SI and ES:DI
        m.seg = 4'b1001; 
        m.gpr[R_SI] = 1'b1; m.gpr[R_DI] = 1'b1;
        if (id_prefix[PREFIX_REP_REPZ_REPE] | id_prefix[PREFIX_REPNE_REPNZ]) m.gpr[R_CX] = 1'b1;
    end
    if (id_opcode inside {8'hAA, 8'hAB, 8'hAE, 8'hAF}) begin  // STOS/SCAS, ES:DI
        m.seg = 4'b0001; 
        m.gpr[R_DI] = 1'b1;
        if (id_prefix[PREFIX_REP_REPZ_REPE] | id_prefix[PREFIX_REPNE_REPNZ]) m.gpr[R_CX] = 1'b1;
    end
    if (id_opcode inside {8'hAC, 8'hAD}) begin                // LODS, DS:SI
        m.seg = 4'b1000; 
        m.gpr[R_SI] = 1'b1;
        if (id_prefix[PREFIX_REP_REPZ_REPE]) m.gpr[R_CX] = 1'b1;
    end

    if (m.gpr[R_SP])     m.stackop = 1'b1;         // SP readers check stackop
    if (ii.is_stack_op)  begin
        m.gpr[R_SP] = 1'b1;       // Stack ops wait for regular SP writes to finish
        m.seg[2] = 1'b1;          // also SS
    end

    if (id_opcode inside {8'hE4,8'hE5,8'hEC,8'hED,8'h6C,8'h6D}) begin   // IN/INS
        m.io = 1'b1;
    end

    return m;
endfunction

localparam [7:0] BASE_INDEX [0:7] = '{
    8'b0100_1000,  // [BX+SI]
    8'b1000_1000,  // [BX+DI]
    8'b0110_0000,  // [BP+SI]
    8'b1010_0000,  // [BP+DI]
    8'b0100_0000,  // [SI]
    8'b1000_0000,  // [DI]
    8'b0010_0000,  // [BP]
    8'b0000_1000   // [BX]
};

function automatic reg_mask_t build_wr_mask();
    reg_mask_t m = 0;
    
    // G-field (almost all ALU / MOV forms)
    if (~ii.g_read_only) begin
        if (ii.g_is_reg) m.gpr[ii.w ? id_modrm[5:3] : {1'b0, id_modrm[4:3]}] = 1'b1;
        if (ii.g_is_a)   m.gpr[R_AX] = 1'b1;
        if (ii.g_is_c)   m.gpr[R_CX] = 1'b1;
        if (ii.g_is_d)   m.gpr[R_DX] = 1'b1;
        if (ii.g_is_sp)  m.gpr[R_SP] = 1'b1;
    end

    // E-field is a register *and* the instruction can write to it
    if (~ii.e_read_only) begin
        if (ii.e_is_rm & ~ii.e_is_m) m.gpr[ii.w ? id_modrm[2:0] : {1'b0,id_modrm[1:0]}] = 1'b1;
        if (ii.e_is_a)               m.gpr[R_AX] = 1'b1;
        if (ii.e_is_op20)            m.gpr[ii.w ? id_opcode[2:0] : {1'b0,id_opcode[1:0]}] = 1'b1;
    end

    // TODO: consider stack operations
    if (ii.e_is_m & ~ii.e_read_only) m.mem = 1'b1;

    m.seg = 0;
    if (ii.g_is_seg & ~ii.g_read_only) m.seg[id_modrm[4:3]] = 1'b1;
    if (ii.e_is_seg & ~ii.e_read_only) m.seg[id_opcode[5:4]] = 1'b1;

    if (id_opcode >= 8'hA4 && id_opcode <= 8'hA7) begin       // MOVS/CMPS, DS:SI and ES:DI
        m.gpr[R_SI] = 1'b1; m.gpr[R_DI] = 1'b1;
        if (id_prefix[PREFIX_REP_REPZ_REPE] | id_prefix[PREFIX_REPNE_REPNZ]) m.gpr[R_CX] = 1'b1;
    end
    if (id_opcode inside {8'hAA, 8'hAB, 8'hAE, 8'hAF}) begin  // STOS/SCAS, ES:DI
        m.gpr[R_DI] = 1'b1;
        if (id_prefix[PREFIX_REP_REPZ_REPE] | id_prefix[PREFIX_REPNE_REPNZ]) m.gpr[R_CX] = 1'b1;
    end
    if (id_opcode inside {8'hAC, 8'hAD}) begin                // LODS, DS:SI
        m.gpr[R_SI] = 1'b1;
        if (id_prefix[PREFIX_REP_REPZ_REPE]) m.gpr[R_CX] = 1'b1;
    end

    // Stack operations sets stackop bit at write time
    m.stackop = ii.is_stack_op;

    if (id_opcode inside {8'hE7,8'hE8,8'hEE,8'hEF,8'h6E,8'h6F} ||  // OUT/OUTS
        id_opcode[7:4]=='h7 || id_opcode inside {8'hE8,8'hE9,8'hEA,8'hEB}) begin       // JMP/CALL, TODO: also need RET and 0xFF
        m.io = 1'b1;
    end

    return m;
endfunction

// ---- Void instruction when branching and INT ──────────────────────────

always @(posedge clk) begin
    if (reset) begin
        id_void_r <= 1'b0;
    end else if (id_valid) begin
        if (id_ready)                            id_void_r <= 1'b0;
        else if (br_taken | int_pipeline_inject) id_void_r <= 1'b1;
    end
end

endmodule

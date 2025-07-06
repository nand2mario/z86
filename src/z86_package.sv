package z86_package;

// -- Config -----------------------------------------------------------------

// Enable forwarding of register file write values to read in the same cycle.
// This reduces Fmax by about 5Mhz but decreases RAW hazard wait cycles from 2 to 1.
localparam CONFIG_FORWARDING_REGFILE = 0;

// --------------------------------------------------------------------------

localparam PREFIX_LOCK = 0;
localparam PREFIX_REPNE_REPNZ = 1;
localparam PREFIX_REP_REPZ_REPE = 2;
localparam PREFIX_CS = 3;
localparam PREFIX_SS = 4;
localparam PREFIX_DS = 5;
localparam PREFIX_ES = 6;

localparam DEBUG = 0;

// Some one-hot encodings to speed up EXECUTE
localparam [16:0] I_ARITH   = 17'h00001, 
                  I_LOGICAL = 17'h00002, 
                  I_SHIFT   = 17'h00004, 
                  I_INCDEC  = 17'h00008,
                  I_MOV     = 17'h00010,
                  I_LOOP    = 17'h00020,
                  I_JUMP    = 17'h00040,
                  I_IO      = 17'h00080,
                  I_JCC     = 17'h00100,
                  I_LEA     = 17'h00200,
                  I_XLAT    = 17'h00400,
                  I_XCHG    = 17'h00800,
                  I_CXD     = 17'h01000,
                  I_FLAGS   = 17'h02000,
                  I_LSAHF   = 17'h04000,
                  I_PUSHPOP = 17'h08000,
                  I_IDLE    = 17'h10000;

// AX, CX, DX, BX, SP, BP, SI, DI  (8086 encoding)
// reg [15:0] regs [8];
localparam [2:0]    R_AX = 3'd0, R_CX = 3'd1, R_DX = 3'd2, R_BX = 3'd3, 
                    R_SP = 3'd4, R_BP = 3'd5, R_SI = 3'd6, R_DI = 3'd7;

localparam [15:0] FLAGS_INIT = 16'h0002;

reg int_is_nmi;

// DECODE stage control signals, mostly from regctrl part of fetch LUT
typedef struct packed {
    // E-related
    logic e_is_rm;               // E is specified by modrm[7:6] and modrm[2:0] (register or memory)
    logic e_is_a;                // E is Ax or AL
    logic e_is_seg;              // E is segment register specified by opcode[5:4]
    logic e_is_op20;             // E is register specified by opcode[2:0]
    logic e_is_imm_addr;         // E is specified by address in imm (A0-A3, MOV A,O)
    logic e_write_only;          // E is write-only, no read
    logic e_read_only;           // E is read-only, no write
    logic e_is_base_index_disp;  // E is something like [BX+SI+disp]
    logic e_is_m;                // E is rm memory (modrm valid and modrm[7:6]!=2'b11)

    // G-related
    logic g_is_reg;              // G is specified by modrm[5:3]
    logic g_is_a;                // G is AX
    logic g_is_c;                // G is CX
    logic g_is_d;                // G is DX
    logic g_is_sp;               // G is SP
    logic g_is_seg;              // G is segment register specified by modrm[5:3]
    logic g_is_imm;              // G is IMM
    logic g_is_imm8_sign_ext;    // G is IMM, width is 8 bits, sign-extended
    logic g_write_only;          // G is write-only, no read (used to identify triple read)
    logic g_read_only;           // G is read-only, no write

    // others
    logic w;                     // Word instruction, otherwise byte
    logic is_two_cycles;         // Decode will need 2 cycles for triple register read or base_index_disp
    logic is_xlat;               // XLAT instruction
    logic is_stack_op;           // PUSH/POP instruction
    logic is_mem_no_stack;       // Accesses memory (not including stack). This includes e_is_m, e_is_imm_addr, xlat and string ops.
    logic e_is_m_base_only;      // E is [SI], [DI] or [BX], modrm=00???1??
    logic e_is_m_disp16;         // E is [disp16], modrm=00???110

    logic [15:0] ip_this;        // PC of this instruction, for error handling like INT0
} id_instruction;

// ─── Register hazard scoreboard ─────────────────────────────────────────────
typedef struct packed {
    logic [7:0] gpr;
    logic       mem;        // block one cycle when last instruction is writing memory and this is reading
                            // TODO: make this finer-grained
    logic [3:0] seg;        // ES/CS/SS/DS
    logic       stackop;    // a stack op is in EX or WB
    logic       io;         // block one cycle for IO RAW
} reg_mask_t;               

endpackage
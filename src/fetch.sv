// z86 instruction fetch buffer and first-stage decoder.
//
// This tightly couples with the cache to continuously fetch instructions from memory,
// then decode them into a more structured format. Each instruction is 1-10 bytes.
// This outputs opcode, modrm, displacement, immediate, etc.
//
// We use a 32-byte buffer that continues to fetch from memory. Whenver we have half of
// the buffer available, we fetch 16 bytes from memory. The buffer is always aligned to
// 16-byte boundary.
//
// First-stage decoding normally takes 1 cycle. It takes more if the instruction has 
// prefixes.
//
// A lot of control signals are generated by the "Fetch LUT"
import z86_package::*;
module fetch (
    input             clk,
    input             reset,

    input             br_taken,      // set new address to fetch from
    input     [19:0]  br_target,     // byte address
    input     [15:0]  br_new_ip,     // IP after branch
    input     [15:0]  br_new_cs,     // CS after branch

    input     [19:0]  reset_addr,    // address after reset
    input     [15:0]  reset_ip,      // IP after reset

    // AXI-style instruction output
    input             id_ready,      // user (decoder) is ready to receive instructions
    output reg        id_valid,      // pulse for instruction output
    output reg [3:0]  id_len,
    output reg [6:0]  id_prefix,     // the fetched instruction fields, PREFIX_* in z486_package.sv
    output reg [7:0]  id_opcode,
    output reg        id_modrm_valid,
    output reg [7:0]  id_modrm,
    output reg        id_disp_valid,
    output reg [31:0] id_disp,       // 8 or 16-bit displacement, or 32-bit pointer
    output reg        id_imm_valid,
    output reg [15:0] id_imm,        // 8 or 16-bit immediate
    output reg [15:0] id_ip_after,   // PC after this instruction
    output reg        id_two_byte,   // 2-byte instruction (0F prefix)

    output reg        id_disp8,
    output reg        id_disp16,
    output reg        id_disp32,     // pointer
    output reg        id_imm8,
    output reg        id_imm16,
    
    output id_instruction id_inst,

    // cache loading interface
    output reg        ld_req,        
    input             ld_ack,
    output reg [19:0] ld_addr,
    input     [127:0] ld_data
);

reg instr_acked = 1;

// 32-byte buffer. We could load into either first or second half.
// `buf_start`. 0: buffer starts at offset 0; 1: buffer starts at offset 16
// `buf_valid` is whether each half of the buffer is valid.
// `buf_addr` is the starting address of the data in the buffer.
// `ptr`. buffer[ptr] is the first byte of the next instruction.

reg [7:0]  buffer[0:31];
reg        buf_start;         // 0: buffer starts at offset 0; 1: buffer starts at offset 16 and wraps around
reg [1:0]  buf_valid;         // which halves of the buffer are valid
reg [19:0] buf_addr;          // 16-byte aligned buffer start address
reg [4:0]  ptr;               // buffer[ptr] is start of next instruction

reg        loading;           // there's an ongoing cache request
reg        loading_half;      // which half of the buffer is being loaded
reg        loading_drop;      // drop the cache request (the buffer is flushed)

reg [3:0]  len;               // instruction length not including prefixes
reg [6:0]  prefix;
reg [1:0]  prefix_len;
wire       w = buffer[ptr][0];
reg        is_modrm, is_disp8, is_disp16, is_disp32, is_imm8, is_imm16;
reg [17:0] regctrl;
reg [31:0] disp_t;
reg [15:0] imm_t;
reg [4:0]  ptr_end;
reg [15:0] ip;                // start of this instruction

wire       is_0f_escape = (buffer[ptr] == 8'h0F);
wire [7:0] opcode_byte = is_0f_escape ? buffer[(ptr+1)%32] : buffer[ptr];
wire [8:0] lut_index = {is_0f_escape, opcode_byte};

wire [4:0] param_base = ptr + is_0f_escape;
wire [7:0] modrm = buffer[(param_base + 1) % 32];

// instruction layout decoding (combinational)
always @* begin
    // 1. parse opcode 
    {regctrl, is_modrm, is_imm8, is_imm16, is_disp32} = FETCH_LUT[lut_index];
    // special case handling
    if (buffer[ptr] == 8'hF6 && modrm[5:4] == 2'b0)
        is_imm8 = 1;
    if (buffer[ptr] == 8'hF7 && modrm[5:4] == 2'b0)
        is_imm16 = 1;

    // 2. parse modrm and displacement
    {is_disp16, is_disp8} = is_modrm ? modrm_disp(modrm) : 2'b00;

    // 3. collect results. in total 4 + 9 = 13 diffrent layouts
    disp_t = {32{1'bX}};
    imm_t = {16{1'bX}};
    casez ({is_disp32, is_modrm, is_imm16, is_imm8, is_disp16, is_disp8})
        6'b1_?_??_??: begin    // opcode + disp32
            len = 5;
            disp_t = {buffer[(param_base+4)%32], buffer[(param_base+3)%32], 
                      buffer[(param_base+2)%32], buffer[(param_base+1)%32]};
        end
        6'b0_0_00_??: begin    // opcode only
            len = 1;
        end
        6'b0_0_01_??: begin    // opcode + imm8
            len = 2;
            imm_t = buffer[(param_base+1)%32];
        end
        6'b0_0_10_??: begin    // opcode + imm16
            len = 3;
            imm_t = {buffer[(param_base+2)%32], buffer[(param_base+1)%32]};
        end
        6'b0_1_??_00: begin    // opcode + modrm + [imm8/imm16]
            len = 2 + (is_imm8 ? 1 : is_imm16 ? 2 : 0);
            imm_t = {is_imm16 ? buffer[(param_base+3)%32] : 8'h0, buffer[(param_base+2)%32]};
        end
        6'b0_1_??_01: begin    // opcode + modrm + disp8 + [imm8/imm16]
            len = 3 + (is_imm8 ? 1 : is_imm16 ? 2 : 0);
            disp_t = buffer[(param_base+2)%32];
            imm_t = {is_imm16 ? buffer[(param_base+4)%32] : 8'h0, buffer[(param_base+3)%32]};
        end
        6'b0_1_??_10: begin    // opcode + modrm + disp16 + [imm8/imm16]
            len = 4 + (is_imm8 ? 1 : is_imm16 ? 2 : 0);
            disp_t = {buffer[(param_base+3)%32], buffer[(param_base+2)%32]};
            imm_t = {is_imm16 ? buffer[(param_base+5)%32] : 8'h0, buffer[(param_base+4)%32]};
        end
        default: begin
            len = 1;
            $display("Unknown instruction layout %h %h", buffer[ptr], buffer[(ptr+1)%32]);
        end
    endcase
    len += is_0f_escape;
    ptr_end = ptr + len;
end

reg ld_ack_r;
always @(posedge clk) begin
    logic [7:0] opcode = 'x;
    logic [4:0] ptr_next = 'x;

    if (id_ready) id_valid <= 0;        // downstream consumed instruction

    if (reset) begin
        buf_valid <= 0;
        buf_addr <= {reset_addr[19:4], 4'b0};
        ptr <= reset_addr[3:0];
        ip <= reset_ip;
        buf_start <= 0;
        ld_req <= ld_ack;
        ld_ack_r <= ld_ack;
        loading <= 0;
        id_valid <= 0;
        id_inst <= 0;
        loading_drop <= 0;
    end else begin
        ld_ack_r <= ld_ack;
        if (ld_ack ^ ld_ack_r) begin  // receive data from cache
            if (!loading_drop) begin
                for (int i = 0; i < 16; i++) begin
                    buffer[loading_half*16 + i] <= ld_data[i*8 +: 8];
                end
                buf_valid[loading_half] <= 1;
            end
            loading <= 0;
            loading_drop <= 0;
        end

        // set address
        if (br_taken) begin
            if (buf_addr[19:4] != br_target[19:4] && buf_addr[19:4] + 1 != br_target[19:4]) begin
                // target address is not in our cache line, flush the cache
                buf_valid <= 0;
                buf_start <= 0;
                buf_addr <= {br_target[19:4], 4'b0};
                ip <= br_new_ip;                            
                ptr <= br_target[3:0];
                prefix_len <= 0;
                if (ld_req != ld_ack)                   // drop any ongoing cache request
                    loading_drop <= 1;
            end else begin
                // target address is in our cache line, just move the pointer
                ptr_next = br_target[4:0] - buf_addr[4:0] + (buf_start ? 16 : 0);
                if (ptr_next[4] != ptr[4]) begin
                    // we need to invalidate the current buffer
                    buf_valid[ptr[4]] <= 0;
                    buf_start <= ~buf_start;
                    buf_addr <= buf_addr + 16;
                end
                ptr <= ptr_next;
                ip <= br_new_ip;
            end
            // id_valid <= 0;                              // pipeline flush
        end else begin
            // request data from cache
            if (!loading) begin
                if (!buf_valid[buf_start]) begin             // load the first half
                    ld_req <= ~ld_req;
                    ld_addr <= buf_addr;
                    loading_half <= buf_start;
                    loading <= 1;
                end else if (!buf_valid[~buf_start]) begin   // load the second half
                    ld_req <= ~ld_req;
                    ld_addr <= buf_addr + 16;
                    loading_half <= ~buf_start;
                    loading <= 1;
                end
            end
        end

        if (!br_taken && buf_valid[ptr[4]]) begin            // consume bytes 
            if (is_prefix(buffer[ptr])) begin                // an extra cycle for prefix
                prefix[prefix_bit(buffer[ptr])] <= 1;
                ptr <= ptr + 1;
                ip <= ip + 1;
                prefix_len <= prefix_len + 1;
                if (ptr == 15 || ptr == 31) begin            // invalidate the buffer
                    buf_valid[ptr[4]] <= 0;
                    buf_start <= ~buf_start;
                    buf_addr <= buf_addr + 16;
                end
            end else if ((~id_valid || id_ready) && buf_valid[ptr_end[4]]) begin  // actually output the instruction
                // ip is at opcode
                if (buffer[ptr] != 8'h90) begin
                    if (DEBUG) $display("FETCH addr=%h, opcode=%h, len=%d, ip_after=%h", ip-prefix_len, buffer[ptr], len + prefix_len, ip+len);
                end
                id_valid <= 1;
                id_len <= len + prefix_len;
                id_opcode <= opcode_byte;
                id_two_byte <= is_0f_escape;
                id_modrm_valid <= is_modrm;
                id_modrm <= modrm;
                id_disp_valid <= is_disp32 || (is_modrm && (is_disp16 || is_disp8));
                id_disp <= disp_t;
                id_imm_valid <= is_imm8 || is_imm16;
                id_imm <= imm_t;
                id_ip_after <= ip + len;
                ip <= ip + len;

                // Control signals from LUT
                // if (opcode != 8'h90)
                //     $display("regread=%h", regctrl);
                id_inst.e_is_rm   <= regctrl[0];   
                id_inst.e_is_a    <= regctrl[1];   
                id_inst.e_is_seg  <= regctrl[2];   
                id_inst.e_is_op20 <= regctrl[3];   
                id_inst.e_is_imm_addr  <= regctrl[4];
                id_inst.e_write_only  <= regctrl[5];
                id_inst.e_read_only   <= regctrl[6];

                id_inst.g_is_reg  <= regctrl[7];   
                id_inst.g_is_a    <= regctrl[8];   
                id_inst.g_is_c    <= regctrl[9];   
                id_inst.g_is_d    <= regctrl[10];   
                id_inst.g_is_sp   <= regctrl[11];   
                id_inst.g_is_seg  <= regctrl[12];
                id_inst.g_is_imm  <= regctrl[13];
                id_inst.g_write_only <= regctrl[14];
                id_inst.g_read_only <= regctrl[15];

                id_inst.w         <= regctrl[16];
                id_inst.is_stack_op <= regctrl[17];
                // DIV is always 16-bit (needs AX/DX as dividend)
                // if (opcode == 8'hF6 && modrm[5:3]==3'd6)
                //     id_inst.w <= 1;
                
                // Generate additional control signals
                id_inst.e_is_base_index_disp <= is_modrm &modrm[7:6] != 2'b11 & ~modrm[2] & 
                                              (modrm[7] ^ modrm[6]);   // E is something like [BX+SI+disp]
                id_inst.e_is_m <= is_modrm & modrm[7:6] != 2'b11;
                id_inst.g_is_imm8_sign_ext <= opcode_byte == 8'h83;
                // Decode normally takes one cycle, but takes 2 cycles for these cases:
                // - Triple-read: memory form and needs both base and index registers, and G also reads GPR
                // - Triple-plus: memory form DS:[BX+SI+disp] or similar. Add BX+BI in 1st cycle, Then 
                //   displacement and segment in 2nd cycle.
                id_inst.is_two_cycles <= is_modrm & modrm[7:6] != 2'b11 &
                        ( ~modrm[2] & ~regctrl[14] & (regctrl[7] | regctrl[8] | regctrl[9] | regctrl[10] | regctrl[11]) | // G reads GPR
                          ~modrm[2] & (modrm[7] ^ modrm[6])   // e.g. memory form[BX+SI+disp] 
                        );

                id_inst.is_xlat <= opcode_byte == 8'hD7;
                id_inst.ip_this <= ip - prefix_len;
                id_inst.is_mem_no_stack <= (is_modrm & modrm[7:6] != 2'b11) |             // e_is_m
                                           (opcode_byte == 8'hD7) |                            // XLAT
                                           (opcode_byte[7:4]==4'hA && opcode_byte[3:1]!=3'b100);    // imm_memory and string ops
                id_inst.e_is_m_base_only <= is_modrm & modrm[7:6] == 2'b00 & modrm[2];
                id_inst.e_is_m_disp16 <= is_modrm & modrm[7:6] == 2'b00 & modrm[2:0] == 3'b110;

                id_disp8 <= is_disp8;
                id_disp16 <= is_disp16;
                id_disp32 <= is_disp32;
                id_imm8 <= is_imm8;
                id_imm16 <= is_imm16;

                id_prefix <= prefix;
                prefix <= 0;
                prefix_len <= 0;

                // if done with the current 16 bytes, invalidate it so we'll fetch another
                if (ptr[4] != ptr_end[4]) begin   
                    buf_valid[ptr[4]] <= 0;
                    buf_start <= ~buf_start;
                    buf_addr <= buf_addr + 16;
                end
                ptr <= ptr_end;
            end
        end
    end
end

// First stage decoding lookup table: 
// opcode => {regctrl, layout} 
//   layout    = {is_modrm, is_i8, is_i16, is_i32}
//   regctrl   = E and G register control bits. See id_instruction for details.
// the LUT is generated by tools/fetch_lut.py
`include "fetch_lut.svh"
// layout special cases:
// 1. For F6, if modrm[5:3]==0, there is an extra i8.
// 2. For F7, if modrm[5:3]==0, there is an extra i16.

// displacement lookup table: modrm -> {disp16, disp8}
function [1:0] modrm_disp(input [7:0] b);
    priority casez ({b[7:6], b[2:0]})
    // special case
    8'b00_110:   return 2'b10;      // modrm + disp16
    // normal cases
    8'b00_???:   return 2'b00;      // modrm
    8'b01_???:   return 2'b01;      // modrm + disp8
    8'b10_???:   return 2'b10;      // modrm + disp16
    8'b11_???:   return 2'b00;      // modrm
    endcase
endfunction

function is_prefix(input [7:0] b);
    return b == 8'hF3 || b == 8'hF2 || b == 8'h26 || b == 8'h2E || b == 8'h36 || b == 8'h3E;
endfunction

function [3:0] prefix_bit(input [7:0] b);
    case (b)
    8'hF0: return PREFIX_LOCK;
    8'hF2: return PREFIX_REPNE_REPNZ;
    8'hF3: return PREFIX_REP_REPZ_REPE;
    8'h26: return PREFIX_ES;
    8'h2E: return PREFIX_CS;
    8'h36: return PREFIX_SS;
    8'h3E: return PREFIX_DS;
    default: return 0;
    endcase
endfunction


endmodule
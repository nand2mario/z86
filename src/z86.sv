// z86 - A pipelined 16-bit x86 CPU.
//
// z86 uses a five stage pipeline to implement a 8086-compatible CPU with
// real-mode 80286 instructions (but no protected mode). For most instructions, it 
// z86 can finish one instruction per CPU cycle. Other more complex instructions 
// are micro-code driven and multi-cycle.
//
// To provide enough memory bandwidth, the memory interface is 32-bit wide,
// so it is possible to sustain one instruction per cycle, assuming 3 bytes
// per instruction.
//
// A cache module is included in the design, but caching is not implemented
// yet.
//
// - nand2mario, 7/2025

module z86 (
`ifndef VERILATOR
    input               clk_g,
`else     
    input               clk,
`endif     
    input               reset,
    input               cpu_reset,

	input               a20_enable,

	//--------------------------------------------------------------------------
	input               interrupt_do,
	input   [7:0]       interrupt_vector,
	output              interrupt_done,

	//-------------------------------------------------------------------------- memory bus
    // 32-bit interface provide enough bandwidth. Every cycles requires on average 3/2=1.5 bytes
    // assuming 3 bytes per instruction, and CPI = 2
	output      [19:0]  avm_address,
	output      [31:0]  avm_writedata,
	output      [3:0]   avm_byteenable,
	output      [3:0]   avm_burstcount,
	output              avm_write,
	output              avm_read,

	input               avm_waitrequest,
	input               avm_readdatavalid,
	input       [31:0]  avm_readdata,

	//-------------------------------------------------------------------------- dma bus
    // driven by DMA controller
	input       [19:0]  dma_address,
	input               dma_16bit,
	input               dma_write,          // pulse
	input       [15:0]  dma_writedata,
	input               dma_read,           // pulse
	output      [15:0]  dma_readdata,
	output              dma_readdatavalid,
	output              dma_waitrequest,

	//-------------------------------------------------------------------------- io bus
	output              io_read_do,
	output      [15:0]  io_read_address,
	output              io_read_word,      // 1: 16-bit, 0: 8-bit
	input       [15:0]  io_read_data,
	input               io_read_done,

	output              io_write_do,
	output      [15:0]  io_write_address,
	output              io_write_word,    // 1: 16-bit, 0: 8-bit
	output      [15:0]  io_write_data,
	input               io_write_done,

	//-------------------------------------------------------------------------- debug
    input               dbg_reg_wr,
    input        [3:0]  dbg_reg_addr,
    input       [15:0]  dbg_reg_din,
    input               dbg_reg_rd,
    output      [15:0]  dbg_reg_dout,
     
    input               dbg_mem_wr,
    input       [19:0]  dbg_mem_addr,
    input       [7:0]   dbg_mem_din,
    input               dbg_mem_rd,
    output      [7:0]   dbg_mem_dout
);

import z86_package::*;

`ifndef VERILATOR
pll u_pll (.clkin(clk_g), .clkout0(clk));
`endif

// cache interface 
wire [19:0] ld_addr;
wire [127:0] ld_data;
wire ld_req, ld_ack;

wire rd_ready;
wire [15:0] rd_data;

wire ex_rd;      // read from EXECUTE
wire ex_rd_io;
wire [19:0] ex_rd_addr;  
wire ex_rd_word;

wire id_rd;      // read from DECODE
wire id_rd_io;
wire [19:0] id_rd_addr;
wire id_rd_word;

wire [19:0] wr_addr;   // write from EXECUTE
wire [15:0] wr_data;
wire wr_word;
wire wr_valid, wr_ready;
wire wr_io;

reg [19:0] dbg_reset_addr = 20'hFFFF0;
reg [15:0] dbg_reset_ip = 16'hFFF0;

// decode2 stage signals
wire id_ready, id_valid;
wire id_done;
wire [3:0] id_len;
wire [15:0] id_ip_after;
wire [6:0] id_prefix;
wire [7:0] id_op;
wire [7:0] id_modrm;
wire [31:0] id_disp;
wire [15:0] id_imm;
wire id_modrm_valid, id_disp_valid, id_imm_valid, id_disp8, id_disp16, id_disp32, id_imm8, id_imm16;
wire id_two_byte;
wire id_mem_read;
// wire [8:0] id_regread;
// wire id_two_cycles, id_base_index_disp;
id_instruction id_inst;

// execute
wire ex_ready /* verilator public */, ex_valid /* verilator public */;
wire [15:0] ex_ip_after /* verilator public */;
wire ex_mem_rd;
wire [15:0] ex_e_addr_val;
wire [19:0] ex_e_fulladdr;
wire [15:0] ex_e_segment;
wire [15:0] ex_g_val;
wire [7:0] ex_opcode /* verilator public */;
wire [16:0] ex_iclass;
wire [7:0] ex_modrm;
wire [15:0] ex_imm /* verilator public */;
wire [31:0] ex_disp;
wire ex_ucode_valid;
wire [8:0] ex_ucode_entry_next;
id_instruction ex_inst;
wire ex_reg;
wire [2:0] ex_reg1_raddr;
wire [15:0] ex_reg1_rdata;
wire [2:0] ex_reg2_raddr;
wire [15:0] ex_reg2_rdata;

// write-back
wire wb_reg_valid;
wire wb_reg2_valid;
wire wb_width, wb_width2;
wire [2:0] wb_reg;
wire [15:0] wb_data;
wire [2:0] wb_reg2;
wire [15:0] wb_data2;
wire [1:0] wb_flags_update_mask;
wire wb_flags_valid;
wire [15:0] wb_flags;
wire wb_seg_valid;
wire [1:0] wb_seg;
wire [15:0] wb_seg_data;
wire br_taken;
wire [19:0] br_target;
wire [15:0] br_new_cs;
wire [15:0] br_new_ip;

// Register file signals
wire [2:0] reg1_raddr;
wire [15:0] reg1_rdata;
wire [2:0] reg2_raddr;
wire [15:0] reg2_rdata;
wire [1:0] reg1_we;
wire [2:0] reg1_waddr;
wire [15:0] reg1_wdata;
wire [1:0] reg2_we;
wire [2:0] reg2_waddr;
wire [15:0] reg2_wdata;

// other registers
wire [15:0] reg_f;
wire [15:0] reg_ip;
wire [15:0] seg_ES;
wire [15:0] seg_CS;
wire [15:0] seg_SS;
wire [15:0] seg_DS;

reg [19:0] dbg_set_addr_val;
reg [15:0] dbg_set_ip_val;
reg dbg_set_addr;

// cache and memory interface serves FETCH (instruction loads), EXECUTE (memory read) and WRITE-BACK (memory write)
cache u_cache(
    .clk                (clk), 
    .reset              (reset),
    .ld_addr            (ld_addr), 
    .ld_req             (ld_req), 
    .ld_ack             (ld_ack), 
    .ld_data            (ld_data),  
    .ld_hit             (),

    .rd                 (ex_rd | id_rd), 
    .rd_addr            (id_rd ? id_rd_addr : ex_rd_addr), 
    .rd_io              (id_rd ? id_rd_io : ex_rd_io),     // I/O requests do not go through cache
    .rd_ready           (rd_ready), 
    .rd_data            (rd_data), 
    .rd_word            (id_rd ? id_rd_word : ex_rd_word),

    .wr_addr            (wr_addr), 
    .wr_valid           (wr_valid), 
    .wr_ready           (wr_ready), 
    .wr_data            (wr_data), 
    .wr_word            (wr_word), 
    .wr_io              (wr_io),

    .avm_address        (avm_address), 
    .avm_writedata      (avm_writedata), 
    .avm_byteenable     (avm_byteenable), 
    .avm_burstcount     (avm_burstcount), 
    .avm_write          (avm_write), 
    .avm_read           (avm_read),
    .avm_waitrequest    (avm_waitrequest), 
    .avm_readdatavalid  (avm_readdatavalid), 
    .avm_readdata       (avm_readdata),

    .io_read_do         (io_read_do),
    .io_read_address    (io_read_address),
    .io_read_word       (io_read_word),
    .io_read_data       (io_read_data),
    .io_read_done       (io_read_done),

    .io_write_do        (io_write_do),
    .io_write_address   (io_write_address),
    .io_write_word      (io_write_word),
    .io_write_data      (io_write_data),
    .io_write_done      (io_write_done),

    .dbg_mem_wr         (dbg_mem_wr), 
    .dbg_mem_addr       (dbg_mem_addr), 
    .dbg_mem_din        (dbg_mem_din), 
    .dbg_mem_rd         (dbg_mem_rd), 
    .dbg_mem_dout       (dbg_mem_dout)
);

// register file
regfile u_regfile (
    .clk                (clk),
    .id_raddr1          (reg1_raddr), 
    .id_rdata1          (reg1_rdata),
    .id_raddr2          (reg2_raddr), 
    .id_rdata2          (reg2_rdata),
    .ex_raddr1          (ex_reg1_raddr), 
    .ex_rdata1          (ex_reg1_rdata),
    .ex_raddr2          (ex_reg2_raddr), 
    .ex_rdata2          (ex_reg2_rdata),

    .we1                (reg1_we), 
    .waddr1             (reg1_waddr), 
    .wdata1             (reg1_wdata),
    .we2                (reg2_we), 
    .waddr2             (reg2_waddr), 
    .wdata2             (reg2_wdata)
);

// z86 5-stage pipeline: 
//     FETCH -> DECODE1 -> DECODE2 -> EXECUTE -> WRITE-BACK

// FETCH: 32-byte instruction fetch buffer that does 16-byte loads from the cache. 
// DECODE1: calculate instruction length and decode into prefix, opcode, modrm, displacement, immediate
fetch u_fetch_decode1 (
    .clk                (clk), 
    .reset              (cpu_reset), 
    .reset_addr         (dbg_reset_addr),  // physical address to load after reset
    .reset_ip           (dbg_reset_ip),
    
    .ld_addr            (ld_addr), 
    .ld_req             (ld_req), 
    .ld_ack             (ld_ack), 
    .ld_data            (ld_data),

    .br_taken           (br_taken), 
    .br_target          (br_target), 
    .br_new_ip          (br_new_ip), 
    .br_new_cs          (br_new_cs),

    .id_ready           (id_ready), 
    .id_valid           (id_valid), 
    .id_len             (id_len), 
    .id_ip_after        (id_ip_after), 
    .id_prefix          (id_prefix), 
    .id_opcode          (id_op), 
    .id_modrm_valid     (id_modrm_valid), 
    .id_modrm           (id_modrm), 
    .id_disp            (id_disp), 
    .id_imm             (id_imm), 
    .id_disp_valid      (id_disp_valid), 
    .id_imm_valid       (id_imm_valid), 
    .id_disp8           (id_disp8), 
    .id_disp16          (id_disp16), 
    .id_disp32          (id_disp32),
    .id_imm8            (id_imm8), 
    .id_imm16           (id_imm16), 
    .id_two_byte        (id_two_byte),
    .id_inst            (id_inst)           // decoded instruction signals
);

// DECODE2: calculate effective address
decode u_decode2 (
    .clk                (clk), 
    .reset              (cpu_reset),

    .id_ready           (id_ready), 
    .id_valid           (id_valid),
    .id_len             (id_len), 
    .id_ip_after        (id_ip_after), 
    .id_prefix          (id_prefix), 
    .id_opcode          (id_op), 
    .id_modrm_valid     (id_modrm_valid), 
    .id_modrm           (id_modrm), 
    .id_disp            (id_disp), 
    .id_imm             (id_imm),
    .id_disp_valid      (id_disp_valid), 
    .id_imm_valid       (id_imm_valid), 
    .id_disp8           (id_disp8), 
    .id_disp16          (id_disp16), 
    .id_disp32          (id_disp32),
    .id_imm8            (id_imm8),
    .id_imm16           (id_imm16), 
    .id_two_byte        (id_two_byte),
    .id_inst            (id_inst),

    .br_taken           (br_taken),         // branch taken signal input

    .ex_ready           (ex_ready), 
    .ex_valid           (ex_valid), 
    .ex_ip_after        (ex_ip_after), 
    .ex_e_segment       (ex_e_segment),
    .ex_mem_rd          (ex_mem_rd), 
    .ex_e_addr_val      (ex_e_addr_val), 
    .ex_e_fulladdr      (ex_e_fulladdr), 
    .ex_g_val           (ex_g_val), 
    .ex_opcode          (ex_opcode), 
    .ex_iclass          (ex_iclass), 
    .ex_modrm           (ex_modrm), 
    .ex_imm             (ex_imm), 
    .ex_disp            (ex_disp),
    .ex_ucode_entry_next(ex_ucode_entry_next), 
    .ex_ucode_valid     (ex_ucode_valid), 
    .ex_inst            (ex_inst),

    .reg1_raddr         (reg1_raddr), 
    .reg1_rdata         (reg1_rdata),
    .reg2_raddr         (reg2_raddr), 
    .reg2_rdata         (reg2_rdata),

    .reg_f              (reg_f), 
    .reg_ip             (reg_ip), 
    .seg_ES             (seg_ES), 
    .seg_CS             (seg_CS), 
    .seg_SS             (seg_SS), 
    .seg_DS             (seg_DS),

    .rd                 (id_rd), 
    .rd_addr            (id_rd_addr), 
    .rd_io              (id_rd_io),
    .rd_ready           (rd_ready), 
    .rd_data            (rd_data), 
    .rd_word            (id_rd_word),

    .intr               (interrupt_do), 
    .nmi                (), 
    .pic_vec            (interrupt_vector), 
    .intra              (interrupt_done),

    .reg_rd             (dbg_reg_rd), 
    .reg_addr           (dbg_reg_addr),
    .reg_dout           (dbg_reg_dout)
);

// EXECUTE: execute the instruction, with hardwired logic as fast path and microcode for complex instructions
execute u_execute (
    .clk                (clk),          
    .reset              (cpu_reset),

    .ex_ready           (ex_ready), 
    .ex_valid           (ex_valid), 
    .ex_ip_after        (ex_ip_after), 
    .ex_e_segment       (ex_e_segment),
    .ex_mem_rd          (ex_mem_rd), 
    .ex_e_addr_val      (ex_e_addr_val), 
    .ex_e_fulladdr      (ex_e_fulladdr),
    .ex_g_val           (ex_g_val), 
    .ex_imm             (ex_imm),
    .ex_opcode          (ex_opcode), 
    .ex_iclass          (ex_iclass), 
    .ex_modrm           (ex_modrm), 
    .ex_disp            (ex_disp),
    .ex_ucode_entry_next(ex_ucode_entry_next), 
    .ex_ucode_valid     (ex_ucode_valid), 
    .ex_inst            (ex_inst),

    .br_taken           (br_taken), 
    .br_target          (br_target), 
    .br_new_cs          (br_new_cs), 
    .br_new_ip          (br_new_ip),

    .wb_reg_valid       (wb_reg_valid), 
    .wb_reg             (wb_reg), 
    .wb_data            (wb_data),
    .wb_width           (wb_width), 
    .wb_reg2_valid      (wb_reg2_valid), 
    .wb_reg2            (wb_reg2), 
    .wb_data2           (wb_data2),
    .wb_width2          (wb_width2),
    .wb_flags_valid     (wb_flags_valid), 
    .wb_flags           (wb_flags), 
    .wb_flags_update_mask(wb_flags_update_mask),
    .wb_seg_valid       (wb_seg_valid), 
    .wb_seg             (wb_seg), 
    .wb_seg_data        (wb_seg_data),

    .ex_reg             (ex_reg), 
    .reg1_raddr         (ex_reg1_raddr), 
    .reg1_rdata         (ex_reg1_rdata),
    .reg2_raddr         (ex_reg2_raddr), 
    .reg2_rdata         (ex_reg2_rdata),

    .reg_f              (reg_f), 
    .seg_ES             (seg_ES), 
    .seg_CS             (seg_CS), 
    .seg_SS             (seg_SS), 
    .seg_DS             (seg_DS),

    .rd                 (ex_rd), 
    .rd_ready           (rd_ready), 
    .rd_addr            (ex_rd_addr), 
    .rd_data            (rd_data), 
    .rd_word            (ex_rd_word),
    .rd_io              (ex_rd_io),
    .wr_valid           (wr_valid), 
    .wr_ready           (wr_ready), 
    .wr_addr            (wr_addr), 
    .wr_data            (wr_data), 
    .wr_word            (wr_word), 
    .wr_io              (wr_io),

    .reg_wr             (dbg_reg_wr), 
    .reg_addr           (dbg_reg_addr),
    .reg_din            (dbg_reg_din)
);

// WRITE-BACK: write the result to the register file or memory
write_back u_write_back (
    .clk                (clk), 
    .reset              (cpu_reset),

    .wb_reg_valid       (wb_reg_valid), 
    .wb_reg             (wb_reg), 
    .wb_data            (wb_data),
    .wb_width           (wb_width), 
    .wb_reg2_valid      (wb_reg2_valid), 
    .wb_reg2            (wb_reg2), 
    .wb_data2           (wb_data2),
    .wb_width2          (wb_width2),
    .wb_flags_valid     (wb_flags_valid), 
    .wb_flags           (wb_flags), 
    .wb_flags_update_mask(wb_flags_update_mask),
    .wb_seg_valid       (wb_seg_valid), 
    .wb_seg             (wb_seg), 
    .wb_seg_data        (wb_seg_data),

    .br_taken           (br_taken), 
    .br_target          (br_target), 
    .br_new_cs          (br_new_cs), 
    .br_new_ip          (br_new_ip),

    .reg1_we            (reg1_we), 
    .reg1_waddr         (reg1_waddr), 
    .reg1_wdata         (reg1_wdata),
    .reg2_we            (reg2_we), 
    .reg2_waddr         (reg2_waddr), 
    .reg2_wdata         (reg2_wdata),

    .reg_ip             (reg_ip), 
    .seg_ES             (seg_ES), 
    .seg_CS             (seg_CS), 
    .seg_SS             (seg_SS), 
    .seg_DS             (seg_DS),
    
    .reg_wr             (dbg_reg_wr), 
    .reg_addr           (dbg_reg_addr), 
    .reg_din            (dbg_reg_din)
);


// debug: set loading address when CS or IP is changed
reg [15:0] dbg_reg_cs;
always @(posedge clk) begin
    dbg_set_addr <= 0;
    if (dbg_reg_wr) begin
        if (dbg_reg_addr == 4'd12) begin  // IP
            dbg_reset_ip <= dbg_reg_din;
            dbg_reset_addr <= {dbg_reg_cs, 4'h0} + dbg_reg_din;
        end
        if (dbg_reg_addr == 4'd8) begin  // CS
            dbg_reg_cs <= dbg_reg_din;
            dbg_reset_addr <= {dbg_reg_din, 4'h0} + dbg_reset_ip;
        end
    end
end

endmodule
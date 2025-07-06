// z86 CPU with simulated sdram for regression tests in `tests/`

module z86_test (
    input         clk_sys,
    input         reset,
    input         cpu_reset,

	input         dbg_mem_wr,
	input  [19:0] dbg_mem_addr,
	input  [7:0]  dbg_mem_din,
	input         dbg_mem_rd,
	output [7:0]  dbg_mem_dout,

	input         dbg_reg_wr,
	input  [3:0]  dbg_reg_addr,
	input  [15:0] dbg_reg_din,
	input         dbg_reg_rd,
	output [15:0] dbg_reg_dout
);

wire [19:0] mem_address;
wire [31:0] mem_writedata;
wire [31:0] mem_readdata;
wire  [3:0] mem_byteenable;
wire  [3:0] mem_burstcount;
wire        mem_write;
wire        mem_read;
wire        mem_waitrequest;
wire        mem_readdatavalid;

wire        interrupt_do;
wire [7:0]  interrupt_vector;
wire        interrupt_done;

wire [15:0] cpu_io_read_address;
wire        cpu_io_read_do;
logic       cpu_io_read_done;
wire [15:0] cpu_io_read_word;
wire [15:0] cpu_io_read_data;
wire [15:0] cpu_io_write_address;
wire        cpu_io_write_do;
wire        cpu_io_write_done;
wire [15:0] cpu_io_write_word;
wire [15:0] cpu_io_write_data;

wire        a20_enable;

wire [19:0] dma_address;
wire        dma_16bit;
wire        dma_read;
wire [15:0] dma_readdata;
wire        dma_readdatavalid;
wire        dma_waitrequest;
wire        dma_write;
wire [15:0] dma_writedata;

sdram_sim ram (
    .clk               (clk_sys),
    .reset             (reset),
   
    .cpu_addr          (dbg_mem_wr ? dbg_mem_addr : mem_address),
    .cpu_din           (dbg_mem_wr ? {4{dbg_mem_din}} : mem_writedata),
    .cpu_dout          (mem_readdata),
    .cpu_dout_ready    (mem_readdatavalid),
    .cpu_be            (dbg_mem_wr ? (1 << dbg_mem_addr[1:0]) : mem_byteenable),
    .cpu_burstcount    (mem_burstcount),
    .cpu_busy          (mem_waitrequest),
    .cpu_rd            (mem_read),
    .cpu_we            (dbg_mem_wr ? 1'b1 : mem_write)
);

z86 z86 (
    .clk               (clk_sys),
    .reset             (reset),
	.cpu_reset         (cpu_reset),

	.avm_address       (mem_address),
	.avm_writedata     (mem_writedata),
	.avm_byteenable    (mem_byteenable),
	.avm_burstcount    (mem_burstcount),
	.avm_write         (mem_write),
	.avm_read          (mem_read),
	.avm_waitrequest   (mem_waitrequest),
	.avm_readdatavalid (mem_readdatavalid),
	.avm_readdata      (mem_readdata),

	.interrupt_do      (interrupt_do),
	.interrupt_vector  (interrupt_vector),
	.interrupt_done    (interrupt_done),

	.io_read_do        (cpu_io_read_do),
	.io_read_address   (cpu_io_read_address),
	.io_read_word      (cpu_io_read_word),
	.io_read_data      (cpu_io_read_data),
	.io_read_done      (cpu_io_read_done),
	.io_write_do       (cpu_io_write_do),
	.io_write_address  (cpu_io_write_address),
	.io_write_word     (cpu_io_write_word),
	.io_write_data     (cpu_io_write_data),
	.io_write_done     (cpu_io_write_done),

	.a20_enable        (a20_enable),

	.dma_address       (dma_address),
	.dma_16bit         (dma_16bit),
	.dma_read          (dma_read),
	.dma_readdata      (dma_readdata),
	.dma_readdatavalid (dma_readdatavalid),
	.dma_waitrequest   (dma_waitrequest),
	.dma_write         (dma_write),
	.dma_writedata     (dma_writedata),

	.dbg_reg_wr        (dbg_reg_wr),
	.dbg_reg_addr      (dbg_reg_addr),
	.dbg_reg_din       (dbg_reg_din),
	.dbg_reg_rd        (dbg_reg_rd),
	.dbg_reg_dout      (dbg_reg_dout)
);

// Simple I/O read simulation, always FFFF
assign cpu_io_read_data = 16'hFFFF;
always @(posedge clk_sys) begin
	cpu_io_read_done <= 0;
    if (cpu_io_read_do) begin
        $display("IO read: %x", cpu_io_read_address);
		cpu_io_read_done <= 1;
    end
end


endmodule
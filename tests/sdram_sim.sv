// SDRAM simulation that supports burst read, and VGA memory access

module sdram_sim (
    input             clk,
    input             reset,
    input      [19:0] cpu_addr,
    input      [31:0] cpu_din,
    output reg [31:0] cpu_dout,
    output reg        cpu_dout_ready,
    input      [3:0]  cpu_be,            // byte enable for writes
    input      [7:0]  cpu_burstcount,    // burst count for reads
    output reg        cpu_busy,
    input             cpu_rd,
    input             cpu_we,

    output reg [16:0] vga_address,
    input      [7:0]  vga_readdata,
    output reg [7:0]  vga_writedata,
    input      [2:0]  vga_memmode,
    output reg        vga_read,
    output reg        vga_write,

    input      [5:0]  vga_wr_seg,
    input      [5:0]  vga_rd_seg,
    input             vga_fb_en
);

localparam DWORD_CNT = 1024*1024/4;

logic [31:0] mem [0:DWORD_CNT-1] /* verilator public */;

logic [2:0] state; 
localparam IDLE = 0;
localparam READ_BURST = 1;
localparam VGA_READ_WAIT = 2;
localparam VGA_READ_BYTE1 = 3;
localparam VGA_READ_BYTE2 = 4;
localparam VGA_WRITE2 = 5;

assign cpu_busy = (state != IDLE);

logic [7:0] burst_left;
logic [19:2] burst_addr;

reg   [1:0] vga_mask;
reg   [1:0] vga_cmp;
reg         vga_word, vga_word_t;
reg   [7:0] vga_writedata2;
reg   [1:0] boff_t, boff;

// = 0xA0000-0xBFFFF (VGA: exact region depends on VGA_MODE)
wire vga_rgn = (cpu_addr[19:17] == 'h5) && ((cpu_addr[16:15] & vga_mask) == vga_cmp); 

always @(posedge clk) begin
    boff_t = 0;
    if (reset) begin
        state <= IDLE;
        burst_left <= 0;
`ifdef VERILATOR
        for (int i = 0; i < DWORD_CNT; i++) begin
            mem[i] = {4{8'hF4}};     // HLT
        end
`endif
    end else begin
        cpu_dout_ready <= 0;
        vga_read <= 0;
        vga_write <= 0;
        case (state)
            IDLE: begin
                if (vga_rgn & (cpu_rd | cpu_we)) begin
                    // We assume one or two consecutive 1's in avm_byteenable
                    vga_word_t = 0;
                    if (cpu_be[0]) begin
                        vga_address <= {cpu_addr[16:2], 2'd0};
                        vga_word_t = cpu_be[1];
                        boff_t = 0;
                    end else if (cpu_be[1]) begin
                        vga_address <= {cpu_addr[16:2], 2'd1};
                        vga_word_t = cpu_be[2];
                        boff_t = 1;
                    end else if (cpu_be[2]) begin
                        vga_address <= {cpu_addr[16:2], 2'd2};
                        vga_word_t = cpu_be[3];
                        boff_t = 2;
                    end else begin
                        vga_address <= {cpu_addr[16:2], 2'd3};
                        boff_t = 3;
                    end
                    boff <= boff_t;
                    vga_word <= vga_word_t;
                end
                if (cpu_rd) begin
                    if (vga_rgn) begin
                        // VGA read, byte or word only
                        vga_read <= 1'b1;
                        state <= VGA_READ_WAIT;
                    end else begin
                        // Main memory read, supports burst
                        cpu_dout <= mem[cpu_addr[19:2]];
                        cpu_dout_ready <= 1;
                        if (cpu_burstcount > 1) begin
                            state <= READ_BURST;
                            burst_left <= cpu_burstcount - 1;
                            burst_addr <= cpu_addr[19:2] + 1;
                        end
                    end
                end else if (cpu_we) begin
                    if (vga_rgn) begin
                        // VGA write, byte or word only
                        vga_write <= 1'b1;
                        vga_writedata <= cpu_din[boff_t*8 +: 8];
                        if (vga_word_t) begin
                            vga_writedata2 <= cpu_din[boff_t*8 + 8 +: 8];
                            state <= VGA_WRITE2;
                        end
                        // $display("VGA write: [%h]=%h, byteenable=%b", cpu_addr, cpu_din, cpu_be); 
                    end else begin
                        // Main memory write, supports burst
                        if (cpu_be[0]) mem[cpu_addr[19:2]][7:0] <= cpu_din[7:0];
                        if (cpu_be[1]) mem[cpu_addr[19:2]][15:8] <= cpu_din[15:8];
                        if (cpu_be[2]) mem[cpu_addr[19:2]][23:16] <= cpu_din[23:16];
                        if (cpu_be[3]) mem[cpu_addr[19:2]][31:24] <= cpu_din[31:24];
                    end
                end
            end
            READ_BURST: begin
                if (burst_left == 0) begin
                    state <= IDLE;
                end else begin
                    cpu_dout <= mem[burst_addr];
                    cpu_dout_ready <= 1;
                    burst_addr <= burst_addr + 1;
                    burst_left <= burst_left - 1;
                end
            end
            VGA_READ_WAIT: begin
                if (vga_word) begin
                    vga_address[1:0] <= vga_address[1:0] + 2'd1;
                end 
                state <= VGA_READ_BYTE1;
            end
            VGA_READ_BYTE1: begin    // byte 1 ready
                cpu_dout[boff * 8 +: 8] <= vga_readdata;
                if (vga_word) begin
                    state <= VGA_READ_BYTE2;
                end else begin
                    cpu_dout_ready <= 1;
                    // $display("VGA byte read: [%h]=%h", vga_address, vga_readdata);
                    state <= IDLE;
                end
            end
            VGA_READ_BYTE2: begin    // byte 2 ready
                cpu_dout[boff * 8 + 8 +: 8] <= vga_readdata;
                cpu_dout_ready <= 1;
                state <= IDLE;
                // $display("VGA word read: [%h]=%02h%02h", vga_address-1, cpu_dout[boff*8+:8], vga_readdata);
            end
            VGA_WRITE2: begin        // write byte 2
                vga_address[1:0] <= vga_address[1:0] + 2'd1;
                vga_writedata <= vga_writedata2;
                vga_write <= 1;
                state <= IDLE;
            end
            default: ;
        endcase
    end
end

always @(posedge clk) begin
	case (vga_memmode)
		3'b100:		// 128K
			begin
				vga_mask <= 2'b00;
				vga_cmp  <= 2'b00;
			end
		
		3'b101:		// lower 64K
			begin
				vga_mask <= 2'b10;
				vga_cmp  <= 2'b00;
			end
		
		3'b110:		// 3rd 32K
			begin
				vga_mask <= 2'b11;
				vga_cmp  <= 2'b10;
			end
		
		3'b111:		// top 32K
			begin
				vga_mask <= 2'b11;
				vga_cmp  <= 2'b11;
			end
		
		default :	// disable VGA RAM
			begin
				vga_mask <= 2'b00;
				vga_cmp  <= 2'b11;
			end
	endcase
end


endmodule
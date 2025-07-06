// Connect FETCH/DECODE/EXECUTE stage to a 32-bit Avalon memory interface
// TODO: actually implement L1 cache

module cache(
    input              clk,
    input              reset,

    // Instruction loads from FETCH
    input       [19:0] ld_addr,
    input              ld_req,          // load request toggle
    output reg         ld_ack,
    output reg [127:0] ld_data,         // whole cacheline output
    output reg         ld_hit,          // 1 if cache hit, combinatorial

    // Data reads from EXECUTE or DECODE (memory or I/O)
    input      [19:0]  rd_addr,
    input              rd,              // read pulse
    output reg         rd_ready = 1'b1, // idle or data ready
    output reg [15:0]  rd_data,
    input              rd_io,
    input              rd_word,

    // Data writes from EXECUTE (memory or I/O)
    input       [19:0] wr_addr,
    input              wr_valid,        // write request, hold high until wr_ready is high
    output reg         wr_ready,        // valid/ready pair, request done when both are high
    input       [15:0] wr_data,
    input              wr_word,         // write 16-bit word
    input              wr_io,

    // Memory interface
    output reg [19:0]  avm_address,
    output reg [31:0]  avm_writedata,
    output reg [3:0]   avm_byteenable,
    output reg [3:0]   avm_burstcount,
    output reg         avm_write,
    output reg         avm_read,
    input              avm_waitrequest, 
    input              avm_readdatavalid,
    input      [31:0]  avm_readdata,

    // I/O interface
    output             io_read_do,
    output     [15:0]  io_read_address,
    output             io_read_word,
    input      [15:0]  io_read_data,
    input              io_read_done,

    output             io_write_do,
    output     [15:0]  io_write_address,
    output             io_write_word,
    output     [15:0]  io_write_data,
    input              io_write_done,

    // DEBUG interface
    input              dbg_mem_wr,
    input       [19:0] dbg_mem_addr,
    input       [7:0]  dbg_mem_din,
    input              dbg_mem_rd,
    output reg [7:0]   dbg_mem_dout
);

// Pending requests as read request is pulse
logic [19:0] rd_addr_pending;
logic rd_pending, rd_io_pending, rd_word_pending;

logic [1:0] ld_offset;
logic [1:0] rd_offset;

// FSM for I/O and memory
logic [1:0] io_state; 
localparam IO_IDLE = 0;
localparam IO_READING = 1;
localparam IO_WRITING = 2;
logic [2:0] mem_state;
localparam MEM_IDLE = 0;
localparam MEM_LOADING = 1;
localparam MEM_READING = 2;
localparam MEM_READING2 = 3;
localparam MEM_WRITING = 4;
localparam MEM_WRITING2 = 5;

logic wr_valid_r;

// Output signals
always_comb begin
    automatic logic [19:0] rd_a = rd_pending ? rd_addr_pending : rd_addr;

    avm_read = 0;
    avm_write = 0;
    avm_burstcount = 1;
    avm_address = 'x;
    avm_writedata = 'x;
    avm_byteenable = 'x;
    io_read_do = 0;
    io_write_do = 0;
    io_write_address = 'x;
    io_write_word = 0;
    io_write_data = 'x;
    io_read_address = 'x;
    io_read_word = 0;

    if (reset) begin
    end else begin
        // Request to memory
        if (mem_state == MEM_IDLE) begin
            if (ld_req ^ ld_ack) begin
                avm_read = 1;
                avm_address = ld_addr;
                avm_burstcount = 4;
            end else if (wr_valid & ~wr_io & ~wr_ready) begin
                if (wr_addr == 20'h3c2) begin
                    // $display("write [%x]=%x", wr_addr, wr_data);
                end
                // write needs to be done before read to avoid RAW hazard
                avm_write = 1;
                avm_address = wr_addr;
                casez ({wr_word, wr_addr[1:0]}) 
                3'b0??: begin 
                    avm_writedata = {4{wr_data[7:0]}};
                    avm_byteenable = 4'b0001 << wr_addr[1:0];
                end
                3'b10?, 3'b110: begin
                    avm_writedata = wr_data << (8*wr_addr[1:0]);
                    avm_byteenable = 4'b11 << wr_addr[1:0];
                end
                3'b111: begin
                    avm_writedata = wr_data[7:0] << 24;
                    avm_byteenable = 4'b1000;
                end
                endcase
            end else if (rd | rd_pending) begin
                avm_read = 1;
                avm_address = rd_a;
                avm_byteenable = (rd_word ? 4'b11 : 4'b01) << rd_a[1:0];
            end
        end
        if (mem_state == MEM_READING & avm_readdatavalid & rd_word & rd_offset == 2'd3) begin
            avm_read = 1;
            avm_address = rd_addr_pending + 20'd4;
            avm_byteenable = 4'b1;
        end
        if (mem_state == MEM_WRITING & ~avm_waitrequest & wr_word & wr_addr[1:0] == 2'd3) begin
            avm_write = 1;
            avm_address = wr_addr + 20'd4;     // wr_addr is valid as long as wr_valid is 1
            avm_writedata = wr_data[15:8];
            avm_byteenable = 4'd1;
        end

        if (io_state == IO_IDLE) begin
            if (wr_valid & wr_io & ~wr_ready) begin
                io_write_do = 1;
                io_write_address = wr_addr;
                io_write_word = wr_word;
                io_write_data = wr_data;
            end else if (rd & rd_io | rd_pending & rd_io_pending) begin
                io_read_do = 1;
                io_read_address = rd_pending ? rd_addr_pending : rd_addr;
                io_read_word = rd_word;
            end
        end
    end
end

// FSM logic
// Memory:
// - If new loading request, enter MEM_LOADING and start loading
// - Enter writing and reading state similarly, if there's no loading
// - When in loading/writing/reading, return to IDLE when done
// - Save reading request to rd_pending when we are not in IDLE, as it is pulse request.
//   Then also check for pending read request in IDLE.
// - Toggle ld_ack after the 4th 32-bit word is received.
// - Pulse wr_ready when in MEM_WRITING and avm_waitrequest is low
// - rd_ready=1 when not in MEM_READING and there's no pending read request
//
// I/O is simpler as there's only read and write, no loading.

// Memory FSM
always @(posedge clk) begin
    if (reset) begin
        mem_state <= MEM_IDLE;
        io_state <= IO_IDLE;
    end else begin
        if (rd) begin
            rd_pending <= 1;
            rd_io_pending <= rd_io;
            rd_ready <= 0;
            rd_addr_pending <= rd_addr;
        end
        wr_ready <= 0;
        
        // Memory FSM
        case (mem_state)
        MEM_IDLE: begin
            if (ld_req ^ ld_ack) begin
                mem_state <= MEM_LOADING;
            end else if (wr_valid & ~wr_io & ~wr_ready) begin
                mem_state <= MEM_WRITING;
            end else if (rd & ~rd_io | rd_pending & ~rd_io_pending) begin
                rd_pending <= 0;
                mem_state <= MEM_READING;
                rd_offset <= rd_pending ? rd_addr_pending[1:0] : rd_addr[1:0];
            end
        end
        MEM_LOADING: begin
            if (avm_readdatavalid) begin
                ld_data[32*ld_offset +: 32] <= avm_readdata;
                ld_offset <= ld_offset + 1;
                if (ld_offset == 2'd3) begin
                    ld_ack <= ld_req;
                    mem_state <= MEM_IDLE;
                end
            end
        end
        MEM_READING: begin
            if (avm_readdatavalid) begin
                rd_data[7:0] <= avm_readdata[8*rd_offset +: 8];
                if (rd_word & rd_offset == 2'd3) begin
                    // need a second read to get the second byte
                    mem_state <= MEM_READING2;
                end else begin
                    rd_data[15:8] <= avm_readdata[8*rd_offset + 8 +: 8];
                    rd_ready <= 1;
                    mem_state <= MEM_IDLE;
                end
            end
        end
        MEM_READING2: begin
            if (avm_readdatavalid) begin
                rd_data[15:8] <= avm_readdata[7:0];
                rd_ready <= 1;
                mem_state <= MEM_IDLE;
            end
        end
        MEM_WRITING: begin
            if (~avm_waitrequest) begin
                if (wr_word & wr_addr[1:0] == 2'd3) begin
                    // need a second write to write the second byte
                    mem_state <= MEM_WRITING2;
                end else begin
                    mem_state <= MEM_IDLE;
                    wr_ready <= 1;
                end
            end
        end
        MEM_WRITING2: begin
            if (~avm_waitrequest) begin
                mem_state <= MEM_IDLE;
                wr_ready <= 1;
            end
        end
        default: ;
        endcase

        // I/O FSM
        case (io_state)
        IO_IDLE: begin
            if (wr_valid & wr_io & ~wr_ready) begin
                io_state <= IO_WRITING;
            end else if (rd & rd_io | rd_pending & rd_io_pending)  begin
                io_state <= IO_READING;
                rd_pending <= 0;
            end
        end
        IO_READING: begin
            if (io_read_done) begin
                io_state <= IO_IDLE;
                rd_data <= io_read_data;
                rd_ready <= 1;
            end
        end
        IO_WRITING: begin
            if (io_write_done) begin
                io_state <= IO_IDLE;
                wr_ready <= 1;
            end
        end
        default: io_state <= IO_IDLE;
        endcase        
    end
end


endmodule
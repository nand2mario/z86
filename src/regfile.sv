// 4-read, 2-write register file
// 0-7 are general purpose registers: AX, CX, DX, BX, SP, BP, SI, DI
module regfile (
    input          clk,
    // read ports for ID stage
    input       [2:0]  id_raddr1,
    output reg [15:0]  id_rdata1,
    input       [2:0]  id_raddr2,
    output reg [15:0]  id_rdata2,

    // read ports for EX stage
    input       [2:0]  ex_raddr1,
    output reg [15:0]  ex_rdata1,
    input       [2:0]  ex_raddr2,
    output reg [15:0]  ex_rdata2,

    // write ports for WB stage
    input       [1:0]  we1,        // byte enabled write
    input       [2:0]  waddr1,
    input      [15:0]  wdata1,
    input      [1:0]   we2,        // byte enabled write
    input      [2:0]   waddr2,
    input     [15:0]   wdata2
);

import z86_package::*;

reg [15:0] regs [0:7] /* verilator public */;
reg [15:0] regs2 [0:7];            // replicated regs for EX stage

wire [15:0] AX /* verilator public */ = regs[R_AX];
wire [15:0] CX /* verilator public */ = regs[R_CX];
wire [15:0] DX /* verilator public */ = regs[R_DX];
wire [15:0] BX /* verilator public */ = regs[R_BX];
wire [15:0] SP /* verilator public */ = regs[R_SP];
wire [15:0] BP /* verilator public */ = regs[R_BP];
wire [15:0] SI /* verilator public */ = regs[R_SI];
wire [15:0] DI /* verilator public */ = regs[R_DI];

always_comb begin
    id_rdata1 = regs[id_raddr1];
    id_rdata2 = regs[id_raddr2];
    ex_rdata1 = regs2[ex_raddr1];
    ex_rdata2 = regs2[ex_raddr2];

    if (CONFIG_FORWARDING_REGFILE) begin
        // Same-cycle register value forwarding
        if (id_raddr1 == waddr1 && we1[0]) id_rdata1[7:0]  = wdata1[7:0];
        if (id_raddr1 == waddr1 && we1[1]) id_rdata1[15:8] = wdata1[15:8];
        if (id_raddr1 == waddr2 && we2[0]) id_rdata1[7:0]  = wdata2[7:0];
        if (id_raddr1 == waddr2 && we2[1]) id_rdata1[15:8] = wdata2[15:8];
        if (id_raddr2 == waddr1 && we1[0]) id_rdata2[7:0]  = wdata1[7:0];
        if (id_raddr2 == waddr1 && we1[1]) id_rdata2[15:8] = wdata1[15:8];
        if (id_raddr2 == waddr2 && we2[0]) id_rdata2[7:0]  = wdata2[7:0];
        if (id_raddr2 == waddr2 && we2[1]) id_rdata2[15:8] = wdata2[15:8];
        
        // Forwarding for EX stage
        if (ex_raddr1 == waddr1 && we1[0]) ex_rdata1[7:0]  = wdata1[7:0];
        if (ex_raddr1 == waddr1 && we1[1]) ex_rdata1[15:8] = wdata1[15:8];
        if (ex_raddr1 == waddr2 && we2[0]) ex_rdata1[7:0]  = wdata2[7:0];
        if (ex_raddr1 == waddr2 && we2[1]) ex_rdata1[15:8] = wdata2[15:8];
        if (ex_raddr2 == waddr1 && we1[0]) ex_rdata2[7:0]  = wdata1[7:0];
        if (ex_raddr2 == waddr1 && we1[1]) ex_rdata2[15:8] = wdata1[15:8];
        if (ex_raddr2 == waddr2 && we2[0]) ex_rdata2[7:0]  = wdata2[7:0];
        if (ex_raddr2 == waddr2 && we2[1]) ex_rdata2[15:8] = wdata2[15:8];
    end
end

always_ff @(posedge clk) begin
    if (we1[0]) begin regs[waddr1][7:0]  <= wdata1[7:0];  regs2[waddr1][7:0]  <= wdata1[7:0];  end
    if (we1[1]) begin regs[waddr1][15:8] <= wdata1[15:8]; regs2[waddr1][15:8] <= wdata1[15:8]; end
    if (we2[0]) begin regs[waddr2][7:0]  <= wdata2[7:0];  regs2[waddr2][7:0]  <= wdata2[7:0];  end
    if (we2[1]) begin regs[waddr2][15:8] <= wdata2[15:8]; regs2[waddr2][15:8] <= wdata2[15:8]; end

    // if (we1) begin
    //     $display("regfile: write %x to %d, we1 = %d", wdata1, waddr1, we1);
    // end
    // if (we2) begin
    //     $display("regfile: write %x to %d, we2 = %d", wdata2, waddr2, we2);
    // end
end

endmodule
// DDRAM Arbiter for Sega 32X RetroAchievements
//
// Sits between ddram module (primary) and RA mirror (secondary).
// ddram is the primary master (handles 32X SH2 SDRAM accesses).
// RA mirror is secondary (toggle req/ack protocol, read+write).
// RA requests are serviced when ddram is idle on the bus.
//
// RA toggle signals originate from clk_sys domain; arbiter runs on
// clk_ram and synchronizes them accordingly.

module ddram_arb_s32x (
input         clk,            // clk_ram

// Physical DDRAM interface (directly to top-level DDRAM ports)
input         PHY_BUSY,
output  [7:0] PHY_BURSTCNT,
output [28:0] PHY_ADDR,
input  [63:0] PHY_DOUT,
input         PHY_DOUT_READY,
output        PHY_RD,
output [63:0] PHY_DIN,
output  [7:0] PHY_BE,
output        PHY_WE,

// ddram module interface (primary master)
output        DDR_BUSY,
input   [7:0] DDR_BURSTCNT,
input  [28:0] DDR_ADDR,
output [63:0] DDR_DOUT,
output        DDR_DOUT_READY,
input         DDR_RD,
input  [63:0] DDR_DIN,
input   [7:0] DDR_BE,
input         DDR_WE,

// RetroAchievements write channel (toggle req/ack, from clk_sys)
input  [28:0] ra_wr_addr,
input  [63:0] ra_wr_din,
input   [7:0] ra_wr_be,
input         ra_wr_req,
output reg    ra_wr_ack,

// RetroAchievements read channel (toggle req/ack, from clk_sys)
input  [28:0] ra_rd_addr,
input         ra_rd_req,
output reg    ra_rd_ack,
output reg [63:0] ra_rd_dout
);

// Synchronize RA toggle signals (come from clk_sys domain)
reg ra_wr_req_s1, ra_wr_req_s2;
reg ra_rd_req_s1, ra_rd_req_s2;
always @(posedge clk) begin
ra_wr_req_s1 <= ra_wr_req; ra_wr_req_s2 <= ra_wr_req_s1;
ra_rd_req_s1 <= ra_rd_req; ra_rd_req_s2 <= ra_rd_req_s1;
end

// State machine
localparam S_PASSTHRU = 2'd0;
localparam S_RA_WR    = 2'd1;
localparam S_RA_RD    = 2'd2;
localparam S_RA_WAIT  = 2'd3;

reg [1:0] state = S_PASSTHRU;

// Track pending ddram read bursts to avoid mid-burst interruption
reg        ddr_rd_active = 0;
reg  [7:0] ddr_burst_cnt = 0;

// Combinational mux
assign PHY_BURSTCNT = (state == S_PASSTHRU) ? DDR_BURSTCNT : 8'd1;
assign PHY_ADDR     = (state == S_PASSTHRU) ? DDR_ADDR     :
                      (state == S_RA_WR)    ? ra_wr_addr   : ra_rd_addr;
assign PHY_DIN      = (state == S_PASSTHRU) ? DDR_DIN      : ra_wr_din;
assign PHY_BE       = (state == S_PASSTHRU) ? DDR_BE       :
                      (state == S_RA_WR)    ? ra_wr_be     : 8'hFF;
assign PHY_WE       = (state == S_RA_WR)    ? 1'b1         :
                      (state == S_PASSTHRU) ? DDR_WE       : 1'b0;
assign PHY_RD       = (state == S_RA_RD)    ? 1'b1         :
                      (state == S_PASSTHRU) ? DDR_RD       : 1'b0;

assign DDR_BUSY       = (state != S_PASSTHRU) ? 1'b1 : PHY_BUSY;
assign DDR_DOUT       = PHY_DOUT;
assign DDR_DOUT_READY = (state == S_PASSTHRU) ? PHY_DOUT_READY : 1'b0;

always @(posedge clk) begin
// Track pending ddram read bursts
if (state == S_PASSTHRU) begin
if (DDR_RD && !PHY_BUSY) begin
ddr_rd_active <= 1'b1;
ddr_burst_cnt <= DDR_BURSTCNT;
end
if (ddr_rd_active && PHY_DOUT_READY) begin
if (ddr_burst_cnt <= 8'd1)
ddr_rd_active <= 1'b0;
else
ddr_burst_cnt <= ddr_burst_cnt - 8'd1;
end
end

case (state)
S_PASSTHRU: begin
// Only steal bus when ddram is idle
if (!DDR_WE && !DDR_RD && !PHY_BUSY && !ddr_rd_active) begin
if (ra_wr_req_s2 != ra_wr_ack)
state <= S_RA_WR;
else if (ra_rd_req_s2 != ra_rd_ack)
state <= S_RA_RD;
end
end

S_RA_WR: begin
if (!PHY_BUSY) begin
ra_wr_ack <= ra_wr_req_s2;
state <= S_PASSTHRU;
end
end

S_RA_RD: begin
if (!PHY_BUSY) begin
state <= S_RA_WAIT;
end
end

S_RA_WAIT: begin
if (PHY_DOUT_READY) begin
ra_rd_dout <= PHY_DOUT;
ra_rd_ack  <= ra_rd_req_s2;
state      <= S_PASSTHRU;
end
end
endcase
end

endmodule

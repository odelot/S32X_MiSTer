// RetroAchievements RAM Mirror for Sega 32X — Option C  (v0x01)
//
// Each VBlank, reads a list of specific addresses from DDRAM (written by ARM),
// fetches byte values from 68K Work RAM (BRAM) or 32X SH2 SDRAM (DDR3),
// and writes them back to DDRAM.
//
// Memory map for rcheevos:
//   0x00000-0x0FFFF : 68K Work RAM (64KB, mapped to BRAM)
//   0x10000-0x4FFFF : 32X SH2 SDRAM (256KB, via DDR3)
//   other           : returns 0
//
// DDRAM Layout (at DDRAM_BASE, ARM phys 0x3D000000):
//   [0x00000] Header:  magic(32) + 0(8) + flags(8) + 0(16)
//   [0x00008] Frame:   frame_counter(32) + 0(32)
//   [0x40000] AddrReq: addr_count(32) + request_id(32)      (ARM -> FPGA)
//   [0x40008] Addrs:   addr[0](32) + addr[1](32), ...       (2 per 64-bit word)
//   [0x48000] ValResp: response_id(32) + response_frame(32) (FPGA -> ARM)
//   [0x48008] Values:  val[0..7](8b each), ...              (8 per 64-bit word)

module ra_ram_mirror_s32x #(
parameter [28:0] DDRAM_BASE = 29'h07A00000  // ARM phys 0x3D000000 >> 3
)(
input             clk,           // clk_sys
input             reset,
input             vblank,

// 68K Work RAM BRAM read interface (dpram port B, two 8-bit halves)
output reg [14:0] bram_addr,     // 15-bit word address
input       [7:0] bram_u_dout,  // high byte
input       [7:0] bram_l_dout,  // low byte

// DDRAM write interface (toggle req/ack)
output reg [28:0] ddram_wr_addr,
output reg [63:0] ddram_wr_din,
output reg  [7:0] ddram_wr_be,
output reg        ddram_wr_req,
input             ddram_wr_ack,

// DDRAM read interface (toggle req/ack)
output reg [28:0] ddram_rd_addr,
output reg        ddram_rd_req,
input             ddram_rd_ack,
input      [63:0] ddram_rd_dout,

// Status
output reg        active,
output reg [31:0] dbg_frame_counter
);

// ======================================================================
// Constants
// ======================================================================
localparam [28:0] ADDRLIST_BASE = DDRAM_BASE + 29'h8000;  // byte offset 0x40000 / 8
localparam [28:0] VALCACHE_BASE = DDRAM_BASE + 29'h9000;  // byte offset 0x48000 / 8
localparam [31:0] WRAM_LIMIT    = 32'h10000;              // 68K Work RAM 64KB
localparam [31:0] S32X_LIMIT    = 32'h50000;              // end of 32X SH2 SDRAM
localparam [12:0] MAX_ADDRS     = 13'd4096;

// ======================================================================
// Clock domain crossing synchronizers for DDRAM ack
// ======================================================================
reg dwr_ack_s1, dwr_ack_s2;
reg drd_ack_s1, drd_ack_s2;
always @(posedge clk) begin
dwr_ack_s1 <= ddram_wr_ack; dwr_ack_s2 <= dwr_ack_s1;
drd_ack_s1 <= ddram_rd_ack; drd_ack_s2 <= drd_ack_s1;
end

// ======================================================================
// VBlank edge detection and sticky pending flag
// ======================================================================
reg vblank_prev;
wire vblank_rising = vblank & ~vblank_prev;
always @(posedge clk) vblank_prev <= vblank;

reg vblank_pending;
always @(posedge clk) begin
if (reset)
vblank_pending <= 1'b0;
else begin
if (vblank_rising)
vblank_pending <= 1'b1;
if (vblank_pending && state == S_IDLE)
vblank_pending <= 1'b0;
end
end

// ======================================================================
// State machine
// ======================================================================
localparam S_IDLE          = 5'd0;
localparam S_DD_WR_WAIT    = 5'd1;
localparam S_DD_RD_WAIT    = 5'd2;
localparam S_READ_HDR      = 5'd3;
localparam S_PARSE_HDR     = 5'd4;
localparam S_READ_PAIR     = 5'd5;
localparam S_PARSE_ADDR    = 5'd6;
localparam S_DISPATCH      = 5'd7;
localparam S_BRAM_WAIT     = 5'd8;
localparam S_BRAM_WAIT2    = 5'd9;
localparam S_SDR_RD_ISSUE  = 5'd10;
localparam S_SDR_RD_DONE   = 5'd11;
localparam S_STORE_VAL     = 5'd12;
localparam S_FLUSH_BUF     = 5'd13;
localparam S_WRITE_RESP    = 5'd14;
localparam S_WR_HDR0       = 5'd15;
localparam S_WR_HDR1       = 5'd16;
localparam S_WR_DBG        = 5'd17;

reg [4:0] state;
reg [4:0] return_state;

reg [31:0] frame_counter;
always @(posedge clk) dbg_frame_counter <= frame_counter;

reg [63:0] rd_data;
reg [31:0] req_count;
reg [31:0] req_id;
reg [12:0] addr_idx;
reg [63:0] addr_word;
reg [31:0] cur_addr;
reg [63:0] collect_buf;
reg  [3:0] collect_cnt;
reg [12:0] val_word_idx;
reg  [7:0] fetch_byte;

// 32X SDRAM byte offset within DDR word
reg [17:0] s32x_offset;  // cur_addr - 0x10000

// Debug counters
reg [15:0] dbg_ok_cnt;
reg [15:0] dbg_oob_cnt;
reg  [7:0] dbg_dispatch_cnt;
reg [15:0] dbg_first_sdr_word;

// ======================================================================
// Main state machine
// ======================================================================
always @(posedge clk) begin
if (reset) begin
state         <= S_IDLE;
active        <= 1'b0;
frame_counter <= 32'd0;
ddram_wr_req  <= dwr_ack_s2;
ddram_rd_req  <= drd_ack_s2;
end
else begin
case (state)

S_IDLE: begin
active <= 1'b0;
if (vblank_pending) begin
active            <= 1'b1;
dbg_ok_cnt        <= 16'd0;
dbg_oob_cnt       <= 16'd0;
dbg_dispatch_cnt  <= 8'd0;
dbg_first_sdr_word <= 16'd0;
// Write header with busy=1
ddram_wr_addr <= DDRAM_BASE;
ddram_wr_din  <= {16'd0, 8'h01, 8'd0, 32'h52414348};
ddram_wr_be   <= 8'hFF;
ddram_wr_req  <= ~ddram_wr_req;
return_state  <= S_READ_HDR;
state         <= S_DD_WR_WAIT;
end
end

S_DD_WR_WAIT: begin
if (ddram_wr_req == dwr_ack_s2)
state <= return_state;
end

S_DD_RD_WAIT: begin
if (ddram_rd_req == drd_ack_s2) begin
rd_data <= ddram_rd_dout;
state   <= return_state;
end
end

S_READ_HDR: begin
ddram_rd_addr <= ADDRLIST_BASE;
ddram_rd_req  <= ~ddram_rd_req;
return_state  <= S_PARSE_HDR;
state         <= S_DD_RD_WAIT;
end

S_PARSE_HDR: begin
req_id <= rd_data[63:32];
if (rd_data[31:0] == 32'd0) begin
req_count <= 32'd0;
state     <= S_WRITE_RESP;
end else begin
req_count    <= (rd_data[31:0] > {19'd0, MAX_ADDRS}) ?
                {19'd0, MAX_ADDRS} : rd_data[31:0];
addr_idx     <= 13'd0;
collect_cnt  <= 4'd0;
collect_buf  <= 64'd0;
val_word_idx <= 13'd0;
state        <= S_READ_PAIR;
end
end

S_READ_PAIR: begin
ddram_rd_addr <= ADDRLIST_BASE + 29'd1 + {16'd0, addr_idx[12:1]};
ddram_rd_req  <= ~ddram_rd_req;
return_state  <= S_PARSE_ADDR;
state         <= S_DD_RD_WAIT;
end

S_PARSE_ADDR: begin
// S32X 68K: no FC1004 inversion, bram_addr = cur_addr[15:1] directly
if (!addr_idx[0]) begin
addr_word <= rd_data;
cur_addr  <= rd_data[31:0];
bram_addr <= rd_data[15:1];
end else begin
cur_addr  <= addr_word[63:32];
bram_addr <= addr_word[47:33];
end
state <= S_DISPATCH;
end

S_DISPATCH: begin
dbg_dispatch_cnt <= dbg_dispatch_cnt + 8'd1;
if (cur_addr < WRAM_LIMIT) begin
// 68K Work RAM via BRAM
dbg_ok_cnt <= dbg_ok_cnt + 16'd1;
state      <= S_BRAM_WAIT;
end else if (cur_addr < S32X_LIMIT) begin
// 32X SH2 SDRAM via DDR3
dbg_ok_cnt     <= dbg_ok_cnt + 16'd1;
s32x_offset    <= cur_addr[17:0] - 18'h10000;
state          <= S_SDR_RD_ISSUE;
end else begin
fetch_byte  <= 8'd0;
dbg_oob_cnt <= dbg_oob_cnt + 16'd1;
state       <= S_STORE_VAL;
end
end

S_BRAM_WAIT: begin
state <= S_BRAM_WAIT2;
end

S_BRAM_WAIT2: begin
// RA convention: even addr -> low byte, odd addr -> high byte
fetch_byte <= cur_addr[0] ? bram_u_dout : bram_l_dout;
state <= S_STORE_VAL;
end

S_SDR_RD_ISSUE: begin
// DDR word address = (cur_addr - 0x10000) >> 3
ddram_rd_addr <= {14'd0, s32x_offset[17:3]};
ddram_rd_req  <= ~ddram_rd_req;
return_state  <= S_SDR_RD_DONE;
state         <= S_DD_RD_WAIT;
end

S_SDR_RD_DONE: begin
// rd_data holds 64-bit DDR word.
// SH2 is big-endian: byte 0 at bits[63:56], byte 7 at bits[7:0].
// RA LE convention: XOR bit 0 of byte_pos.
if (dbg_first_sdr_word == 16'd0)
dbg_first_sdr_word <= rd_data[63:48];
case (s32x_offset[2:0] ^ 3'd1)
3'd0: fetch_byte <= rd_data[63:56];
3'd1: fetch_byte <= rd_data[55:48];
3'd2: fetch_byte <= rd_data[47:40];
3'd3: fetch_byte <= rd_data[39:32];
3'd4: fetch_byte <= rd_data[31:24];
3'd5: fetch_byte <= rd_data[23:16];
3'd6: fetch_byte <= rd_data[15:8];
3'd7: fetch_byte <= rd_data[7:0];
endcase
state <= S_STORE_VAL;
end

S_STORE_VAL: begin
case (collect_cnt[2:0])
3'd0: collect_buf[ 7: 0] <= fetch_byte;
3'd1: collect_buf[15: 8] <= fetch_byte;
3'd2: collect_buf[23:16] <= fetch_byte;
3'd3: collect_buf[31:24] <= fetch_byte;
3'd4: collect_buf[39:32] <= fetch_byte;
3'd5: collect_buf[47:40] <= fetch_byte;
3'd6: collect_buf[55:48] <= fetch_byte;
3'd7: collect_buf[63:56] <= fetch_byte;
endcase
collect_cnt <= collect_cnt + 4'd1;
addr_idx    <= addr_idx + 13'd1;

if (collect_cnt == 4'd7 || (addr_idx + 13'd1 >= req_count[12:0])) begin
state <= S_FLUSH_BUF;
end else if (addr_idx[0]) begin
state <= S_READ_PAIR;
end else begin
state <= S_PARSE_ADDR;
end
end

S_FLUSH_BUF: begin
ddram_wr_addr <= VALCACHE_BASE + 29'd1 + {16'd0, val_word_idx};
ddram_wr_din  <= collect_buf;
ddram_wr_be   <= (collect_cnt == 4'd8) ? 8'hFF
                 : ((8'd1 << collect_cnt[2:0]) - 8'd1);
ddram_wr_req  <= ~ddram_wr_req;
val_word_idx  <= val_word_idx + 13'd1;
collect_cnt   <= 4'd0;
collect_buf   <= 64'd0;

if (addr_idx >= req_count[12:0]) begin
return_state <= S_WRITE_RESP;
end else if (!addr_idx[0]) begin
return_state <= S_READ_PAIR;
end else begin
return_state <= S_PARSE_ADDR;
end
state <= S_DD_WR_WAIT;
end

S_WRITE_RESP: begin
ddram_wr_addr <= VALCACHE_BASE;
ddram_wr_din  <= {frame_counter + 32'd1, req_id};
ddram_wr_be   <= 8'hFF;
ddram_wr_req  <= ~ddram_wr_req;
return_state  <= S_WR_HDR0;
state         <= S_DD_WR_WAIT;
end

S_WR_HDR0: begin
ddram_wr_addr <= DDRAM_BASE;
ddram_wr_din  <= {16'd0, 8'h00, 8'd0, 32'h52414348};
ddram_wr_be   <= 8'hFF;
ddram_wr_req  <= ~ddram_wr_req;
return_state  <= S_WR_HDR1;
state         <= S_DD_WR_WAIT;
end

S_WR_HDR1: begin
ddram_wr_addr <= DDRAM_BASE + 29'd1;
ddram_wr_din  <= {32'd0, frame_counter + 32'd1};
ddram_wr_be   <= 8'hFF;
ddram_wr_req  <= ~ddram_wr_req;
frame_counter <= frame_counter + 32'd1;
return_state  <= S_WR_DBG;
state         <= S_DD_WR_WAIT;
end

// Debug word @ DDRAM_BASE+2 (offset 0x10):
// {ver(8), dispatch(8), ok_cnt(16), oob_cnt(16), first_sdr_word(16)}
S_WR_DBG: begin
ddram_wr_addr <= DDRAM_BASE + 29'd2;
ddram_wr_din  <= {8'h01, dbg_dispatch_cnt, dbg_ok_cnt, dbg_oob_cnt, dbg_first_sdr_word};
ddram_wr_be   <= 8'hFF;
ddram_wr_req  <= ~ddram_wr_req;
return_state  <= S_IDLE;
state         <= S_DD_WR_WAIT;
end

default: state <= S_IDLE;
endcase
end
end

endmodule


`default_nettype none
module secdec #(
parameter DATA_WIDTH = 8,
parameter FIFO_DEPTH = 4
)(
input wire clk,
input wire rst,
input wire wr_en,
input wire [DATA_WIDTH-1:0] data_in,
input wire rd_en,
output reg [DATA_WIDTH-1:0] data_out,
output reg ack,
output reg nack,
input wire [1:0] err_mode
);
// encoder
wire [12:0] cw_in;
secdec_encoder enc (.data_in(data_in), .codeword_out(cw_in));
// fifo
wire [12:0] cw_fifo;
wire fifo_dv, fifo_empty;
fifo #(.DATA_WIDTH(13), .DEPTH(FIFO_DEPTH)) u_fifo (
.clk(clk), .rst(rst),
.wr_en(wr_en), .rd_en(rd_en),
.din(cw_in),
.dout(cw_fifo), .dout_valid(fifo_dv),
.full(),
.empty(fifo_empty)
);
// latch err_mode on read
reg [1:0] err_q;
always @(posedge clk or posedge rst) begin
if (rst) err_q <= 2'b00;
else if (rd_en && !fifo_empty) err_q <= err_mode;
end
// corrupt data if requested
wire [12:0] cw_corrupt =
(err_q == 2'b01) ? (cw_fifo ^ 13'h004) :
(err_q == 2'b10) ? (cw_fifo ^ 13'h014) :
cw_fifo;
// decoder
wire [7:0] dec_data;
wire d_err;
secdec_decoder dec (
.codeword_in(cw_corrupt),
.data_out(dec_data),
.single_err(),
.double_err(d_err)
);
// direct use of fifo_dv (no extra cycle delay)
always @(posedge clk or posedge rst) begin
if (rst) begin
data_out <= {DATA_WIDTH{1'b0}};
ack <= 1'b0;
nack <= 1'b0;
end else begin
ack <= 1'b0;
nack <= 1'b0;
if (fifo_dv) begin
data_out <= dec_data;
if (d_err) nack <= 1'b1;
else ack <= 1'b1;
end
end
end
endmodule
module secdec_encoder (
input wire [7:0] data_in,
output wire [12:0] codeword_out
);
wire p1, p2, p4, p8;
assign p1 = data_in[0] ^ data_in[1] ^ data_in[3] ^ data_in[4] ^ data_in[6];
assign p2 = data_in[0] ^ data_in[2] ^ data_in[3] ^ data_in[5] ^ data_in[6];
assign p4 = data_in[1] ^ data_in[2] ^ data_in[3] ^ data_in[7];
assign p8 = data_in[4] ^ data_in[5] ^ data_in[6] ^ data_in[7];
wire p0 = ^{data_in, p1, p2, p4, p8};
assign codeword_out = {
p0,
data_in[7],
data_in[6],
data_in[5],
data_in[4],
p8,
data_in[3],
data_in[2],
data_in[1],
p4,
data_in[0],
p2,
p1
};
endmodule
module secdec_decoder (
input wire [12:0] codeword_in,
output wire [7:0] data_out,
output wire single_err,
output wire double_err
);
wire p1_rx = codeword_in[0];
wire p2_rx = codeword_in[1];
wire d0 = codeword_in[2];
wire p4_rx = codeword_in[3];
wire d1 = codeword_in[4];
wire d2 = codeword_in[5];
wire d3 = codeword_in[6];
wire p8_rx = codeword_in[7];
wire d4 = codeword_in[8];
wire d5 = codeword_in[9];
wire d6 = codeword_in[10];
wire d7 = codeword_in[11];
wire p0_rx = codeword_in[12];
wire p1_calc = d0 ^ d1 ^ d3 ^ d4 ^ d6;
wire p2_calc = d0 ^ d2 ^ d3 ^ d5 ^ d6;
wire p4_calc = d1 ^ d2 ^ d3 ^ d7;
wire p8_calc = d4 ^ d5 ^ d6 ^ d7;
wire [3:0] syndrome = {p8_rx ^ p8_calc, p4_rx ^ p4_calc, p2_rx ^ p2_calc, p1_rx ^ p1_calc};
wire overall_calc = ^{d0,d1,d2,d3,d4,d5,d6,d7, p1_calc, p2_calc, p4_calc, p8_calc};
wire parity_mismatch = (p0_rx != overall_calc);
assign double_err = (syndrome != 4'd0) && !parity_mismatch;
wire single_data_or_parity = (syndrome != 4'd0) && parity_mismatch;
wire single_p0_only = (syndrome == 4'd0) && parity_mismatch;
assign single_err = single_data_or_parity || single_p0_only;
reg [12:0] corrected;
always @* begin
corrected = codeword_in;
if (single_data_or_parity) begin
case (syndrome)
4'd1 : corrected[0] = ~corrected[0];
4'd2 : corrected[1] = ~corrected[1];
4'd3 : corrected[2] = ~corrected[2];
4'd4 : corrected[3] = ~corrected[3];
4'd5 : corrected[4] = ~corrected[4];
4'd6 : corrected[5] = ~corrected[5];
4'd7 : corrected[6] = ~corrected[6];
4'd8 : corrected[7] = ~corrected[7];
4'd9 : corrected[8] = ~corrected[8];
4'd10: corrected[9] = ~corrected[9];
4'd11: corrected[10] = ~corrected[10];
4'd12: corrected[11] = ~corrected[11];
default: ;
endcase
end else if (single_p0_only) begin
corrected[12] = ~corrected[12];
end
end
assign data_out = {
corrected[11],
corrected[10],
corrected[9],
corrected[8],
corrected[6],
corrected[5],
corrected[4],
corrected[2]
};
endmodule
module fifo #(
parameter DATA_WIDTH = 13,
parameter DEPTH = 4
)(
input wire clk,
input wire rst,
input wire wr_en,
input wire rd_en,
input wire [DATA_WIDTH-1:0] din,
output reg [DATA_WIDTH-1:0] dout,
output reg dout_valid,
output wire full,
output wire empty
);
reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
reg [$clog2(DEPTH):0] count;
reg [$clog2(DEPTH)-1:0] wptr,rptr;
integer i;
assign full = (count == DEPTH);
assign empty = (count == 0);
always @(posedge clk or posedge rst) begin
if (rst) begin
wptr <= 0; rptr <= 0; count <= 0;
dout <= {DATA_WIDTH{1'b0}}; dout_valid <= 1'b0;
for (i=0;i<DEPTH;i=i+1) mem[i] <= {DATA_WIDTH{1'b0}};
end else begin
if (wr_en && !full) begin
mem[wptr] <= din;
wptr <= wptr + 1'b1;
count <= count + 1'b1;
end
if (rd_en && !empty) begin
dout <= mem[rptr];
rptr <= rptr + 1'b1;
count <= count - 1'b1;
dout_valid <= 1'b1;
end else begin
dout_valid <= 1'b0;
end
end
end
endmodule

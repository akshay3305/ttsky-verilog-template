`default_nettype none
`timescale 1ns/1ps
module tb_secdec_fifo_arq;
 reg clk = 0;
 always #5 clk = ~clk;
 reg rst;
 reg wr_en;
 reg rd_en;
 reg [7:0] data_in;
 reg [1:0] err_mode;
 wire [7:0] data_out;
 wire ack, nack;
 secdec_fifo_arq #(.DATA_WIDTH(8), .FIFO_DEPTH(4)) dut (
 .clk(clk), .rst(rst),
 .wr_en(wr_en), .data_in(data_in),
 .rd_en(rd_en), .data_out(data_out),
 .ack(ack), .nack(nack),
 .err_mode(err_mode)
 );
 task write_word(input [7:0] w);
 begin
 @(posedge clk);
 data_in <= w; wr_en <= 1'b1;
 @(posedge clk);
 wr_en <= 1'b0;
 end
 endtask
 task read_with_arq(input [7:0] last_sent);
 begin
 @(posedge clk);
 rd_en <= 1'b1;
 @(posedge clk);
 rd_en <= 1'b0;
 // wait two cycles for registered ack/nack
 @(posedge clk);
 @(posedge clk);
 if (nack) begin
 $display("[%0t] NACK observed for 0x%0h -> retransmit clean", $time, last_sent);
 err_mode <= 2'b00;
 write_word(last_sent);
 @(posedge clk);
 rd_en <= 1'b1;
 @(posedge clk);
 rd_en <= 1'b0;
 @(posedge clk); @(posedge clk);
 if (ack) $display("[%0t] Retransmit ACK OK 0x%0h", $time, data_out);
 end
 else if (ack) begin
 $display("[%0t] ACK OK 0x%0h", $time, data_out);
 end
 end
 endtask
 initial begin
 $dumpfile("secdec_fifo_arq.vcd");
 $dumpvars(0, tb_secdec_fifo_arq);
 rst = 1; wr_en = 0; rd_en = 0; data_in = 8'h00; err_mode = 2'b00;
 #20 rst = 0;
 write_word(8'hA5);
 err_mode <= 2'b00;
 read_with_arq(8'hA5);
 write_word(8'h3C);
 err_mode <= 2'b01;
 read_with_arq(8'h3C);
 write_word(8'h5A);
 err_mode <= 2'b10;
 read_with_arq(8'h5A);
 #50 $finish;
 end
endmodule

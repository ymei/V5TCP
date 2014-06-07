`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    16:20:21 10/14/2013 
// Design Name: 
// Module Name:    wr_inter 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module wr_inter(reset,clk,data,DIN,SCLK,SYN,over
    );
 input reset;
 input clk;
 input [31:0] data;
 output reg DIN;
 output reg SCLK;
 output reg SYN;
 output reg over;
 
 reg [5:0] cnt;
 reg [31:0] data_buf;
 
 parameter s0=4'b0000,s1=4'b0001,s2=4'b0010,s3=4'b0100,s4=4'b1000;
 reg [3:0] current_state,next_state;
 
 always@(posedge clk or negedge reset)
 begin
 if(!reset)
	current_state<=s0;
 else
	current_state<=next_state;
 end
 
 always@(cnt,current_state)
 begin
 case(current_state)
 s0:begin next_state<=s1; end
 s1:begin next_state<=s2; end
 s2:begin if(cnt>6'h20) next_state<=s4; else next_state<=s3; end
 s3:begin next_state<=s2; end
 s4:begin next_state<=s0; end
 default:begin next_state<=s0; end
 endcase
 end
 
 always@(posedge clk)
 begin
 case(next_state)
 s0:begin cnt<=6'h00; data_buf<=32'h0a000000; DIN<=1'b0; SCLK<=1'b1; SYN<=1'b1; over<=1'b0; end
 s1:begin cnt<=6'h00; data_buf<=data; DIN<=1'b0; SCLK<=1'b1; SYN<=1'b0; over<=1'b0; end
 s2:begin cnt<=cnt+1'b1; data_buf<=data_buf; DIN<=data_buf[31]; SCLK<=1'b1; SYN<=1'b0; over<=1'b0; end
 s3:begin cnt<=cnt; data_buf<=data_buf<<1; DIN<=DIN; SCLK<=1'b0; SYN<=1'b0; over<=1'b0; end
 s4:begin cnt<=6'h00; data_buf<=32'h00000000; DIN<=1'b0; SCLK<=1'b1; SYN<=1'b1; over<=1'b1; end
 default:begin cnt<=6'h00; data_buf<=32'h0a000000; DIN<=1'b0; SCLK<=1'b1; SYN<=1'b1; over<=1'b0; end
 endcase
 end

endmodule

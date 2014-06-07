`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    16:13:45 10/14/2013 
// Design Name: 
// Module Name:    dac_inter8568 
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
module dac_inter8568(reset,clk,dataa,datac,datae,datag,din,sclk,syn
    );  //negedge reset trigger the data action, config DAC channel a, c, e, g;
input reset;
input clk;
input [15:0] dataa;
input [15:0] datac;
input [15:0] datae;
input [15:0] datag;
output din;
output sclk;
output syn;

reg [15:0] a_dac_reg;
reg [15:0] c_dac_reg;
reg [15:0] e_dac_reg;
reg [15:0] g_dac_reg;
reg [31:0] pdata;
wire over;
reg wr_reset;

wr_inter wr_inter_inst(.reset(wr_reset),.clk(clk),.data(pdata),.DIN(din),.SCLK(sclk),.SYN(syn),.over(over));

always@(posedge clk)
begin
  a_dac_reg<=dataa;
  c_dac_reg<=datac;
  e_dac_reg<=datae;
  g_dac_reg<=datag;
end
	 
parameter s0=8'h00,s1=8'h01,s2=8'h02,s3=8'h04,
          s4=8'h08,s5=8'h10,s6=8'h20,s7=8'h40,s8=8'h80;
			 		 
reg [7:0] current_state,next_state;

 always@(posedge clk or negedge reset)
 begin
 if(!reset)
	current_state<=s0;
 else
	current_state<=next_state;
 end
 
 always@(over,current_state)
 begin
 case(current_state)
 s0:begin if(over) next_state<=s1; else next_state<=s0;  end
 s1:begin if(over) next_state<=s2; else next_state<=s1; end
 s2:begin if(over) next_state<=s3; else next_state<=s2; end
 s3:begin if(over) next_state<=s4; else next_state<=s3; end
 s4:begin if(over) next_state<=s5; else next_state<=s4; end
 s5:begin if(over) next_state<=s6; else next_state<=s5; end
 s6:begin if(over) next_state<=s7; else next_state<=s6; end
 s7:begin if(over) next_state<=s8; else next_state<=s7; end
 s8:begin next_state<=s8; end	
 default:begin next_state<=s0; end
 endcase
 end
 
 always@(posedge clk)
 begin
 case(next_state)
 s0:begin wr_reset<=1'b1; pdata<=32'h0a000000; end  //hardware reset
 s1:begin wr_reset<=1'b1; pdata<=32'h07000000; end  //software reset
 s2:begin wr_reset<=1'b1; pdata<=32'h090a0000; end  //inter reference voltage on
 s3:begin wr_reset<=1'b1; pdata<=32'h040002AA; end  //turn on channel a,c,e,g
 s4:begin wr_reset<=1'b1; pdata<={12'h030,a_dac_reg,4'h0}; end  //a
 s5:begin wr_reset<=1'b1; pdata<={12'h032,c_dac_reg,4'h0}; end  //c
 s6:begin wr_reset<=1'b1; pdata<={12'h034,e_dac_reg,4'h0}; end  //e
 s7:begin wr_reset<=1'b1; pdata<={12'h036,g_dac_reg,4'h0}; end  //g
 s8:begin wr_reset<=1'b0; pdata<=32'h0a000000; end
 default:begin wr_reset<=1'b1; pdata<=32'h0a000000; end
 endcase
 end

endmodule

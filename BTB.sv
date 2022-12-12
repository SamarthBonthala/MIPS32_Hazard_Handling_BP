`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/22/2022 10:29:14 AM
// Design Name: 
// Module Name: BTB
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module BTB(
input clk,
input rst,
input [31:0] PC,
input mispred,
input [31:0] TARGET_PC,
input [31:0] PC_plus4,
input [31:0] PC_plus4_M,
output [31:0] NPC
    );
    
logic [32:0] BTB [127:0]; //[32] is valid bit and [31:0] is the NPC stored at a BTB entry

//if valid bit is set then give BTB entry else increment PC by 4    
assign NPC = ((BTB[PC[8:2]][32] == 1'b1) && rst) ? BTB[PC[8:2]][31:0] : PC_plus4; // BTB has 128 entries

//genvar i;
//generate 
//    for(i=0; i < 128; i++) begin
        always @(posedge clk) begin
            if(!rst) begin
		BTB <= '{default:'0};
	    end
            if(rst) begin
                if(!BTB[PC[8:2]][32] && !mispred) begin
                    BTB[PC[8:2]] <= {1'b1,(PC_plus4)};
                end
                else if(mispred) begin
                    //BTB[PC[8:2] - 7'h3] <= {1'b1,(TARGET_PC)};
                    BTB[PC_plus4_M[8:2] - 'h1] <= {1'b1,(TARGET_PC)};
               //     BTB[i] <= TARGET_PC;
                end
            end
        end
//    end
//endgenerate    
//genvar i;
//    for(i=0; i < 128; i++) begin
//        always @(posedge clk) begin
//            if(!rst) begin
//		BTB[i] <= 33'h0;
//	    end
//	end
//    end
endmodule

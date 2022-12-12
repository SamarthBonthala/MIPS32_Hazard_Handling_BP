`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/22/2022 11:15:10 AM
// Design Name: 
// Module Name: Branch_1bit_predictor
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


module Branch_2bit_predictor(
input clk,
input rst,
input [31:0] PC,
input [31:0] BPC,
input [31:0] PC_plus4_M,
input is_Branch,
input actual_result,
output predicted_dir,
output flush_pipeline,
output update_btb
    );
parameter SNT = 2'b00; 
parameter WNT = 2'b01; 
parameter WT = 2'b10; 
parameter ST = 2'b11; 
logic [1:0] pred_array [127:0];
assign predicted_dir = ((pred_array[PC[8:2]]==WNT) || (pred_array[PC[8:2]]==SNT)) ? 1'b0 : 1'b1; //128 PC's accomodated in pred_dir
assign update_btb = (^pred_array[PC_plus4_M[8:2] - 'h1]);
always @(posedge clk) begin
    if(!rst) begin
        pred_array <= '{default:SNT};
    end
    else begin
        if(is_Branch) begin
		if((pred_array[BPC[8:2]] == SNT) && actual_result) begin
            		pred_array[BPC[8:2]] <= WNT;
		end
		else if((pred_array[BPC[8:2]] == WNT) && actual_result) begin
            		pred_array[BPC[8:2]] <= WT;
		end
		else if((pred_array[BPC[8:2]] == WNT) && !actual_result) begin
            		pred_array[BPC[8:2]] <= SNT;
		end
		else if((pred_array[BPC[8:2]] == WT) && !actual_result) begin
            		pred_array[BPC[8:2]] <= WNT;
		end
		else if((pred_array[BPC[8:2]] == WT) && actual_result) begin
            		pred_array[BPC[8:2]] <= ST;
		end
		else if((pred_array[BPC[8:2]] == ST) && !actual_result) begin
            		pred_array[BPC[8:2]] <= WT;
		end
        end
    end
end

//genvar i;
//    for(i=0; i < 128; i++) begin
//        always @(posedge clk) begin
//            if(!rst) begin
//		pred_array[i] <= WNT;
//	    end
//	end
//    end

endmodule

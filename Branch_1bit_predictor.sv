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


module Branch_1bit_predictor(
input clk,
input rst,
input [31:0] PC,
input [31:0] BPC,
input is_Branch,
input actual_result,
output predicted_dir,
output flush_pipeline
    );
    
logic [127:0] pred_array;
assign predicted_dir = pred_array[PC[8:2]]; //128 PC accomodated in pred_dir
//assign flush_pipeline = (actual_result != pred_array[PC[8:2]]);
//assign pred_array[PC[8:2]] = (actual_result != pred_array[PC[8:2]]) ? actual_result : 1'b0;
always @(posedge clk) begin
    if(!rst) begin
        pred_array <= 128'b0;
    end
    else begin
        if((actual_result != pred_array[BPC[8:2]]) && is_Branch) begin
            //pred_array[PC[8:2] - 7'h3] = actual_result;
            pred_array[BPC[8:2]] <= actual_result;
            //flush_pipeline <= 1'b1;
        end
    end
end
endmodule

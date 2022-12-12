
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/22/2022 05:00:54 PM
// Design Name: 
// Module Name: MIPS_PIPELINED_PI
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


module MIPS_PIPELINED_PI(
    input CLK,
    input RSTn,
    input [31:0] Instr,
    output logic [31:0] PC
    );

    //special instructions (opcode == 000000), values of F code (bits 5-0):
    parameter add = 6'b100000;
    parameter sub = 6'b100010;
    parameter xor1 = 6'b100110;
    parameter and1 = 6'b100100;
    parameter or1 = 6'b100101;
    parameter slt = 6'b101010;
    parameter srl = 6'b000010;
    parameter sll = 6'b000000;
    parameter jr = 6'b001000;
    
    //non-special instructions, values of opcodes:
    parameter addi = 6'b001000;
    parameter andi = 6'b001100;
    parameter ori = 6'b001101;
    parameter lw = 6'b100011;
    parameter sw = 6'b101011;
    parameter beq = 6'b000100;
    parameter bne = 6'b000101;
    parameter j = 6'b000010;
    
    //Fetch Stage
    logic [31:0] PCPlus4;
    
    //Decode Stage
    logic [31:0] REG_FILE [0:31];
    logic [31:0] SignImm;
    logic [31:0] RD1, RD2;
    logic [5:0] opcode, func;
    
    logic RegWrite_D;
    logic MemtoReg_D;
    logic MemWrite_D;
    logic Branch_D;
    logic BranchT_D;
    logic [5:0] ALUControl_D;
    logic ALUSrc_D;
    logic RegDst_D;
    logic [4:0] Shamt_D;
    
    //Execute Stage
    logic signed [31:0] SrcA_E, SrcB_E;
    logic [4:0] WriteReg_E;
    logic [31:0] ALUOut_E;
    logic Zero_E;
    logic [31:0] PCBranch_E;
    
    //Memory Stage
    logic PCSrc_M;
    
    logic [31:0] DATA_MEM [127:0];  //Just 128 locations - 7 bit address
    
    //WriteBack Stage
    logic [31:0] Result_WB;
    
    //Decode Stage Pipeline Registers
    logic [31:0] Instr_D;
    logic [31:0] PCPlus4_D;
    
    //Execute Stage Pipeline Registers
    logic RSTn_E;
    logic [31:0] SignImm_E;
    logic [31:0] PCPlus4_E;
    logic [4:0] Rt_E, Rd_E;
    logic [4:0] Shamt_E;
    
    logic RegWrite_E;
    logic MemtoReg_E;
    logic MemWrite_E;
    logic Branch_E;
    logic BranchT_E;
    logic [5:0] ALUControl_E;
    logic ALUSrc_E;
    logic RegDst_E;
    
    //Memory Stage Pipeline Registers
    logic RSTn_M;
    logic [31:0] PCBranch_M;
    logic [31:0] WriteData_M;
    logic [4:0] WriteReg_M;
    logic [31:0] ALUOut_M;
    logic Zero_M;
    
    
    logic RegWrite_M;
    logic MemtoReg_M;
    logic MemWrite_M;
    logic Branch_M, Branch_M_1;
    logic BranchT_M;
    
    //WB Stage Pipeline Regsiters
    logic RSTn_WB;
    logic RegWrite_WB;
    logic MemtoReg_WB;
    logic [31:0] ALUOut_WB;
    logic [31:0] ReadData_WB;
    logic [4:0] WriteReg_WB;
    
    //Data Dependency Signals
    logic Stall_Data_E, Stall_Data_M, Stall_Data_WB;
    
    //Fetch Stage
    
    always_ff@(posedge CLK) begin
        if(!RSTn) begin
            PC <= 32'h00000000;
        end
        else if(!(Stall_Data_E || Stall_Data_M || Stall_Data_WB || Branch_D || Branch_E)) begin
            if(PCSrc_M) begin
                PC <= PCBranch_M; 
            end
            else if (!Branch_M) begin
                PC <= PCPlus4;
            end
        end
    end
    
    assign PCPlus4 = PC + 32'h00000004;
    
    //Fetch-Decode Pipeline Register Update
        
    always_ff@(posedge CLK) begin
        if(!RSTn || ((Branch_D || Branch_E || Branch_M ) && !(Stall_Data_E || Stall_Data_M || Stall_Data_WB))) begin
            Instr_D <= 32'h00000000;
        end
        else if (!(Stall_Data_E || Stall_Data_M || Stall_Data_WB)) begin
            Instr_D <= Instr;
        end
    end
    
    always_ff@(posedge CLK) begin
        if (!(Stall_Data_E || Stall_Data_M || Stall_Data_WB)) begin
        	PCPlus4_D <= PCPlus4;
    	end
    end
    
    //Decode Stage
    always_ff@(posedge CLK) begin
    	if (!RSTn) begin
		REG_FILE <= '{default:'0};
	end else begin
        	if(RegWrite_WB) begin
        	    REG_FILE[WriteReg_WB] <= Result_WB;
        	end
        	
        	RD1 <= REG_FILE[Instr_D[25:21]];
        	RD2 <= REG_FILE[Instr_D[20:16]];
	end
    end
    
    always_comb begin   //ALUControl generation
        if (opcode == 6'h00) //register instructions
            ALUControl_D = func;
        else if (!((opcode == 6'h00) || (opcode == 6'h02))) begin //immediate instructions
            if ((opcode == lw)||(opcode == sw)||(opcode == addi)) ALUControl_D = add;
            else if ((opcode == beq)||(opcode == bne)) begin
                ALUControl_D = sub;
            end
            else if (opcode == andi) ALUControl_D = and1;
            else if (opcode == ori) ALUControl_D = or1;
            else ALUControl_D = 6'h00;
        end
        else begin
            ALUControl_D = 6'h00;
        end
    end
    
    assign SignImm = (Instr_D[15] == 1)? ({16'hFFFF, Instr_D[15:0]}) : ({16'h0000, Instr_D[15:0]});
    assign opcode = Instr_D[31:26];
    assign Shamt_D = Instr_D[10:6];
    assign func = Instr_D[5:0];
    assign RegWrite_D = (opcode == 6'h00) || (opcode == addi) || (opcode == andi) || (opcode == ori) || (opcode == lw);
    assign MemtoReg_D = (opcode == lw);
    assign MemWrite_D = (opcode == sw);
    assign Branch_D = (opcode == beq) || (opcode == bne);
    assign BranchT_D = opcode[0];
    assign ALUSrc_D = !((opcode == 6'h00) || (opcode == 6'h02) || Branch_D);
    assign RegDst_D = (opcode == 6'h00);
    
    //Decode to Execute Stage Pipeline Registers
    
    always_ff@(posedge CLK) begin
        if(!RSTn_E || (Stall_Data_E || Stall_Data_M || Stall_Data_WB) || Branch_E || Branch_M || Branch_M_1) begin
            RegWrite_E <= 1'b0;
            MemWrite_E <= 1'b0;
            Branch_E <= 1'b0;
        end
        else begin
            RegWrite_E <= RegWrite_D;
            MemtoReg_E <= MemtoReg_D;
            MemWrite_E <= MemWrite_D;
            Branch_E <= Branch_D;
            BranchT_E <= BranchT_D;
            ALUControl_E <= ALUControl_D;
            ALUSrc_E <= ALUSrc_D;
            RegDst_E <= RegDst_D;
            Shamt_E <= Shamt_D;
            
            Rt_E <= Instr_D[20:16];
            Rd_E <= Instr_D[15:11];
            
            SignImm_E <= SignImm;
            
            PCPlus4_E <= PCPlus4_D;
        end
        RSTn_E <= RSTn;
    end
    
    //Execute Stage
    
    always_comb begin
        if (ALUControl_E == and1) ALUOut_E = SrcA_E & SrcB_E;
        else if (ALUControl_E == or1) ALUOut_E = SrcA_E | SrcB_E;
        else if (ALUControl_E == add) ALUOut_E = SrcA_E + SrcB_E;
        else if (ALUControl_E == sub) ALUOut_E = SrcA_E - SrcB_E;
        else if (ALUControl_E == srl) ALUOut_E = SrcB_E >> Shamt_E;
        else if (ALUControl_E == sll) ALUOut_E = SrcB_E << Shamt_E;
        else if (ALUControl_E == slt) ALUOut_E = (SrcA_E < SrcB_E)? 32'd1 : 32'd0;
        else if (ALUControl_E == xor1) ALUOut_E = SrcA_E ^ SrcB_E;
        else ALUOut_E = 32'h00000000;
    end
    
    assign SrcA_E = RD1;
    assign SrcB_E = ALUSrc_E ? SignImm_E : RD2;
    assign WriteReg_E = RegDst_E ? Rd_E : Rt_E;
    assign PCBranch_E = PCPlus4_E + (SignImm_E << 2);
    
    assign Zero_E = (ALUOut_E == 32'h00000000);
    
    //Execute-Memory Pipeline Registers
    
    always_ff@(posedge CLK) begin
        //if(!RSTn_M || ((Stall_Data_M || Stall_Data_WB))) begin
        if(!RSTn_M) begin
            RegWrite_M <= 1'b0;
            MemWrite_M <= 1'b0;
            Branch_M <= 1'b0;
        end
        else begin
            RegWrite_M <= RegWrite_E;
            MemtoReg_M <= MemtoReg_E;
            MemWrite_M <= MemWrite_E;
            Branch_M <= Branch_E;
            BranchT_M <= BranchT_E;
            
            Zero_M <= Zero_E;
            ALUOut_M <= ALUOut_E;
            
            WriteData_M <= RD2;
            WriteReg_M <= WriteReg_E;
            
            PCBranch_M <= PCBranch_E;
        end
        RSTn_M <= RSTn_E;
    end
    
    //Memory Stage
    
    always_ff@(posedge CLK) begin
        if(MemWrite_M) begin
            DATA_MEM[ALUOut_M] <= WriteData_M;
        end
    end
    
    assign PCSrc_M = Branch_M && (Zero_M ^ BranchT_M);
    
    //Memory-WriteBack Pipeline Registers
    
    always_ff@(posedge CLK) begin
        //if(!RSTn_WB || ((Stall_Data_WB))) begin
        if(!RSTn_WB) begin
            RegWrite_WB <= 1'b0;
        end
        else begin
            RegWrite_WB <= RegWrite_M;
            MemtoReg_WB <= MemtoReg_M;
            ALUOut_WB <= ALUOut_M;
            ReadData_WB <= DATA_MEM[ALUOut_M];
            WriteReg_WB <= WriteReg_M;
        end
        RSTn_WB <= RSTn_M;
    end
    
    //Writeback Stage
    
    assign Result_WB = MemtoReg_WB ? ReadData_WB : ALUOut_WB;
    
    //Data Dependency Stall Generation
    
    assign Stall_Data_E = RegWrite_E && (WriteReg_E==Instr_D[25:21] || ((WriteReg_E==Instr_D[20:16]) && (!ALUSrc_D)));
    assign Stall_Data_M = RegWrite_M && (WriteReg_M==Instr_D[25:21] || ((WriteReg_M==Instr_D[20:16]) && (!ALUSrc_D)));
    assign Stall_Data_WB = RegWrite_WB && (WriteReg_WB==Instr_D[25:21] || ((WriteReg_WB==Instr_D[20:16]) && (!ALUSrc_D)));

    always_ff @(posedge CLK) begin
    	Branch_M_1 <= Branch_M;
    end
    
endmodule

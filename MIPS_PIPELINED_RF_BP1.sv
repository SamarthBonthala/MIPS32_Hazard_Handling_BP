`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/22/2022 08:55:20 AM
// Design Name: 
// Module Name: MIPS_PIPELINED
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


module MIPS_PIPELINED_RF_BP1(
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
    logic [31:0] RD2_temp;
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
    logic [31:0] PCPlus4_M;
    
    
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
    logic Stall_Data_E;

    //Data Forwarding Signals
    logic [1:0] Forward_AE, Forward_BE;
    //1-bit Branch History and BTB
    logic pred_dir;
    logic Pred_Dir_E;
    logic Pred_Dir_M;
    logic Pred_Dir_D;
    logic actual_dir;
    logic mispred;
    logic [31:0] BTB [127:0];
    logic flush;
    logic [7:0] BTB_addr;
    logic [31:0] TARGET_PC;
    logic [31:0] NPC;
    logic [31:0] Branch_PC_Predict;
    logic [31:0] Branch_addr_delayed_M;
    logic [31:0] Branch_addr_delayed_E;
    //Fetch Stage
    
    always_ff@(posedge CLK) begin
        if(!RSTn) begin
	    PC <= 32'b0;
        end
        else begin
	if(mispred) begin
		if(PCSrc_M) begin
			PC <= PCBranch_M;
		end
		else
			PC <= PCPlus4_M;
	end
	else begin
	if((Instr[31:26] == bne) || (Instr[31:26] == beq))
            PC <= NPC;
	else
	   PC <= PCPlus4;
        end
	end
    end
//assign PC = RSTn ? NPC : 32'b0;
assign TARGET_PC = PCSrc_M ? PCBranch_M : PCPlus4_M;
BTB Buffer(
.clk(CLK),
.rst(RSTn),
.PC(PC),
.mispred(mispred),
.TARGET_PC(TARGET_PC),
.PC_plus4(PCPlus4),
.PC_plus4_M(PCPlus4_M),
.NPC(NPC)
 );
 
 Branch_1bit_predictor bit1pred(
.clk(CLK),
.rst(RSTn),
.PC(PC),
.BPC(Branch_addr_delayed_M),
.is_Branch(Branch_M),
.actual_result(actual_dir),
.predicted_dir(pred_dir),
.flush_pipeline()
);

    assign PCPlus4 = PC + 32'h00000004;
    //Fetch-Decode Pipeline Register Update
        
    always_ff@(posedge CLK) begin
        if(!RSTn ||  mispred) begin
            Instr_D <= 32'h00000000;
        end
        else if (!(Stall_Data_E)) begin
            Instr_D <= Instr;
        end
    end
    
    always_ff@(posedge CLK) begin
        PCPlus4_D <= PCPlus4;
    end
//agoel assign BTB_addr = PC[9:2];
    //Decode Stage
    
    always_ff@(posedge CLK) begin
    	if (!RSTn) begin
		REG_FILE <= '{default:'0};
		Pred_Dir_D <= 1'b0;
	end else begin
        	if(RegWrite_WB) begin
        	    REG_FILE[WriteReg_WB] <= Result_WB;
        	end
        	
        	if (RegWrite_WB && WriteReg_WB==Instr_D[25:21]) 
			RD1 <= Result_WB;
		else
        		RD1 <= REG_FILE[Instr_D[25:21]];

		if (RegWrite_WB && WriteReg_WB==Instr_D[20:16])
			RD2 <= Result_WB;
		else
        		RD2 <= REG_FILE[Instr_D[20:16]];
		Pred_Dir_D <= pred_dir;
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
    assign Branch_PC_Predict = Branch_D ? PCPlus4_D - 'h4 : Branch_PC_Predict;
    always@(posedge CLK) begin
    	Branch_addr_delayed_E <=  Branch_PC_Predict;
    	Branch_addr_delayed_M <=  Branch_addr_delayed_E;
    end
    //always@(posedge Branch_M) begin
    //	Branch_addr_store <= Branch_addr_delayed;
    //end
    assign BranchT_D = opcode[0];
    assign ALUSrc_D = !((opcode == 6'h00) || (opcode == 6'h02) || Branch_D);
    assign RegDst_D = (opcode == 6'h00);
    
    //Decode to Execute Stage Pipeline Registers
    
    always_ff@(posedge CLK) begin
        if(!RSTn_E || (Stall_Data_E) ||  mispred || Branch_M_1) begin
            RegWrite_E <= 1'b0;
            MemWrite_E <= 1'b0;
		Branch_E <= 'h0;
	    	Pred_Dir_E <= 'b0;
	end
    	else if (RSTn) begin
        	RegWrite_E <= RegWrite_D;
        	MemtoReg_E <= MemtoReg_D;
        	MemWrite_E <= MemWrite_D;
        	Branch_E <= Branch_D;
        	BranchT_E <= BranchT_D;
        	ALUControl_E <= ALUControl_D;
        	ALUSrc_E <= ALUSrc_D;
        	RegDst_E <= RegDst_D;
        	Shamt_E <= Shamt_D;
		Pred_Dir_E <= Pred_Dir_D;
        	
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
   
    // Data Forwarding Logic
    always_ff@(posedge CLK) begin
    	if (RegWrite_E && WriteReg_E==Instr_D[25:21]) begin // To establish priority - mem_stage always has the most recently executed instruction
		Forward_AE <= 2'b10; // Forward from Memory Stage
	end else if (RegWrite_M && WriteReg_M==Instr_D[25:21]) begin
		Forward_AE <= 2'b01; // Forward from Writeback Stage
	end else
		Forward_AE <= 2'b00;
    end

    always_ff@(posedge CLK) begin
    	if (RegWrite_E && WriteReg_E==Instr_D[20:16]) begin // To establish priority - mem_stage always has the most recently executed instruction
		Forward_BE <= 2'b10; // Forward from Memory Stage
	end else if (RegWrite_M && WriteReg_M==Instr_D[20:16]) begin
		Forward_BE <= 2'b01; // Forward from Writeback Stage
	end else
		Forward_BE <= 2'b00;
    end
    
    // SrcA and SrcB for ALU
    always_comb begin
    	case(Forward_AE)
		2'b00 : SrcA_E = RD1;
		2'b10 : SrcA_E = ALUOut_M;
		2'b01 : SrcA_E = Result_WB; 
		2'b11 : SrcA_E = 'h0;
	endcase
    end

    always_comb begin
    	case(Forward_BE)
		2'b00 : RD2_temp = RD2;
		2'b10 : RD2_temp = ALUOut_M;
		2'b01 : RD2_temp = Result_WB;
		2'b11 : RD2_temp = 'h0;
	endcase
    end 
     
    assign SrcB_E = ALUSrc_E ? SignImm_E : RD2_temp;
    assign WriteReg_E = RegDst_E ? Rd_E : Rt_E;
    assign PCBranch_E = PCPlus4_E + (SignImm_E << 2);
    
    assign Zero_E = (ALUOut_E == 32'h00000000);
    
    //Execute-Memory Pipeline Registers
    
    always_ff@(posedge CLK) begin
	if(!RSTn_M || mispred) begin
            RegWrite_M <= 1'b0;
            MemWrite_M <= 1'b0;
		Branch_M <= 'h0;
		Zero_M <= 'h0;
	    	Pred_Dir_M <= 'b0;
	end
    	else if (RSTn) begin
        	RegWrite_M <= RegWrite_E;
        	MemtoReg_M <= MemtoReg_E;
        	MemWrite_M <= MemWrite_E;
        	Branch_M <= Branch_E;
        	BranchT_M <= BranchT_E;
        	
        	Zero_M <= Zero_E;
        	ALUOut_M <= ALUOut_E;
        	
        	WriteData_M <= RD2_temp;
        	WriteReg_M <= WriteReg_E;
        	
        	PCBranch_M <= PCBranch_E;
		Pred_Dir_M <= Pred_Dir_E;
		PCPlus4_M <= PCPlus4_E;
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
    assign actual_dir = RSTn & PCSrc_M;
    assign mispred = (actual_dir != Pred_Dir_M);
    
    //Memory-WriteBack Pipeline Registers
    
    always_ff@(posedge CLK) begin
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
    
    assign Stall_Data_E = RegWrite_E && MemtoReg_E && (WriteReg_E==Instr_D[25:21] || ((WriteReg_E==Instr_D[20:16]) && (!ALUSrc_D))); // Stall at the Execute stage when instruction is load word and stall for one cycle

    always_ff @(posedge CLK) begin
    	Branch_M_1 <= mispred;
    end
    
endmodule

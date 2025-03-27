`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/23/2025 02:24:15 PM
// Design Name: 
// Module Name: cpu
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

module cpu(
    input logic rst_n, clk, 
    input logic [31:0] imem_insn, //instruction
    output logic dmem_wen,  // 0 to read, 1 to write
    output logic [31:0] imem_addr, dmem_addr, //Address lines
    inout logic [31:0] dmem_data //Write & Read data
    );
    
    wire [31:0] alu_out; //Output for ALU
    wire [31:0] regData1, regData2; //Reg file outputs
    wire Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite; //Control Unit Output
    wire [1:0] ALUOp; //Control Unit Output
    wire [4:0] ALUCtrl; //ALU Control

    wire [31:0] data;
    
    int file1, file2;
    logic [15:0] CC = 16'd0;
    logic [31:0] PC = 32'd0;
    
    logic signed [11:0] imm; //immediate
    logic signed [31:0] imm_extend; //immediate extended
    logic [4:0] rs1, rs2, rd; //read registers
    logic [2:0] func3;
    logic [6:0] func7; 
    logic [6:0] opcode; //opcode
    
    reg [63:0] IF_ID; //Fetch and Decode carry register
    reg [146:0] ID_EX; //Decode and Execute carry register
    reg [69:0] EX_MEM; //Execute and Memory carry register
    reg [69:0] MEM_WB; //Memory and Write Back carry register
    
    wire [63:0] IF_IDwire; //Wire from IF_ID to ID_EX
    wire [146:0] ID_EXwire; //Wire from ID_EX to EX_MEM
    wire [69:0] EX_MEMwire; //Wire from EX_MEM to MEM_WB
    wire [69:0] MEM_WBwire; //Wire from MEM_WB to WB
    
    logic [31:0] alu_A, alu_B;
    logic [31:0] writeData;
    logic [4:0] writeReg;
    logic regW;
    
     
    //Stage One - Fetch
    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
            PC <= 32'b0;
            CC <= 16'b0;
        end 
        else begin         
            dmem_wen = 0;
            imem_addr = PC; //Address of current instruction
            PC = PC + 4; //increment pc for next instruction
            CC = CC + 1;
            IF_ID = {PC[31:0], imem_insn[31:0]}; //Current Bits: 64

        end
    end
    
    assign IF_IDwire = IF_ID;
    
    
    
    
    //Stage Two - Decode and register read            
    controlUnit control(.opcode(IF_IDwire[6:0]), .Branch(Branch), .MemRead(MemRead),
                .MemtoReg(MemtoReg), .MemWrite(MemWrite), .ALUSrc(ALUSrc), 
                .RegWrite(RegWrite), .ALUOp(ALUOp));
             
    reg_file regfile(.clk(clk), .regWrite(regW), .RR1(IF_IDwire[19:15]), .RR2(IF_IDwire[24:20]), 
                     .WR(writeReg), .WD(writeData), .RD1(regData1), 
                     .RD2(regData2));
                
    //opcode can determine the proper field for immediate value
//    assign rd = IF_ID[11:7];
//    assign func3 = IF_ID[14:12];
//    assign rs1 = IF_ID[19:15];
//    assign rs2 = IF_ID[24:20];
//    assign func7 = IF_ID[31:25];
      assign imm = IF_IDwire[31:20];    
      assign imm_extend = { {20{imm[11]}}, imm }; //sign-extend to 32 bits 

    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
           
        end
        else begin    
            ID_EX = { IF_IDwire[14:12], IF_IDwire[31:25], RegWrite, ALUSrc, ALUOp, IF_IDwire[11:7], imm_extend, regData2, regData1, IF_IDwire[63:32]}; //Current Bits: 147
            // ID_EX(func3, func7, RegWrite, ALUSrc, ALUOp, rd, imm_extend, regData2, regData1, PC)
        end 
    end
    
    assign ID_EXwire = ID_EX;
    
    
    //Stage Three - Execute    
    aluControlUnit aluControl(.ALUOp(ALUOp), .func3(ID_EXwire[146:144]), .func7(ID_EXwire[143:137]), .ALUCtrl(ALUCtrl));
    
    assign alu_B = (ID_EX[135] == 1) ? ID_EX[127:96] : ID_EX[95:64];
    alu calc(.A(alu_A), .B(alu_B), .ALUCtrl(ALUCtrl), .out(alu_out));
    
    always@(posedge clk)begin 
        alu_A = regData1;
        EX_MEM = { ID_EXwire[136], ID_EXwire[132:128], alu_out }; //Holds RegWrite, rd, alu_out
    end
    
    assign EX_MEMwire = EX_MEM;
    
    
    //Stage Four - Memory Access             
    always@(posedge clk)begin
            dmem_wen <= 1;
            
            MEM_WB = { EX_MEMwire[37], EX_MEMwire[36:32], EX_MEMwire[31:0]}; // Current Bits: 38 bit
                                                              //Holds RegWrite, rd, alu_out
    end
    
    assign MEM_WBwire = MEM_WB;
    
    
    //Stage Five - Write Back
    assign data = (MEM_WB[37] && MEM_WB[31:0]) ? MEM_WB[31:0] : writeData; //change later
    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
        end
        else begin
            writeData = data;
            regW = MEM_WBwire[37];
            writeReg = MEM_WBwire[36:32];
        end
          
    end

    
endmodule

module reg_file(
    input logic clk, regWrite,
    input logic [4:0] RR1, RR2, WR,
    input logic [31:0] WD,
    output logic [31:0] RD1, RD2
);
    logic [31:0] register [31:0] = '{default: 32'b0};

    assign RD1 = register[RR1]; //read data from register 1
    assign RD2 = register[RR2]; //read data from register 2
    
    always@(posedge clk) begin
        if(regWrite) register[WR] <= WD; //write data to destination register
    end 
    
endmodule

module controlUnit( // we can try to combine control and alu control into one module
    input logic [6:0] opcode,
    output logic Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite,
    output logic [1:0] ALUOp
    );
    
    reg [7:0] ctrl;
    
    assign {ALUOp[1:0], ALUSrc, Branch, MemRead, MemWrite, RegWrite, MemtoReg} = ctrl;
    
    always @(*) begin
        casez(opcode)
        //I-type
            7'b0010011: ctrl = 8'b00101011;
            
        //R-type
            7'b0110011: ctrl = 8'b10000010;
            
        endcase
    end
endmodule


module aluControlUnit(
    input logic [1:0] ALUOp,
    input logic [2:0] func3,
    input logic [6:0] func7,
    output logic [4:0] ALUCtrl
    );
    
    always @(*) begin
        casez({ALUOp, func3})
        //Itype
            ({2'b00, 3'b000}): ALUCtrl = 5'b00000; //ADDI
            ({2'b00, 3'b010}): ALUCtrl = 5'b00001; //SLTI
            ({2'b00, 3'b011}): ALUCtrl = 5'b00010; //SLTIU
            ({2'b00, 3'b100}): ALUCtrl = 5'b00011; //XORI
            ({2'b00, 3'b110}): ALUCtrl = 5'b00100; //ORI
            ({2'b00, 3'b111}): ALUCtrl = 5'b00101; //ANDI
            ({2'b00, 3'b001}): ALUCtrl = 5'b00110; //SLLI
            ({2'b00, 3'b101}): ALUCtrl = (func7[5] == 1 ? 5'b00111 : 5'b01000); //SRAI or SRLI
        endcase
    
    end
endmodule

module alu(
    input logic [31:0] A,B,
    input logic [4:0] ALUCtrl, 
    output logic [31:0] out
);    
    
    always_comb begin
        case(ALUCtrl)
            5'h00: out = A + B; //ADDI
            5'h01: out = (A < B) ? 1:0; //SLTI
            5'h02: out = (A < B) ? 1:0; //SLTIU
            5'h03: out = A ^ B; //XORI
            5'h04: out = A | B; //ORI
            5'h05: out = A & B; //ANDI
            5'h06: out = A << B; //SLLI
            5'h07: out = A >>> B; //SRAI
            5'h08: out = A >> B; //SRLI
            default: out = 32'b0;
        endcase
    end
    
endmodule
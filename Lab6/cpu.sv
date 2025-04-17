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
    input logic [31:0] imem_insn, //ROM instruction
    output logic dmem_wen,  // 0 to read, 1 to write
    output logic [3:0] byte_en, //Send which bytes to store
    output logic [31:0] imem_addr, dmem_addr, //Address lines
    inout logic [31:0] dmem_data //Memory Write & Read data
    );
    
    wire [31:0] alu_out; //Output for ALU
    wire [31:0] regData1, regData2; //Register File Read outputs
    wire Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite; //Control Unit Output
    wire [2:0] ALUOp; //Control Unit Output
    wire [4:0] ALUCtrl; //ALU Control
    wire [1:0] forwardA, forwardB; //Forwarding MUXs
    wire [3:0] byte_en_line; //Byte enable line

    wire [31:0] data; //Write Back variable
    
    int file1, file2; //For file text output
    //Trace files found in ..\[project folder]\[project name].sim\sim_1\behav\xsim
    //output-PC.txt and output-CC.txt
    
    logic [15:0] CC = 16'd0; //Clock Cycle Counter
    logic [31:0] PC = 32'd0; //Program Counter
    
    logic signed [11:0] imm; //Immediate value
    logic signed [31:0] imm_extend; //Immediate extended
    logic [4:0] rs1, rs2, rd; // Register File Read registers
    logic [2:0] func3; //Arithmetic functions
    logic [6:0] func7; //Shift functions
    logic [4:0] shamt; //I-Type 'shift amount' for SLLI, SRLI, and SRAI
    logic [6:0] opcode; //opcode
    
    reg [63:0] IF_ID; //Fetch and Decode carry register
    reg [165:0] ID_EX; //Decode and Execute carry register
    reg [76:0] EX_MEM; //Execute and Memory carry register
    reg [70:0] MEM_WB; //Memory and Write Back carry register
    
    wire [63:0] IF_IDwire; //Wire from IF_ID to ID_EX
    wire [165:0] ID_EXwire; //Wire from ID_EX to EX_MEM
    wire [76:0] EX_MEMwire; //Wire from EX_MEM to MEM_WB
    wire [70:0] MEM_WBwire; //Wire from MEM_WB to WB
    
    logic [31:0] alu_A, alu_B, bestAlu_B; //To determine ALU inputs
    logic signed [31:0] writeData; //Write data for register
    logic [4:0] writeReg; //The register to write to
    logic regW; //Register write enable (1 to write, 0 to not write)
    
    //Stage One - Fetch
    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
            PC <= 32'b0;
            CC <= 16'b0;
        end 
        else begin     
            file1 = $fopen("output-PC.txt", "a");        
            file2 = $fopen("output-CC.txt", "a");
            $fdisplay(file1, "PC(int, hex) = (%1d, %h)", imem_addr, imem_addr); 
            dmem_wen = 0;
            imem_addr = PC; //Address of current instruction
            IF_ID = {PC[31:0], imem_insn[31:0]}; //Current Bits: 64
            PC = PC + 4; //increment pc for next instruction
            CC = CC + 1;
        end
    end
    assign IF_IDwire = IF_ID;
    
    //Stage Two - Decode and register read            
    controlUnit control(.opcode(IF_IDwire[6:0]), .Branch(Branch), .MemRead(MemRead),
                .MemtoReg(MemtoReg), .MemWrite(MemWrite), .ALUSrc(ALUSrc), 
                .RegWrite(RegWrite), .ALUOp(ALUOp));
             
    reg_file regfile(.clk(clk), .regWrite(MEM_WBwire[37]), .RR1(IF_IDwire[19:15]), .RR2(IF_IDwire[24:20]), 
                     .WR(MEM_WBwire[36:32]), .WD(writeData), .RD1(regData1), 
                     .RD2(regData2));
                
    //DECODE VARIABLES, DO NOT NEED TO CHANGE ID_IF REGISTER PLACEMENT
    //rd = IF_ID[11:7]
    //func3 = IF_ID[14:12]
    //rs1 = IF_ID[19:15]
    //rs2 = IF_ID[24:20]
    //shamt = IF_ID[24:20]
    //func7 = IF_ID[31:25]
    //imm = IF_ID[31:20]
    
    always@(*) begin
        if(ALUOp == 3'b100) imm = {IF_IDwire[31:25], IF_IDwire[11:7]};
        else imm = IF_IDwire[31:20];
    end
//      assign imm = IF_IDwire[31:20]; //Retrieve immediate value
      assign imm_extend = { {20{imm[11]}}, imm }; //Sign-extend immediate value to 32 bits 

    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
           //Nothing here for now
        end
        else begin    
            ID_EX = {MemtoReg, MemWrite, MemRead, IF_IDwire[24:20], IF_IDwire[19:15], IF_IDwire[24:20], IF_IDwire[14:12], IF_IDwire[31:25], 
                      RegWrite, ALUSrc, ALUOp, IF_IDwire[11:7], imm_extend, regData2, 
                      regData1, IF_IDwire[63:32]}; //Current Bits: 162
                 
            //UPDATE THIS LIST WHEN ADDING TO CARRY REG
            // ID_EX( MemtoReg, MemWrite, MemRead, shamt, rs1, rs2, func3, func7, RegWrite, ALUSrc, ALUOp, rd, imm_extend, regData2, regData1, PC)
            
            // MemtoReg = ID_EX[165]
            // MemWrite = ID_EX[164]
            // MemRead = ID_EX[163]
            // shamt = ID_EX[162:158]
            // rs1 = ID_EX[157:153]
            // rs2 = ID_EX[152:148]
            // func3 = ID_EX[147:145]
            // func7 = ID_EX[144:138] or imm[11:5]
            // regWrite = ID_EX[137]
            // ALUSrc = ID_EX[136]
            // ALUOp = ID_EX[135:133]
            // rd = ID_EX[132:128] or imm[4:0]
            // imm_extend = ID_EX[127:96]
            // regData2 = ID_EX[95:64]
            // regData1 = ID_EX[63:32]
            // PC = ID_EX[31:0]
            
        end 
    end
    assign ID_EXwire = ID_EX;
    
    
    //Stage Three - Execute 
    forwardingUnit forward(.EX_MEMregWrite(EX_MEMwire[37]), .MEM_WBregWrite(MEM_WBwire[37]),
    .EX_MEMregRd(EX_MEMwire[36:32]), .MEM_WBregRd(MEM_WBwire[36:32]), 
    .rs1(ID_EXwire[157:153]), .rs2(ID_EXwire[152:148]),
    .forwardA(forwardA), .forwardB(forwardB));
    
    aluControlUnit aluControl(.ALUOp(ALUOp), .func3(ID_EXwire[147:145]), .func7(ID_EXwire[144:138]), .ALUCtrl(ALUCtrl), .byte_en(byte_en_line));
    
    alu calc(.shamt(ID_EXwire[162:158]), .A(alu_A), .B(bestAlu_B), .ALUCtrl(ALUCtrl), .out(alu_out));
    
    //Check ALUSrc to decide which to use
    assign bestAlu_B = (ID_EXwire[136] == 1) ? ID_EXwire[127:96] : alu_B; //imm_extend or regData2

    always@(*) begin
        case(forwardA)                       //     STAGE         SOURCE
            2'b00: alu_A = ID_EXwire[63:32]; // stage 2 (DECODE) regData1
            2'b10: alu_A = EX_MEMwire[31:0]; // stage 4 (MEMORY) ALU out
            2'b01: alu_A = writeData;        // stage 5 (WRITE BACK) ALU out
        endcase
        case(forwardB)                       //     STAGE         SOURCE
            2'b00: alu_B = ID_EXwire[95:64]; // stage 2 (DECODE) regData2
            2'b10: alu_B = EX_MEMwire[31:0]; // stage 4 (MEMORY) ALU out
            2'b01: alu_B = writeData;        // stage 5 (WRITE BACK) ALU out
        endcase
    end
    always@(posedge clk)begin 
        
       EX_MEM = { byte_en_line, ID_EXwire[165], ID_EXwire[164], ID_EXwire[163], ID_EXwire[95:64], ID_EXwire[137], ID_EXwire[132:128], alu_out }; 
      //UPDATE THIS LIST WHEN ADDING TO CARRY REG
      //EX_MEM(byte_en_line, MemtoReg, MemWrite, MemRead ,regData2, RegWrite, rd, alu_out)
      
      //byte_en_line = EX_MEM[76:73]
      //MemtoReg = EX_MEM[72]
      //MemWrite = EX_MEM[71]
      //MemRead = EX_MEM[70]
      //regData2 = EX_MEM[69:38]
      //RegWrite = EX_MEM[37]
      //rd = EX_MEM[36:32]
      //alu_out = EX_MEM[31:0]
      
    end
    assign EX_MEMwire = EX_MEM;
    
    //Stage Four - Memory Access             
    assign dmem_data = EX_MEMwire[69:38]; //regData2 always written to memory
    assign dmem_addr = EX_MEMwire[31:0]; //alu_out sets the memory address
    assign dmem_wen = EX_MEMwire[71]; //If 1 write, if 0 read
    assign byte_en = EX_MEMwire[76:73]; //Byte_enable is set
    
    always@(posedge clk)begin
        if(EX_MEMwire[70] == 1) begin //If MemRead, read data to write back
            MEM_WB = { dmem_data, EX_MEMwire[72], EX_MEMwire[37], EX_MEMwire[36:32], EX_MEMwire[31:0]};
            //MEM_WB(dmem_data, MemtoReg, RegWrite, rd, alu_out)
        end
        else begin
            MEM_WB = { EX_MEMwire[72], EX_MEMwire[37], EX_MEMwire[36:32], EX_MEMwire[31:0]};
            //MEM_WB(MemtoReg, RegWrite, rd, alu_out)
        end
            
        //UPDATE THIS LIST WHEN ADDING TO CARRY REG
        //MEM_WB(dmem_data, MemtoReg, RegWrite, rd, alu_out)
        
        //dmem_data = MEM_WB[70:39]
        //MemtoReg = MEM_WB[38]
        //RegWrite = MEM_WB[37]
        //rd = MEM_WB[36:32]
        //alu_out = MEM_WB[31:0]
            
           
    end
    assign MEM_WBwire = MEM_WB;
    
    
    //Stage Five - Write Back
    //Conditional checks RegWrite to write to register file
    always@(MEM_WBwire)begin

//        if(MEM_WBwire[38] == 1) begin //Write memory data to the register
//            if(MEM_WBwire[70:39] >= 0) begin
//                writeData = MEM_WBwire[70:39];
//            end
//        end
//        else if(MEM_WBwire[38] == 0) begin //Write alu_out to the register
//            if(MEM_WBwire[37] && MEM_WBwire[31:0] >= 0) begin
//                writeData = MEM_WBwire[31:0];
//            end
//        end
            
        regW = MEM_WBwire[37];
        writeReg = MEM_WBwire[36:32];
        $fdisplay(file2, "At CC: %1d, register[%1d] = %2d", CC, writeReg, writeData);
        $fclose(file1);
        $fclose(file2);
    end  
    
    assign writeData = (MEM_WBwire[38] == 1) ? MEM_WBwire[70:39] : MEM_WBwire[31:0];
    // If MemtoReg = 1, use RAM data, else use alu_out
   

    
endmodule

//----------------------------------------------------------------------
//----------------------------- END OF CPU -----------------------------
//----------------------------------------------------------------------

module reg_file(
    input logic clk, regWrite,
    input logic [4:0] RR1, RR2, WR,
    input logic [31:0] WD,
    output logic [31:0] RD1, RD2
);
    logic [31:0] register [31:0] = '{default: 32'b0};

    assign RD1 = register[RR1]; //read data from register 1
    assign RD2 = register[RR2]; //read data from register 2
    
    always@(regWrite, WR, WD) begin
        if(regWrite) register[WR] <= WD; //write data to destination register
    end 
    
endmodule

module forwardingUnit(
    input logic EX_MEMregWrite, MEM_WBregWrite,
    input logic [4:0] EX_MEMregRd, MEM_WBregRd, rs1, rs2,
    output logic [1:0] forwardA, forwardB
);
    //Determining ALU input A
    always@(rs1, EX_MEMregRd, MEM_WBregRd, EX_MEMregWrite, MEM_WBregWrite ) begin
        if (EX_MEMregWrite && (EX_MEMregRd != 0) && (EX_MEMregRd == rs1))begin
            forwardA = 2'b10;
        end
        else if(MEM_WBregWrite && (MEM_WBregRd != 0) && (MEM_WBregRd == rs1)) begin
            forwardA = 2'b01;
        end
        else begin
            forwardA = 2'b00;
        end
    end
    
    //Determining ALU input B
    always@(rs2, EX_MEMregRd, MEM_WBregRd, EX_MEMregWrite, MEM_WBregWrite ) begin
        if (EX_MEMregWrite && (EX_MEMregRd != 0) && (EX_MEMregRd == rs2))begin
            forwardB = 2'b10;
        end
        else if(MEM_WBregWrite && (MEM_WBregRd != 0) && (MEM_WBregRd == rs2)) begin
            forwardB = 2'b01;
        end
        else begin
            forwardB = 2'b00;
        end
    end

endmodule

//need for load and store
module hazardDetectionUnit (
    input logic ID_EXMemRead,
    input logic [4:0] rs1, rs2, ID_EXregRd, 
    output logic stall
    );

always @(ID_EXMemRead, rs1, rs2, ID_EXregRd)
	begin
	  if((ID_EXMemRead == 1) && ((ID_EXregRd == rs1)||(ID_EXregRd == rs2)))
	    begin
	      stall = 1;
	    end
	  else
	    begin
	      stall = 0;
	    end
	end

endmodule

// we can try to combine control and alu control into one module
module controlUnit(
    input logic [6:0] opcode,
    output logic Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite,
    output logic [2:0] ALUOp
    );
    
    reg [8:0] ctrl;
    
    assign {ALUOp[2:0], ALUSrc, Branch, MemRead, MemWrite, RegWrite, MemtoReg} = ctrl;
    
    always @(*) begin
        casez(opcode)
        //I-type
            7'b0010011: ctrl = 9'b000100010;
            
        //R-type
            7'b0110011: ctrl = 9'b010000010;
            
        //Load
            7'b0000011: ctrl = 9'b011101011;
            
        //Store
            7'b0100011: ctrl = 9'b100100100;
        endcase
    end
endmodule

module aluControlUnit(
    input logic [2:0] ALUOp,
    input logic [2:0] func3,
    input logic [6:0] func7,
    output logic [4:0] ALUCtrl,
    output logic [3:0] byte_en
    );
    
    always @(*) begin
        casez({ALUOp, func3})
        //I-type
            ({3'b000, 3'b000}): ALUCtrl = 5'b00000; //ADDI
            ({3'b000, 3'b010}): ALUCtrl = 5'b00001; //SLTI
            ({3'b000, 3'b011}): ALUCtrl = 5'b00010; //SLTIU
            ({3'b000, 3'b100}): ALUCtrl = 5'b00011; //XORI
            ({3'b000, 3'b110}): ALUCtrl = 5'b00100; //ORI
            ({3'b000, 3'b111}): ALUCtrl = 5'b00101; //ANDI
            ({3'b000, 3'b001}): ALUCtrl = 5'b00110; //SLLI
            ({3'b000, 3'b101}): ALUCtrl = (func7[5] == 1 ? 5'b00111 : 5'b01000); //SRAI or SRLI
            
        //R-type
            ({3'b010, 3'b000}): ALUCtrl = (func7[5] == 1 ? 5'b01001 : 5'b01010); //SUB or ADD
            ({3'b010, 3'b001}): ALUCtrl = 5'b01011; //SLL
            ({3'b010, 3'b010}): ALUCtrl = 5'b01100; //SLT
            ({3'b010, 3'b011}): ALUCtrl = 5'b01101; //SLTU
            ({3'b010, 3'b100}): ALUCtrl = 5'b01110; //XOR
            ({3'b010, 3'b101}): ALUCtrl = (func7[5] == 1 ? 5'b01111 : 5'b10000); //SRA or SRL
            ({3'b010, 3'b110}): ALUCtrl = 5'b10001; //OR
            ({3'b010, 3'b111}): ALUCtrl = 5'b10010; //AND 
            
        //Load , set byte enables here
            ({3'b011, 3'b000}): begin 
                                    ALUCtrl = 5'b00000; //LB
                                    byte_en = 4'b0001;
                                end
            ({3'b011, 3'b001}): begin 
                                    ALUCtrl = 5'b00000; //LH
                                    byte_en = 4'b0011;
                                end
            ({3'b011, 3'b010}): begin 
                                    ALUCtrl = 5'b00000; //LW
                                    byte_en = 4'b1111;
                                end
            ({3'b011, 3'b100}): begin 
                                    ALUCtrl = 5'b00000; //LBU
                                    byte_en = 4'b1000;
                                end
            ({3'b011, 3'b101}): begin 
                                    ALUCtrl = 5'b00000; //LHU
                                    byte_en = 4'b1100;
                                end
            
        //Store
            ({3'b100, 3'b000}): begin 
                                    ALUCtrl = 5'b00000; //SB
                                    byte_en = 4'b0001;
                                end
            ({3'b100, 3'b001}): begin 
                                    ALUCtrl = 5'b00000; //SH
                                    byte_en = 4'b0011;
                                end
            ({3'b100, 3'b010}): begin 
                                    ALUCtrl = 5'b00000; //SW
                                    byte_en = 4'b1111;
                                end
        endcase 
    
    end
endmodule

module alu(
    input logic [4:0] shamt,
    input logic signed [31:0] A,B,
    input logic [4:0] ALUCtrl, 
    output logic [31:0] out
);    
    
    always_comb begin
        case(ALUCtrl)
        
        //I-Type
            0: out = A + B; //ADDI
            1: out = (A < B) ? 1:0; //SLTI
            2: out = (A < B) ? 1:0; //SLTIU
            3: out = A ^ B; //XORI
            4: out = A | B; //ORI
            5: out = A & B; //ANDI
            6: out = A << shamt; //SLLI
            7: out = A >>> shamt; //SRAI
            8: out = A >> shamt; //SRLI
        //R-Type
            9: out = A - B; //SUB
            10: out = A + B; //ADD
            11: begin  //SLL
                    if(B < 0) 
                        begin 
                            out = ~(A >> ~B); //Bizarre condition when B is negative
                        end
                    else out = A << B;
                end 
            12: out = (A < B) ? 1:0; //SLTI
            13: out = (A < B) ? 1:0; //SLTU
            14: out = A ^ B; //XOR
            15: begin  //SRA
                    if(B < 0) 
                        begin 
                            out = ~(A <<< ~B); //Bizarre condition when B is negative
                        end
                    else out = A >>> B;
                end 
            16: begin  //SRL
                    if(B < 0) 
                        begin 
                            out = ~(A << ~B); //Bizarre condition when B is negative
                        end
                    else out = A >> B;
                end 
            17: out = A | B; //OR
            18: out = A & B; //AND
            default: out = 32'b0;
        endcase
    end
    
endmodule
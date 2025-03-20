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
    wire [31:0] write_alu;
    wire [4:0] writeReg1; 
    wire [4:0] writeReg2;
    wire regWriteCarry1, regWriteCarry2;

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
    reg [136:0] ID_EX; //Decode and Execute carry register
    reg [69:0] EX_MEM; //Execute and Memory carry register
    reg [69:0] MEM_WB; //Memory and Write Back carry register
    
    logic [31:0] ALUinput;
    logic [31:0] writeData;
     
    //Stage One - Fetch
    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
            PC <= 32'd0;
        end 
        else begin         
            dmem_wen = 0;
            imem_addr = PC; //Address of current instruction
            PC = PC + 4; //increment pc for next instruction
            CC = CC + 1;
            IF_ID <= {PC[31:0], imem_insn[31:0]}; //Current Bits: 64
        end
    end
    
   
    
    //Stage Two - Decode and register read            
    controlUnit control(.opcode(opcode), .Branch(Branch), .MemRead(MemRead),
                .MemtoReg(MemtoReg), .MemWrite(MemWrite), .ALUSrc(ALUSrc), 
                .RegWrite(RegWrite), .ALUOp(ALUOp));
             
    reg_file regfile(.clk(clk), .regWrite(MEM_WB[37]), .RR1(rs1), .RR2(rs2), 
                     .WR(MEM_WB[36:32]), .WD(writeData), .RD1(regData1), 
                     .RD2(regData2));
                
                
    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
            opcode = 7'b0;
            rd = 5'b0;
            func3 = 3'b0;
            rs1 = 5'b0;
            rs2 = 5'b0;
           
            func7 = 7'b0 ;
            imm = 12'b0;
        end
        else begin
            //opcode can determine the proper field for immediate value
            opcode = IF_ID[6:0];
            rd = IF_ID[11:7];
            func3 = IF_ID[14:12];
            rs1 = IF_ID[19:15];
            rs2 = IF_ID[24:20];
           
            func7 = IF_ID[31:25];
            imm = IF_ID[31:20];
    
            imm_extend = { {20{imm[11]}}, imm}; //sign-extend to 32 bits
            ID_EX = { RegWrite, ALUSrc, ALUOp, rd, imm_extend, regData2, regData1, IF_ID[63:32]}; //Current Bits: 137
    
    end
    end
    //Stage Three - Execute    
    aluControlUnit aluControl(.ALUOp(ALUOp), .func3(func3), .func7(func7), .ALUCtrl(ALUCtrl));
    
    alu calc(.A(regData1), .B(ALUinput), .ALUCtrl(ALUCtrl), .out(alu_out));
     
    always@(posedge clk, negedge rst_n)begin 
        if(!rst_n) begin
            
        end
        else begin
            if(ID_EX[135] == 1)begin //I-type
                ALUinput = ID_EX[127:96];
                $display("assigned immediate");
            end
            else if(ID_EX[135] == 0) begin //R-type
                ALUinput = ID_EX[95:64];
            end
            $display("alu_out = %b", alu_out);  
            $display("ID_EX: %b", ID_EX);

        end 
        
    end
    assign EX_MEM = { regWriteCarry1, writeReg1, alu_out }; //Holds RegWrite, rd, alu_out
    assign writeReg1 = ID_EX[132:128];
    assign regWriteCarry1 = ID_EX[136];
    
    
    //Stage Four - Memory Access             
    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
            dmem_wen <= 0;
        end
        else begin
            dmem_wen <= 1;
            
            MEM_WB = { regWriteCarry2, writeReg2, write_alu}; // Current Bits: 38 bits
        end
    end
    assign write_alu = EX_MEM[31:0];
    assign writeReg2 = EX_MEM[36:32];
    assign regWriteCarry2 = EX_MEM[37];
    
 
    //Stage Five - Write Back
    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
            dmem_wen <= 0;
        end
        else begin
            if(MEM_WB[37] && MEM_WB[31:0]) begin
                writeData = MEM_WB[31:0];
                  $display("MEM_WB: %b" , MEM_WB[31:0]);
            end
            else begin
                
            end
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
            7'b0110011: ctrl = 8'b1000010;
            
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
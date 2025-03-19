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
    
    wire [31:0] alu_out, write_data;
    wire [31:0] regData1, regData2; 
      
        
    int file1, file2;
    logic [1:0] ALUsrc;
    logic [15:0] CC = 16'd0;
    logic [63:0] PC = 64'd0;
    
    logic signed [11:0] i; //immediate
    logic signed [31:0] i_extend; //immediate extended
    logic [4:0] r1, r2; //read registers
    logic [2:0] func; //func3
    logic [6:0] opcode; //opcode
    
    logic [63:0] IF_ID; //Fetch and Decode carry register
    logic [136:0] ID_EX; //Decode and Execute carry register
    logic [192:0] EX_MEM; //Execute and Memory carry register
    logic [127:0] MEM_WB; //Memory and Write Back carry register
    
    logic [31:0] ALUinput;
     
    //Stage One - Fetch
    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
            PC <= 32'd0;
        end 
        else begin         
            dmem_wen <= 0;
            imem_addr <= PC; //Address of current instruction
            PC <= PC + 4; //increment pc for next instruction
            CC <= CC + 1;
        end
    end
    assign IF_ID = {imem_insn[31:0], PC[31:0]};
    
    //Stage Two - Decode and register read
    reg_file read(.clk(clk), .RR1(r1), .RR2(r2), .WR(R2), .WD(write_data),
                  .RD1(regData1), .RD2(regData2));
    always@(posedge clk, negedge rst_n)begin
        
        //opcode can determine the proper field for immediate value
        opcode = IF_ID[38:32];
        case(opcode)
            7'b0010011: begin
                            ALUsrc = 0;                           
                            i = IF_ID[63:52];                              
                            r1 = IF_ID[51:47];                         
                            func = IF_ID[46:44];
                            r2 = IF_ID[43:39];
                        end
        endcase

        i_extend = { {20{i[11]}}, i}; //sign-extend to 32 bits
        $display("ID/EX: %b" , ID_EX);
        $display("ID/EX[135:104]:%b", ID_EX[135:104]);
        $display("ID/EX[103:72]:%b", ID_EX[103:72]);
        $display("ID/EX[71:40]:%b", ID_EX[71:40]);
    
    end
    
    assign ID_EX = {ALUsrc, i_extend, regData1, regData2, func, r2, IF_ID[31:0]};
    
    //Stage Three - Execute    
    alu calc(.A(ID_EX[103:72]), .B(ALUinput), .func3(ID_EX[39:37]), 
             .func7(ID_EX[116:110]), .out(alu_out));
     
    always@(posedge clk, negedge rst_n)begin
        if(ID_EX[136] == 0)begin //I-type
             ALUinput = ID_EX[135:104];
        end
        else if(ID_EX[136] == 1) begin //R-type
            ALUinput = ID_EX[71:40];
        end
        
        $display("alu_out = %b", alu_out);  
    end
    
    //Stage Four - Memory Access             
    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
            dmem_wen = 0;
        end
        else begin
            dmem_wen = 1;
        end
    end
        

 
    //Stage Five - Write Back
    always@(posedge clk, negedge rst_n)begin
        
//                   $display("x%1d = %h", dmem_addr, dmem_data);
//                file2 = $fopen("C:\\CMPE140\\Lab3\\data.txt", "a");
//                $fdisplay(file2, "x%1d = %h", dmem_addr, dmem_data);
//                $fclose(file2);
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
    output logic branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite,
    output logic [1:0] ALUOp
    );
    
    reg [7:0] ctrl;
    
    assign {ALUOp[1:0], ALUSrc, Branch, MemRead, MemWrite, RegWrite, MemtoReg} = ctrl;
    
    always @(*) begin
        casez(opcode)
        //I-type
            7'b00100011: ctrl = 8'b00101011;
            
        //R-type
            7'b01100011: ctrl = 8'b10000010;
            
        endcase
    end
endmodule


module aluControlUnit(
    input logic [1:0] ALUOp,
    input logic [2:0] func3,
    input logic [6:0] func7,
    output logic [3:0] ALUCtrl
    );
    
    always @(*) begin
        casez({ALUOp, func3})
        //Itype
            ({2'b00, 3'b000}): ALUCtrl = 4'b0000; //ADDI
            ({2'b00, 3'b010}): ALUCtrl = 4'b0001; //SLTI
            ({2'b00, 3'b011}): ALUCtrl = 4'b0010; //SLTIU
            ({2'b00, 3'b100}): ALUCtrl = 4'b0011; //XORI
            ({2'b00, 3'b110}): ALUCtrl = 4'b0100; //ORI
            ({2'b00, 3'b111}): ALUCtrl = 4'b0101; //ANDI
            ({2'b00, 3'b001}): ALUCtrl = 4'b0110; //SLLI
            ({2'b00, 3'b101}): ALUCtrl = (func7[5] == 1 ? 4'b0111 : 4'b1000); //SRAI or SRLI
        endcase
    
    end
    
    
    

endmodule

module alu(
    input logic [31:0] A,B,
    input logic [2:0] func3,
    input logic [6:0] func7,
    output logic [31:0] out
);    
    
    always_comb begin
        case(func3)
            3'b000: out = A + B; //ADDI
            3'b010: out = (A < B) ? 1:0; //SLTI
            3'b011: out = (A < B) ? 1:0; //SLTIU
            3'b100: out = A ^ B; //XORI
            3'b110: out = A | B; //ORI
            3'b111: out = A & B; //ANDI
            3'b001: out = A << B; //SLLI
            3'b101: if(func7 == 7'b0)begin //SRLI
                        out = A >> B;
                    end
                    else if(func7 == 7'b0100000)begin //SRAI
                        out = A >>> B;
                    end
            default: out = 32'b0;
        endcase
    end
    

endmodule
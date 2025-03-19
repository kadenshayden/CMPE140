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
    
    wire [31:0] alu_out;
    wire [31:0] regData1, regData2; 

    int file1, file2;
    logic ALUsrc, regWrite;
    logic [15:0] CC = 16'd0;
    logic [31:0] PC = 32'd0;
    
    logic signed [11:0] i; //immediate
    logic signed [31:0] i_extend; //immediate extended
    logic [4:0] r1, r2; //read registers
    logic [2:0] func; //func3
    logic [6:0] opcode; //opcode
    
    logic [63:0] IF_ID; //Fetch and Decode carry register
    logic [137:0] ID_EX; //Decode and Execute carry register
    logic [69:0] EX_MEM; //Execute and Memory carry register
    logic [69:0] MEM_WB; //Memory and Write Back carry register
    
    logic [31:0] ALUinput;
    logic [31:0] writeData;
     
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
    reg_file read(.clk(clk), .regWrite(MEM_WB[37]), .RR1(r1), .RR2(r2), 
                  .WR(MEM_WB[4:0]), .WD(writeData), .RD1(regData1), 
                  .RD2(regData2));
    always@(posedge clk, negedge rst_n)begin
        
        //opcode can determine the proper field for immediate value
        opcode = IF_ID[38:32];
        case(opcode)
            7'b0010011: begin
                            ALUsrc = 0; 
                            regWrite = 1;
                            dmem_wen = 0;                          
                            i = IF_ID[63:52];                              
                            r1 = IF_ID[51:47];                         
                            func = IF_ID[46:44];
                            r2 = IF_ID[43:39];
                        end
        endcase

        i_extend = { {20{i[11]}}, i}; //sign-extend to 32 bits
    end
    
    assign ID_EX = {regWrite, ALUsrc, i_extend, regData1, regData2, 
                    func, r2, IF_ID[31:0]};
    
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
        $display("ID_EX: %b", ID_EX);
    end
    
    assign  EX_MEM = {ID_EX[137], alu_out, ID_EX[36:32], ID_EX[31:0]};
    
    //Stage Four - Memory Access             
    always@(posedge clk, negedge rst_n)begin
        if(!rst_n) begin
            dmem_wen = 0;
        end
        else begin
            dmem_wen = 1;
        end
        
//        $display("EX_MEM: %b" , EX_MEM);
    end
        
    assign MEM_WB = {EX_MEM[69], EX_MEM[68:37], EX_MEM[36:32]};
 
    //Stage Five - Write Back
    always@(posedge clk, negedge rst_n)begin
//         $display("MEM_WB: %b" , MEM_WB);
        if(MEM_WB[37]) begin
            writeData = MEM_WB[36:5];
            $display("MEM_WB: %b" , MEM_WB[36:5]);
            $display("MEM_WB: %b" , MEM_WB[4:0]);
        end
        else begin
            
        end
          
    end
    
    
endmodule

module reg_file(
    input logic clk, regWrite,
    input logic [4:0] RR1, RR2, WR,
    input logic [31:0] WD,
    output logic [31:0] RD1, RD2
);
    logic [31:0] register [31:0] = '{default: 32'b0};; 

    assign RD1 = register[RR1]; //read data from register 1
    assign RD2 = register[RR2]; //read data from register 2
    
    always@(posedge clk) begin
        if(regWrite) register[WR] <= WD; //write data to destination register
    end 
    
endmodule

module controlUnit(
    input logic [6:0] opcode
    );
    
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
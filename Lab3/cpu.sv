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
    wire [31:0] readData1, readData2;   
        
    int file1, file2;
    logic [1:0] ALUsrc;
    logic [2:0] state = 1;
    logic [15:0] cc = 16'd0;
    logic [63:0] pc = 64'd0;
    
    logic signed [11:0] i; //immediate
    logic signed [31:0] i_extend; //immediate extended
    logic [4:0] r1, r2; //read registers
    logic [2:0] func; //func3
    logic [6:0] opcode; //opcode
    
    logic [31:0] register [31:0] = '{default: 32'b0};
    
    logic [63:0] IF_ID; //Fetch and Decode carry register
    logic [136:0] ID_EX; //Decode and Execute carry register
    logic [192:0] EX_MEM; //Execute and Memory carry register
    logic [127:0] MEM_WB; //Memory and Write Back carry register
    
    logic [31:0] ALUinput;
    
    alu calc(.A(ID_EX[103:72]), .B(ALUinput), .func3(ID_EX[39:37]), .func7(ID_EX[116:110]), .out(alu_out));
     
    always@(posedge clk, negedge rst_n)begin
    register[0] <= 0; //Ensure x0 is zero value
        case(state)
            1: begin
                //Stage One - Fetch
                if(!rst_n) begin
                    pc <= 32'd0;
                    cc <= 16'd0;
                end 
                else begin         
                    dmem_wen <= 0;
                    imem_addr <= pc; //Address of current instruction
                    IF_ID <= {imem_insn[31:0], pc[31:0]};
                    pc <= pc + 4; //increment pc for next instruction
                    cc <= cc + 1;
                    state <= 2;
                    //write to file
    //                file1 = $fopen("C:\\CMPE140\\Lab3\\PC.txt", "a");
    //                $fdisplay(file1, "PC = %h", pc);
    //                $fclose(file1);
                end
                end
            2: begin
                //Stage Two - Decode and register read
                if(!rst_n) begin
                    i = 12'd0;
                    func = 3'd0;
                    r1 = 5'd0;
                    r2 = 5'd0;
                    opcode = 7'd0;
                end
                //opcode can determine the proper field for immediate value
    //            opcode = imem_insn[6:0];
                opcode <= IF_ID[38:32];
                
                case(opcode)
                    7'b0010011: begin
                                    ALUsrc <= 0;                           
                                    i <= IF_ID[63:52];                              
                                    r1 <= IF_ID[51:47];                         
                                    func <= IF_ID[46:44];
                                    r2 <= IF_ID[43:39];
                                end
                endcase

//                $display("%b", IF_ID[63:32]);
//                $display("r1 = %1d", IF_ID[51:47]);
//                $display("r2 = %1d", IF_ID[43:39]);

                i_extend <= { {20{i[11]}}, i}; //sign-extend to 32 bits
                $display("i_extend: %b", i_extend);
                ID_EX <= {ALUsrc, i_extend, register[r1], register[r2], func, r2, IF_ID[31:0]};
                $display("ID/EX: %b" , ID_EX);
//                alu calc(.A(ID_EX[104:73]), .B(ID_EX[136:105]), .func3(ID_EX[39:37]), .func7(ID_EX[116:110]), .out(alu_out));
//                $display("%b", ID_EX[104:73]);
                $display("ID/EX[135:104]:%b", ID_EX[135:104]);
                $display("ID/EX[103:72]:%b", ID_EX[103:72]);
                $display("ID/EX[71:40]:%b", ID_EX[71:40]);
//                $display("%b", ID_EX[39:37]);
//                $display("%b", ID_EX[116:110]);
                
                cc <= cc + 1;
                state <= 3;
            end
            3: begin
                $display("ID/EX[136]:%b", ID_EX[136]);
                //Stage Three - Execute    
                if(!rst_n) begin
                       
                end 
                if(ID_EX[136] == 0)begin //I-type
                     ALUinput = ID_EX[135:104];
                end
                else if(ID_EX[136] == 1) begin //R-type
                    ALUinput = ID_EX[71:40];
                end
                
                $display("alu_out = %b", alu_out);                
                cc <= cc + 1;
                state <= 4;
            end 
            4: begin
                //Stage Four - Memory Access
                if(!rst_n) begin
                    dmem_wen <= 0;
                end
                dmem_wen <= 1;
                cc <= cc + 1;
                state <= 5;
            end
            5: begin
                //Stage Five - Write Back
//                $display("x%1d = %h", dmem_addr, dmem_data);
//                file2 = $fopen("C:\\CMPE140\\Lab3\\data.txt", "a");
//                $fdisplay(file2, "x%1d = %h", dmem_addr, dmem_data);
//                $fclose(file2);
              
                cc <= cc + 1;
                state <= 1;
            end
            default: state <= 1;
        endcase
    end
    
endmodule

module reg_file(
    input logic clk, regWrite,
    input logic [4:0] RR1, RR2, WR,
    input logic [31:0] WD,
    output logic [31:0] RD1, RD2
);
    logic [31:0] register [31:0];

    assign RD1 = register[RR1]; //read data from register 1
    assign RD2 = register[RR2]; //read data from register 2
    
    always@(posedge clk) begin
        if(regWrite) register[WR] <= WD; //write data to destination register
    end 
    
endmodule

module alu(
    input logic [31:0] A,B,
    input logic [2:0] func3,
    input logic [6:0] func7,
    output logic [31:0] out
);    
    
    always@(*) begin
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
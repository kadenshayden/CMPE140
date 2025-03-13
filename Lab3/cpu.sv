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
    logic [1:0] AlUsrc;
    logic [2:0] state;
    logic [15:0] cc = 16'd0;
    logic [63:0] pc = 64'd0;
    
    logic signed [11:0] i; //immediate
    logic signed [31:0] i_extend; //immediate extended
    logic [4:0] r1, r2; //read registers
    logic [2:0] func; //func3
    logic [6:0] opcode; //opcode
    
    logic [31:0] register [31:0];
    
    logic [63:0] IF_ID; //Fetch and Decode carry register
    logic [135:0] ID_EX; //Decode and Execute carry register
    logic [192:0] EX_MEM; //Execute and Memory carry register
    logic [127:0] MEM_WB; //Memory and Write Back carry register
    
    
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
                    $display("PC = %h", pc); //Display to console
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
                                    ALUsrc <= 1;
    //                                i <= imem_insn[31:20];
                                    i <= IF_ID[63:52];
    //                                rs <= imem_insn[19:15];
                                    r1 <= IF_ID[51:47];
    //                                func <= imem_insn[14:12];
                                    func <= IF_ID[46:44];
    //                                rd <= imem_insn[11:7];
                                    r2 <= IF_ID[43:39];
                                end
                endcase

                $display("%b", IF_ID[63:32]);
                $display("rs = %b", IF_ID[51:47]);
                $display("rd = %b", IF_ID[43:39]);
                
                i_extend <= { {20{i[11]}}, i}; //sign-extend to 32 bits
                ID_EX <= {i_extend, register[r1], register[r2], func, r2, IF_ID[31:0]};
                cc <= cc + 1;
                state <= 3;
            end
            3: begin
                //Stage Three - Execute                
                if(!rst_n) begin
                       
                end 
                else begin
                    alu calc(.A(register[rs]), .B(ID_EX[127:95]), .func3(func), .func7(i[11:5]), .out(alu_out));
                end  
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
                $display("x%1d = %h", dmem_addr, dmem_data);
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

    logic signed [31:0] B_extend = { {20{B[11]}}, B};
    
    always@(*) begin
        case(func3)
            3'b000: out = A + B_extend; //ADDI
            3'b010: out = (A < B_extend) ? 1:0; //SLTI
            3'b011: out = (A < B_extend) ? 1:0; //SLTIU
            3'b100: out = A ^ B_extend; //XORI
            3'b110: out = A | B_extend; //ORI
            3'b111: out = A & B_extend; //ANDI
            3'b001: out = A << B_extend; //SLLI
            3'b101: if(func7 == 7'b0)begin //SRLI
                        out = A >> B_extend;
                    end
                    else if(func7 == 7'b0100000)begin //SRAI
                        out = A >>> B_extend;
                    end
            default: out = 32'b0;
        endcase
    end
    

endmodule
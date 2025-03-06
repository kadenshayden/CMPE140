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

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/19/2025 10:08:09 AM
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
//Change [data_width - 1: 0] mem [(1<<addr_width)-1:0] to [data_width - 1: 0] mem [ addr_width - 1 :0] in ram.sv lol
//////////////////////////////////////////////////////////////////////////////////


module cpu(
    input logic rst_n, clk, 
    input logic [31:0] imem_insn,
    output logic dmem_wen, 
    output logic [31:0] imem_addr, dmem_addr,
    inout logic [31:0] dmem_data
    );
    
    int file1, file2;
    
    logic [15:0] cc = 16'd0;
    logic [15:0] pc = 16'd0;
    
    logic [11:0] i; //immediate
    logic [4:0] rs; //source register
    logic [2:0] func; //func3
    logic [4:0] rd; //destination register
    logic [6:0] opcode; //opcode
    
    logic [4:0] dest_addr;
    logic [31:0] result3, result4;
    
    //Stage One - Fetch
    always@(posedge clk, negedge rst_n)begin
        if(cc % 5 == 0) begin
            if(!rst_n) begin
                pc <= 16'd0;
                cc <= 16'd0;
            end else begin         
                dmem_wen <= 0;
                imem_addr <= pc; //Address of current instruction
                pc <= pc + 4; //increment pc for next instruction
                cc <= cc + 1;
                $display("PC = %h", pc); //Display to console
                //write to file
                file1 = $fopen("C:\\CMPE140\\Lab3\\PC.txt", "a");
                $fdisplay(file1, "PC = %h", pc);
                $fclose(file1);   
            end
         end
    end

    //Stage Two - Decode
    always@(posedge clk, negedge rst_n)begin
        if(cc % 5 == 1) begin
            if(!rst_n) begin
                i <= 12'd0;
                func <= 3'd0;
                rd <= 5'd0;
                opcode <= 7'd0;
            end
            $display("%b", imem_insn);
            $display("%b", imem_insn[19:15]);
            i <= imem_insn[31:20];
            func <= imem_insn[14:12];
            rd <= imem_insn[11:7];
            opcode <= imem_insn[6:0];
            
            dmem_addr <= imem_insn[19:15];
            cc <= cc + 1;
        end
    end
    
    //Stage Three - Execute
    always@(posedge clk, negedge rst_n)begin
        if(cc % 5 == 2) begin
            if(!rst_n) begin
                result3 <= 32'd0;
            end 
            if(opcode == 7'b0010011 && func == 3'b000) begin //addi
                if(dmem_addr == 32'b0) begin
                    result3 <= { {20{i[11]}}, {i[11:0]}};  
                end else begin
                    result3 <= dmem_data + { {20{i[11]}}, {i[11:0]}};
            end
        end
        cc <= cc + 1;
        dest_addr <= rd;
    end
    end
    
    //Stage Four - Memory Access
    always@(posedge clk, negedge rst_n)begin
        if(cc % 5 == 3) begin
            if(!rst_n) begin
                result4 <= 32'd0;
            end
            dmem_addr <= dest_addr;
            dmem_wen <= 1;
            result4 <= result3;
            cc <= cc + 1;
        end
    end
    
    //Stage Five - Write Back
    always@(posedge clk, negedge rst_n)begin
        if(cc % 5 == 4) begin
                dmem_addr <= rd;
                $display("x%1d = %h", dmem_addr, dmem_data);
                file2 = $fopen("C:\\CMPE140\\Lab3\\data.txt", "a");
                $fdisplay(file2, "x%1d = %h", dmem_addr, dmem_data);
                $fclose(file2);
              
                cc <= cc + 1;
        end
    end
    assign dmem_data = rst_n ? (dmem_wen ? result4 : 32'bz) :  32'bz;
    
endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/13/2023 09:55:51 PM
// Design Name: 
// Module Name: tb
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


module tb(
    );
    
    parameter word_size = 32;
    parameter address_size = 32;
    
    reg rst_n, clk;
    
    always #10 clk = ~clk;
    
    initial
        begin
            rst_n = 0;
            clk = 0;
            
            #35 rst_n = 1;
            #800;
            $stop;
        end
    
wire [address_size-1:0] imem_addr;
wire [word_size-1:0] imem_insn;
wire [address_size-1:0] dmem_addr;
wire [word_size-1:0] dmem_data;
wire dmem_wen;
wire [3:0] byte_en;


cpu dut 
(
    .*
);
    
    // Change to the file you need
    rom #( .addr_width (address_size), .data_width (word_size), .init_file ("line.dat") )
imem (
.addr(imem_addr),
.data(imem_insn)
);

    ram #( .addr_width (address_size), .data_width (word_size), .init_file ("dmem.dat") )
dmem (
.rst_n (rst_n),
.clk (clk),
.wen (dmem_wen),
.byte_en (byte_en),
.addr (dmem_addr),
.data (dmem_data)
);
    
    
    
endmodule

`timescale 1ns/1ps

module rom #(addr_width = 32, data_width = 32, string init_file = "dummy.dat" )
(
input [addr_width-1:0]addr,
output [data_width-1:0]data
);

reg [7:0] mem [255:0]; //[ (1 << addr_width)-1:0];

initial
    begin
        $readmemb (init_file, mem);
        /*for (integer i = 0; i < 16; i = i + 1)
        begin 
            mem[i] = 4'hf - i;
        end*/
    end
    
assign data = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]};


endmodule
`timescale 1ns/1ps

module ram #(addr_width = 32, data_width = 32, string init_file = "dummy.dat" )
(
input rst_n,
input clk,
input wen,
input [3:0] byte_en,
input [addr_width-1:0]addr,
//input [data_width-1:0] data_in,
inout [data_width-1:0] data
);

reg [7:0] mem [ 255 :0];
wire [7:0] addr_p;
assign addr_p = addr & 8'hff;
/*reg [data_width-1:0] mem [ 4294967295 : 0 ]
wire [31:0] addr_p;
assign addr_p = addr;*/


initial
    begin
        $readmemb (init_file, mem);
    end
    
assign data = rst_n ? ( wen ? 32'hz : {mem[addr_p+3], mem[addr_p+2], mem[addr_p+1], mem[addr_p]}) : 'x;

/*
logic [3:0] wen_4;
assign wen_4 = {wen, wen, wen, wen};
logic [3:0] wen_b;
assign wen_b = wen_4 & byte_en;
*/
always_ff @ (posedge clk)
    begin
        if (rst_n)
            begin
            $display ("%b %b\n", wen, byte_en );
                if (wen & byte_en[3])
                        mem[addr_p+3] <= #0.1 data[31:24];
                if (wen & byte_en[2])
                        mem[addr_p+2] <= #0.1 data[23:16];
                if (wen & byte_en[1])
                        mem[addr_p+1] <= #0.1 data[15:8];
                if (wen & byte_en[0])
                        mem[addr_p] <= #0.1 data[7:0];
            end
    end
    
endmodule

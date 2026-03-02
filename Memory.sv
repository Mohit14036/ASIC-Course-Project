
module Memory#(
    parameter int Data_Width = 16,
    parameter int Data_Depth = 64,
    parameter int Address_Bit_Width = 6
)
(     // THIS IS MODELLED LIKE SDP (SIMPLE DUAL PORT RAM) . CHANGE CODE IF U FIND THE BEHAVIOR OF RAM IS INNACURATE.

    input logic WE,RE,
    input logic [Data_Width-1:0] Din,  
    input logic [Address_Bit_Width-1:0] RA,WA,
    output logic [Data_Width-1:0] Dout,
     
    input logic clk

);
    logic [Data_Width-1:0] RAM [0:Data_Depth-1];    
    always_ff@(posedge clk) begin 
        if (WE) RAM[WA] <= Din;
    end


    always_ff @(posedge clk) begin
        if (RE)
            Dout <= RAM[RA];
    end



endmodule

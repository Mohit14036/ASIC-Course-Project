package pkg_line_memory;

    parameter A = 128;
    parameter A_Bit_Width = 7;

    parameter N = 32;
    parameter N_Bit_Width = 5;

    parameter K =16;
   // parameter delta = 64;
    //parameter Delta_Bit_Width = 6;      // DELTA_BIT_WIDTH = $CLOG2(delta). TO AVOID LINTER WARNINGS I PUT VALUE DIRECTLY.

    //parameter  Z =64 ;
    //parameter Z_Bit_Width = 6;          // Z_BIT_WIDTH = $CLOG2(M). TO AVOID LINTER WARNINGS I PUT VALUE DIRECTLY.
    //parameter Line_Selection_Width = 5; // LINE_SELECTION_WIDTH = $CLOG2(M). TO AVOID LINTER WARNINGS I PUT VALUE DIRECTLY.

endpackage



module Line_Memory

import pkg_line_memory::*;
(
        input logic signed [K-1:0] I,
        input logic clk,
        input logic Write_Selector,
        input logic Read_Selector,

        input logic [A_Bit_Width-1:0] RA_n,RA_r,

        input logic Next_Stride,
        input logic Reuse_Selector,
        input logic [A_Bit_Width+N_Bit_Width-1:0]r_ns,

        output logic signed [K-1:0] O[0:N-1]
);

/////////////////////////////////////// MEMORY ////////////////////////////////////////////////////

    wire [N_Bit_Width-1:0] ns = r_ns[N_Bit_Width-1:0];   //  n/s
    wire [A_Bit_Width-1:0] r = r_ns[A_Bit_Width+N_Bit_Width-1:N_Bit_Width]; 
    

    wire  IDM_out = WA >r ; // NO READING IF WA <=r

    wire RE = Read_Selector && IDM_out; //READ ENABLE FOR MEMORY, NO READ IF WA<=r

    

    
    logic [A_Bit_Width-1:0] WA;  // WRITE ADDRESS FOR MEMORY
    WAG wag(.Write_Selector(Write_Selector),.WA(WA),.clk(clk)); // WAG

    logic [N_Bit_Width-1:0] Output_Mux_Select; // MUX SELECT LINE
    logic Address_Decoder_Enb;           
    logic [N_Bit_Width-1:0] Address_Decoder_In;  //ADDRESS DECODER INPUT
 
    wire [A_Bit_Width-1:0] Rd_Ptr = Reuse_Selector? RA_r:RA_n; //READ POINTER
    logic [A_Bit_Width-1:0] RA;  // READ ADDRESS FOR MEMORY
    RAG rag(.ns(ns),.Rd_Ptr(Rd_Ptr),.Next_Stride(Next_Stride),.RE(RE),.RA(RA),.Address_Decoder_In(Address_Decoder_In),
            .Output_Mux_Select(Output_Mux_Select),.Address_Decoder_Enb(Address_Decoder_Enb),.clk(clk));

    logic [K-1:0] Output_Buffer_Din; // DOUT OF MEMORY AND DIN OF BUFFER.
    Memory #(.Data_Width(K),.Data_Depth(A),.Address_Bit_Width(A_Bit_Width)) 
    Memory_Line_Memory(.WE(Write_Selector),.RE(RE),.Din(I),.RA(RA),.WA(WA),.Dout(Output_Buffer_Din),.clk(clk));


/////////////////////////////////////// MEMORY  END////////////////////////////////////////////////////



////////////////////////////////////////////// OUTPUT REGISTERS AND MUXES ///////////////////////////////////
    logic [K-1:0] Output_Buffer[0:N-1]; //OUTPUT BUFFER OF WIDTH K BITS AND LENGTH N
    logic [N-1:0]Output_Buffer_Enables; // ENABLES FOR BUFFER.


    always_ff@(posedge clk) begin   
        for (int i=0;i<N;i++) begin
            if(Output_Buffer_Enables[i]) Output_Buffer[i] <= Output_Buffer_Din;  //ONLY UPDATE UF CORRESPONDING ENABLE IS HIGH.
        end
    end


    logic [K-1:0] Mux_Inputs[0:N-1][0:N-1]; // N MUXES WITH N INPUTS FOR EACH MUX.
 /*

FOR MUX 0, INPUT 0 WILL COME FROM INPUT_BUFFER([(0+0)%27) = INPUT_BUFFER[0], 
FOR MUX 1 INPUT 0 WILL COME FROM  INPUT_BUFFER([(1+0)%27) = INPUT_BUFFER[1], 
FOR MUX 26 INPUT 0 WILL COME FROM  INPUT_BUFFER([(26+0)%27) = INPUT_BUFFER[26],
FOR MUX 26 INPUT 1 WILL COME FROM  INPUT_BUFFER([(26+1)%27) = INPUT_BUFFER[0],
FOR MUX 26 INPUT 2 WILL COME FROM  INPUT_BUFFER([(26+2)%27) = INPUT_BUFFER[1], 

*/
    always_comb begin
        for (int i=0;i<N;i++) begin
            for(int j=0;j<N;j++) begin
                Mux_Inputs[i][j]  = Output_Buffer[(i+j)%N];
            end
        end
    end

    always_comb begin
        for (int i=0;i<N;i++) begin
            O[i]  = Mux_Inputs[i][Output_Mux_Select];
        end
    end




    Address_Decoder address_Decoder
    (.enable(Address_Decoder_Enb),.in(Address_Decoder_In),.out(Output_Buffer_Enables));

////////////////////////////////////////////// OUTPUT REGISTERS  AND MUXSES END ///////////////////////////////////
    
endmodule



module  Address_Decoder
import pkg_line_memory::*;
 
(
    input  logic enable, 
    input  wire [N_Bit_Width-1:0]  in,     
    output wire [N-1:0] out    
);

    assign out = enable ? (N'(1) << in) : N'(0); //LEFT SHIFT BY N.

endmodule




module  WAG
    import pkg_line_memory::*;
(
    input logic Write_Selector,
    output logic [A_Bit_Width-1:0] WA,

    input logic clk
);
    always_ff@(posedge clk) begin
        if(WA == A) WA <=0;   //TO PREVENT  WA CROSSING A (NOT NEEDED IF A IS POWER OF 2)
        else WA <= WA+ Write_Selector;
    end


endmodule

module  RAG
    import pkg_line_memory::*;
(
    input logic [N_Bit_Width-1:0] ns,
    input logic [A_Bit_Width-1:0] Rd_Ptr,
    input logic Next_Stride,
    input logic RE,

    output logic [A_Bit_Width-1:0] RA,
    output logic [N_Bit_Width-1:0] Address_Decoder_In,
    output logic [N_Bit_Width-1:0] Output_Mux_Select,
    output logic Address_Decoder_Enb,

    input logic clk
);
    
    logic Set_Reset;
        
    wire  Reset = (ns == Address_Decoder_In); //RESET WHEN DECODER ADDRESS ==NS.

    always@(posedge clk) begin
        if(Next_Stride) Set_Reset <=1;
        else if(Reset) Set_Reset <=0;
    end

    assign Address_Decoder_Enb  = Set_Reset && RE;

    always_ff@(posedge clk) begin
        
        if(Address_Decoder_Enb) begin
            Output_Mux_Select <= Output_Mux_Select + ns;
        end
    end


    always_ff@(posedge clk) begin
        Address_Decoder_In <= Address_Decoder_In + Set_Reset;
    end

    
    assign RA = Rd_Ptr + Address_Decoder_In;

    
        



endmodule


//import pkg::*;


package pkg_cnn;

    parameter M = 32;
    parameter K =16;
    parameter delta = 63;
    parameter Delta_Bit_Width = 6;      // DELTA_BIT_WIDTH = $CLOG2(delta). TO AVOID LINTER WARNINGS I PUT VALUE DIRECTLY.

    parameter  Z =63 ;
    parameter Z_Bit_Width = 6;          // Z_BIT_WIDTH = $CLOG2(M). TO AVOID LINTER WARNINGS I PUT VALUE DIRECTLY.
    parameter Line_Selection_Width = 5; // LINE_SELECTION_WIDTH = $CLOG2(M). TO AVOID LINTER WARNINGS I PUT VALUE DIRECTLY.

endpackage


import pkg_cnn::*;



/* NOTE ::  1) PARTIAL SUM OR BIAS IS K BITS BECAUSE, ONLY MSB K BITS OF MULTIPLIER OUTPUT ARE TAKEN AND REST ARE DISCARDED.
            2) THE MSB K BITS OF MULTIPLICATION MOVE TO NEXT PE AS PSUM AND WILL BE ADDED TO  MSB K BITS OF MULTIPLIER OUTPUT THERE.

*/

module PE (
    input logic S_Ovd,                                              //SIGNAL FOR OVERRIDING SIGN ZERO DETECTOR TO ALLOW NEGETIVE AND NULL INPUTS.
    input logic [Line_Selection_Width-1:0] Line_Selection_Control,  // SIGNAL TO SELECT INPUT FROM M INPUTS
    input logic signed [K-1:0] B_Psum,         //BIAS OR PARTIAL SUM.

    input logic [1:0] Wr_Rr,                   //CONTROL SIGNALS FOR MEMORY.
    input logic signed [K-1:0] W,              // INPUT WEIGHTS.

    input logic [2*Z_Bit_Width:0] R6_r_Delta,  //R6,R,DELTA CONCATENATED TOGETHER. I SPLIT THEM INTERNALLY.


    input logic signed [K-1:0] Ix [0:M-1],  // INPUT BUS CARRYING M INPUTS FROM M LINE MEMORIES.
    input logic MAC_MAX,                    // CONTROL SIGNAL TO DETERMINE BETWEEN MAC AND MAX OPERATION.


    
    
    
    input logic clk,
    
    
    
    output signed [K-1:0] Psum,            // OUTPUT PARTIAL SUM FOR NEXT PE.
    output logic Stride_Request            // STRIDE REQUEST.


);




/////////////////////////////////////// INPUT REGSITER and SZ DETECTION ////////////////////////////////////////////
    logic signed [K-1:0] input_reg; 
    wire  SZD_enb = S_Ovd?(input_reg <=0):1'b0;   // THIS SIGNAL GOES HIHG WHEN INPUT IS NEGETIVE OR NULL.

    always_ff@(posedge clk) begin
        if(Stride_Request) input_reg <= Ix[Line_Selection_Control];  // REGISTER READS ONLY DURING STRIDE REQUEST.
    end

    wire signed [K-1:0] Null_Mux_Out = SZD_enb? 'b0:input_reg;   // THIS MUX GIVES OUTPUT  0 IF INPUT IS NEGETIVE OR 0.






 
///////////////////////////////////////  AGU AND MEMORY  /////////////////////////////////////////
 
    logic [Z_Bit_Width-1:0] WA,RA;  //WRITE ADDRESS, READ ADDRESS FOR MEMORY/.

    wire [Z_Bit_Width-1:0] r = R6_r_Delta[2*Z_Bit_Width-1:Z_Bit_Width];  //SPLITTING r FROM THE BUS "R6_r_Delta"
    wire [Z_Bit_Width-1:0] Delta = R6_r_Delta[Z_Bit_Width-1:0];   //SPLITTING Delta FROM THE BUS "R6_r_Delta"

    wire  IDM_out = (WA >r);   //IDM IS HIGH ONLY WA OF MEMORY IS GREATER THAN r.

    wire RE =  IDM_out && Wr_Rr[0]; //NO READING WHEN IDM_out IS LOW.
    wire WE = Wr_Rr[0];   //EXTERNAL SIGNAL Wr_Wr

    logic [K-1:0] Dout;  //MEMORY OUTPUT.


    AGU_Read  read_agu(
        .Delta(Delta),.clk(clk),.RE(RE),.RA(RA),.rst(Stride_Request)
    );

    AGU_Write   write_agu(
        .Delta(Delta),.clk(clk),.Wr(Wr_Rr[1]), .WA(WA)
    );


    Memory memory(
        .WE(WE),.RE(RE),.Din(W),.RA(RA),.WA(WA),.Dout(Dout),.clk(clk)

    );

/////////////////////////////////////// AGU AND MEMORY END /////////////////////////////////////////







/////////////////////////////////////// MAC, MAX AND MIN/////////////////////////////////////////


    wire R6 = R6_r_Delta[2*Z_Bit_Width];  //SPLITTING R6 FROM BUS R6_r_Delta.

   

    logic signed [K-1:0] Mac_Out;  //MAC MODULE OUTPUT 
    logic signed [K-1:0] Min_Out;  //MIN MODULE OUTPUT 
    logic signed [K-1:0] Max_Out;  //MAX MODULE OUTPUT 

    logic signed [K-1:0] six = 6;  //CONSTANT 6 FOR MIN MODULE INPUT PORT B.

    MAC mac(.weight(Dout),.in(input_reg),.enb(SZD_enb),.clk(clk),.Bias(B_Psum),. out(Mac_Out));

    MIN min(.A(Mac_Out),.B(six),.out(Min_Out),.R6(R6));

    MAX max(.A(Null_Mux_Out),.B(B_Psum),.out(Max_Out));
///////////////////////////////////////  MAC,MAX AND MIN END/////////////////////////////////////////


    assign Psum = MAC_MAX ? Min_Out : Max_Out;

endmodule










module AGU_Write(
    input logic [Delta_Bit_Width-1:0] Delta,
    input clk,

    input logic Wr,  //EXTERNAL SIGNAL TO CONTROL COUNTER.

    output logic [Z_Bit_Width-1:0] WA  //WRITE ADDRESS OF MEMORY.

);

    logic rst;
    wire [Delta_Bit_Width-1:0] Counter;      //TO COUNT TO Z/DELTA

    assign Counter  = Wr + WA;
    assign rst = (Counter == Delta);        //RESET HIGH WHEN COUNTER REACHES DELTA

    always@(posedge clk) begin
        if(rst) WA<= 0;
        else  WA <= Counter;
    end



endmodule

module AGU_Read(
    input logic [Delta_Bit_Width-1:0] Delta,    
    input clk,

    input logic RE,  //READ ENABLE OF MEMORY

    output logic [Z_Bit_Width-1:0] RA, //READ ADDRESS
    output wire  rst   // THE RST IS STRIDE REQUEST 

);

    wire [Delta_Bit_Width-1:0] Counter;  //TO COUNT TO Z/DELTA

    assign Counter  = RE + RA;              
    assign rst = (Counter == Delta);  //RESET HIGH WHEN COUNTER REACHES DELTA

    always@(posedge clk) begin
        if(rst) RA <= 0;
        else  RA <= Counter;
    end



endmodule



module Memory(     // THIS IS MODELLED LIKE SDP (SIMPLE DUAL PORT RAM) . CHANGE CODE IF U FIND THE BEHAVIOR OF RAM IS INNACURATE.

    input logic WE,RE,
    input logic [K-1:0] Din,  
    input logic [Z_Bit_Width-1:0] RA,WA,
    output logic [K-1:0] Dout,
     
    input logic clk

);
    logic [K-1:0] RAM [0:Z-1];    
    always_ff@(posedge clk) begin 
        if (WE) RAM[WA] <= Din;
    end


    always_ff @(posedge clk) begin
        if (RE)
            Dout <= RAM[RA];
    end



endmodule


module MAC(
    input logic signed [K-1:0] weight, //WEIGHT FROM MEMORY
    input logic signed [K-1:0] in,    //INPUT FROM REGISTER
    input logic enb,                    // EMABLE FROM SZD
    input logic clk,
    input logic signed  [K-1:0] Bias,   // BIAS/PSUM FROM PREVIOUS PE

    output wire signed [K-1:0] out  // OUTPUT OF ACCUMULATOR
);
 


    wire signed [K-1:0] weight_gated  = (~enb) ? weight:'0;   // WEIGHT_GATED BECOMOES  0 WHEN ENB=1(THIS IS TO SAVE POWER FROM SWITCHING)
    wire signed [K-1:0] in_gated  = (~enb)? in : '0;             // INPUT_GATED BECOMOES  0 WHEN ENB=1(THIS IS TO SAVE POWER FROM SWITCHING)

    wire signed [2*K-1:0] multiplier_out = weight_gated* in_gated; //TEMPORARY VARIABLE TO STORE MULTIP OUTPUT, THIS STORES 32 OR 2K BITS.
    reg signed [K-1:0] multiplier_out_reg;                          // REG TO HOLD 16 MSB BITS OF MULTIPLICATON.


    always_ff@(posedge clk) begin
         multiplier_out_reg <= multiplier_out[2*K-1:K];  // LOWER 16 OR K BITS ARE DISCARDED
    end

    assign out = multiplier_out_reg + Bias;  // ACCUMUALTOR OUTPUT.


endmodule 

module MAX(
    input logic signed [K-1:0] A,B,  //K BIT INPUTS 
    output logic signed [K-1:0] out  // K BIT OUTPUT.
);


    wire signed [K-1:0] result = A-B; 


    always_comb begin
        if(result[K-1]) out = B; // IF MSB OF RESLT IS 1, THEN A<B aAND B WILL BE SELECTED
        else out =A;
    end



endmodule

module MIN(
    input logic signed [K-1:0] A,B,  // A IN MAC OUTPUT  AND B IS 6
    output logic signed [K-1:0] out,

    input logic R6   // EMABLE SIGNAL FOR THIS MODULE.
);


    wire signed [K-1:0] result = B-A;   //IS MSB OF REUSLT IS 1, THEN A>B (A>6)

    wire  select = result[K-1] && R6;  


    always_comb begin
        if(select) out = B;    // IF SELECT  IS 1, THEN A>B AND B(6) WILL BE SELECTED.
        else out =A;
    end



endmodule

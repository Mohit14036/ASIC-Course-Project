
package pkg_KPU;
    parameter M = 27;
    parameter  M_Bit_Width = 5;

    parameter N = 32;
    parameter N_Bit_Width =5 ;
    parameter K =16;

     parameter delta = 64;
    parameter Delta_Bit_Width = 6;      // DELTA_BIT_WIDTH = $CLOG2(delta). TO AVOID LINTER WARNINGS I PUT VALUE DIRECTLY.

    parameter  Z =64 ;
    parameter Z_Bit_Width = 6;          // Z_BIT_WIDTH = $CLOG2(M). TO AVOID LINTER WARNINGS I PUT VALUE DIRECTLY.
    parameter Line_Selection_Width = 5; 


    parameter A = 128;
    parameter A_Bit_Width = 7;
endpackage



module KPU
import pkg_KPU::*;
(
    input logic signed [K-1:0] BIAS_Bus[0:M-1],
    input logic signed [K-1:0] Line_Memory_INPUTS_Bus[0:M-1],
    input logic signed [K-1:0] WEIGHTS_Bus[0:M-1][0:N-1],

    input logic clk,

    input logic layer_information
);


    /// BUSES FOR PES

    logic signed [K-1:0] Ix_Bus[0:N-1][0:M-1];
    logic Stride_Request_Bus[0:N-1][0:M-1];

    logic S_Ovd_Bus[0:M-1][0:N-1];
    logic MAC_MAX_Bus[0:M-1][0:N-1];
    logic [M_Bit_Width-1:0] Line_Selection_Control_Bus[0:M-1][0:N-1];    
    logic [1:0] Wr_Rr_Bus[0:M-1][0:N-1];   
    logic [2*Z_Bit_Width:0] R6_r_Delta_Bus[0:M-1][0:N-1];    

    logic [K-1:0] Psum_Collection_Bus[0:M-1][0:N-1];

 


 

   


    genvar m, n;

    generate
        for(m = 0; m < M; m++) begin : PE_row
            for(n = 0; n < N; n++) begin : PE_colomn


                if(n==0) begin : colomn0;
                     PE pe(
                        .S_Ovd(S_Ovd_Bus[m][n]),                                              
                        .Line_Selection_Control(Line_Selection_Control_Bus[m][n]),  
                        .B_Psum(BIAS_Bus[m]),       
                        .Wr_Rr(Wr_Rr_Bus[m][n]),                   
                        .W(WEIGHTS_Bus[m][n]),              
                        .R6_r_Delta(R6_r_Delta_Bus[m][n]),  
                        .Ix(Ix_Bus[n]),  
                        .MAC_MAX(MAC_MAX_Bus[m][n]),                    
                        .clk(clk),    
                        .Stride_Request(Stride_Request_Bus[n][m]),
                        .Psum(Psum_Collection_Bus[m][n])      );
                end
                else begin :other_colomns
                      PE pe(
                        .S_Ovd(S_Ovd_Bus[m][n]),                                              
                        .Line_Selection_Control(Line_Selection_Control_Bus[m][n]),  
                        .B_Psum(Psum_Collection_Bus[m][n-1]),       
                        .Wr_Rr(Wr_Rr_Bus[m][n]),                   
                        .W(WEIGHTS_Bus[m][n]),              
                        .R6_r_Delta(R6_r_Delta_Bus[m][n]),  
                        .Ix(Ix_Bus[n]),  
                        .MAC_MAX(MAC_MAX_Bus[m][n]),                    
                        .clk(clk),    
                        .Stride_Request(Stride_Request_Bus[n][m]),
                        .Psum(Psum_Collection_Bus[m][n])   );
                end

               
            end
        end
    endgenerate

     /// BUSES FOR LINE MEMORIES
    logic Read_Selector_Bus[0:M-1];
    logic Write_Selector_Bus[0:M-1];
    logic Reuse_Selector_Bus[0:M-1];
    logic [A_Bit_Width-1:0] RA_n_Bus[0:M-1];
    logic [A_Bit_Width-1:0] RA_r_Bus[0:M-1];
    logic                   Next_Stride_Bus[0:M-1];
     logic [A_Bit_Width+N_Bit_Width-1:0]r_ns_Bus[0:M-1];

     logic signed [K-1:0] Output_Bus_Line_Memory[0:M-1][0:N-1];

      generate
        for(m = 0; m < M; m++) begin : line_memory_row
         Line_Memory line_memory
        (
                .I(Line_Memory_INPUTS_Bus[m]),
                .clk(clk),
                .Write_Selector(Write_Selector_Bus[m]),
                .Read_Selector(Read_Selector_Bus[m]),

                .RA_n(RA_n_Bus[m]),
                .RA_r(RA_r_Bus[m]),

                .Next_Stride(Next_Stride_Bus[m]),
                .Reuse_Selector(Reuse_Selector_Bus[m]),
                .r_ns(r_ns_Bus[m]),

                .O(Output_Bus_Line_Memory[m])
        );

        end
    endgenerate

        always_comb begin
             for(int j=0;j<N;j++)begin 
                for(int i=0;i<M;i++)begin
                        Ix_Bus[j][i] = Output_Bus_Line_Memory[i][j];
                end
            end
        end


   KPC  kpc
(
    .Stride_Request_Bus(Stride_Request_Bus),

    .S_Ovd_Bus(S_Ovd_Bus),
     .MAC_MAX_Bus(MAC_MAX_Bus),
    .Line_Selection_Control_Bus(Line_Selection_Control_Bus),    
    .Wr_Rr_Bus(Wr_Rr_Bus),
    .R6_r_Delta_Bus(R6_r_Delta_Bus)   ,
    
    .Read_Selector_Bus(Read_Selector_Bus),
    .Write_Selector_Bus(Write_Selector_Bus),
    .Reuse_Selector_Bus(Reuse_Selector_Bus),
    .RA_n_Bus(RA_n_Bus),
    .RA_r_Bus(RA_r_Bus),
    .Next_Stride_Bus(Next_Stride_Bus),
    .r_ns_Bus(r_ns_Bus),

    .clk(clk),
    .layer_information(layer_information)
);

endmodule


module KPC

import pkg_KPU::*;
(
    input logic Stride_Request_Bus[0:N-1][0:M-1],

    output logic S_Ovd_Bus[0:M-1][0:N-1],
    output logic MAC_MAX_Bus[0:M-1][0:N-1],
    output logic [M_Bit_Width-1:0] Line_Selection_Control_Bus[0:M-1][0:N-1],    
    output logic [1:0] Wr_Rr_Bus[0:M-1][0:N-1],
    output logic [2*Z_Bit_Width:0] R6_r_Delta_Bus[0:M-1][0:N-1]   ,
    
    output logic Read_Selector_Bus[0:M-1],
    output logic Write_Selector_Bus[0:M-1],
    output logic Reuse_Selector_Bus[0:M-1],
    output logic [A_Bit_Width-1:0] RA_n_Bus[0:M-1],
    output logic [A_Bit_Width-1:0] RA_r_Bus[0:M-1],
    output logic                   Next_Stride_Bus[0:M-1],
    output logic [A_Bit_Width+N_Bit_Width-1:0]r_ns_Bus[0:M-1],

    input logic clk,

    input logic layer_information
);



endmodule



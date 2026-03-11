`timescale 1ns/1ps
package pkg_IEC;
    parameter M = 9;
    parameter  M_Bit_Width = 4;

    parameter N = 6;
    parameter N_Bit_Width =3 ;
    parameter K =16;

     parameter delta = 64;
    parameter Delta_Bit_Width = 4;      // DELTA_BIT_WIDTH = $CLOG2(delta). TO AVOID LINTER WARNINGS I PUT VALUE DIRECTLY.

    parameter  Z =16 ;
    parameter Z_Bit_Width = 4;          // Z_BIT_WIDTH = $CLOG2(M). TO AVOID LINTER WARNINGS I PUT VALUE DIRECTLY.
    parameter Line_Selection_Width = 4; 


    parameter A = 128;
    parameter A_Bit_Width = 7;
endpackage

module IEC 
import pkg_IEC::*;
    ( 
    input logic clk,
    input logic rst,
    output logic [15:0] I[0:8],
    output logic [15:0] W[0:8][0:5],
    output logic [15:0] Psum
);

    


endmodule
// four  25*multi and add tree, output 4 number(16bit width)
`timescale 1ns/1ps
`define DATA_SIZE 8
`define KENNEL_SIZE 5
`define halfword_width 16
`define inputMapWidth  6*6*8 // get a 6*6 input map to rearrange to 4 5*5 input map which will be input to 4 different conv_mul_add_array
`define inputWeightWidth 5*5*8 //

module mul25_4convolutor(
                input clk,rst,

                input iwVld,imVld,

                input [`inputMapWidth-1:0] im,    //input maps see note above
                input [`inputWeightWidth-1:0] iw, //one channel kennel weights 5*5*8
                input [`DATA_SIZE-1:0] ib,        // one channel bia

                output covResVld,
                output[`halfword_width-1:0] num1,num2,num3,num4);     //output pixels of a 2*2 pixels no relu and no cut out. These operations will be done in cutoutPool module

//result valid wire
wire cov1Vld, cov2Vld, cov3Vld, cov4Vld;
assign covResVld = cov1Vld & cov2Vld & cov3Vld & cov4Vld;

//****************Convalote input maps********************//
conv_mul_add_array u_mul_add_array1(
.clk(clk), .rst(rst),

.iwVld(iwVld), .imVld(imVld),
.weights(iw), .bia(ib),
.imaps({
im[`inputMapWidth-1 - 0*48:`inputMapWidth - 0*48 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 1*48:`inputMapWidth - 1*48 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 2*48:`inputMapWidth - 2*48 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 3*48:`inputMapWidth - 3*48 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 4*48:`inputMapWidth - 4*48 - `KENNEL_SIZE * 8]}),
.covSum(num1),
.covResVld(cov1Vld)
);

conv_mul_add_array u_mul_add_array2(
.clk(clk), .rst(rst),

.iwVld(iwVld), .imVld(imVld),    
.weights(iw), .bia(ib),
.imaps({
im[`inputMapWidth-1 - 0*48 - 8:`inputMapWidth - 0*48 - 8 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 1*48 - 8:`inputMapWidth - 1*48 - 8 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 2*48 - 8:`inputMapWidth - 2*48 - 8 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 3*48 - 8:`inputMapWidth - 3*48 - 8 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 4*48 - 8:`inputMapWidth - 4*48 - 8 - `KENNEL_SIZE * 8]}),
.covSum(num2),
.covResVld(cov2Vld)
);

conv_mul_add_array u_mul_add_array3(
.clk(clk), .rst(rst),

.iwVld(iwVld), .imVld(imVld),   
.weights(iw), .bia(ib),
.imaps({
im[`inputMapWidth-1 - 1*48:`inputMapWidth - 1*48 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 2*48:`inputMapWidth - 2*48 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 3*48:`inputMapWidth - 3*48 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 4*48:`inputMapWidth - 4*48 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 5*48:`inputMapWidth - 5*48 - `KENNEL_SIZE * 8]}),
.covSum(num3),
.covResVld(cov3Vld)
);

conv_mul_add_array u_mul_add_array4(
.clk(clk), .rst(rst),

.iwVld(iwVld), .imVld(imVld),       
.weights(iw), .bia(ib),
.imaps({
im[`inputMapWidth-1 - 1*48 - 8:`inputMapWidth - 1*48 - 8 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 2*48 - 8:`inputMapWidth - 2*48 - 8 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 3*48 - 8:`inputMapWidth - 3*48 - 8 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 4*48 - 8:`inputMapWidth - 4*48 - 8 - `KENNEL_SIZE * 8],
im[`inputMapWidth-1 - 5*48 - 8:`inputMapWidth - 5*48 - 8 - `KENNEL_SIZE * 8]}),
.covSum(num4),
.covResVld(cov4Vld)
);



endmodule


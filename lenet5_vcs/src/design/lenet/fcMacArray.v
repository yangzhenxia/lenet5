`timescale 1ns/1ps
`define DATA_SIZE 8
`define KENNEL_SIZE 5
`define halfword_width 16
//NOTE:50 weights * 50 feature points
// ATTENTION : piping design note
    //every level will be inserted a reg

`define index 399
`define DATA_SIZE 8

// four peArray_1 modules will be 1 pearray
module fcMacArray(
                    input clk,
                    input rst,

                    input [399:0] weights, //输入200bit带宽，一次输5*5*8bit数据
                    input [`DATA_SIZE - 1:0] bia,
                    input iwVld,

                    input [399:0] imaps,

                    output reg [`DATA_SIZE - 1:0] covSum,
                    output reg covResVld
                    );


// internal param
reg [2:0] cycle;


//mul level wire
wire [`halfword_width-1:0]
omul00, omul01, omul02, omul03, omul04, omul05, omul06, omul07, omul08, omul09,
omul10, omul11, omul12, omul13, omul14, omul15, omul16, omul17, omul18, omul19,
omul20, omul21, omul22, omul23, omul24, omul25, omul26, omul27, omul28, omul29,
omul30, omul31, omul32, omul33, omul34, omul35, omul36, omul37, omul38, omul39,
omul40, omul41, omul42, omul43, omul44, omul45, omul46, omul47, omul48, omul49;
reg [`halfword_width-1:0]
regomul00, regomul01, regomul02, regomul03, regomul04, regomul05, regomul06, regomul07, regomul08, regomul09,
regomul10, regomul11, regomul12, regomul13, regomul14, regomul15, regomul16, regomul17, regomul18, regomul19,
regomul20, regomul21, regomul22, regomul23, regomul24, regomul25, regomul26, regomul27, regomul28, regomul29,
regomul30, regomul31, regomul32, regomul33, regomul34, regomul35, regomul36, regomul37, regomul38, regomul39,
regomul40, regomul41, regomul42, regomul43, regomul44, regomul45, regomul46, regomul47, regomul48, regomul49;

//adders level 1
wire [`halfword_width-1:0]
add100, add101, add102, add103, add104, add105, add106, add107, add108, add109,
add110, add111, add112, add113, add114, add115, add116, add117, add118, add119,
add120, add121, add122, add123, add124, add125;
reg [`halfword_width-1:0]
regadd100, regadd101, regadd102, regadd103, regadd104, regadd105, regadd106, regadd107, regadd108, regadd109,
regadd110, regadd111, regadd112, regadd113, regadd114, regadd115, regadd116, regadd117, regadd118, regadd119,
regadd120, regadd121, regadd122, regadd123, regadd124, regadd125;
//adders level 2;
wire [`halfword_width-1:0]
add200, add201, add202, add203, add204, add205, add206, add207, add208, add209,
add210, add211, add212;
reg [`halfword_width-1:0]
regadd200, regadd201, regadd202, regadd203, regadd204, regadd205, regadd206, regadd207, regadd208, regadd209,
regadd210, regadd211, regadd212;
//adders level 3
wire [`halfword_width-1:0] add300, add301, add302, add303, add304, add305;
reg [`halfword_width-1:0] regadd300, regadd301, regadd302, regadd303, regadd304, regadd305;
//adders level 4
wire [`halfword_width-1:0] add400, add401, add402;
reg [`halfword_width-1:0] regadd400, regadd401, regadd402;
//adders level 5
wire [`halfword_width-1:0] add500, add501;
reg [`halfword_width-1:0] regadd500, regadd501;
//add 6
wire [`halfword_width-1:0] fc_res;
//relu
reg [15:0] regfc_res;
wire [`DATA_SIZE-1:0] fc_res8bit;

assign fc_res8bit = regfc_res[15]? 0: (regfc_res [3])?{regfc_res [15],regfc_res [10:4]+1'b1}:{regfc_res [15],regfc_res [10:4]}; //cut out = round(num/(32*4))
//multi array
multi mul00(.a(weights[`index - 0*8:`index -0*8 -7]), .b(imaps[`index - 0*8:`index -0*8 - 7]), .o(omul00));
multi mul01(.a(weights[`index - 1*8:`index -1*8 -7]), .b(imaps[`index - 1*8:`index -1*8 - 7]), .o(omul01));
multi mul02(.a(weights[`index - 2*8:`index -2*8 -7]), .b(imaps[`index - 2*8:`index -2*8 - 7]), .o(omul02));
multi mul03(.a(weights[`index - 3*8:`index -3*8 -7]), .b(imaps[`index - 3*8:`index -3*8 - 7]), .o(omul03));
multi mul04(.a(weights[`index - 4*8:`index -4*8 -7]), .b(imaps[`index - 4*8:`index -4*8 - 7]), .o(omul04));
multi mul05(.a(weights[`index - 5*8:`index -5*8 -7]), .b(imaps[`index - 5*8:`index -5*8 - 7]), .o(omul05));
multi mul06(.a(weights[`index - 6*8:`index -6*8 -7]), .b(imaps[`index - 6*8:`index -6*8 - 7]), .o(omul06));
multi mul07(.a(weights[`index - 7*8:`index -7*8 -7]), .b(imaps[`index - 7*8:`index -7*8 - 7]), .o(omul07));
multi mul08(.a(weights[`index - 8*8:`index -8*8 -7]), .b(imaps[`index - 8*8:`index -8*8 - 7]), .o(omul08));
multi mul09(.a(weights[`index - 9*8:`index -9*8 -7]), .b(imaps[`index - 9*8:`index -9*8 - 7]), .o(omul09));

multi mul10(.a(weights[`index - 10*8:`index - 10*8 -7]), .b(imaps[`index - 10*8:`index - 10*8 - 7]), .o(omul10));
multi mul11(.a(weights[`index - 11*8:`index - 11*8 -7]), .b(imaps[`index - 11*8:`index - 11*8 - 7]), .o(omul11));
multi mul12(.a(weights[`index - 12*8:`index - 12*8 -7]), .b(imaps[`index - 12*8:`index - 12*8 - 7]), .o(omul12));
multi mul13(.a(weights[`index - 13*8:`index - 13*8 -7]), .b(imaps[`index - 13*8:`index - 13*8 - 7]), .o(omul13));
multi mul14(.a(weights[`index - 14*8:`index - 14*8 -7]), .b(imaps[`index - 14*8:`index - 14*8 - 7]), .o(omul14));
multi mul15(.a(weights[`index - 15*8:`index - 15*8 -7]), .b(imaps[`index - 15*8:`index - 15*8 - 7]), .o(omul15));
multi mul16(.a(weights[`index - 16*8:`index - 16*8 -7]), .b(imaps[`index - 16*8:`index - 16*8 - 7]), .o(omul16));
multi mul17(.a(weights[`index - 17*8:`index - 17*8 -7]), .b(imaps[`index - 17*8:`index - 17*8 - 7]), .o(omul17));
multi mul18(.a(weights[`index - 18*8:`index - 18*8 -7]), .b(imaps[`index - 18*8:`index - 18*8 - 7]), .o(omul18));
multi mul19(.a(weights[`index - 19*8:`index - 19*8 -7]), .b(imaps[`index - 19*8:`index - 19*8 - 7]), .o(omul19));

multi mul20(.a(weights[`index - 20*8:`index - 20*8 -7]), .b(imaps[`index - 20*8:`index - 20*8 - 7]), .o(omul20));
multi mul21(.a(weights[`index - 21*8:`index - 21*8 -7]), .b(imaps[`index - 21*8:`index - 21*8 - 7]), .o(omul21));
multi mul22(.a(weights[`index - 22*8:`index - 22*8 -7]), .b(imaps[`index - 22*8:`index - 22*8 - 7]), .o(omul22));
multi mul23(.a(weights[`index - 23*8:`index - 23*8 -7]), .b(imaps[`index - 23*8:`index - 23*8 - 7]), .o(omul23));
multi mul24(.a(weights[`index - 24*8:`index - 24*8 -7]), .b(imaps[`index - 24*8:`index - 24*8 - 7]), .o(omul24));
multi mul25(.a(weights[`index - 25*8:`index - 25*8 -7]), .b(imaps[`index - 25*8:`index - 25*8 - 7]), .o(omul25));
multi mul26(.a(weights[`index - 26*8:`index - 26*8 -7]), .b(imaps[`index - 26*8:`index - 26*8 - 7]), .o(omul26));
multi mul27(.a(weights[`index - 27*8:`index - 27*8 -7]), .b(imaps[`index - 27*8:`index - 27*8 - 7]), .o(omul27));
multi mul28(.a(weights[`index - 28*8:`index - 28*8 -7]), .b(imaps[`index - 28*8:`index - 28*8 - 7]), .o(omul28));
multi mul29(.a(weights[`index - 29*8:`index - 29*8 -7]), .b(imaps[`index - 29*8:`index - 29*8 - 7]), .o(omul29));

multi mul30(.a(weights[`index - 30*8:`index - 30*8 -7]), .b(imaps[`index - 30*8:`index - 30*8 - 7]), .o(omul30));
multi mul31(.a(weights[`index - 31*8:`index - 31*8 -7]), .b(imaps[`index - 31*8:`index - 31*8 - 7]), .o(omul31));
multi mul32(.a(weights[`index - 32*8:`index - 32*8 -7]), .b(imaps[`index - 32*8:`index - 32*8 - 7]), .o(omul32));
multi mul33(.a(weights[`index - 33*8:`index - 33*8 -7]), .b(imaps[`index - 33*8:`index - 33*8 - 7]), .o(omul33));
multi mul34(.a(weights[`index - 34*8:`index - 34*8 -7]), .b(imaps[`index - 34*8:`index - 34*8 - 7]), .o(omul34));
multi mul35(.a(weights[`index - 35*8:`index - 35*8 -7]), .b(imaps[`index - 35*8:`index - 35*8 - 7]), .o(omul35));
multi mul36(.a(weights[`index - 36*8:`index - 36*8 -7]), .b(imaps[`index - 36*8:`index - 36*8 - 7]), .o(omul36));
multi mul37(.a(weights[`index - 37*8:`index - 37*8 -7]), .b(imaps[`index - 37*8:`index - 37*8 - 7]), .o(omul37));
multi mul38(.a(weights[`index - 38*8:`index - 38*8 -7]), .b(imaps[`index - 38*8:`index - 38*8 - 7]), .o(omul38));
multi mul39(.a(weights[`index - 39*8:`index - 39*8 -7]), .b(imaps[`index - 39*8:`index - 39*8 - 7]), .o(omul39));

multi mul40(.a(weights[`index - 40*8:`index - 40*8 -7]), .b(imaps[`index - 40*8:`index - 40*8 - 7]), .o(omul40));
multi mul41(.a(weights[`index - 41*8:`index - 41*8 -7]), .b(imaps[`index - 41*8:`index - 41*8 - 7]), .o(omul41));
multi mul42(.a(weights[`index - 42*8:`index - 42*8 -7]), .b(imaps[`index - 42*8:`index - 42*8 - 7]), .o(omul42));
multi mul43(.a(weights[`index - 43*8:`index - 43*8 -7]), .b(imaps[`index - 43*8:`index - 43*8 - 7]), .o(omul43));
multi mul44(.a(weights[`index - 44*8:`index - 44*8 -7]), .b(imaps[`index - 44*8:`index - 44*8 - 7]), .o(omul44));
multi mul45(.a(weights[`index - 45*8:`index - 45*8 -7]), .b(imaps[`index - 45*8:`index - 45*8 - 7]), .o(omul45));
multi mul46(.a(weights[`index - 46*8:`index - 46*8 -7]), .b(imaps[`index - 46*8:`index - 46*8 - 7]), .o(omul46));
multi mul47(.a(weights[`index - 47*8:`index - 47*8 -7]), .b(imaps[`index - 47*8:`index - 47*8 - 7]), .o(omul47));
multi mul48(.a(weights[`index - 48*8:`index - 48*8 -7]), .b(imaps[`index - 48*8:`index - 48*8 - 7]), .o(omul48));
multi mul49(.a(weights[`index - 49*8:`index - 49*8 -7]), .b(imaps[`index - 49*8:`index - 49*8 - 7]), .o(omul49));

//adders level 1
add u_add100(.a(regomul00), .b(regomul25), .o(add100));
add u_add101(.a(regomul01), .b(regomul26), .o(add101));
add u_add102(.a(regomul02), .b(regomul27), .o(add102));
add u_add103(.a(regomul03), .b(regomul28), .o(add103));
add u_add104(.a(regomul04), .b(regomul29), .o(add104));
add u_add105(.a(regomul05), .b(regomul30), .o(add105));
add u_add106(.a(regomul06), .b(regomul31), .o(add106));
add u_add107(.a(regomul07), .b(regomul32), .o(add107));
add u_add108(.a(regomul08), .b(regomul33), .o(add108));
add u_add109(.a(regomul09), .b(regomul33), .o(add109));

add u_add110(.a(regomul10), .b(regomul34), .o(add110));
add u_add111(.a(regomul11), .b(regomul35), .o(add111));
add u_add112(.a(regomul12), .b(regomul36), .o(add112));
add u_add113(.a(regomul13), .b(regomul37), .o(add113));
add u_add114(.a(regomul14), .b(regomul38), .o(add114));
add u_add115(.a(regomul15), .b(regomul39), .o(add115));
add u_add116(.a(regomul16), .b(regomul40), .o(add116));
add u_add117(.a(regomul17), .b(regomul41), .o(add117));
add u_add118(.a(regomul18), .b(regomul42), .o(add118));
add u_add119(.a(regomul19), .b(regomul43), .o(add119));

add u_add120(.a(regomul20), .b(regomul44), .o(add120));
add u_add121(.a(regomul21), .b(regomul45), .o(add121));
add u_add122(.a(regomul22), .b(regomul46), .o(add122));
add u_add123(.a(regomul23), .b(regomul47), .o(add123));
add u_add124(.a(regomul24), .b(regomul48), .o(add124));
add u_add125(.a(regomul25), .b({bia[7],3'b0,bia[6:0],5'b0}), .o(add125));

//adders level2
add u_add200(.a(regadd100), .b(regadd113), .o(add200));
add u_add201(.a(regadd101), .b(regadd114), .o(add201));
add u_add202(.a(regadd102), .b(regadd115), .o(add202));
add u_add203(.a(regadd103), .b(regadd116), .o(add203));
add u_add204(.a(regadd104), .b(regadd117), .o(add204));
add u_add205(.a(regadd105), .b(regadd118), .o(add205));
add u_add206(.a(regadd106), .b(regadd119), .o(add206));
add u_add207(.a(regadd107), .b(regadd120), .o(add207));
add u_add208(.a(regadd108), .b(regadd121), .o(add208));
add u_add209(.a(regadd109), .b(regadd122), .o(add209));

add u_add210(.a(regadd110), .b(regadd123), .o(add210));
add u_add211(.a(regadd111), .b(regadd124), .o(add211));
add u_add212(.a(regadd112), .b(regadd125), .o(add212));

//adders level 3
add u_add300(.a(regadd200), .b(regadd206), .o(add300));
add u_add301(.a(regadd201), .b(regadd207), .o(add301));
add u_add302(.a(regadd202), .b(regadd208), .o(add302));
add u_add303(.a(regadd203), .b(regadd209), .o(add303));
add u_add304(.a(regadd204), .b(regadd210), .o(add304));
add u_add305(.a(regadd205), .b(regadd211), .o(add305));

//adders level 4
add u_add400(.a(regadd300), .b(regadd303), .o(add400));
add u_add401(.a(regadd301), .b(regadd304), .o(add401));
add u_add402(.a(regadd302), .b(regadd305), .o(add402));

//adders level 5
add u_add500(.a(regadd400), .b(regadd402), .o(add500));
add u_add501(.a(regadd401), .b(regadd212), .o(add501));

//adders level 6
add u_add600(.a(regadd500), .b(regadd501), .o(fc_res));

always@(posedge clk or  posedge rst) begin
    if(rst == 1) begin
        cycle <= 0;
        //output
        covSum <= 0; covResVld <= 0;regfc_res <= 0;
        //mul
        regomul00 <= 0 ; regomul01 <= 0 ; regomul02 <= 0 ; regomul03 <= 0 ; regomul04 <= 0 ; regomul05 <= 0 ; regomul06 <= 0 ; regomul07 <= 0 ; regomul08 <= 0 ; regomul09 <= 0 ;
        regomul10 <= 0 ; regomul11 <= 0 ; regomul12 <= 0 ; regomul13 <= 0 ; regomul14 <= 0 ; regomul15 <= 0 ; regomul16 <= 0 ; regomul17 <= 0 ; regomul18 <= 0 ; regomul19 <= 0 ;
        regomul20 <= 0 ; regomul21 <= 0 ; regomul22 <= 0 ; regomul23 <= 0 ; regomul24 <= 0 ; regomul25 <= 0 ; regomul26 <= 0 ; regomul27 <= 0 ; regomul28 <= 0 ; regomul29 <= 0 ;
        regomul30 <= 0 ; regomul31 <= 0 ; regomul32 <= 0 ; regomul33 <= 0 ; regomul34 <= 0 ; regomul35 <= 0 ; regomul36 <= 0 ; regomul37 <= 0 ; regomul38 <= 0 ; regomul39 <= 0 ;
        regomul40 <= 0 ; regomul41 <= 0 ; regomul42 <= 0 ; regomul43 <= 0 ; regomul44 <= 0 ; regomul45 <= 0 ; regomul46 <= 0 ; regomul47 <= 0 ; regomul48 <= 0 ; regomul49 <= 0 ;
        //add1
        regadd100 <= 0 ; regadd101 <= 0 ; regadd102 <= 0 ; regadd103 <= 0 ; regadd104 <= 0 ; regadd105 <= 0 ; regadd106 <= 0 ; regadd107 <= 0 ; regadd108 <= 0 ; regadd109 <= 0 ;
        regadd110 <= 0 ; regadd111 <= 0 ; regadd112 <= 0 ; regadd113 <= 0 ; regadd114 <= 0 ; regadd115 <= 0 ; regadd116 <= 0 ; regadd117 <= 0 ; regadd118 <= 0 ; regadd119 <= 0 ;
        regadd120 <= 0 ; regadd121 <= 0 ; regadd122 <= 0 ; regadd123 <= 0 ; regadd124 <= 0 ; regadd125 <= 0 ;
        //add2
        regadd200 <= 0 ; regadd201 <= 0 ; regadd202 <= 0 ; regadd203 <= 0 ; regadd204 <= 0 ; regadd205 <= 0 ; regadd206 <= 0 ; regadd207 <= 0 ; regadd208 <= 0 ; regadd209 <= 0 ;
        regadd210 <= 0 ; regadd211 <= 0 ; regadd212 <= 0 ;
        //add3
        regadd300 <= 0 ; regadd301 <= 0 ; regadd302 <= 0 ; regadd303 <= 0 ; regadd304 <= 0 ; regadd305 <= 0 ;
        //add4
        regadd400 <= 0 ; regadd401 <= 0 ; regadd402 <= 0 ;
        //add5
        regadd500 <= 0 ; regadd501 <= 0 ;
    end
    else if(iwVld == 1) begin
        if(cycle == 0) begin 
            cycle <= cycle + 1;
            regomul00 <= omul00 ; regomul01 <= omul01 ; regomul02 <= omul02 ; regomul03 <= omul03 ; regomul04 <= omul04 ; regomul05 <= omul05 ; regomul06 <= omul06 ; regomul07 <= omul07 ; regomul08 <= omul08 ; regomul09 <= omul09 ;
            regomul10 <= omul10 ; regomul11 <= omul11 ; regomul12 <= omul12 ; regomul13 <= omul13 ; regomul14 <= omul14 ; regomul15 <= omul15 ; regomul16 <= omul16 ; regomul17 <= omul17 ; regomul18 <= omul18 ; regomul19 <= omul19 ;
            regomul20 <= omul20 ; regomul21 <= omul21 ; regomul22 <= omul22 ; regomul23 <= omul23 ; regomul24 <= omul24 ; regomul25 <= omul25 ; regomul26 <= omul26 ; regomul27 <= omul27 ; regomul28 <= omul28 ; regomul29 <= omul29 ;
            regomul30 <= omul30 ; regomul31 <= omul31 ; regomul32 <= omul32 ; regomul33 <= omul33 ; regomul34 <= omul34 ; regomul35 <= omul35 ; regomul36 <= omul36 ; regomul37 <= omul37 ; regomul38 <= omul38 ; regomul39 <= omul39 ;
            regomul40 <= omul40 ; regomul41 <= omul41 ; regomul42 <= omul42 ; regomul43 <= omul43 ; regomul44 <= omul44 ; regomul45 <= omul45 ; regomul46 <= omul46 ; regomul47 <= omul47 ; regomul48 <= omul48 ; regomul49 <= omul49 ;
        end
        else if(cycle == 1) begin
            cycle <= cycle + 1;
            regadd100 <= add100 ; regadd101 <= add101 ; regadd102 <= add102 ; regadd103 <= add103 ; regadd104 <= add104 ; regadd105 <= add105 ; regadd106 <= add106 ; regadd107 <= add107 ; regadd108 <= add108 ; regadd109 <= add109 ;
            regadd110 <= add110 ; regadd111 <= add111 ; regadd112 <= add112 ; regadd113 <= add113 ; regadd114 <= add114 ; regadd115 <= add115 ; regadd116 <= add116 ; regadd117 <= add117 ; regadd118 <= add118 ; regadd119 <= add119 ;
            regadd120 <= add120 ; regadd121 <= add121 ; regadd122 <= add122 ; regadd123 <= add123 ; regadd124 <= add124 ; regadd125 <= add125 ;
        end
        else if(cycle == 2) begin
            cycle <= cycle + 1;
            regadd200 <= add200 ; regadd201 <= add201 ; regadd202 <= add202 ; 
            regadd203 <= add203 ; regadd204 <= add204 ; regadd205 <= add205 ; 
            regadd206 <= add206 ; regadd207 <= add207 ; regadd208 <= add208 ; 
            regadd209 <= add209 ; regadd210 <= add210 ; regadd211 <= add211 ; regadd212 <= add212 ;
        end
        else if(cycle == 3) begin
            cycle <= cycle + 1;
            regadd300 <= add300 ; regadd301 <= add301 ; 
            regadd302 <= add302 ; regadd303 <= add303 ; 
            regadd304 <= add304 ; regadd305 <= add305 ;
        end
        else if(cycle == 4) begin
            cycle <= cycle + 1;
            regadd400 <= add400 ; regadd401 <= add401 ; regadd402 <= add402 ;
        end
        else if(cycle == 5) begin
            cycle <= cycle + 1;
            regadd500 <= add500 ; regadd501 <= add501 ;
        end
        else if(cycle == 6) begin
            regfc_res <=  fc_res;
            cycle <= cycle + 1;
        end
        else if(cycle == 7) begin
            covResVld <= 1;
            covSum <= fc_res8bit;
        end
            
    end
    else begin
        cycle <= 0;
        //output
        covSum <= 0; covResVld <= 0;
        //mul
        regomul00 <= 0 ; regomul01 <= 0 ; regomul02 <= 0 ; regomul03 <= 0 ; regomul04 <= 0 ; regomul05 <= 0 ; regomul06 <= 0 ; regomul07 <= 0 ; regomul08 <= 0 ; regomul09 <= 0 ;
        regomul10 <= 0 ; regomul11 <= 0 ; regomul12 <= 0 ; regomul13 <= 0 ; regomul14 <= 0 ; regomul15 <= 0 ; regomul16 <= 0 ; regomul17 <= 0 ; regomul18 <= 0 ; regomul19 <= 0 ;
        regomul20 <= 0 ; regomul21 <= 0 ; regomul22 <= 0 ; regomul23 <= 0 ; regomul24 <= 0 ; regomul25 <= 0 ; regomul26 <= 0 ; regomul27 <= 0 ; regomul28 <= 0 ; regomul29 <= 0 ;
        regomul30 <= 0 ; regomul31 <= 0 ; regomul32 <= 0 ; regomul33 <= 0 ; regomul34 <= 0 ; regomul35 <= 0 ; regomul36 <= 0 ; regomul37 <= 0 ; regomul38 <= 0 ; regomul39 <= 0 ;
        regomul40 <= 0 ; regomul41 <= 0 ; regomul42 <= 0 ; regomul43 <= 0 ; regomul44 <= 0 ; regomul45 <= 0 ; regomul46 <= 0 ; regomul47 <= 0 ; regomul48 <= 0 ; regomul49 <= 0 ;
        //add1
        regadd100 <= 0 ; regadd101 <= 0 ; regadd102 <= 0 ; regadd103 <= 0 ; regadd104 <= 0 ; regadd105 <= 0 ; regadd106 <= 0 ; regadd107 <= 0 ; regadd108 <= 0 ; regadd109 <= 0 ;
        regadd110 <= 0 ; regadd111 <= 0 ; regadd112 <= 0 ; regadd113 <= 0 ; regadd114 <= 0 ; regadd115 <= 0 ; regadd116 <= 0 ; regadd117 <= 0 ; regadd118 <= 0 ; regadd119 <= 0 ;
        regadd120 <= 0 ; regadd121 <= 0 ; regadd122 <= 0 ; regadd123 <= 0 ; regadd124 <= 0 ; regadd125 <= 0 ;
        //add2
        regadd200 <= 0 ; regadd201 <= 0 ; regadd202 <= 0 ; regadd203 <= 0 ; regadd204 <= 0 ; regadd205 <= 0 ; regadd206 <= 0 ; regadd207 <= 0 ; regadd208 <= 0 ; regadd209 <= 0 ;
        regadd210 <= 0 ; regadd211 <= 0 ; regadd212 <= 0 ;
        //add3
        regadd300 <= 0 ; regadd301 <= 0 ; regadd302 <= 0 ; regadd303 <= 0 ; regadd304 <= 0 ; regadd305 <= 0 ;
        //add4
        regadd400 <= 0 ; regadd401 <= 0 ; regadd402 <= 0 ;
        //add5
        regadd500 <= 0 ; regadd501 <= 0 ;
    end
end
endmodule

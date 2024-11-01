`timescale 1ps/1ps
//NOTE:this module will relu and pool a 8x8 map to 1x1map

`define halfword 16 
module reluPoolcov2(
                    input clk,rst,
                    input dataVld,
                    input [`halfword-1:0] 
                    num00,num01,num02,num03,num04,num05,num06,num07,num08,num09,
                    num10,num11,num12,num13,num14,num15,num16,num17,num18,num19,
                    num20,num21,num22,num23,num24,num25,num26,num27,num28,num29,
                    num30,num31,num32,num33,num34,num35,num36,num37,num38,num39,
                    num40,num41,num42,num43,num44,num45,num46,num47,num48,num49,
                    num50,num51,num52,num53,num54,num55,num56,num57,num58,num59,
                    num60,num61,num62,num63,

                    output reg reluPoolResVld,
                    output reg [7:0] reluPoolRes
);
//internal param
reg [2:0] cycle;

//***************reLu****************//
wire [7:0]
num00Relu, num01Relu, num02Relu, num03Relu, num04Relu, num05Relu, num06Relu, num07Relu, num08Relu, num09Relu, 
num10Relu, num11Relu, num12Relu, num13Relu, num14Relu, num15Relu, num16Relu, num17Relu, num18Relu, num19Relu, 
num20Relu, num21Relu, num22Relu, num23Relu, num24Relu, num25Relu, num26Relu, num27Relu, num28Relu, num29Relu, 
num30Relu, num31Relu, num32Relu, num33Relu, num34Relu, num35Relu, num36Relu, num37Relu, num38Relu, num39Relu, 
num40Relu, num41Relu, num42Relu, num43Relu, num44Relu, num45Relu, num46Relu, num47Relu, num48Relu, num49Relu, 
num50Relu, num51Relu, num52Relu, num53Relu, num54Relu, num55Relu, num56Relu, num57Relu, num58Relu, num59Relu, 
num60Relu, num61Relu, num62Relu, num63Relu;

assign num00Relu = num00[15]? 0: (num00 [6])?{num00 [15],num00 [13:7]+1'b1}:{num00 [15],num00 [13:7]}; //cut out = round(num/(32*4))
assign num01Relu = num01[15]? 0: (num01 [6])?{num01 [15],num01 [13:7]+1'b1}:{num01 [15],num01 [13:7]};
assign num02Relu = num02[15]? 0: (num02 [6])?{num02 [15],num02 [13:7]+1'b1}:{num02 [15],num02 [13:7]};
assign num03Relu = num03[15]? 0: (num03 [6])?{num03 [15],num03 [13:7]+1'b1}:{num03 [15],num03 [13:7]};
assign num04Relu = num04[15]? 0: (num04 [6])?{num04 [15],num04 [13:7]+1'b1}:{num04 [15],num04 [13:7]};
assign num05Relu = num05[15]? 0: (num05 [6])?{num05 [15],num05 [13:7]+1'b1}:{num05 [15],num05 [13:7]};
assign num06Relu = num06[15]? 0: (num06 [6])?{num06 [15],num06 [13:7]+1'b1}:{num06 [15],num06 [13:7]};
assign num07Relu = num07[15]? 0: (num07 [6])?{num07 [15],num07 [13:7]+1'b1}:{num07 [15],num07 [13:7]};
assign num08Relu = num08[15]? 0: (num08 [6])?{num08 [15],num08 [13:7]+1'b1}:{num08 [15],num08 [13:7]};
assign num09Relu = num09[15]? 0: (num09 [6])?{num09 [15],num09 [13:7]+1'b1}:{num09 [15],num09 [13:7]};
assign num10Relu = num10[15]? 0: (num10 [6])?{num10 [15],num10 [13:7]+1'b1}:{num10 [15],num10 [13:7]}; //cut out = round(num/(32*4))
assign num11Relu = num11[15]? 0: (num11 [6])?{num11 [15],num11 [13:7]+1'b1}:{num11 [15],num11 [13:7]};
assign num12Relu = num12[15]? 0: (num12 [6])?{num12 [15],num12 [13:7]+1'b1}:{num12 [15],num12 [13:7]};
assign num13Relu = num13[15]? 0: (num13 [6])?{num13 [15],num13 [13:7]+1'b1}:{num13 [15],num13 [13:7]};
assign num14Relu = num14[15]? 0: (num14 [6])?{num14 [15],num14 [13:7]+1'b1}:{num14 [15],num14 [13:7]};
assign num15Relu = num15[15]? 0: (num15 [6])?{num15 [15],num15 [13:7]+1'b1}:{num15 [15],num15 [13:7]};
assign num16Relu = num16[15]? 0: (num16 [6])?{num16 [15],num16 [13:7]+1'b1}:{num16 [15],num16 [13:7]};
assign num17Relu = num17[15]? 0: (num17 [6])?{num17 [15],num17 [13:7]+1'b1}:{num17 [15],num17 [13:7]};
assign num18Relu = num18[15]? 0: (num18 [6])?{num18 [15],num18 [13:7]+1'b1}:{num18 [15],num18 [13:7]};
assign num19Relu = num19[15]? 0: (num19 [6])?{num19 [15],num19 [13:7]+1'b1}:{num19 [15],num19 [13:7]};
assign num20Relu = num20[15]? 0: (num20 [6])?{num20 [15],num20 [13:7]+1'b1}:{num20 [15],num20 [13:7]}; //cut out = round(num/(32*4))
assign num21Relu = num21[15]? 0: (num21 [6])?{num21 [15],num21 [13:7]+1'b1}:{num21 [15],num21 [13:7]};
assign num22Relu = num22[15]? 0: (num22 [6])?{num22 [15],num22 [13:7]+1'b1}:{num22 [15],num22 [13:7]};
assign num23Relu = num23[15]? 0: (num23 [6])?{num23 [15],num23 [13:7]+1'b1}:{num23 [15],num23 [13:7]};
assign num24Relu = num24[15]? 0: (num24 [6])?{num24 [15],num24 [13:7]+1'b1}:{num24 [15],num24 [13:7]};
assign num25Relu = num25[15]? 0: (num25 [6])?{num25 [15],num25 [13:7]+1'b1}:{num25 [15],num25 [13:7]};
assign num26Relu = num26[15]? 0: (num26 [6])?{num26 [15],num26 [13:7]+1'b1}:{num26 [15],num26 [13:7]};
assign num27Relu = num27[15]? 0: (num27 [6])?{num27 [15],num27 [13:7]+1'b1}:{num27 [15],num27 [13:7]};
assign num28Relu = num28[15]? 0: (num28 [6])?{num28 [15],num28 [13:7]+1'b1}:{num28 [15],num28 [13:7]};
assign num29Relu = num29[15]? 0: (num29 [6])?{num29 [15],num29 [13:7]+1'b1}:{num29 [15],num29 [13:7]};
assign num30Relu = num30[15]? 0: (num30 [6])?{num30 [15],num30 [13:7]+1'b1}:{num30 [15],num30 [13:7]}; //cut out = round(num/(32*4))
assign num31Relu = num31[15]? 0: (num31 [6])?{num31 [15],num31 [13:7]+1'b1}:{num31 [15],num31 [13:7]};
assign num32Relu = num32[15]? 0: (num32 [6])?{num32 [15],num32 [13:7]+1'b1}:{num32 [15],num32 [13:7]};
assign num33Relu = num33[15]? 0: (num33 [6])?{num33 [15],num33 [13:7]+1'b1}:{num33 [15],num33 [13:7]};
assign num34Relu = num34[15]? 0: (num34 [6])?{num34 [15],num34 [13:7]+1'b1}:{num34 [15],num34 [13:7]};
assign num35Relu = num35[15]? 0: (num35 [6])?{num35 [15],num35 [13:7]+1'b1}:{num35 [15],num35 [13:7]};
assign num36Relu = num36[15]? 0: (num36 [6])?{num36 [15],num36 [13:7]+1'b1}:{num36 [15],num36 [13:7]};
assign num37Relu = num37[15]? 0: (num37 [6])?{num37 [15],num37 [13:7]+1'b1}:{num37 [15],num37 [13:7]};
assign num38Relu = num38[15]? 0: (num38 [6])?{num38 [15],num38 [13:7]+1'b1}:{num38 [15],num38 [13:7]};
assign num39Relu = num39[15]? 0: (num39 [6])?{num39 [15],num39 [13:7]+1'b1}:{num39 [15],num39 [13:7]};
assign num40Relu = num40[15]? 0: (num40 [6])?{num40 [15],num40 [13:7]+1'b1}:{num40 [15],num40 [13:7]}; //cut out = round(num/(32*4))
assign num41Relu = num41[15]? 0: (num41 [6])?{num41 [15],num41 [13:7]+1'b1}:{num41 [15],num41 [13:7]};
assign num42Relu = num42[15]? 0: (num42 [6])?{num42 [15],num42 [13:7]+1'b1}:{num42 [15],num42 [13:7]};
assign num43Relu = num43[15]? 0: (num43 [6])?{num43 [15],num43 [13:7]+1'b1}:{num43 [15],num43 [13:7]};
assign num44Relu = num44[15]? 0: (num44 [6])?{num44 [15],num44 [13:7]+1'b1}:{num44 [15],num44 [13:7]};
assign num45Relu = num45[15]? 0: (num45 [6])?{num45 [15],num45 [13:7]+1'b1}:{num45 [15],num45 [13:7]};
assign num46Relu = num46[15]? 0: (num46 [6])?{num46 [15],num46 [13:7]+1'b1}:{num46 [15],num46 [13:7]};
assign num47Relu = num47[15]? 0: (num47 [6])?{num47 [15],num47 [13:7]+1'b1}:{num47 [15],num47 [13:7]};
assign num48Relu = num48[15]? 0: (num48 [6])?{num48 [15],num48 [13:7]+1'b1}:{num48 [15],num48 [13:7]};
assign num49Relu = num49[15]? 0: (num49 [6])?{num49 [15],num49 [13:7]+1'b1}:{num49 [15],num49 [13:7]};
assign num50Relu = num50[15]? 0: (num50 [6])?{num50 [15],num50 [13:7]+1'b1}:{num50 [15],num50 [13:7]}; //cut out = round(num/(32*4))
assign num51Relu = num51[15]? 0: (num51 [6])?{num51 [15],num51 [13:7]+1'b1}:{num51 [15],num51 [13:7]};
assign num52Relu = num52[15]? 0: (num52 [6])?{num52 [15],num52 [13:7]+1'b1}:{num52 [15],num52 [13:7]};
assign num53Relu = num53[15]? 0: (num53 [6])?{num53 [15],num53 [13:7]+1'b1}:{num53 [15],num53 [13:7]};
assign num54Relu = num54[15]? 0: (num54 [6])?{num54 [15],num54 [13:7]+1'b1}:{num54 [15],num54 [13:7]};
assign num55Relu = num55[15]? 0: (num55 [6])?{num55 [15],num55 [13:7]+1'b1}:{num55 [15],num55 [13:7]};
assign num56Relu = num56[15]? 0: (num56 [6])?{num56 [15],num56 [13:7]+1'b1}:{num56 [15],num56 [13:7]};
assign num57Relu = num57[15]? 0: (num57 [6])?{num57 [15],num57 [13:7]+1'b1}:{num57 [15],num57 [13:7]};
assign num58Relu = num58[15]? 0: (num58 [6])?{num58 [15],num58 [13:7]+1'b1}:{num58 [15],num58 [13:7]};
assign num59Relu = num59[15]? 0: (num59 [6])?{num59 [15],num59 [13:7]+1'b1}:{num59 [15],num59 [13:7]};
assign num60Relu = num60[15]? 0: (num60 [6])?{num60 [15],num60 [13:7]+1'b1}:{num60 [15],num60 [13:7]}; //cut out = round(num/(32*4))
assign num61Relu = num61[15]? 0: (num61 [6])?{num61 [15],num61 [13:7]+1'b1}:{num61 [15],num61 [13:7]};
assign num62Relu = num62[15]? 0: (num62 [6])?{num62 [15],num62 [13:7]+1'b1}:{num62 [15],num62 [13:7]};
assign num63Relu = num63[15]? 0: (num63 [6])?{num63 [15],num63 [13:7]+1'b1}:{num63 [15],num63 [13:7]};

//******************comparator************//
//level 1
reg [7:0] 
max100, max101, max102, max103, max104, max105, max106, max107, max108, max109, 
max110, max111, max112, max113, max114, max115, max116, max117, max118, max119, 
max120, max121, max122, max123, max124, max125, max126, max127, max128, max129, 
max130, max131;
//level 2
reg [7:0]
max200, max201, max202, max203, max204, max205, max206, max207, max208, max209, 
max210, max211, max212, max213, max214, max215;
//level 3
reg [7:0]
max300, max301, max302, max303, max304, max305, max306, max307;
//level 4
reg [7:0]
max400, max401, max402, max403;
//level 5
reg [7:0]
max500, max501;




always @(posedge clk or posedge rst) begin
    if(rst == 1) begin
        cycle <= 0;
        //level 1
        max100 <= 0; max101 <= 0; max102 <= 0; max103 <= 0; max104 <= 0; max105 <= 0; max106 <= 0; max107 <= 0; max108 <= 0; max109 <= 0; 
        max110 <= 0; max111 <= 0; max112 <= 0; max113 <= 0; max114 <= 0; max115 <= 0; max116 <= 0; max117 <= 0; max118 <= 0; max119 <= 0; 
        max120 <= 0; max121 <= 0; max122 <= 0; max123 <= 0; max124 <= 0; max125 <= 0; max126 <= 0; max127 <= 0; max128 <= 0; max129 <= 0; 
        max130 <= 0; max131 <= 0;
        //level 2
        max200 <= 0 ; max201 <= 0 ; max202 <= 0 ; max203 <= 0 ; max204 <= 0 ; max205 <= 0 ; max206 <= 0 ; max207 <= 0 ; max208 <= 0 ; max209 <= 0 ; 
        max210 <= 0 ; max211 <= 0 ; max212 <= 0 ; max213 <= 0 ; max214 <= 0 ; max215 <= 0 ;
        //level 3
        max300 <= 0 ; max301 <= 0 ; max302 <= 0 ; max303 <= 0 ; max304 <= 0 ; max305 <= 0 ; max306 <= 0 ; max307 <= 0 ;
        //level 4
        max400 <= 0 ; max401 <= 0 ; max402 <= 0 ; max403 <= 0 ;
        //level 5
        max500 <= 0 ; max501 <= 0 ; 
        //out put
        reluPoolResVld <= 0;
        reluPoolRes <= 0;
    end
    else begin
        if(dataVld == 1)begin
            if(cycle == 0) begin
                max100 <= (num00Relu > num10Relu)?num00Relu : num10Relu; 
                max101 <= (num01Relu > num11Relu)?num01Relu : num11Relu; 
                max102 <= (num02Relu > num12Relu)?num02Relu : num12Relu; 
                max103 <= (num03Relu > num13Relu)?num03Relu : num13Relu; 
                max104 <= (num04Relu > num14Relu)?num04Relu : num14Relu; 
                max105 <= (num05Relu > num15Relu)?num05Relu : num15Relu; 
                max106 <= (num06Relu > num16Relu)?num06Relu : num16Relu; 
                max107 <= (num07Relu > num17Relu)?num07Relu : num17Relu; 
                max108 <= (num08Relu > num18Relu)?num08Relu : num18Relu; 
                max109 <= (num09Relu > num19Relu)?num09Relu : num19Relu; 
                max110 <= (num20Relu > num30Relu)?num20Relu : num30Relu; 
                max111 <= (num21Relu > num31Relu)?num21Relu : num31Relu; 
                max112 <= (num22Relu > num32Relu)?num22Relu : num32Relu; 
                max113 <= (num23Relu > num33Relu)?num23Relu : num33Relu; 
                max114 <= (num24Relu > num34Relu)?num24Relu : num34Relu; 
                max115 <= (num25Relu > num35Relu)?num25Relu : num35Relu; 
                max116 <= (num26Relu > num36Relu)?num26Relu : num36Relu; 
                max117 <= (num27Relu > num37Relu)?num27Relu : num37Relu; 
                max118 <= (num28Relu > num38Relu)?num28Relu : num38Relu; 
                max119 <= (num29Relu > num39Relu)?num29Relu : num39Relu; 
                max120 <= (num40Relu > num50Relu)?num40Relu : num50Relu; 
                max121 <= (num41Relu > num51Relu)?num41Relu : num51Relu; 
                max122 <= (num42Relu > num52Relu)?num42Relu : num52Relu; 
                max123 <= (num43Relu > num53Relu)?num43Relu : num53Relu; 
                max124 <= (num44Relu > num54Relu)?num44Relu : num54Relu; 
                max125 <= (num45Relu > num55Relu)?num45Relu : num55Relu; 
                max126 <= (num46Relu > num56Relu)?num46Relu : num56Relu; 
                max127 <= (num47Relu > num57Relu)?num47Relu : num57Relu; 
                max128 <= (num48Relu > num58Relu)?num48Relu : num58Relu; 
                max129 <= (num49Relu > num59Relu)?num49Relu : num59Relu; 
                max130 <= (num60Relu > num62Relu)?num60Relu : num62Relu; 
                max131 <= (num61Relu > num63Relu)?num61Relu : num63Relu;

                cycle <= cycle + 1;
            end 
            else if(cycle == 1) begin
                max200 <= (max100 > max116)?max100 : max116; 
                max201 <= (max101 > max117)?max101 : max117; 
                max202 <= (max102 > max118)?max102 : max118; 
                max203 <= (max103 > max119)?max103 : max119; 
                max204 <= (max104 > max120)?max104 : max120; 
                max205 <= (max105 > max121)?max105 : max121; 
                max206 <= (max106 > max122)?max106 : max122; 
                max207 <= (max107 > max123)?max107 : max123; 
                max208 <= (max108 > max124)?max108 : max124; 
                max209 <= (max109 > max125)?max109 : max125; 
                max210 <= (max110 > max126)?max110 : max126; 
                max211 <= (max111 > max127)?max111 : max127; 
                max212 <= (max112 > max128)?max112 : max128; 
                max213 <= (max113 > max129)?max113 : max129; 
                max214 <= (max114 > max130)?max114 : max130; 
                max215 <= (max115 > max131)?max115 : max131;

                cycle <= cycle + 1;
            end
            else if(cycle == 2) begin
                max300 <= (max200 > max208)?max200 : max208; 
                max301 <= (max201 > max209)?max201 : max209; 
                max302 <= (max202 > max210)?max202 : max210; 
                max303 <= (max203 > max211)?max203 : max211; 
                max304 <= (max204 > max212)?max204 : max212; 
                max305 <= (max205 > max213)?max205 : max213; 
                max306 <= (max206 > max214)?max206 : max214; 
                max307 <= (max207 > max215)?max207 : max215;

                cycle <= cycle + 1;
            end
            else if(cycle == 3) begin
                max400 <= (max300 > max304)?max300 : max304; 
                max401 <= (max301 > max305)?max301 : max305; 
                max402 <= (max302 > max306)?max302 : max306; 
                max403 <= (max303 > max307)?max303 : max307;

                cycle <= cycle + 1;  
            end
            else if(cycle == 4) begin
                max500 <= (max400 > max402)?max400 : max402;
                max501 <= (max401 > max403)?max401 : max403;

                cycle <= cycle + 1;
            end
            else if(cycle == 5) begin
                reluPoolRes <= (max500 > max501)?max500 : max501;
                reluPoolResVld <= 1;
            end
        end
        else begin
            cycle <= 0;
            //level 1
            max100 <= 0; max101 <= 0; max102 <= 0; max103 <= 0; max104 <= 0; max105 <= 0; max106 <= 0; max107 <= 0; max108 <= 0; max109 <= 0; 
            max110 <= 0; max111 <= 0; max112 <= 0; max113 <= 0; max114 <= 0; max115 <= 0; max116 <= 0; max117 <= 0; max118 <= 0; max119 <= 0; 
            max120 <= 0; max121 <= 0; max122 <= 0; max123 <= 0; max124 <= 0; max125 <= 0; max126 <= 0; max127 <= 0; max128 <= 0; max129 <= 0; 
            max130 <= 0; max131 <= 0;
            //level 2
            max200 <= 0 ; max201 <= 0 ; max202 <= 0 ; max203 <= 0 ; max204 <= 0 ; max205 <= 0 ; max206 <= 0 ; max207 <= 0 ; max208 <= 0 ; max209 <= 0 ; 
            max210 <= 0 ; max211 <= 0 ; max212 <= 0 ; max213 <= 0 ; max214 <= 0 ; max215 <= 0 ;
            //level 3
            max300 <= 0 ; max301 <= 0 ; max302 <= 0 ; max303 <= 0 ; max304 <= 0 ; max305 <= 0 ; max306 <= 0 ; max307 <= 0 ;
            //level 4
            max400 <= 0 ; max401 <= 0 ; max402 <= 0 ; max403 <= 0 ;
            //level 5
            max500 <= 0 ; max501 <= 0 ; 
            //out put
            reluPoolResVld <= 0;
            reluPoolRes <= 0;
        end
    end
end
endmodule
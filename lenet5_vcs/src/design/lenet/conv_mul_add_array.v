`timescale 1ns/1ps
`define DATA_SIZE 8
`define KENNEL_SIZE 5
`define halfword_width 16
`define mindex 199

// ATTENTION : piping design note
//every level will be inserted a reg



// four peArray_1 modules will be 1 pearray
module conv_mul_add_array(
                    input clk,
                    input rst,

                     input [`KENNEL_SIZE*`KENNEL_SIZE*`DATA_SIZE - 1:0] weights, //输入200bit带宽，一次输5*5*8bit数据
                     input [`DATA_SIZE - 1:0] bia,
                     input iwVld,

                     input [`KENNEL_SIZE*`KENNEL_SIZE*`DATA_SIZE - 1:0] imaps,
                     input imVld,

                     output reg [`halfword_width - 1:0] covSum,
                     output reg covResVld
                     );
// internal param
reg [2:0] cycle;
reg [`KENNEL_SIZE*`KENNEL_SIZE*`DATA_SIZE - 1:0] imInternal;
reg [`KENNEL_SIZE*`KENNEL_SIZE*`DATA_SIZE - 1:0] iwInternal;
reg [`DATA_SIZE-1:0] ibInternal;



//乘法阵列输出，用于加分树
wire [`halfword_width - 1:0]
Omul1, Omul2, Omul3, Omul4, Omul5, Omul6, Omul7, Omul8, Omul9, Omul10, Omul11, Omul12, Omul13,
Omul14, Omul15, Omul16, Omul17, Omul18, Omul19, Omul20, Omul21, Omul22, Omul23, Omul24, Omul25;
//registers inserted in mul level and adders level 1
reg [`halfword_width - 1:0] 
regOmul1, regOmul2, regOmul3, regOmul4, regOmul5, regOmul6, regOmul7, regOmul8, regOmul9, regOmul10, regOmul11, regOmul12, regOmul13,
regOmul14, regOmul15, regOmul16, regOmul17, regOmul18, regOmul19, regOmul20, regOmul21, regOmul22, regOmul23, regOmul24, regOmul25;

//adders level 1
wire [`halfword_width - 1:0]
addtreeTemp01, addtreeTemp02, addtreeTemp03, addtreeTemp04, addtreeTemp05, addtreeTemp06, addtreeTemp07,
addtreeTemp08, addtreeTemp09, addtreeTemp10, addtreeTemp11, addtreeTemp12, addtreeBia;//level 1
//registers inserted in adders level 1 and level 2
reg [`halfword_width - 1:0]
regaddtreeTemp01, regaddtreeTemp02, regaddtreeTemp03, regaddtreeTemp04, regaddtreeTemp05, regaddtreeTemp06, regaddtreeTemp07,
regaddtreeTemp08, regaddtreeTemp09, regaddtreeTemp10, regaddtreeTemp11, regaddtreeTemp12, regaddtreeBia;

//adders level 2
wire [`halfword_width - 1:0]
addtreeTemp21, addtreeTemp22, addtreeTemp23, addtreeTemp24, addtreeTemp25, addtreeTemp26; //level 2
//insert reg in add tree level 2 and level 3
reg [`halfword_width -1 :0]
regaddtreeTemp21, regaddtreeTemp22, regaddtreeTemp23, regaddtreeTemp24, regaddtreeTemp25, regaddtreeTemp26; 

//adders level 3
wire [`halfword_width - 1:0]
addtreeTemp31, addtreeTemp32, addtreeTemp33; //level 3
reg [`halfword_width - 1:0]
regaddtreeTemp31, regaddtreeTemp32, regaddtreeTemp33;


wire [`halfword_width - 1:0]
addtreeTemp41, addtreeTemp42,
addSum;

//insert reg in lv4 and lv5
reg [`halfword_width - 1:0]
regaddtreeTemp41, regaddtreeTemp42;



//*************************乘法阵列************************//
multi mul1(.a(iwInternal[`mindex - 0*8:`mindex -0*8 -7]), .b(imInternal[`mindex - 0*8:`mindex -0*8 - 7]), .o(Omul1));
multi mul2(.a(iwInternal[`mindex - 1*8:`mindex -1*8 -7]), .b(imInternal[`mindex - 1*8:`mindex -1*8 - 7]), .o(Omul2));
multi mul3(.a(iwInternal[`mindex - 2*8:`mindex -2*8 -7]), .b(imInternal[`mindex - 2*8:`mindex -2*8 - 7]), .o(Omul3));
multi mul4(.a(iwInternal[`mindex - 3*8:`mindex -3*8 -7]), .b(imInternal[`mindex - 3*8:`mindex -3*8 - 7]), .o(Omul4));
multi mul5(.a(iwInternal[`mindex - 4*8:`mindex -4*8 -7]), .b(imInternal[`mindex - 4*8:`mindex -4*8 - 7]), .o(Omul5));
multi mul6(.a(iwInternal[`mindex - 5*8:`mindex -5*8 -7]), .b(imInternal[`mindex - 5*8:`mindex -5*8 - 7]), .o(Omul6));
multi mul7(.a(iwInternal[`mindex - 6*8:`mindex -6*8 -7]), .b(imInternal[`mindex - 6*8:`mindex -6*8 - 7]), .o(Omul7));
multi mul8(.a(iwInternal[`mindex - 7*8:`mindex -7*8 -7]), .b(imInternal[`mindex - 7*8:`mindex -7*8 - 7]), .o(Omul8));
multi mul9(.a(iwInternal[`mindex - 8*8:`mindex -8*8 -7]), .b(imInternal[`mindex - 8*8:`mindex -8*8 - 7]), .o(Omul9));
multi mul0(.a(iwInternal[`mindex - 9*8:`mindex -9*8 -7]), .b(imInternal[`mindex - 9*8:`mindex -9*8 - 7]), .o(Omul10));
multi mul11(.a(iwInternal[`mindex - 10*8:`mindex -10*8 -7]), .b(imInternal[`mindex - 10*8:`mindex -10*8 - 7]), .o(Omul11));
multi mul12(.a(iwInternal[`mindex - 11*8:`mindex -11*8 -7]), .b(imInternal[`mindex - 11*8:`mindex -11*8 - 7]), .o(Omul12));
multi mul13(.a(iwInternal[`mindex - 12*8:`mindex -12*8 -7]), .b(imInternal[`mindex - 12*8:`mindex -12*8 - 7]), .o(Omul13));
multi mul14(.a(iwInternal[`mindex - 13*8:`mindex -13*8 -7]), .b(imInternal[`mindex - 13*8:`mindex -13*8 - 7]), .o(Omul14));
multi mul15(.a(iwInternal[`mindex - 14*8:`mindex -14*8 -7]), .b(imInternal[`mindex - 14*8:`mindex -14*8 - 7]), .o(Omul15));
multi mul16(.a(iwInternal[`mindex - 15*8:`mindex -15*8 -7]), .b(imInternal[`mindex - 15*8:`mindex -15*8 - 7]), .o(Omul16));
multi mul17(.a(iwInternal[`mindex - 16*8:`mindex -16*8 -7]), .b(imInternal[`mindex - 16*8:`mindex -16*8 - 7]), .o(Omul17));
multi mul18(.a(iwInternal[`mindex - 17*8:`mindex -17*8 -7]), .b(imInternal[`mindex - 17*8:`mindex -17*8 - 7]), .o(Omul18));
multi mul19(.a(iwInternal[`mindex - 18*8:`mindex -18*8 -7]), .b(imInternal[`mindex - 18*8:`mindex -18*8 - 7]), .o(Omul19));
multi mul20(.a(iwInternal[`mindex - 19*8:`mindex -19*8 -7]), .b(imInternal[`mindex - 19*8:`mindex -19*8 - 7]), .o(Omul20));
multi mul21(.a(iwInternal[`mindex - 20*8:`mindex -20*8 -7]), .b(imInternal[`mindex - 20*8:`mindex -20*8 - 7]), .o(Omul21));
multi mul22(.a(iwInternal[`mindex - 21*8:`mindex -21*8 -7]), .b(imInternal[`mindex - 21*8:`mindex -21*8 - 7]), .o(Omul22));
multi mul23(.a(iwInternal[`mindex - 22*8:`mindex -22*8 -7]), .b(imInternal[`mindex - 22*8:`mindex -22*8 - 7]), .o(Omul23));
multi mul24(.a(iwInternal[`mindex - 23*8:`mindex -23*8 -7]), .b(imInternal[`mindex - 23*8:`mindex -23*8 - 7]), .o(Omul24));
multi mul25(.a(iwInternal[`mindex - 24*8:`mindex -24*8 -7]), .b(imInternal[`mindex - 24*8:`mindex -24*8 - 7]), .o(Omul25));


//**********************加法************************//
//第一 13次并行加
add add1(.a(regOmul1), .b(regOmul14), .o(addtreeTemp01));
add add2(.a(regOmul2), .b(regOmul15), .o(addtreeTemp02));
add add3(.a(regOmul3), .b(regOmul16), .o(addtreeTemp03));
add add4(.a(regOmul4), .b(regOmul17), .o(addtreeTemp04));
add add5(.a(regOmul5), .b(regOmul18), .o(addtreeTemp05));
add add6(.a(regOmul6), .b(regOmul19), .o(addtreeTemp06));
add add7(.a(regOmul7), .b(regOmul20), .o(addtreeTemp07));
add add8(.a(regOmul8), .b(regOmul21), .o(addtreeTemp08));
add add9(.a(regOmul9), .b(regOmul22), .o(addtreeTemp09));
add add10(.a(regOmul10), .b(regOmul23), .o(addtreeTemp10));
add add11(.a(regOmul11), .b(regOmul24), .o(addtreeTemp11));
add add12(.a(regOmul12), .b(regOmul25), .o(addtreeTemp12));
add add13(.a(regOmul13), .b({ibInternal[7],3'b0,ibInternal[6:0],5'b0}), .o(addtreeBia)); //加入权重
//第二 6次并行加
add add21(.a(regaddtreeTemp01), .b(regaddtreeTemp07), .o(addtreeTemp21));
add add22(.a(regaddtreeTemp02), .b(regaddtreeTemp08), .o(addtreeTemp22));
add add23(.a(regaddtreeTemp03), .b(regaddtreeTemp09), .o(addtreeTemp23));
add add24(.a(regaddtreeTemp04), .b(regaddtreeTemp10), .o(addtreeTemp24));
add add25(.a(regaddtreeTemp05), .b(regaddtreeTemp11), .o(addtreeTemp25));
add add26(.a(regaddtreeTemp06), .b(regaddtreeTemp12), .o(addtreeTemp26));
//第三 3次并行加
add add31(.a(regaddtreeTemp21), .b(regaddtreeTemp24), .o(addtreeTemp31));
add add32(.a(regaddtreeTemp22), .b(regaddtreeTemp25), .o(addtreeTemp32));
add add33(.a(regaddtreeTemp23), .b(regaddtreeTemp26), .o(addtreeTemp33));
//第四 2次并行加
add add41(.a(regaddtreeTemp31), .b(regaddtreeTemp33), .o(addtreeTemp41));
add add42(.a(regaddtreeTemp32), .b(regaddtreeBia), .o(addtreeTemp42));
//第五 1次加，出结果
add add51(.a(regaddtreeTemp41), .b(regaddtreeTemp42), .o(addSum));

always@(posedge clk or posedge rst) begin
    if(rst == 1'b1) begin
        //mul leve
        regOmul1<= 0; regOmul2<= 0; regOmul3<= 0; regOmul4<= 0; regOmul5<= 0; regOmul6<= 0; regOmul7<= 0; regOmul8<= 0; regOmul9<= 0; regOmul10<= 0; regOmul11<= 0; regOmul12<= 0; regOmul13<= 0;
        regOmul14<= 0; regOmul15<= 0; regOmul16<= 0; regOmul17<= 0; regOmul18<= 0; regOmul19<= 0; regOmul20<= 0; regOmul21<= 0; regOmul22<= 0; regOmul23<= 0; regOmul24<= 0; regOmul25 <= 0;
        //adders level 1
        regaddtreeTemp01 <= 0; regaddtreeTemp02 <= 0; regaddtreeTemp03 <= 0; regaddtreeTemp04 <= 0; regaddtreeTemp05 <= 0; regaddtreeTemp06 <= 0; regaddtreeTemp07 <= 0;
        regaddtreeTemp08 <= 0; regaddtreeTemp09 <= 0; regaddtreeTemp10 <= 0; regaddtreeTemp11 <= 0; regaddtreeTemp12 <= 0; regaddtreeBia <= 0;
        //adders level 2
        regaddtreeTemp21<= 0; regaddtreeTemp22<= 0; regaddtreeTemp23<= 0; regaddtreeTemp24<= 0; regaddtreeTemp25<= 0; regaddtreeTemp26<= 0; regaddtreeBia<= 0; 
        //adders level 3
        regaddtreeTemp31 <= 0; regaddtreeTemp32 <= 0; regaddtreeTemp33 <= 0;
        //adders level 4
        regaddtreeTemp41<= 0; regaddtreeTemp42<= 0;
        //internal param
        iwInternal <= 0; ibInternal <= 0;   imInternal <= 0;
        cycle <= 0;
        covResVld <= 0;
        covSum <= 0;
    end
    else begin
        if(imVld&iwVld == 1) begin
            if(cycle == 0) begin
                iwInternal <= weights;
                ibInternal <= bia;
                imInternal <= imaps;

                cycle <= cycle + 1;
            end
            if(cycle == 1) begin
                regOmul1 <= Omul1;
                regOmul2 <= Omul2;
                regOmul3 <= Omul3;
                regOmul4 <= Omul4;
                regOmul5 <= Omul5;
                regOmul6 <= Omul6;
                regOmul7 <= Omul7;
                regOmul8 <= Omul8;
                regOmul9 <= Omul9;
                regOmul10 <= Omul10;
                regOmul11 <= Omul11;
                regOmul12 <= Omul12;
                regOmul13 <= Omul13;
                regOmul14 <= Omul14;
                regOmul15 <= Omul15;
                regOmul16 <= Omul16;
                regOmul17 <= Omul17;
                regOmul18 <= Omul18;
                regOmul19 <= Omul19;
                regOmul20 <= Omul20;
                regOmul21 <= Omul21;
                regOmul22 <= Omul22;
                regOmul23 <= Omul23;
                regOmul24 <= Omul24;
                regOmul25 <= Omul25;

                cycle <= cycle + 1;                
            end
            else if(cycle == 2) begin
                regaddtreeTemp01  <= addtreeTemp01; regaddtreeTemp02  <= addtreeTemp02; regaddtreeTemp03  <= addtreeTemp03; regaddtreeTemp04  <= addtreeTemp04; regaddtreeTemp05  <= addtreeTemp05; regaddtreeTemp06  <= addtreeTemp06; regaddtreeTemp07  <= addtreeTemp07;
                regaddtreeTemp08  <= addtreeTemp08; regaddtreeTemp09  <= addtreeTemp09; regaddtreeTemp10  <= addtreeTemp10; regaddtreeTemp11  <= addtreeTemp11; regaddtreeTemp12  <= addtreeTemp12; regaddtreeBia  <= addtreeBia;

                cycle <= cycle + 1;
            end
            else if(cycle == 3) begin
                regaddtreeTemp21 <= addtreeTemp21;
                regaddtreeTemp22 <= addtreeTemp22;
                regaddtreeTemp23 <= addtreeTemp23;
                regaddtreeTemp24 <= addtreeTemp24;
                regaddtreeTemp25 <= addtreeTemp25;
                regaddtreeTemp26 <= addtreeTemp26;
                regaddtreeBia   <= addtreeBia;

                cycle <= cycle + 1;
            end
            else if(cycle == 4) begin
                regaddtreeTemp31 <= addtreeTemp31;
                regaddtreeTemp32 <= addtreeTemp32;
                regaddtreeTemp33 <= addtreeTemp33;

                cycle <= cycle + 1;
            end
            else if(cycle == 5) begin
                regaddtreeTemp41 <= addtreeTemp41;
                regaddtreeTemp42 <= addtreeTemp42;
                
                cycle <= cycle + 1;
            end
            else if(cycle == 6) begin
                covSum <= addSum;
                covResVld <= 1;
            end
        end
        else begin
            //mul leve
            regOmul1<= 0; regOmul2<= 0; regOmul3<= 0; regOmul4<= 0; regOmul5<= 0; regOmul6<= 0; regOmul7<= 0; regOmul8<= 0; regOmul9<= 0; regOmul10<= 0; regOmul11<= 0; regOmul12<= 0; regOmul13<= 0;
            regOmul14<= 0; regOmul15<= 0; regOmul16<= 0; regOmul17<= 0; regOmul18<= 0; regOmul19<= 0; regOmul20<= 0; regOmul21<= 0; regOmul22<= 0; regOmul23<= 0; regOmul24<= 0; regOmul25 <= 0;
            //adders level 1
            regaddtreeTemp01 <= 0; regaddtreeTemp02 <= 0; regaddtreeTemp03 <= 0; regaddtreeTemp04 <= 0; regaddtreeTemp05 <= 0; regaddtreeTemp06 <= 0; regaddtreeTemp07 <= 0;
            regaddtreeTemp08 <= 0; regaddtreeTemp09 <= 0; regaddtreeTemp10 <= 0; regaddtreeTemp11 <= 0; regaddtreeTemp12 <= 0; regaddtreeBia <= 0;
            //adders level 2
            regaddtreeTemp21<= 0; regaddtreeTemp22<= 0; regaddtreeTemp23<= 0; regaddtreeTemp24<= 0; regaddtreeTemp25<= 0; regaddtreeTemp26<= 0; regaddtreeBia<= 0; 
            //adders level 3
            regaddtreeTemp31 <= 0; regaddtreeTemp32 <= 0; regaddtreeTemp33 <= 0;
            //adders level 4
            regaddtreeTemp41<= 0; regaddtreeTemp42<= 0;
            //internal param
            iwInternal <= 0; ibInternal <= 0;   imInternal <= 0;
            cycle <= 0;
            covResVld <= 0;
        end
    end
end
endmodule

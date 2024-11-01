`timescale 1ns / 1ps


`define DATA_SIZE 8
`define halfword 16

module reluPool(
    input clk, rst,
    input numsVld,
    input [`halfword-1:0] num1,num2,num3,num4,

    output reg dout_vld,
    output reg [`DATA_SIZE-1:0] dout);
    //***************Internal reg********************//
    reg [2:0] cycle;
    //level 1
    reg [`halfword-1:0] InterNum1, InterNum2, InterNum3, InterNum4;
    //level 2
    reg [`DATA_SIZE-1:0] regnum1Cutout, regnum2Cutout, regnum3Cutout, regnum4Cutout;
    //level 3
    reg [`DATA_SIZE-1:0] regMax1, regMax2;
    //level 4 out put

    //***************CUT OUT & RELU******************// //level 1
    
    wire [`DATA_SIZE-1:0] num1Cutout, num2Cutout, num3Cutout, num4Cutout;

    assign num1Cutout = InterNum1[15]? 0: (InterNum1 [6])?{InterNum1 [15],InterNum1 [13:7]+1'b1}:{InterNum1 [15],InterNum1 [13:7]}; //cut out = round(num/(32*4))
    assign num2Cutout = InterNum2[15]? 0: (InterNum2 [6])?{InterNum2 [15],InterNum2 [13:7]+1'b1}:{InterNum2 [15],InterNum2 [13:7]};
    assign num3Cutout = InterNum3[15]? 0: (InterNum3 [6])?{InterNum3 [15],InterNum3 [13:7]+1'b1}:{InterNum3 [15],InterNum3 [13:7]};
    assign num4Cutout = InterNum4[15]? 0: (InterNum4 [6])?{InterNum4 [15],InterNum4 [13:7]+1'b1}:{InterNum4 [15],InterNum4 [13:7]};

    //*******************MAX POOL******************// level 2~4
    //level 2
    wire [`DATA_SIZE-1:0] max_1;
    wire [`DATA_SIZE-1:0] max_2;

    assign max_1 = (regnum1Cutout > regnum2Cutout)? regnum1Cutout:regnum2Cutout;
    assign max_2 = (regnum3Cutout > regnum4Cutout)? regnum3Cutout:regnum4Cutout;
    
    //leve3
    wire [`DATA_SIZE-1:0] maxx;

    assign maxx = (regMax1 > regMax2)? regMax1:regMax2;


always @(posedge clk or posedge rst ) begin
    if(rst == 1) begin
        //**output reg
            dout_vld <= 0;
            dout <= 0;
        //**internal reg
            cycle <= 0;
            //level 1
            InterNum1 <= 0; InterNum2 <= 0; InterNum3 <= 0; InterNum4 <= 0;
            //level 2
            regnum1Cutout <= 0; regnum2Cutout <= 0; regnum3Cutout <= 0; regnum4Cutout <= 0;
            //level 3
            regMax1 <= 0; regMax2 <= 0;
            //level 4 out put
    end
    else begin
        if(numsVld == 1) begin
            if(cycle == 0) begin
                InterNum1 <= num1; InterNum2 <= num2; InterNum3 <= num3; InterNum4 <= num4;
                cycle <= cycle + 1;
            end
            else if(cycle == 1) begin
                regnum1Cutout <= num1Cutout; regnum2Cutout <= num2Cutout; regnum3Cutout <= num3Cutout; regnum4Cutout <= num4Cutout;
                cycle <= cycle + 1;
            end
            else if(cycle == 2) begin
                regMax1 <= max_1; regMax2 <= max_2;
                cycle <= cycle + 1;
            end
            else if(cycle == 3) begin
                dout_vld <= 1;
                dout <= maxx;
            end
        end
        else begin
            //**output reg
                dout_vld <= 0;
                dout <= 0;
            //**internal reg
                cycle <= 0;
                //level 1
                InterNum1 <= 0; InterNum2 <= 0; InterNum3 <= 0; InterNum4 <= 0;
                //level 2
                regnum1Cutout <= 0; regnum2Cutout <= 0; regnum3Cutout <= 0; regnum4Cutout <= 0;
                //level 3
                regMax1 <= 0; regMax2 <= 0;
                //level 4 out put
        end
    end
end
endmodule

`timescale 1ps/1ps

`define DATA_SIZE 8

module fc_classify(
    input clk, rst,
    input classify_en,
    input [10*`DATA_SIZE-1:0] fc_result,

    output reg classify_res_vld,
    output reg [`DATA_SIZE-1:0] classify_res
);
//internal param
reg [2:0] cycle;

//max level 1
reg [`DATA_SIZE-1:0]
max10, max11, max12, max13, max14;

//max level 2
reg [`DATA_SIZE-1:0]
max20,max21;

//max level 3
reg [`DATA_SIZE-1:0] max30;

//max level 4
reg [`DATA_SIZE-1:0] max40;

always@(posedge clk or posedge rst) begin
    if(rst == 1) begin
        //**internal param
            cycle <= 0;
            //max level 1
            max10 <= 0; max11 <= 0; max12 <= 0; max13 <= 0; max14 <= 0;
            //max level 2
            max20 <= 0;max21 <= 0;
            //max level 3
            max30 <= 0;
            //max level 4
            max40 <= 0;
        //**output param
            classify_res_vld <= 0;
            classify_res <= 0;
    end
    else begin
        if(classify_en == 1) begin
            if(cycle == 0) begin
                max10 <= (fc_result[10*`DATA_SIZE-1:9*`DATA_SIZE] >fc_result[9*`DATA_SIZE-1:8*`DATA_SIZE] )? fc_result[10*`DATA_SIZE-1:9*`DATA_SIZE]:fc_result[9*`DATA_SIZE-1:8*`DATA_SIZE] ;
                max11 <= (fc_result[8*`DATA_SIZE-1:7*`DATA_SIZE] >fc_result[7*`DATA_SIZE-1:6*`DATA_SIZE] )? fc_result[8*`DATA_SIZE-1:7*`DATA_SIZE]:fc_result[7*`DATA_SIZE-1:6*`DATA_SIZE] ;
                max12 <= (fc_result[6*`DATA_SIZE-1:5*`DATA_SIZE] >fc_result[5*`DATA_SIZE-1:4*`DATA_SIZE] )? fc_result[6*`DATA_SIZE-1:5*`DATA_SIZE]:fc_result[5*`DATA_SIZE-1:4*`DATA_SIZE] ;
                max13 <= (fc_result[4*`DATA_SIZE-1:3*`DATA_SIZE] >fc_result[3*`DATA_SIZE-1:2*`DATA_SIZE] )? fc_result[4*`DATA_SIZE-1:3*`DATA_SIZE]:fc_result[3*`DATA_SIZE-1:2*`DATA_SIZE] ;
                max14 <= (fc_result[2*`DATA_SIZE-1:1*`DATA_SIZE] >fc_result[1*`DATA_SIZE-1:0*`DATA_SIZE] )? fc_result[2*`DATA_SIZE-1:1*`DATA_SIZE]:fc_result[1*`DATA_SIZE-1:0*`DATA_SIZE] ;
            
                cycle <= cycle + 1;
            end
            else if(cycle == 1) begin
                max20 <= (max10 > max11)? max10:max11;
                max21 <= (max12 > max13)? max12:max13;

                cycle <= cycle + 1;
            end
            else if(cycle == 2) begin
                max30 <= (max20 > max21)? max20:max21;

                cycle <= cycle + 1;
            end
            else if(cycle == 3) begin
                max40 <= (max30 > max14)? max30:max14;

                cycle <= cycle + 1;                
            end
            else if(cycle == 4) begin
                if(max40 == fc_result[10*`DATA_SIZE-1:9*`DATA_SIZE]) classify_res <= 8'd0;
                if(max40 == fc_result[8*`DATA_SIZE-1:7*`DATA_SIZE] ) classify_res <= 8'd2;
                if(max40 == fc_result[6*`DATA_SIZE-1:5*`DATA_SIZE] ) classify_res <= 8'd4;
                if(max40 == fc_result[4*`DATA_SIZE-1:3*`DATA_SIZE] ) classify_res <= 8'd6;
                if(max40 == fc_result[2*`DATA_SIZE-1:1*`DATA_SIZE] ) classify_res <= 8'd8;
                if(max40 == fc_result[9*`DATA_SIZE-1:8*`DATA_SIZE] ) classify_res <= 8'd1;
                if(max40 == fc_result[7*`DATA_SIZE-1:6*`DATA_SIZE] ) classify_res <= 8'd3;
                if(max40 == fc_result[5*`DATA_SIZE-1:4*`DATA_SIZE] ) classify_res <= 8'd5;
                if(max40 == fc_result[3*`DATA_SIZE-1:2*`DATA_SIZE] ) classify_res <= 8'd7;
                if(max40 == fc_result[1*`DATA_SIZE-1:0*`DATA_SIZE] ) classify_res <= 8'd9;
            
                cycle <= cycle + 1;
            end
            else if(cycle == 5) classify_res_vld <= 1;
        end
        else begin
            //**internal param
            cycle <= 0;
            //max level 1
            max10 <= 0; max11 <= 0; max12 <= 0; max13 <= 0; max14 <= 0;
            //max level 2
            max20 <= 0;max21 <= 0;
            //max level 3
            max30 <= 0;
            //max level 4
            max40 <= 0;
        //**output param
            classify_res_vld <= 0;
            classify_res <= 0;
        end

    end
end
endmodule
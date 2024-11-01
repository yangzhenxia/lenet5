`timescale 1ns / 1ps

`define DATA_SIZE 8

`define inputMapWidth  6*6*8 // get a 6*6 input map to rearrange to 4 5*5 input map which will be input to 4 different mul_add_array
`define inputWeightWidth 5*5*8 // get a whole kennel weights

`define halfword 16

`define weightRam_addra_width 11
`define mapRam_addra_width 9


module convPool2(
                input clk,rst,
                input conv_2_en,
                output reg conv_2_finish,

                //**interact with convolution level weights&bias sram array
                    //data from 13 sram
                    input [`inputWeightWidth+`DATA_SIZE-1:0] conv2_weightsBias,
                    //addra and ena
                    output reg [`weightRam_addra_width-1:0] conv2_weightsBias_sram_addra,
                    output reg conv2_weightsBias_sram_ena,
                //**interact with feature map sram
                    //data from 18 sram
                    input [`inputMapWidth-1:0] conv2_inputMap,
                    //addra , ena ,wea
                    output reg [`mapRam_addra_width-1:0] conv2_featureMap_sram_addra,
                    output reg conv2_featureMap_sram_ena,
                //**interact with convoluters
                    //data from convoluters
                        input convResVld,
                        input [`halfword - 1: 0] conv2_num1, conv2_num2, conv2_num3, conv2_num4, 
                    //data to convoluters
                        output reg imVld,iwVld,
                        output reg [`inputMapWidth-1:0] im,
                        output reg [`inputWeightWidth-1:0] iw,
                //**conv2 level results (50points)
                    output reg [399:0] convPool2Res
                );
//debug mem
// reg[7:0]mem[0:0];
// reg[15:0] memm [0:0];
// reg [199:0] memW[0:0];
// reg [287:0] memM[0:0];
// integer optFile;

//counter param
reg [5:0] filter; //in cov1 20 filter in total
reg [4:0] channel; 
reg [2:0] cycle; 

//temp
reg [15:0] convResMem [0:63];
integer i;

//addra set
parameter 
conv2_weightsBias_base = 20,
conv2_featureMap_base = 144;

reg [9:0] conv2_weightsBias_index;//0~1000
reg [8:0] conv2_featureMap_index;//0~320

//***********8x8 reLu pool module************//
reg reluPool_en;
wire reluPoolResVld;
wire [7:0] reluPoolRes;

reluPoolcov2 u_reluPoolcov2(
.clk(clk),.rst(rst),
.dataVld(reluPool_en),
.num00(convResMem[0]),.num01(convResMem[1]),.num02(convResMem[2]),.num03(convResMem[3]),.num04(convResMem[4]),.num05(convResMem[5]),.num06(convResMem[6]),.num07(convResMem[7]),.num08(convResMem[8]),.num09(convResMem[9]),
.num10(convResMem[10]),.num11(convResMem[11]),.num12(convResMem[12]),.num13(convResMem[13]),.num14(convResMem[14]),.num15(convResMem[15]),.num16(convResMem[16]),.num17(convResMem[17]),.num18(convResMem[18]),.num19(convResMem[19]),
.num20(convResMem[20]),.num21(convResMem[21]),.num22(convResMem[22]),.num23(convResMem[23]),.num24(convResMem[24]),.num25(convResMem[25]),.num26(convResMem[26]),.num27(convResMem[27]),.num28(convResMem[28]),.num29(convResMem[29]),
.num30(convResMem[30]),.num31(convResMem[31]),.num32(convResMem[32]),.num33(convResMem[33]),.num34(convResMem[34]),.num35(convResMem[35]),.num36(convResMem[36]),.num37(convResMem[37]),.num38(convResMem[38]),.num39(convResMem[39]),
.num40(convResMem[40]),.num41(convResMem[41]),.num42(convResMem[42]),.num43(convResMem[43]),.num44(convResMem[44]),.num45(convResMem[45]),.num46(convResMem[46]),.num47(convResMem[47]),.num48(convResMem[48]),.num49(convResMem[49]),
.num50(convResMem[50]),.num51(convResMem[51]),.num52(convResMem[52]),.num53(convResMem[53]),.num54(convResMem[54]),.num55(convResMem[55]),.num56(convResMem[56]),.num57(convResMem[57]),.num58(convResMem[58]),.num59(convResMem[59]),
.num60(convResMem[60]),.num61(convResMem[61]),.num62(convResMem[62]),.num63(convResMem[63]),

.reluPoolResVld(reluPoolResVld),
.reluPoolRes(reluPoolRes)
);
//*****************accumulaters******************//
reg [15:0] 
accu1_current,
accu2_current,
accu3_current,
accu4_current;

reg [15:0] regaccu1_res,regaccu2_res,regaccu3_res,regaccu4_res;
wire [15:0] accu1_res,accu2_res,accu3_res,accu4_res;

add u_conv2_accu1(.a(accu1_current), .b(conv2_num1), .o(accu1_res));
add u_conv2_accu2(.a(accu2_current), .b(conv2_num2), .o(accu2_res));
add u_conv2_accu3(.a(accu3_current), .b(conv2_num3), .o(accu3_res));
add u_conv2_accu4(.a(accu4_current), .b(conv2_num4), .o(accu4_res));


//*******************state mechine********************//
reg [5:0] state, next_state;
parameter 
S_IDLE =  6'b100000,
S_LOAD_BIA = 6'b010000,
S_LOAD_W = 6'b001000,
S_LOAD_M = 6'b000100,
S_CONVOLUTE = 6'b000010,
S_RELU_POOL = 6'b000001;

//S_CONVLUTE param
reg convState_finish;
reg [4:0] convResCnt;
reg[2:0] row, col;
//S_LOAD_BIA param
reg ibVld;
//S_RELUPOOL param
reg poolState_finish;

always @(posedge clk or posedge rst) begin
    if(rst == 1) begin
        state <= S_IDLE;
    end
    else begin
        state <= next_state;
    end
end

always@(*) begin
    if(rst == 1) begin
        next_state = S_IDLE;
    end
    else begin
        case(state) 
            S_IDLE: next_state = (conv_2_en == 0)?S_IDLE:S_LOAD_BIA;
            S_LOAD_BIA: next_state = (ibVld == 0)?S_LOAD_BIA:S_LOAD_W;
            S_LOAD_W: next_state = (iwVld == 0)?S_LOAD_W:S_LOAD_M;
            S_LOAD_M: next_state = (imVld == 0)?S_LOAD_M:S_CONVOLUTE;
            // S_CONVOLUTE: next_state = (convState_finish == 0)?S_CONVOLUTE:(channel == 20)?S_RELU_POOL:(oneFeatureMap_finish == 0)?S_LOAD_M:S_LOAD_W;
            S_CONVOLUTE: next_state = (convState_finish == 0)?S_CONVOLUTE:(convResCnt < 16)?S_LOAD_M:(channel < 20)?S_LOAD_W:S_RELU_POOL;
            S_RELU_POOL: next_state = (poolState_finish == 0)?S_RELU_POOL:(filter < 50)?S_LOAD_BIA:S_IDLE;
            default:next_state = S_IDLE;
        endcase
    end
end

//S_IDLE
always@(posedge clk or posedge rst) begin
    if(rst == 1) begin
        //S_IDLE
        conv_2_finish <= 0;
        //S_LOAD_BIA
         ibVld <= 0;
            //init mem
            for(i=0;i<64;i=i+1) begin
                convResMem[i] <= 0;
            end
            //init weights and bias ram ena..
            conv2_weightsBias_sram_ena <= 0;
            conv2_weightsBias_sram_addra <= 0;

            cycle <= 0;
        //S_LOAD_W
            iw <= 0;
            iwVld <= 0;
            conv2_weightsBias_index <= 0;
        //S_LOAD_M
            im <= 0;
            imVld <= 0;
            //feature ram
            conv2_featureMap_sram_addra <= 0;
            conv2_featureMap_sram_ena <= 0;
            //index
            conv2_featureMap_index <= 0;
        //S_CONVOLUTE
            convState_finish <= 0;
            //init accumulaters param
            accu1_current <= 0;
            accu2_current <= 0;
            accu3_current <= 0;
            accu4_current <= 0;
            //internal param
            channel <= 0;
            convResCnt <= 0;
            //accumulate param
            regaccu1_res <= 0; regaccu2_res <= 0; regaccu3_res <= 0; regaccu4_res <= 0;
            //
            row <= 0; col <= 0;
        //S_RELU_POOL
            convPool2Res <= 0;
            filter <= 0;
            poolState_finish<=0;
            reluPool_en <= 0;
           
    end
    else begin
        case(state)
            S_IDLE:
            begin
                if(filter == 50) begin
                    conv_2_finish <= 1;
                end
            end
            //********
            S_LOAD_BIA:
            begin
                if(ibVld == 0) begin
                    if(cycle == 0) begin
                        conv2_weightsBias_sram_ena <= 1;
                        conv2_weightsBias_sram_addra <=  conv2_weightsBias_base + conv2_weightsBias_index;

                        cycle <= cycle + 1;
                    end
                    else if(cycle == 2) begin
                        ibVld <= 1;
                        for(i=0;i<64;i=i+1) begin
                            convResMem[i] <= {conv2_weightsBias[`DATA_SIZE-1],3'b0,conv2_weightsBias[`DATA_SIZE-2:0],5'b0};
                        end
                        //reset feature map index 
                        conv2_featureMap_index <= 0;

                        cycle <= 0;
                    end
                    else begin
                        cycle <= cycle + 1;
                    end
                end
            end
            //*********
            S_LOAD_W:
            begin
                if(state == S_LOAD_W & iwVld == 0) begin
                    if(cycle == 0) begin
                        row <= 0; col <= 0;

                        convResCnt <= 0;

                        conv2_weightsBias_sram_ena <= 1;
                        conv2_weightsBias_sram_addra <=  conv2_weightsBias_base + conv2_weightsBias_index;

                        cycle <= cycle + 1;
                    end
                    else if(cycle == 2) begin
                        iwVld <= 1;
                        iw <= conv2_weightsBias[`inputWeightWidth + `DATA_SIZE -1:`DATA_SIZE];
                    // //***************debug******************//
                    //     if(filter == 0)
                    //     begin
                    //         // memm[0]  = {1'b0,accu4_res[14:0]};
                    //         memW[0]  = iw;
                    //         optFile = $fopen("D:/Projects/FPGA/SEU_MNIST_LeNet/Lenet_Fukami_v3/_usr_Src/sim/data_txt/conv2Filter0Channels.txt","a");
                    //         $fwrite(optFile, "%b\n",memW[0]);
                    //         $fclose(optFile);
                    //     end
                    // //*************************************//
                        //update addra index
                        conv2_weightsBias_index <= conv2_weightsBias_index + 1;
                        //stop ram
                        conv2_weightsBias_sram_ena <= 0;
                        //update channel
                        channel <= channel + 1;

                        cycle <= 0;
                    end
                    else begin
                        cycle <= cycle + 1;
                    end
                end
            end
            //**********
            S_LOAD_M:
            begin
                if(imVld == 0) begin
                    if(cycle == 0) begin
                        conv2_featureMap_sram_ena <= 1;
                        conv2_featureMap_sram_addra <=  conv2_featureMap_base + conv2_featureMap_index;

                        convState_finish <= 0;
                        cycle <= cycle + 1;
                    end
                    else if(cycle == 2) begin
                        imVld <= 1;
                        im <= conv2_inputMap;

                    // //***************debug******************//
                    //     if(filter == 0 & row == 1 & col == 3)
                    //     begin
                    //         memM[0]  = im;
                    //         optFile = $fopen("D:/Projects/FPGA/SEU_MNIST_LeNet/Lenet_Fukami_v3/_usr_Src/sim/data_txt/conv2Filter0Maps20.txt","a");
                    //         $fwrite(optFile, "%b\n",memM[0]);
                    //         $fclose(optFile);
                    //     end
                    // //*************************************//

                        
                        //stop ram
                        conv2_featureMap_sram_ena <= 0;

                        cycle <= 0;
                    end
                    else begin
                        cycle <= cycle + 1;
                    end
                end
            end
            //********
            S_CONVOLUTE:
            begin
               if(convState_finish == 0)begin
                    if(col < 4) begin
                        if(convResVld == 1) begin
                            if(cycle == 0) begin
                                accu1_current <= convResMem[0+2*col+16*row];
                                accu2_current <= convResMem[1+2*col+16*row];
                                accu3_current <= convResMem[8+2*col+16*row];
                                accu4_current <= convResMem[9+2*col+16*row];
                            
                                cycle <= cycle + 1;
                            end
                            else if(cycle == 1) begin
                               regaccu1_res <= accu1_res;
                               regaccu2_res <= accu2_res;
                               regaccu3_res <= accu3_res;
                               regaccu4_res <= accu4_res;

                               cycle <= cycle + 1;
                            end
                            else if(cycle == 2) begin
                                if(convResCnt == 15) begin
                                    iwVld <= 0;
                                end
                                if(channel == 20) begin
                                    poolState_finish <= 0;
                                end

                                convResMem[0+2*col+16*row] <= regaccu1_res ;
                                convResMem[1+2*col+16*row] <= regaccu2_res ;
                                convResMem[8+2*col+16*row] <= regaccu3_res ;
                                convResMem[9+2*col+16*row] <= regaccu4_res ;

                                imVld <= 0;
                                //update addra index
                                conv2_featureMap_index <= conv2_featureMap_index + 1;

                                convState_finish <= 1;
                                convResCnt <= convResCnt + 1;

                                col <= col + 1;

                                cycle <= 0;
                                // //***************debug******************//
                                // if(filter == 0 & row == 1 & col == 2)
                                // begin
                                //     // memm[0]  = {1'b0,accu4_res[14:0]};
                                //     memm[0]  = accu4_res;
                                //     optFile = $fopen("D:/Projects/FPGA/SEU_MNIST_LeNet/Lenet_Fukami_v3/_usr_Src/sim/data_txt/optConv2Map1P2Acum.txt","a");
                                //     $fwrite(optFile, "%d\n",memm[0]);
                                //     $fclose(optFile);
                                // end
                            end
                        end
                    end
                    else begin
                        col <= 0;
                        row <= row + 1;
                    end
                end
            end
            //********
            S_RELU_POOL:
            begin
                if(state == S_RELU_POOL & poolState_finish == 0) begin
                    if(reluPoolResVld == 0) begin
                        reluPool_en <= 1;
                    end
                    else begin

                    reluPool_en <= 0;
                    poolState_finish <= 1;
                    convPool2Res <= {convPool2Res[391:0],reluPoolRes};

                    filter <= filter + 1;
                    channel <= 0;

                    ibVld <= 0;

                    // //***************debug******************//
                    //     mem[0]  = reluPoolRes;
                    //     optFile = $fopen("D:/Projects/FPGA/SEU_MNIST_LeNet/Lenet_Fukami_v3/_usr_Src/sim/data_txt/optConv2Channel50.txt","a");
                    //     $fwrite(optFile, "%d\n",mem[0]);
                    //     $fclose(optFile);
                    end
                end
            end
        endcase
    end
end
endmodule

`timescale 1ns / 1ps

`define DATA_SIZE 8
`define inputMapWidth  6*6*8 // get a 6*6 input map to rearrange to 4 5*5 input map which will be input to 4 different mul_add_array
`define inputWeightWidth 5*5*8 // get a whole kennel weights
`define halfword 16 

`define weightRam_addra_width 11
`define mapRam_addra_width 9

module convPool1(
                input clk,rst,
                input conv_1_en,
                output reg conv_1_finish,

                //**interact with weights and bias ram
                    //data from controler
                    input [`inputWeightWidth+`DATA_SIZE-1:0] conv1_weightsBias,
                    //addra and ena
                    output reg [`weightRam_addra_width-1:0] conv1_weightsBias_sram_addra,
                    output reg conv1_weightsBias_sram_ena,
                //**interact with feature map sram
                    //data from 18 sram
                    input [`inputMapWidth-1:0] conv1_inputMap,
                    //data to 18 sram
                    output reg [`inputMapWidth-1:0] conv1_optMap,
                    //addra , ena ,wea
                    output reg [`mapRam_addra_width-1:0] conv1_featureMap_sram_addra,
                    output reg conv1_featureMap_sram_ena,
                    output reg conv1_featureMap_sram_wea,
                //**interact with convoluters
                    //data from convoluters
                        input convResVld,
                        input [`halfword - 1: 0] conv1_num1, conv1_num2, conv1_num3, conv1_num4, 
                    //data to convoluters
                        output reg imVld,iwVld,
                        output reg [`DATA_SIZE-1:0] ib,
                        output reg [`inputMapWidth-1:0] im,
                        output reg [`inputWeightWidth-1:0] iw
                );

//debug mem
// reg[7:0]mem[0:0];
// integer optFile;

//counter param
reg [4:0] filter; //in cov1 20 filter in total
reg [2:0] cycle; 
reg [7:0] convResCnt;//when 36point have been done ,will be store to sram array

//state finish label
reg convState_finish;
reg storeState_finish;

//temp res data
reg [7:0] convResMem [0:143];
integer i;


//***********relu Pool module
wire reluPool_vld;
wire [`DATA_SIZE-1:0] poolRes;
reg reluPool_en;
reluPool u_reluPool_cov1(.clk(clk), .rst(rst), 
.numsVld(reluPool_en), 
.num1(conv1_num1), .num2(conv1_num2), 
.num3(conv1_num3), .num4(conv1_num4), 
.dout_vld(reluPool_vld), .dout(poolRes));

//***************state mechine
//state define
reg [4:0] state, next_state;
parameter 
S_IDLE = 5'b10000,
S_LOAD_WB = 5'b01000,//load weights and bias
S_LOAD_M = 5'b00100, // load map
S_CONVOLUTE = 5'b00010,
S_STORE_RES = 5'b00001;


//RAM address set
parameter
conv1_featureMap_base = 0,
conv1_weightsBias_base = 0,
conv1_result_base = 144;

reg [7:0] inputMap_addraIndex;
reg [8:0] outputMap_addraIndex; //one 12*12map will be reranged to 16 maps(6*6) ,and 20 channels deep in 16*20 = 320

//S_STORE_RES
reg [2:0] row,col;

always@(posedge clk or posedge rst) begin
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
            S_IDLE: next_state = (conv_1_en == 0)?S_IDLE:(filter == 20)?S_IDLE:S_LOAD_WB;
            S_LOAD_WB:next_state = (iwVld == 1)? S_LOAD_M:S_LOAD_WB;
            S_LOAD_M:next_state = (imVld == 1)?S_CONVOLUTE:S_LOAD_M;
            S_CONVOLUTE:next_state = (convState_finish == 0)?S_CONVOLUTE:(convResCnt < 144)?S_LOAD_M:S_STORE_RES;
            S_STORE_RES:next_state = (storeState_finish == 0)?S_STORE_RES:(filter < 20)?S_LOAD_WB:S_IDLE;
            default: next_state = S_IDLE;
        endcase
    end
end


always @(posedge clk or posedge rst) begin
    if(rst == 1) begin
        conv_1_finish <= 0;
        //**S_LOAD_WB
            iwVld <= 0;
            //weights and bias ram init
            conv1_weightsBias_sram_addra <= 0;
            conv1_weightsBias_sram_ena <= 0;
            //convoluter param init
            iw <= 0;ib <= 0;
            //internal param init
            cycle <= 0;
            filter <= 0;
        //**S_LOAD_M
            imVld <= 0;
            //weights and bias ram init
            conv1_featureMap_sram_addra <= 0;
            conv1_featureMap_sram_ena <= 0;
            conv1_featureMap_sram_wea <= 0;
            //convoluter param init
            im <= 0;
            //addra init
            inputMap_addraIndex <= 0;
        //**S_CONVOLUTE
            conv1_optMap <= 0;
            convState_finish <= 0;
            //convResMem init
            convResCnt <= 0;
            for(i=0;i<144;i=i+1) begin
                convResMem[i] <= 0;
            end
            reluPool_en <= 0;
        //**S_STORE_RES
            storeState_finish <= 0;
            //param init
            filter <= 0;
            //S_STORE_RES internal param
            row<=0;col <=0;
            //addra param init
            outputMap_addraIndex <= 0;

            
    end 
    else begin
        case(state)
            //*******
            S_IDLE:
            begin
                if(state == S_IDLE & filter == 20) begin
                conv_1_finish <= 1;
                end
            end
            //*******
            S_LOAD_WB:
            begin
                if(iwVld == 0) begin
                    if (cycle == 0) begin
                        iwVld <= 1'b0; //tell to convoluter weights and bia is NOT ready
                        //reset S_STORE_RES
                        storeState_finish <= 0;

                        conv1_weightsBias_sram_ena   <= 1'b1;
                        conv1_weightsBias_sram_addra <= conv1_featureMap_base + filter;

                        cycle                   <= cycle + 1;
                    end
                    else if (cycle == 2) begin// wait 2 cycle to get data
                        iw <= conv1_weightsBias[`inputWeightWidth+ `DATA_SIZE-1:`DATA_SIZE];
                        ib <= conv1_weightsBias[`DATA_SIZE-1:0];
                        //stop enable the ram
                        conv1_weightsBias_sram_ena <= 1'b0; 
                        //tell to convoluter weights and bia is ready
                        iwVld <= 1'b1; 

                        cycle <= 0;
                    end
                    else begin
                      cycle <= cycle + 1;
                    end
                end
            end
            //*************
            S_LOAD_M:
            begin
                if(imVld == 0) begin
                    if (cycle == 0) begin
                        imVld <= 1'b0; //tell to convoluter weights and bia is NOT ready
                        //convolute init
                        convState_finish <= 0;
                        //enable ram
                        conv1_featureMap_sram_wea <= 0;
                        conv1_featureMap_sram_ena   <= 1;
                        conv1_featureMap_sram_addra <= conv1_featureMap_base + inputMap_addraIndex;

                        cycle                   <= cycle + 1;
                    end
                    else if (cycle == 2) begin// wait 2 cycle to get data
                        im <= conv1_inputMap;
                        //stop enable the ram
                        conv1_featureMap_sram_ena <= 0;
                        conv1_featureMap_sram_wea <= 0;
                        //index update
                        inputMap_addraIndex <= inputMap_addraIndex + 1;
                        //tell to convoluter weights and bia is ready
                        imVld <= 1'b1; 

                        cycle <= 0;
                    end
                     else begin
                      cycle <= cycle + 1;
                    end
                end
            end
            //**********
            S_CONVOLUTE:
            begin
                if(convState_finish == 0) begin
                    if(convResVld == 1) begin
                        if(reluPool_vld == 0) begin
                            reluPool_en <= 1;
                        end
                        else begin
                        convState_finish <= 1; //one feature map point done

                        convResMem[convResCnt]<= poolRes;
                        convResCnt <= convResCnt + 1;
                        //one map array done
                        imVld <= 0;
                        reluPool_en <= 0;
                    
                        // //****debug****//
                        //     mem[0]  = poolRes;
                        //     optFile = $fopen("D:/Projects/FPGA/SEU_MNIST_LeNet/Lenet_Fukami_v5/_usr_Src/sim/data_txt/optConv1Channel20.txt","a");
                        //     $fwrite(optFile, "%d\n",mem[0]);
                        //     $fclose(optFile);
                        end
                    end
                end
            end
            //**************
            S_STORE_RES:
            begin
                if(storeState_finish == 0) begin
                    if(row < 4) begin
                        if(col < 4) begin
                            if(cycle == 0)begin
                                conv1_featureMap_sram_addra <= conv1_result_base + outputMap_addraIndex;
                                conv1_featureMap_sram_ena <= 1;
                                conv1_featureMap_sram_wea <= 1;
                                //wright data to ram
                                conv1_optMap <= 
                                    {convResMem[0 +col*2 +row*24],convResMem[1 +col*2 +row*24],
                                    convResMem[2 +col*2 +row*24],convResMem[3 +col*2 +row*24],
                                    convResMem[4 +col*2 +row*24],convResMem[5 +col*2 +row*24],
                                    convResMem[12+col*2 +row*24],convResMem[13+col*2 +row*24],
                                    convResMem[14+col*2 +row*24],convResMem[15+col*2 +row*24],
                                    convResMem[16+col*2 +row*24],convResMem[17+col*2 +row*24],
                                    convResMem[24+col*2 +row*24],convResMem[25+col*2 +row*24],
                                    convResMem[26+col*2 +row*24],convResMem[27+col*2 +row*24],
                                    convResMem[28+col*2 +row*24],convResMem[29+col*2 +row*24],
                                    convResMem[36+col*2 +row*24],convResMem[37+col*2 +row*24],
                                    convResMem[38+col*2 +row*24],convResMem[39+col*2 +row*24],
                                    convResMem[40+col*2 +row*24],convResMem[41+col*2 +row*24],
                                    convResMem[48+col*2 +row*24],convResMem[49+col*2 +row*24],
                                    convResMem[50+col*2 +row*24],convResMem[51+col*2 +row*24],
                                    convResMem[52+col*2 +row*24],convResMem[53+col*2 +row*24],
                                    convResMem[60+col*2 +row*24],convResMem[61+col*2 +row*24],
                                    convResMem[62+col*2 +row*24],convResMem[63+col*2 +row*24],
                                    convResMem[64+col*2 +row*24],convResMem[65+col*2 +row*24]};

                                cycle <= cycle + 1;
                            end
                            else if(cycle == 2) begin
                                conv1_featureMap_sram_ena <= 0;
                                conv1_featureMap_sram_wea <= 0;
                                //update index
                                outputMap_addraIndex <= outputMap_addraIndex + 1;
                                //update col
                                col <= col + 1;

                                cycle <= 0;
                            end
                            else begin
                                cycle <= cycle + 1;
                            end
                        end
                        else begin
                            row <= row + 1;
                            col <= 0;    
                        end
                    end
                    else begin
                        row <= 0;
                        storeState_finish <= 1;
                        //one filter done
                        filter <= filter + 1;
                        iwVld <= 0;//reload weights and bias
                        //reload the input map
                        inputMap_addraIndex <= 0;
                        //init convRescnt
                        convResCnt <= 0;
                    end
                end
            end
        endcase    
    end
end
endmodule

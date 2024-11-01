`timescale 1ns / 1ps

`define DATA_SIZE 8
`define halfword 16

`define inputMapWidth 6*6*8
`define inputWeightWidth 5*5*8

`define weightRam_addra_width 11
`define mapRam_addra_width 9


module lenetTop(
                //**input
                    input clk, sys_rst,
                    input ctrl_rst,//reset from controler
                    input start,
                    //data from sram pass controler
                    input [`inputWeightWidth+`DATA_SIZE-1:0] weightsBias,
                    input [`inputMapWidth-1:0] featureMap_sram_douta,
                //**output
                    //to sram
                    output reg weightsBias_sram_ena,
                    output reg [`weightRam_addra_width-1:0] weightsBias_sram_addra,
                    output reg featureMap_sram_ena, featureMap_sram_wea,
                    output reg [`mapRam_addra_width-1:0] featureMap_sram_addra,
                    output reg [`inputMapWidth-1:0] featureMap_sram_din,
                    //to controler
                    wire [`DATA_SIZE-1:0] lenet_res,
                    output reg lenet_finish);

//*****************set reset signal******************//
wire rst;
assign  rst = ctrl_rst | sys_rst;


//*********mul*add array**********//
reg imVld,iwVld;
reg [`inputMapWidth-1:0] im;    //input maps see note above
reg [`inputWeightWidth-1:0] iw; //one channel kennel weights 5*5*8
reg [`DATA_SIZE-1:0] ib;

wire [`halfword-1:0] num1, num2, num3, num4;
wire covResVld;

mul25_4convolutor u_mul25_4convolutor(.clk(clk), .rst(rst), .imVld(imVld), .iwVld(iwVld), .im(im), .ib(ib), .iw(iw),
.num1(num1), .num2(num2), .num3(num3), .num4(num4), .covResVld(covResVld));

//****************covPool1********************//
reg conv1_en;
wire conv1_finish;
//weights bram
wire conv1_weightsBias_sram_ena;
wire [`weightRam_addra_width-1:0] conv1_weightsBias_sram_addra;
reg [`inputWeightWidth+`DATA_SIZE-1:0] conv1_weightsBias;

//feature bram
wire conv1_featureMap_sram_ena;
wire [0:0] conv1_featureMap_sram_wea;
wire [`mapRam_addra_width-1:0] conv1_featureMap_sram_addra;
wire [`inputMapWidth-1:0] conv1_optMap;
reg [`inputMapWidth-1:0] conv1_inputMap;

//interact with mull&add array
wire conv1_imVld, conv1_iwVld;
wire [`inputMapWidth-1:0] conv1_im;
wire [`inputWeightWidth-1:0] conv1_iw;
wire [`DATA_SIZE-1:0] conv1_ib;

reg conv1_covResVld;
reg [`halfword-1:0] conv1_num1, conv1_num2, conv1_num3, conv1_num4;
//module
convPool1 u_convPool1(
.clk(clk), .rst(rst),
.conv_1_en(conv1_en),.conv_1_finish(conv1_finish),
//**interact with convolution level weights&bias sram array
//data from 13 sram
.conv1_weightsBias(conv1_weightsBias),
//addra and ena
.conv1_weightsBias_sram_addra(conv1_weightsBias_sram_addra),
.conv1_weightsBias_sram_ena(conv1_weightsBias_sram_ena),
//**interact with feature map sram
//data from 18 sram
.conv1_inputMap(conv1_inputMap),
//data to 18 sram
.conv1_optMap(conv1_optMap),
//addra , ena ,wea
.conv1_featureMap_sram_addra(conv1_featureMap_sram_addra),
.conv1_featureMap_sram_ena(conv1_featureMap_sram_ena),
.conv1_featureMap_sram_wea(conv1_featureMap_sram_wea),
//**interact with convoluters
//data from convoluters
.convResVld(conv1_covResVld),
.conv1_num1(conv1_num1), .conv1_num2(conv1_num2), .conv1_num3(conv1_num3), .conv1_num4(conv1_num4), 
//data to convoluters
.imVld(conv1_imVld),.iwVld(conv1_iwVld),
.ib(conv1_ib),.im(conv1_im),.iw(conv1_iw)
);
//***********************ConvPool2**************************//
reg conv2_en;
wire conv2_finish;
//weights bram
wire conv2_weightsBias_sram_ena;
wire [`weightRam_addra_width-1:0] conv2_weightsBias_sram_addra;
reg [`inputWeightWidth+`DATA_SIZE-1:0] conv2_weightsBias;
//feature bram
wire conv2_featureMap_sram_ena;
wire [`mapRam_addra_width-1:0] conv2_featureMap_sram_addra;
reg [`inputMapWidth-1:0] conv2_inputMap;
//interact with mull&add array
wire conv2_imVld, conv2_iwVld;
wire [`inputMapWidth-1:0] conv2_im;
wire [`inputWeightWidth-1:0] conv2_iw;

reg conv2_covResVld;
reg [`halfword-1:0] conv2_num1, conv2_num2, conv2_num3, conv2_num4;
//***conv2 level results 50 points
wire [399:0] convPool2Res;

//module
convPool2 u_convPool2(
.clk(clk), .rst(rst),
.conv_2_en(conv2_en),.conv_2_finish(conv2_finish),
//**interact with convolution level weights&bias sram array
//data from 13 sram
.conv2_weightsBias(conv2_weightsBias),
//addra and ena
.conv2_weightsBias_sram_addra(conv2_weightsBias_sram_addra),
.conv2_weightsBias_sram_ena(conv2_weightsBias_sram_ena),
//**interact with feature map sram
//data from 18 sram
.conv2_inputMap(conv2_inputMap),
//addra , ena ,wea
.conv2_featureMap_sram_addra(conv2_featureMap_sram_addra),
.conv2_featureMap_sram_ena(conv2_featureMap_sram_ena),
//**interact with convoluters
//data from convoluters
.convResVld(conv2_covResVld),
.conv2_num1(conv2_num1), .conv2_num2(conv2_num2), .conv2_num3(conv2_num3), .conv2_num4(conv2_num4), 
//data to convoluters
.imVld(conv2_imVld),.iwVld(conv2_iwVld),
.im(conv2_im),.iw(conv2_iw),
//**results to fc
.convPool2Res(convPool2Res)
);

//************************FC*************************//
reg fc_en;
reg [`inputWeightWidth+`DATA_SIZE-1:0] fc_weightsBias;

wire fc_finish;
wire [`weightRam_addra_width-1:0] fc_weightsBias_sram_addra;
wire fc_weightsBias_sram_ena;


fc u_fc(
.clk(clk),.rst(rst),
.fc_en(fc_en),
.fc_finish(fc_finish),

//**interact with feature map sram
//data from 18 sram
.fc_weightsBias(fc_weightsBias),
//addra , ena
.fc_weightsBias_sram_addra(fc_weightsBias_sram_addra),
.fc_weightsBias_sram_ena(fc_weightsBias_sram_ena),
//**interact with conv2 results
.conv2_results(convPool2Res),
//**result
.classResult(lenet_res)
);



//************************state mechine*************************//
reg [3:0] state, next_state;
parameter
S_IDLE = 4'b1000,
S_CONV_1 = 4'b0100,
S_CONV_2 = 4'b0010,
S_FC     = 4'b0001;

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
            S_IDLE: next_state = (start == 1)?S_CONV_1:S_IDLE;
            S_CONV_1: next_state = (conv1_finish == 0)?S_CONV_1:S_CONV_2;
            S_CONV_2: next_state = (conv2_finish == 0)?S_CONV_2:S_FC;
            S_FC: next_state = (fc_finish == 0)?S_FC:S_IDLE;
            default:next_state = S_IDLE;
        endcase
    end
end
//
always@(posedge clk or posedge rst) begin
    if(rst == 1) begin
        lenet_finish <= 0;
        //S_CONV1
            conv1_en <= 0;
        //S_CONV2
            conv2_en <= 0;
        //S_FC
            fc_en <= 0;
        
    end
    else begin
        case(state)
            S_IDLE:
            begin
                if(fc_finish == 1)begin
                    lenet_finish <= 1;
                end
            end
            //******
            S_CONV_1:
            begin
                if(conv1_finish == 0)begin
                    conv1_en <= 1;
                end
                else begin
                    conv1_en <= 0;
                end
            end
            //*****
            S_CONV_2:
            begin
                if(conv2_finish == 0)begin
                    conv2_en <= 1;
                end
                else begin
                    conv2_en <= 0;
                end
            end
            //****
            S_FC:
            begin
                if(fc_finish == 0)begin
                    fc_en <= 1;
                end
                else begin
                    fc_en <= 0;
                end
            end
        endcase
    end
end


//**********************Schedule routing************************//
//weights ram
always@(*)begin
    if(rst == 1) begin 
        //lenet top param
        weightsBias_sram_ena = 0;weightsBias_sram_addra = 0;
        //conv1 reg
        conv1_weightsBias = 0;
        //conv2 reg
        conv2_weightsBias = 0;
        //fc reg
        fc_weightsBias = 0;
    end
    else begin
        case(state)
            S_CONV_1:
            begin
                //ram addra & ena
                weightsBias_sram_ena = conv1_weightsBias_sram_ena;
                weightsBias_sram_addra = conv1_weightsBias_sram_addra;
                //conv1 reg
                conv1_weightsBias = weightsBias;
            end
            S_CONV_2:
            begin
                //ram addra & ena
                weightsBias_sram_ena = conv2_weightsBias_sram_ena;
                weightsBias_sram_addra = conv2_weightsBias_sram_addra;
                //conv2 reg
                conv2_weightsBias = weightsBias;
            end
            S_FC:
            begin
                //ram addra & ena
                weightsBias_sram_ena = fc_weightsBias_sram_ena;
                weightsBias_sram_addra = fc_weightsBias_sram_addra;
                //fc reg
                fc_weightsBias = weightsBias;
            end
            default:
            begin
                //lenet top param
                weightsBias_sram_ena = 0;weightsBias_sram_addra = 0;
                //conv1 reg
                conv1_weightsBias = 0;
                //conv2 reg
                conv2_weightsBias = 0;
                //fc reg
                fc_weightsBias = 0;
            end
        endcase
    end
end

//feature ram
always@(*)begin
    if(rst == 1) begin 
        //lenet top param
        featureMap_sram_ena = 0; featureMap_sram_wea = 0;
        featureMap_sram_addra = 0;
        featureMap_sram_din = 0;
        //conv1 reg
        conv1_inputMap = 0;
        //conv2 reg
        conv2_inputMap = 0;
    end
    else begin
        case(state)
            S_CONV_1:
            begin
                //ram addra & ena
                featureMap_sram_ena = conv1_featureMap_sram_ena;
                featureMap_sram_wea = conv1_featureMap_sram_wea;
                featureMap_sram_addra = conv1_featureMap_sram_addra;
                //conv1 reg
                conv1_inputMap = featureMap_sram_douta;
                //ram reg din
                featureMap_sram_din = conv1_optMap;
            end
            S_CONV_2:
            begin
                //ram addra & ena
                featureMap_sram_ena = conv2_featureMap_sram_ena;
                featureMap_sram_wea = 0;
                featureMap_sram_addra = conv2_featureMap_sram_addra;
                //conv2 reg
                conv2_inputMap = featureMap_sram_douta;
            end
            default:
            begin
                //lenet top param
                featureMap_sram_ena = 0; featureMap_sram_wea = 0;
                featureMap_sram_addra = 0;
                featureMap_sram_din = 0;
                //conv1 reg
                conv1_inputMap = 0;
                //conv2 reg
                conv2_inputMap = 0;
            end
        endcase
    end
end

//convolutors
 always@(*)
    begin
        if (rst == 1'b1)begin
            //module reg
            im=0; ib=0; iw=0;
            imVld=0; iwVld = 0;
            //conv1
            conv1_covResVld = 0; 
            conv1_num1 = 0; conv1_num2 = 0; conv1_num3 = 0; conv1_num4 = 0;
            //conv2
            conv2_covResVld = 0; 
            conv2_num1 = 0; conv2_num2 = 0; conv2_num3 = 0; conv2_num4 = 0;
        end
        else begin
            case(state)
                S_CONV_1:
                begin
                    //to convolutors
                    im=conv1_im; ib=conv1_ib; iw=conv1_iw;
                    imVld=conv1_imVld; iwVld=conv1_iwVld;
                    //to conv level
                    conv1_covResVld = covResVld; 
                    conv1_num1 = num1; conv1_num2 = num2; conv1_num3 = num3; conv1_num4 = num4;
                end
                S_CONV_2:
                begin
                    //to convolutors
                    im=conv2_im; ib=0; iw=conv2_iw;
                    imVld=conv2_imVld; iwVld=conv2_iwVld;
                    //to conv level
                    conv2_covResVld = covResVld; 
                    conv2_num1 = num1; conv2_num2 = num2; conv2_num3 = num3; conv2_num4 = num4;
                end
                default:
                begin
                    //module reg
                        im=0; ib=0; iw=0;
                        imVld=0; iwVld = 0;
                        //conv1
                        conv1_covResVld = 0; 
                        conv1_num1 = 0; conv1_num2 = 0; conv1_num3 = 0; conv1_num4 = 0;
                        //conv2
                        conv2_covResVld = 0; 
                        conv2_num1 = 0; conv2_num2 = 0; conv2_num3 = 0; conv2_num4 = 0;
                end
            endcase
        end
    end
endmodule

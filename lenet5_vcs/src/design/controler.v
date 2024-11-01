`timescale 1ns/1ps

`define inputWeightWidth 5*5*8
`define inputMapWidth 6*6*8
`define DATA_SIZE 8  

`define mapRam_addra_width 9
`define weightRam_addra_width 11 

module controler(
    input clk,rst,
    //control
    input inference_start,
    //to uart
    output reg controller2Uart_ena,
    output reg [7:0] controller2Uart_data,
    //from uart
    input input_uart_ena,
    input [`inputMapWidth-1:0] input_uart_data,
    output reg uart_done_led,
    //from flash
    output reg init_done_led,
    output reg flash_start,
    input input_flash_ena,
    input [`inputWeightWidth+`DATA_SIZE-1:0] input_flash_data
);

//***************Sram Array******************//
//weight ram
reg weightsBias_sram_ena, weightsBias_sram_wea;
reg [`weightRam_addra_width-1:0] weightsBias_sram_addra;
wire [`inputWeightWidth+`DATA_SIZE-1:0] weightsBias;
reg [`inputWeightWidth+`DATA_SIZE-1:0] weightsBias_load;//from external bram
//map ram
reg featureMap_sram_ena, featureMap_sram_wea;
reg [`mapRam_addra_width-1:0] featureMap_sram_addra;
reg [`inputMapWidth-1:0] featureMap_sram_din;//from conv1 to sram ;or from uart
wire [`inputMapWidth-1:0] featureMap_sram_douta; // from sram to lenet

sramArray u_sramArray(
.clk(clk),.rst(rst),
//weights&bias ram
.weightsBias_sram_ena(weightsBias_sram_ena),
.weightsBias_sram_wea(weightsBias_sram_wea),
.weightsBias_sram_addra(weightsBias_sram_addra),
.weightsBias(weightsBias),
.weightsBias_load(weightsBias_load),
//feature map ram
.featureMap_sram_ena(featureMap_sram_ena),
.featureMap_sram_wea(featureMap_sram_wea),
.featureMap_sram_addra(featureMap_sram_addra),
.featureMap_sram_din(featureMap_sram_din),
.featureMap_sram_douta(featureMap_sram_douta)
);

//*********************Lenet***********************//
//control signal
reg ctrl_rst;
reg start;
//weight
reg [`inputWeightWidth+`DATA_SIZE-1:0] net_weightsBias;
wire  net_weightsBias_sram_ena;
wire  [`weightRam_addra_width-1:0] net_weightsBias_sram_addra;
//map
reg [`inputMapWidth-1:0] net_inputMap;
wire net_featureMap_sram_ena, net_featureMap_sram_wea;
wire [`mapRam_addra_width-1:0] net_featureMap_sram_addra;
wire [`inputMapWidth-1:0] net_optMap;
//net result signal
wire [`DATA_SIZE-1:0] lenet_res;
wire lenet_finish;

lenetTop u_lenet(
    //**input
        .clk(clk), .sys_rst(rst),
        .ctrl_rst(ctrl_rst),//reset from controler
        .start(start),
        //data from sram pass controler
        .weightsBias(net_weightsBias),
        .featureMap_sram_douta(net_inputMap),
    //**output
        //to sram
        .weightsBias_sram_ena(net_weightsBias_sram_ena),
        .weightsBias_sram_addra(net_weightsBias_sram_addra),
        .featureMap_sram_ena(net_featureMap_sram_ena), 
        .featureMap_sram_wea(net_featureMap_sram_wea),
        .featureMap_sram_addra(net_featureMap_sram_addra),
        .featureMap_sram_din(net_optMap),
        //to controler
        .lenet_res(lenet_res),
        .lenet_finish(lenet_finish)
);

/////////////////////////////////////////////////////
//*******************State Mechine****************//
reg [2:0] cycle;
//S_INIT param
reg initState_finish;
reg [`weightRam_addra_width-1:0] flash_load_cnt; //indexs of sram and bram are same
reg flash_data_vld;

reg flash_weightsBias_sram_ena;
reg [`weightRam_addra_width-1:0] flash_weightsBias_sram_addra;
reg [`inputWeightWidth+`DATA_SIZE-1:0]flash_weightsBias_load;


//S_LOAD_UART param
reg loadUart_finish;
reg uart_featureMap_sram_ena;
reg uart_data_vld;
reg [`mapRam_addra_width-1:0] uart_featureMap_sram_addra;
reg [`inputMapWidth-1:0] uart_map;
reg [7:0] uart_load_cnt;

//S_LENET
reg lenetState_finish;

//state param
reg [2:0] state, next_state;
parameter 
S_INIT = 3'b100,
S_LOAD_UART = 3'b010,
S_LENET = 3'b001;


always@(posedge clk or posedge rst) begin
    if(rst == 1) begin
        state <= S_INIT;
    end
    else begin
        state <= next_state;
    end
end

always@(*) begin
    if(rst == 1) begin
        next_state = S_INIT;
    end
    else begin
        case(state)
            S_INIT:next_state = (initState_finish == 0)?S_INIT:S_LOAD_UART;
            S_LOAD_UART:next_state = (loadUart_finish == 0)?S_LOAD_UART:(inference_start == 0)?S_LOAD_UART:S_LENET;
            S_LENET:next_state = (lenetState_finish == 0)?S_LENET:S_LOAD_UART;
            default: next_state = S_INIT;
        endcase
    end
end

always@(posedge clk or posedge rst) begin
    if(rst == 1) begin
        //**output reg
            //lenet 
                //ctrl sig
                ctrl_rst <= 0;
                start <= 0;
            //2uart
                controller2Uart_data <= 0;
                controller2Uart_ena <= 0;
                uart_done_led <= 0;
            //flash
                flash_start <= 0;
                init_done_led <= 0;
        //**state reg
            cycle <= 0;
            //S_INIT
            flash_load_cnt <= 0;
            initState_finish <= 0;
            flash_data_vld <= 0;
            flash_weightsBias_sram_ena <= 0;
            flash_weightsBias_sram_addra <= 0;
            flash_weightsBias_load <= 0;
            //S_LENET
            lenetState_finish <= 0;
            //S_LOAD_UART
            loadUart_finish <= 0;
            uart_featureMap_sram_ena <= 0;
            uart_featureMap_sram_addra <= 0;
            uart_map <= 0;
            uart_load_cnt <= 0;
            uart_data_vld <= 0;
    end
    else begin
        case(state)
            S_INIT:
            begin
                if(initState_finish == 0)begin
                    //enable external flash
                    if(cycle == 0) begin
                        cycle <= cycle + 1;
                        flash_start <= 1;
                    end
                    else if(cycle == 1) begin
                        cycle <= cycle + 1;
                        flash_start <= 0;
                    end
                    if(flash_load_cnt < 1040) begin
                        if(input_flash_ena == 1) begin
                            flash_data_vld <= 1;
                            flash_weightsBias_load <= input_flash_data;
                        end
                        else if(flash_data_vld == 1) begin
                            if(cycle == 2) begin
                                flash_weightsBias_sram_ena <= 1;
                                flash_weightsBias_sram_addra <= flash_load_cnt;
                                cycle <= cycle + 1;
                            end
                            else if(cycle == 4) begin
                                flash_data_vld <= 0;
                                flash_weightsBias_sram_ena <= 0;
                                flash_load_cnt <= flash_load_cnt + 1;
                                cycle <= 2;
                            end
                            else cycle <= cycle + 1;
                        end
                    end
                    else if(cycle == 2) begin
                        cycle <= cycle + 1;
                        //init ready, send mess to uart
                        controller2Uart_ena <= 1; // will be H one clk
                        controller2Uart_data <= 8'hFA;
                    end
                    else if(cycle == 3) begin
                        cycle <= 0;
                        controller2Uart_ena <= 0;
                        controller2Uart_data <= 0;
                        //init state finish
                        initState_finish <= 1;
                        init_done_led <= 1;
                    end
                end
            end
            //**
            S_LOAD_UART:
            begin
                if(loadUart_finish == 0)begin
                    if(uart_load_cnt < 144) begin
                        if(input_uart_ena == 1) begin
                            uart_data_vld <= 1;
                            uart_map <= input_uart_data;
                        end
                        else if(uart_data_vld == 1) begin
                            if(cycle == 0) begin
                                uart_featureMap_sram_ena <= 1;
                                uart_featureMap_sram_addra <= uart_load_cnt;
                                cycle <= cycle + 1;
                            end
                            else if(cycle == 1) begin
                                uart_data_vld <= 0;
                                uart_featureMap_sram_ena <= 0;
                                uart_load_cnt <= uart_load_cnt + 1;
                                cycle <= 0;
                            end
                        end
                    end
                    else if(cycle == 0)begin
                        cycle <= cycle + 1;
                        //send mess to uart "11" which means map have been loaded
                        controller2Uart_data <= 8'hFB;
                        controller2Uart_ena <= 1;
                    end
                    else if(cycle == 1) begin
                        cycle <= 0;
                        //reset param
                        uart_load_cnt <= 0;
                        controller2Uart_ena <= 0; //have been H one clk
                        //load uart map state finish
                        loadUart_finish <= 1;
                        uart_done_led <= 1;
                        //enable lenet inference
                        lenetState_finish <= 0;
                    end
                end
            end
            //**
            S_LENET:
            begin
                if(lenetState_finish == 0)begin
                    if(cycle == 0) begin
                        ctrl_rst <= 1;
                        cycle <= cycle + 1;
                    end
                    else if(cycle == 1) begin
                        ctrl_rst <= 0;
                        start <= 1;
                        cycle <= cycle + 1;
                    end
                    else if(cycle == 2) begin
                        start <= 0;
                        cycle <= cycle + 1;
                    end
                    if(lenet_finish == 1) begin
                        if(cycle == 3) begin        
                            cycle <= cycle + 1;
                            //output result
                            controller2Uart_ena <= 1;//inference finish will be H 1 clk
                            controller2Uart_data <= lenet_res;
                            
                        end
                        else if(cycle == 4) begin
                            //inference finish
                            lenetState_finish <= 1;
                            // enable S_UART_LOAD
                            loadUart_finish <= 0;
                            uart_done_led <= 0;
                            //reset param
                            controller2Uart_ena <= 0;
                            cycle <= 0;
                        end
                    end
                end
            end
        endcase
    end
end

//*************Shcedule Lines***********//
//weight ram
always@(*) begin
    if(rst == 1) begin
        //sram param
        weightsBias_sram_ena = 0;
        weightsBias_sram_wea = 0;
        weightsBias_sram_addra = 0;
        weightsBias_load = 0;
        //lenet param
        net_weightsBias = 0;
    end
    else begin
        case(state) 
        S_INIT:
        begin
            weightsBias_sram_ena = flash_weightsBias_sram_ena;
            weightsBias_sram_wea = 1;//wright only
            weightsBias_sram_addra = flash_weightsBias_sram_addra;
            weightsBias_load = flash_weightsBias_load;
        end
        S_LENET:
        begin
            weightsBias_sram_ena = net_weightsBias_sram_ena;
            weightsBias_sram_wea = 0;//read only
            weightsBias_sram_addra = net_weightsBias_sram_addra;
            net_weightsBias = weightsBias;
        end
        default:
        begin
            //sram param
            weightsBias_sram_ena = 0;
            weightsBias_sram_wea = 0;
            weightsBias_sram_addra = 0;
            weightsBias_load = 0;
            //lenet param
            net_weightsBias = 0;
        end
        endcase
    end
end

//map ram
always@(*) begin
    if(rst == 1) begin
        //map ram
        featureMap_sram_ena = 0;
        featureMap_sram_wea = 0;
        featureMap_sram_addra = 0;
        net_inputMap = 0;
        featureMap_sram_din = 0;
        //lenet
        
    end
    else begin
        case(state)
            S_LOAD_UART:
            begin
                featureMap_sram_ena = uart_featureMap_sram_ena;
                featureMap_sram_wea = 1;
                featureMap_sram_addra = uart_featureMap_sram_addra;
                featureMap_sram_din = uart_map;
            end
            S_LENET:
            begin
                featureMap_sram_ena = net_featureMap_sram_ena;
                featureMap_sram_wea = net_featureMap_sram_wea;
                featureMap_sram_addra = net_featureMap_sram_addra;
                featureMap_sram_din = net_optMap;
                net_inputMap = featureMap_sram_douta;
            end
        default:
        begin
            //map ram
            featureMap_sram_ena = 0;
            featureMap_sram_wea = 0;
            featureMap_sram_addra = 0;
            featureMap_sram_din = 0;
            //lenet
            net_inputMap = 0;
        end
        endcase
    end
end
endmodule

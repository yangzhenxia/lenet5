`timescale 1ns / 1ps

`define halfword 16
`define DATA_SIZE 8

`define inputWeightWidth 5*5*8 

`define weightRam_addra_width 11

module  fc(
        input clk,rst,
        input fc_en,
        output reg fc_finish,

        //**interact with feature map sram
            //data from 18 sram
            input [`inputWeightWidth+`DATA_SIZE-1:0] fc_weightsBias,
            //addra , ena
            output  reg [`weightRam_addra_width-1:0] fc_weightsBias_sram_addra,
            output reg fc_weightsBias_sram_ena,

        //**interact with conv2 results
            input [399:0] conv2_results,
        //**output result
            output reg [7:0] classResult
        );
//****************FC Mac Array Module *********************//
reg [399:0] iw;
reg [`DATA_SIZE-1:0] ib;
reg iwVld;

wire [`DATA_SIZE-1:0] fc_res;
wire covResVld;

fcMacArray u_fcMacArray(
.clk(clk),
.rst(rst),

.weights(iw),
.bia(ib),
.iwVld(iwVld),

.imaps(conv2_results),

.covSum(fc_res),
.covResVld(covResVld)
);
//**********************Classify Module*****************//
reg classify_en;
reg [10*`DATA_SIZE-1:0] result;//fc result 10points
//output param
wire classify_res_vld;
wire [`DATA_SIZE-1:0] classify_res; 
fc_classify u_fc_classify(
    //input
    .clk(clk), .rst(rst), 
    .classify_en(classify_en), 
    .fc_result(result), 
    //output
    .classify_res_vld(classify_res_vld), 
    .classify_res(classify_res));

//******************state mechine**************//
reg [2:0] cycle;
//S_FC param
reg fcState_finish;
reg [3:0] resCnt; //0~10

//S_LOAD_WB param
parameter 
fc_weightsBias_base = 1020;
reg [4:0] fc_weightsBias_index;
reg [1:0] loadCnt;
//S_CLASSIFY param
reg classifyState_finish;



reg [3:0] state, next_state;
parameter 
S_IDLE = 4'b1000,
S_LOAD_WB = 4'b0100,
S_FC = 4'b0010,
S_CLASSIFY = 4'b0001;

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
        S_IDLE:next_state = (fc_en == 0)?S_IDLE:(resCnt == 10)?S_IDLE:S_LOAD_WB;
        S_LOAD_WB:next_state = (iwVld == 0)?S_LOAD_WB:S_FC;
        S_FC:next_state = (fcState_finish == 0)?S_FC:(resCnt < 10)?S_LOAD_WB:S_CLASSIFY; 
        S_CLASSIFY:next_state = (classifyState_finish == 0)?S_CLASSIFY:S_IDLE;
        default:next_state = S_IDLE;
        endcase
    end
end


//S_IDLE
always@(posedge clk or posedge rst) begin
    if (rst == 1) begin
        //S_IDLE
        fc_finish <= 0;
        //S_LOAD_WB
            iwVld <= 0;iw <= 0;
            ib <= 0;
            fc_weightsBias_sram_addra <= 0;
            fc_weightsBias_index <= 0;
            cycle <= 0;
            loadCnt <= 0;
            fc_weightsBias_sram_ena <= 0;
        //S_FC
            fcState_finish <= 0;
            resCnt <= 0;
            result <= 0;
        //S_CLASSIFY
            classResult <= 0;
            classifyState_finish <= 0;
            classify_en <= 0;
    end
    else begin
        case(state)
            S_IDLE:
            begin
                if(resCnt == 10) begin
                    fc_finish <= 1;
                end
            end
            //*****
            S_LOAD_WB:
            begin
                if(iwVld == 0) begin
                    if(loadCnt < 2) begin
                        if(cycle == 0) begin
                            fc_weightsBias_sram_ena <= 1;
                            fc_weightsBias_sram_addra <= fc_weightsBias_base + fc_weightsBias_index;

                            cycle <= cycle + 1;
                        end
                        else if(cycle == 2) begin
                            fc_weightsBias_sram_ena <= 0;
                            fc_weightsBias_index <= fc_weightsBias_index + 1;

                            iw <= {iw[199:0], fc_weightsBias[`inputWeightWidth+`DATA_SIZE-1:`DATA_SIZE]};
                            ib <= fc_weightsBias[`DATA_SIZE-1:0];
                            loadCnt <= loadCnt + 1;
                            cycle <= 0;
                        end
                        else begin
                            cycle <= cycle + 1;
                        end
                    end
                    else begin
                        loadCnt <= 0;
                        iwVld <= 1;
                        fcState_finish <= 0;
                    end
                end
            end
        //******
        S_FC:
        begin
            if(fcState_finish == 0) begin
                if(covResVld == 1) begin
                    result <= {result[71:0],fc_res};
                    resCnt <= resCnt + 1;
                    iwVld  <= 0;
                    fcState_finish <= 1;
                end
            end
        end
        //****
        S_CLASSIFY:
        begin
            if(classifyState_finish == 0) begin
                if(classify_res_vld == 0) begin
                    classify_en <= 1;
                end
                else begin
                    classify_en <= 0;
                    classResult <= classify_res;
                    classifyState_finish <= 1;
                end
            end
        end
        endcase
    end
end
endmodule

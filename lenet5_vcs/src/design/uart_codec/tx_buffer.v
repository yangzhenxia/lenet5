/*
 * @Author: Wang Yuanhao
 * @Date: 2022-09-16 23:25:11
 * @LastEditTime: 2022-09-25 18:51:31
 * @LastEditors: Wang Yuanhao
 * @Description: transporter encoder(trans 128 bit data to 8 bit data)
 * @FilePath: \rtl\tx_buffer.v
 * Change history:
 */
module tx_buffer(
    input clk,
    input rstn,
    input out_en,
    input [127:0] out_data,
    output free,
    input tx_end,
    output reg  tx_en,
    output reg [7:0] data_tx
);
    localparam IDLE=0,
               RECEIVE=1,
               SEND=2,
               WAIT=3;

    reg [1:0] cur_st;
    reg [1:0] nxt_st;
    reg [3:0] count;

    always@(posedge clk or negedge rstn) begin
        if(!rstn) begin
            cur_st<=0;
        end
        else begin
            cur_st<=nxt_st;
        end
    end


    always@(*) begin
        nxt_st=cur_st;
        case(cur_st)
            IDLE: if(out_en) nxt_st=RECEIVE;
            RECEIVE: nxt_st=SEND;
            SEND: nxt_st=WAIT;
            WAIT: nxt_st=(tx_end? (count==15? IDLE: SEND): WAIT);
            default: nxt_st=IDLE;
        endcase
    end

    always@(posedge clk or negedge rstn) begin
        if(!rstn) begin
            count<=0;
        end
        else begin
            if(tx_end) begin
                count<=count+1;
            end
        end
    end

    reg [127:0]out_data_buffer;
    always@(posedge clk or negedge rstn) begin
        if(!rstn) begin
            out_data_buffer<=0;
        end
        else begin
            if(nxt_st==RECEIVE) begin
                out_data_buffer<=out_data;
            end
        end
    end


    always@(posedge clk or negedge rstn) begin
        if(!rstn) begin
            data_tx<=0;
        end
        else begin
            data_tx<=out_data_buffer[127-count*8-:8];
        end
    end

    always@(posedge clk or negedge rstn) begin
        if(!rstn) begin
            tx_en<=0;
        end
        else begin
            tx_en<=(nxt_st==SEND);
        end
    end


//    always@(posedge clk or negedge rstn) begin
//        if(!rstn) begin
//            free<=0;
//        end
//        else begin
//            free<=(nxt_st==IDLE);
//        end
//    end
    
    assign free = (cur_st==IDLE)&&!out_en;      // need to be advanced for one cycle!

endmodule

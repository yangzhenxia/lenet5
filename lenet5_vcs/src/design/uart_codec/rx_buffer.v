/*
 * @Author: Wang Yuanhao
 * @Date: 2022-09-16 22:28:34
 * @LastEditTime: 2022-09-25 18:46:36
 * @LastEditors: Wang Yuanhao
 * @Description: receiver decoder(trans 8 bit data to 128 bit data)
 * @FilePath: \rtl\rx_buffer.v
 * Change history:
 */
 `define inputMapWidth 6*6*8
 
module rx_buffer(
    input clk,
    input rstn,
    input rx_en,
    input [7:0] data_rx,
    output reg wr_en,
    output reg [`inputMapWidth-1:0]wr_data
);
    localparam inputMapWord = 'd36-'d1;

    localparam IDLE=0,
               SPLICE=1,
               SEND=2;

    reg [2:0] cur_st;
    reg [2:0] nxt_st;
    reg [5:0] count;
    
    // ila_2 u_ila_2(
    //     .clk(clk),
    //     .probe0(cur_st),
    //     .probe1(count)
    // );

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
            IDLE: if(rx_en) nxt_st=SPLICE;
            SPLICE: if(count==inputMapWord) nxt_st=SEND;
            SEND: nxt_st=IDLE;
            default: nxt_st=IDLE;
        endcase
    end

    always@(posedge clk or negedge rstn) begin
        if(!rstn) begin
            count<=0;
            wr_data<=0;
        end
        else begin
            if(rx_en&&(cur_st==IDLE||cur_st==SPLICE)) begin
                if(count==inputMapWord) begin
                    count <= 'd0;
                end
                else begin
                    count<=count+1;
                    wr_data[(`inputMapWidth-1-count*8)-:8]<=data_rx;                           //when rx_en==1, collect data_rx
                end
            end
            
        end
    end

    always@(posedge clk or negedge rstn) begin
        if(!rstn) begin
            wr_en<=0;
        end
        else begin
            wr_en<=(cur_st==SEND);
        end
    end

endmodule

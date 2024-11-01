/*
 * @Author: Wang Yuanhao
 * @Date: 2022-08-12 18:58:36
 * @LastEditTime: 2022-09-25 18:53:14
 * @LastEditors: Wang Yuanhao
 * @Description: top layer
 * @FilePath: \rtl\uart_codec_top.v
 * Change history:
 */

module uart_codec_top#(
    parameter  UART_BPS=9600
)(
    input [31:0]clk_freq,
    input clk,
    input rstn,
    input rxd,
    output txd,
    //output free,
    output wr_en,
    output [287:0]wr_data,
    input out_en,
    input [7:0] out_data,
    output  reg out_free
);

    wire rx_en;
    wire [7:0]data_rx;
    wire tx_end;
    
    // ila_1 u_ila_1(
    //     .clk(clk),
    //     .probe0(rx_en),
    //     .probe1(data_rx),
    //     .probe2(wr_en),
    //     .probe3(wr_data)
    // );


    uart_rx#(
        .UART_BPS(UART_BPS)    //串口波特�?
 //       .CLK_FREQ(CLK_FREQ)    //时钟频率
    ) u_uart_rx(
	.clk_freq(clk_freq),
        .clk(clk),          //系统时钟40MHz
        .rstn(rstn),        //全局复位
        .rxd(rxd),          //串口接收数据

        .data_rx(data_rx),  //串转并后�?8bit数据
        .rx_en(rx_en)       //串转并后的数据有效标志信�?
    );

    rx_buffer u_rx_buffer(
        .clk(clk),
        .rstn(rstn),
        .rx_en(rx_en),
        .data_rx(data_rx),
        .wr_en(wr_en),
        .wr_data(wr_data)
);


    uart_tx#(
        .UART_BPS(UART_BPS)    //串口波特�?
//        .CLK_FREQ(CLK_FREQ)    //时钟频率
    )u_uart_tx(
	.clk_freq(clk_freq),
        .clk(clk),
        .rstn(rstn),
        .data_tx(out_data),
        .txd(txd),
        .tx_en(out_en),
        .tx_end(tx_end)
    );
    
    
    always@(posedge clk or negedge rstn) begin
        if(!rstn) begin
            out_free <= 'd1;
        end
        else begin
            if(out_en) out_free <= 'd0;
            else if(tx_end) out_free <= 'd1;
        end
    end

endmodule

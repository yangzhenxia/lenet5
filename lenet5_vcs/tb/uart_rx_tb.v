/*
 * @Author: Wang Yuanhao
 * @Date: 2022-09-17 09:44:33
 * @LastEditTime: 2022-11-16 23:54:40
 * @LastEditors: Wang Yuanhao
 * @Description: uart receiver testbench
 * @FilePath: \undefinedc:\Users\11580\Desktop\Lenet_Fukami_competition\tb\uart_rx_tb.v
 * Change history:
 */
`timescale 1ns/1ps
module uart_rx_tb();
    reg     clk200M;
    reg     rstn;
    reg     key;
    reg     rxd;
    wire    txd;
    wire    free;
    wire    init_done_led;
    wire    uart_done_led;
    wire    miso;
    wire    sck;
    wire    cs_n;
    wire    mosi;
    

    wire    [31:0]clk_freq;
    assign clk_freq = 'd200_000_000;    //ʱ��Ƶ��
    parameter UART_BPS = 'd20_000_00;          //������
    
    localparam  BPS_CNT = 'd200_000_000/UART_BPS;
    
    fpga_top_with_uart#(
        .UART_BPS(UART_BPS)
    )u_fpga_top_with_uart(
    //control signal
        .clk200M(clk200M),
    // input fxdla_clk,
        .rstn(rstn),
        .key(key),

    //UART Interface
        .rxd(rxd),
        .txd(txd),

    //internal signal
        .free(free),  //free=1 ---> UART_TX is not working
        .init_done_led(init_done_led),
        .uart_done_led(uart_done_led),
    
    //flash interface
        .miso(miso),
        .sck(sck),
        .cs_n(cs_n),
        .mosi(mosi)
    );
    
    wire    WPn;
    wire    HOLDn;
    
    assign WPn = 1'd1;
    assign HOLDn = 1'd1;
    
    W25Q128JVxIM u_W25Q128JVxIM(
        .CSn(cs_n),
        .CLK(sck),
        .DIO(mosi),
        .DO(miso),
        .WPn(WPn), 
        .HOLDn(HOLDn)
    );

    reg [7:0] rx_data_temp [0:10000];
    reg [9:0] rx_data [0:10000];

    integer i;
    integer j;
    
    always #2.5 clk200M<=~clk200M;

    initial begin
                clk200M<=0; rstn<=0; rxd<=1; key<=1;
		#200    rstn<=1;
        #100_000_000 ;
	/******************************************uart_input****************************************/
        $readmemh("../tb/data_txt/num7_uart.txt",rx_data_temp);
        for(i=0;i<5183;i=i+1) begin
            rx_data[i]<={1'b1,rx_data_temp[i],1'b0};
        end
        for(i=0;i<5183;i=i+1) begin
            for(j=0;j<=9;j=j+1) begin
                #(5*BPS_CNT) rxd<=rx_data[i][j];
            end
        end
        rxd<=1;
        #1000   key <= 0;
        #25_000_000 key<=1;
        #10_000_000 $stop();
    

    end

endmodule


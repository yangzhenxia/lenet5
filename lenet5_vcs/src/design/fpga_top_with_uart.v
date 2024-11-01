`define inputMapWidth 6*6*8
`timescale 1ns/1ps
module fpga_top_with_uart#(
    parameter UART_BPS = 'd115200          //������
)(
    //control signal
     input  clk200M,
    // input fxdla_clk,
    input rstn,
    input key,

    //UART Interface
    input rxd,
    output txd,

    //internal signal
    output free,  //free=1 ---> UART_TX is not working
    output init_done_led,
    output wire uart_done_led,
    
    //flash interface
    input                   miso,
    output                  sck,
    output                  cs_n,
    output                  mosi

);
    wire key_flag;
    key_filter#(
        .CNT_MAX(22'd3_999_999) //计数器计数最大�??
    )u_key_filter(
        .sys_clk(clk200M),   //系统时钟50Mhz
        .sys_rst_n(rstn),   //全局复位
        .key_in(key),   //按键输入信号
        .key_flag(key_flag)        //key_flag�?1时表示消抖后�?测到按键被按�?
                                    //key_flag�?0时表示没有检测到按键被按�?
    );
    
    //generate clk50M from clk200M
    reg [1:0]   clk_cnt;
    always@(posedge clk200M or negedge rstn) begin
        if(!rstn) begin
            clk_cnt <= 'd0;
        end
        else begin
            clk_cnt <= clk_cnt + 'd1;
        end
    end
    assign clk50M = clk_cnt[1];
    

    wire    wr_en;
    wire    [287:0] wr_data;
    wire    controller2Uart_ena;
    wire    [7:0] controller2Uart_data;
    wire    flash_start;

    assign  controller2NICE_data = controller2Uart_data;

    
    wire rst;
    wire [207:0] input_flash_data;
    wire [255:0] out_sram_data;
    wire out_sram_en_shrink;
    
    assign input_flash_data = out_sram_data[207:0];
    assign rst = ~rstn;
    
    controler u_controller(
        .clk(clk200M),
        .rst(rst),
        .inference_start(key_flag),
        .controller2Uart_ena(controller2Uart_ena),
        .controller2Uart_data(controller2Uart_data),
        .input_uart_ena(wr_en),
        .input_uart_data(wr_data),
        .init_done_led(init_done_led),
        .uart_done_led(uart_done_led),
        .flash_start(flash_start),//pull up one clk to make flash module start
        .input_flash_ena(out_sram_en_shrink),
        .input_flash_data(input_flash_data)
    );
    
    // turn flash_start to flash_start_ext
    wire    flash_start_ext;
    reg     flash_start_dly1;
    reg     flash_start_dly2;
    reg     flash_start_dly3;   

    always@(posedge clk200M or negedge rstn) begin
        if(!rstn) begin
            flash_start_dly1  <= 'd0;
            flash_start_dly2  <= 'd0;
            flash_start_dly3  <= 'd0;
        end
        else begin
            flash_start_dly1 <= flash_start;
            flash_start_dly2 <= flash_start_dly1;
            flash_start_dly3 <= flash_start_dly2;
        end
    end
    assign flash_start_ext = flash_start | flash_start_dly1 | flash_start_dly2 | flash_start_dly3;
    
    wire    [31:0]clk_freq;
    assign clk_freq = 'd200_000_000;    //ʱ��Ƶ��
    
    // wire    free;


    uart_codec_top#(
        .UART_BPS(UART_BPS)
    )u_uart_codec_top(
        .clk_freq(clk_freq),
        .clk(clk200M),
        .rstn(rstn),
        .rxd(rxd),
        .txd(txd),
        .wr_en(wr_en),
        .wr_data(wr_data),
        .out_en(controller2Uart_ena),   //�����������ʹ���ź�???
        .out_data(controller2Uart_data),//���������������ź�
        .out_free(free) //uart����״̬�����ź�,Ϊ1��ʾuart����
    );
    
    wire out_sram_en;
    
    data_loader_top u_data_loader_top(
        .sys_clk(clk50M),
        .sys_rst_n(rstn),
        .layer_load_start(flash_start_ext),
        .out_sram_en(out_sram_en),
        .out_sram_data(out_sram_data),
        .layer_load_finish(),

        .miso(miso),
        .sck(sck),
        .cs_n(cs_n),
        .mosi(mosi)
    );
    
    // turn out_sram_data to out_sram_en_shrink
    reg out_sram_en_dly;
    always@(posedge clk200M or negedge rstn) begin
        if(!rstn) begin
            out_sram_en_dly <= 'd0;
        end
        else begin
            out_sram_en_dly <= out_sram_en;
        end
    end
    assign out_sram_en_shrink = ~out_sram_en_dly & out_sram_en;
    

endmodule

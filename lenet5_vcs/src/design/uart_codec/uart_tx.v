////////////////////////////////////////////////////////////////////////
// Author        : EmbedFire
// Create Date   : 2019/06/12
// Module Name   : uart_tx
// Project Name  : rs232
// Target Devices: Altera EP4CE10F17C8N
// Tool Versions : Quartus 13.0
// Description   : 
// 
// Revision      : V1.0
// Additional Comments:
// 
// 实验平台: 野火_征�?�Pro_FPGA�?发板
// 公司    : http://www.embedfire.com
// 论坛    : http://www.firebbs.cn
// 淘宝    : https://fire-stm32.taobao.com
////////////////////////////////////////////////////////////////////////

module  uart_tx
#(
    parameter   UART_BPS    =   'd9600         //串口波特�?
)
(
     input   wire    [31:0]  clk_freq ,
     input   wire            clk     ,   //系统时钟40MHz
     input   wire            rstn   ,   //全局复位
     input   wire    [7:0]   data_tx     ,   //模块输入�?8bit数据
     input   wire            tx_en     ,   //并行数据有效标志信号
 
     output  reg             txd       ,       //串转并后�?1bit数据
     output  reg             tx_end
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//localparam    define
wire    [31:0]  baud_cnt_max ;
assign  baud_cnt_max    =   clk_freq/UART_BPS   ;

//reg   define
reg [12:0]  baud_cnt;
reg         bit_flag;
reg [3:0]   bit_cnt ;
reg         work_en ;

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
//work_en:接收数据工作使能信号
always@(posedge clk or negedge rstn)
        if(rstn == 1'b0)
            work_en <= 1'b0;
        else    if(tx_en == 1'b1)
            work_en <= 1'b1;
        else    if((bit_flag == 1'b1) && (bit_cnt == 4'd9))
            work_en <= 1'b0;

//baud_cnt:波特率计数器计数，从0计数到baud_cnt_max - 1
always@(posedge clk or negedge rstn)
        if(rstn == 1'b0)
            baud_cnt <= 13'b0;
        else    if((baud_cnt == baud_cnt_max - 1) || (work_en == 1'b0))
            baud_cnt <= 13'b0;
        else    if(work_en)
            baud_cnt <= baud_cnt + 1'b1;

//bit_flag:当baud_cnt计数器计数到1时让bit_flag拉高�?个时钟的高电�?
always@(posedge clk or negedge rstn)
        if(rstn == 1'b0)
            bit_flag <= 1'b0;
        else    if(baud_cnt == baud_cnt_max - 1)
            bit_flag <= 1'b1;
        else
            bit_flag <= 1'b0;

//bit_cnt:数据位数个数计数�?10个有效数据（含起始位和停止位）到来后计数器清�?
always@(posedge clk or negedge rstn)
    if(rstn == 1'b0)
        bit_cnt <= 4'b0;
    else    if((bit_flag == 1'b1) && (bit_cnt == 4'd9))
        bit_cnt <= 4'b0;
    else    if((bit_flag == 1'b1) && (work_en == 1'b1))
        bit_cnt <= bit_cnt + 1'b1;

//txd:输出数据在满足rs232协议（起始位�?0，停止位�?1）的情况下一位一位输�?
always@(posedge clk or negedge rstn)
        if(rstn == 1'b0) begin
            txd <= 1'b1; //空闲状�?�时为高电平
            tx_end <= 0;
        end
        else begin
            tx_end <= 0;
            if(bit_flag == 1'b1)
                case(bit_cnt)
                    0       : txd <= 1'b0;
                    1       : txd <= data_tx[0];
                    2       : txd <= data_tx[1];
                    3       : txd <= data_tx[2];
                    4       : txd <= data_tx[3];
                    5       : txd <= data_tx[4];
                    6       : txd <= data_tx[5];
                    7       : txd <= data_tx[6];
                    8       : txd <= data_tx[7];
                    9       : begin txd <= 1'b1; tx_end <= 1; end
                    default : txd <= 1'b1;
                endcase
        end

endmodule



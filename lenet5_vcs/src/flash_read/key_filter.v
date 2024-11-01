/*
 * @Author: Wang Yuanhao
 * @Date: 2022-10-17 23:03:58
 * @LastEditTime: 2022-10-17 23:05:59
 * @LastEditors: Wang Yuanhao
 * @Description: 
 * @FilePath: \flash_write\key_filter.v
 * Change history:
 */


module  key_filter
#(
    parameter CNT_MAX = 22'd3_999_999 //计数器计数最大�??
)
(
    input   wire    sys_clk     ,   //系统时钟50Mhz
    input   wire    sys_rst_n   ,   //全局复位
    input   wire    key_in      ,   //按键输入信号

    output  reg     key_flag        //key_flag�?1时表示消抖后�?测到按键被按�?
                                    //key_flag�?0时表示没有检测到按键被按�?
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//reg   define
reg     [21:0]  cnt_20ms    ;   //计数�?

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

//cnt_20ms:如果时钟的上升沿�?测到外部按键输入的�?�为低电平时，计数器�?始计�?
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        cnt_20ms <= 20'b0;
    else    if(key_in == 1'b1)
        cnt_20ms <= 20'b0;
    else    if(cnt_20ms == CNT_MAX && key_in == 1'b0)
        cnt_20ms <= cnt_20ms;
    else
        cnt_20ms <= cnt_20ms + 1'b1;

//key_flag:当计数满20ms后产生按键有效标志位
//且key_flag�?999_999时拉�?,维持�?个时钟的高电�?
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        key_flag <= 1'b0;
    else    if(cnt_20ms == CNT_MAX - 1'b1)
        key_flag <= 1'b1;
    else
        key_flag <= 1'b0;

endmodule

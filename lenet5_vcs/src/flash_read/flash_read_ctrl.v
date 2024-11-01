/*
 * @Author: Wang Yuanhao
 * @Date: 2022-10-13 10:04:41
 * @LastEditTime: 2022-10-13 16:15:26
 * @LastEditors: Wang Yuanhao
 * @Description: QSPI flash read controller based on EmbedFire education version.
 * @FilePath: \flash_read\flash_read_ctrl.v
 * Change history:
 */

module  flash_read_ctrl(
    input   wire            sys_clk     ,
    input   wire            sys_rst_n   ,
    input   wire            start       ,
    input   wire    [23:0]  start_addr  ,   //读取的初始地�?
    input   wire    [23:0]  byte_num    ,   //�?次需要读出的字节�?
    input   wire            miso        ,   //读出flash数据

    output  reg             sck         ,   //串行时钟
    output  reg             cs_n        ,   //片�?�信�?
    output  reg             mosi        ,   //主输出从输入数据
    output  reg             po_flag     ,   //输出数据
    output  reg     [7:0]   po_data         //输出数据
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

//parameter define
parameter   IDLE    =   2'b01  ,   //初始状�??
            READ    =   2'b10  ;   //数据读状�?

wire [7:0]  READ_INST;
assign  READ_INST  =   8'b0000_0011;   //        ?


//read config
reg [23:0]  num_data;               //读出字节个数
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        num_data <= 'd0;
    end
    else begin
        if(start) begin
            num_data <= byte_num;
        end
    end
end

reg [7:0] sector_addr;
reg [7:0] page_addr;
reg [7:0] byte_addr;
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        sector_addr <= 'd0;
        page_addr <= 'd0;
        byte_addr <= 'd0;
    end
    else begin
        if(start) begin
            sector_addr <= start_addr[23:16];
            page_addr <= start_addr[15:8];
            byte_addr <= start_addr[7:0];
        end
    end
end


//reg   define
reg     [4:0]   cnt_clk         ;   //系统时钟计数�?
reg     [2:0]   state           ;   //状�?�机状�??
reg     [15:0]  cnt_byte        ;   //字节计数�?
reg     [1:0]   cnt_sck         ;   //串行时钟计数�?
reg     [2:0]   cnt_bit         ;   //比特计数�?
reg             miso_flag       ;   //miso提取标志信号
reg     [7:0]   data            ;   //拼接数据
reg             po_flag_reg     ;   //输出数据标志信号

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

//state：两段式状�?�机第一段，状�?�跳�?
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        state   <=  IDLE;
    else
    case(state)
        IDLE:   if(start == 1'b1)
                    state   <=  READ;
        READ:   if((cnt_byte == num_data + 16'd3) && (cnt_clk == 5'd31))
                    state   <=  IDLE;
    endcase

//mosi：两段式状�?�机第二段，逻辑输出
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        mosi    <=  1'b0;
    else    if((state == READ) && (cnt_byte>= 16'd4))
        mosi    <=  1'b0;
    else    if((state == READ) && (cnt_byte == 16'd0) && (cnt_sck == 2'd0))
        mosi    <=  READ_INST[7 - cnt_bit];  //读指�?
    else    if((state == READ) && (cnt_byte == 16'd1) && (cnt_sck == 2'd0))
        mosi    <=  sector_addr[7 - cnt_bit];  //扇区地址
    else    if((state == READ) && (cnt_byte == 16'd2) && (cnt_sck == 2'd0))
        mosi    <=  page_addr[7 - cnt_bit];    //页地�?
    else    if((state == READ) && (cnt_byte == 16'd3) && (cnt_sck == 2'd0))
        mosi    <=  byte_addr[7 - cnt_bit];    //字节地址

//cnt_clk：系统时钟计数器，用以记录单个字�?
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        cnt_clk  <=  5'd0;
    else    if(state == READ)
        cnt_clk  <=  cnt_clk + 1'b1;

//cnt_byte：记录输出字节个数和等待时间
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        cnt_byte    <=  16'd0;
    else    if((cnt_clk == 5'd31) && (cnt_byte == num_data + 16'd3))
        cnt_byte    <=  16'd0;
    else    if(cnt_clk == 5'd31)
        cnt_byte    <=  cnt_byte + 1'b1;

//cnt_sck：串行时钟计数器，用以生成串行时�?
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        cnt_sck <=  2'd0;
    else    if(state == READ)
        cnt_sck <=  cnt_sck + 1'b1;

//cs_n：片选信�?
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        cs_n    <=  1'b1;
    else    if(start == 1'b1)
        cs_n    <=  1'b0;
    else    if((cnt_byte == num_data + 16'd3) && (cnt_clk == 5'd31) && (state == READ))
        cs_n    <=  1'b1;

//sck：输出串行时�?
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        sck <=  1'b0;
    else    if(cnt_sck == 2'd0)
        sck <=  1'b0;
    else    if(cnt_sck == 2'd2)
        sck <=  1'b1;

//cnt_bit：高低位对调，控制mosi输出
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        cnt_bit <=  3'd0;
    else    if(cnt_sck == 2'd2)
        cnt_bit <=  cnt_bit + 1'b1;

//miso_flag：miso提取标志信号
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        miso_flag   <=  1'b0;
    else    if((cnt_byte >= 16'd4) && (cnt_sck == 2'd1))
        miso_flag   <=  1'b1;
    else
        miso_flag   <=  1'b0;

//data：拼接数�?
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        data    <=  8'd0;
    else    if(miso_flag == 1'b1)
        data    <=  {data[6:0],miso};

//po_flag_reg:输出数据标志信号
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        po_flag_reg <=  1'b0;
    else    if((cnt_bit == 3'd7) && (miso_flag == 1'b1))
        po_flag_reg <=  1'b1;
    else
        po_flag_reg <=  1'b0;

//po_flag:输出数据标志信号
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        po_flag <=  1'b0;
    else
        po_flag <=  po_flag_reg;

//po_data:输出数据
always@(posedge sys_clk or  negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        po_data <=  8'd0;
    else    if(po_flag_reg == 1'b1)
        po_data <=  data;
    else
        po_data <=  po_data;

endmodule

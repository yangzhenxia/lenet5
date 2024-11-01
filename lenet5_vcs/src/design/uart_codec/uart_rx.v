//****************************************Copyright (c)***********************************//
//鍘熷瓙鍝ュ湪绾挎暀瀛﹀钩鍙帮細www.yuanzige.com
//鎶?鏈敮鎸侊細www.openedv.com
//娣樺疂搴楅摵锛歨ttp://openedv.taobao.com 
//鍏虫敞寰俊鍏紬骞冲彴寰俊鍙凤細"姝ｇ偣鍘熷瓙"锛屽厤璐硅幏鍙朲YNQ & FPGA & STM32 & LINUX璧勬枡銆?
//鐗堟潈鎵?鏈夛紝鐩楃増蹇呯┒銆?
//Copyright(C) 姝ｇ偣鍘熷瓙 2018-2028
//All rights reserved                                  
//----------------------------------------------------------------------------------------
// File name:           uart_recv
// Last modified Date:  2019/10/9 9:56:36
// Last Version:        V1.1
// Descriptions:        UART涓插彛鎺ユ敹妯″潡
//----------------------------------------------------------------------------------------
// Created by:          姝ｇ偣鍘熷瓙
// Created date:        2019/10/9 9:56:36
// Version:             V1.0
// Descriptions:        The original version
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module uart_rx#(
    parameter  UART_BPS = 9600                    //涓插彛娉㈢壒鐜?
)(
    input   [31:0]    clk_freq,
    input             clk,                  //绯荤粺鏃堕挓
    input             rstn,                //绯荤粺澶嶄綅锛屼綆鐢靛钩鏈夋晥
    
    input             rxd,                 //UART鎺ユ敹绔彛
    output  reg       rx_en,                //鎺ユ敹涓?甯ф暟鎹畬鎴愭爣蹇?
    //output  reg       rx_flag,                  //鎺ユ敹杩囩▼鏍囧織淇″彿,del by wyh
    //output  reg [ 3:0] rx_cnt,                  //鎺ユ敹鏁版嵁璁℃暟鍣?,del by wyh
    //output  reg [ 7:0] rxdata,                //del by wyh
    output  reg [7:0] data_rx                 //鎺ユ敹鐨勬暟鎹?
    );
    
//parameter define
wire    [31:0]      bps_cnt;
assign  bps_cnt  = clk_freq/UART_BPS;       //涓哄緱鍒版寚瀹氭尝鐗圭巼锛?
                                                //闇?瑕佸绯荤粺鏃堕挓璁℃暟bps_cnt娆?
//reg define
reg [ 3:0] rx_cnt;
reg [ 7:0] rxdata;
reg        rxd_d0;
reg        rxd_d1;
reg [15:0] clk_cnt;                              //绯荤粺鏃堕挓璁℃暟鍣?
reg        rx_flag;

//wire define
wire       start_flag;

//*****************************************************
//**                    main code
//*****************************************************
//鎹曡幏鎺ユ敹绔彛涓嬮檷娌?(璧峰浣?)锛屽緱鍒颁竴涓椂閽熷懆鏈熺殑鑴夊啿淇″彿
assign  start_flag = rxd_d1 & (~rxd_d0);    

//瀵筓ART鎺ユ敹绔彛鐨勬暟鎹欢杩熶袱涓椂閽熷懆鏈?
always @(posedge clk or negedge rstn) begin 
    if (!rstn) begin 
        rxd_d0 <= 1'b0;
        rxd_d1 <= 1'b0;          
    end
    else begin
        rxd_d0  <= rxd;                   
        rxd_d1  <= rxd_d0;
    end   
end

//褰撹剦鍐蹭俊鍙穝tart_flag鍒拌揪鏃讹紝杩涘叆鎺ユ敹杩囩▼           
always @(posedge clk or negedge rstn) begin         
    if (!rstn)                                  
        rx_flag <= 1'b0;
    else begin
        if(start_flag)                          //妫?娴嬪埌璧峰浣?
            rx_flag <= 1'b1;                    //杩涘叆鎺ユ敹杩囩▼锛屾爣蹇椾綅rx_flag鎷夐珮
                                                //璁℃暟鍒板仠姝綅涓棿鏃讹紝鍋滄鎺ユ敹杩囩▼
        else if((rx_cnt == 4'd9) && (clk_cnt == bps_cnt/2))
            rx_flag <= 1'b0;                    //鎺ユ敹杩囩▼缁撴潫锛屾爣蹇椾綅rx_flag鎷変綆
        else
            rx_flag <= rx_flag;
    end
end

//杩涘叆鎺ユ敹杩囩▼鍚庯紝鍚姩绯荤粺鏃堕挓璁℃暟鍣?
always @(posedge clk or negedge rstn) begin         
    if (!rstn)                             
        clk_cnt <= 16'd0;                                  
    else if ( rx_flag ) begin                   //澶勪簬鎺ユ敹杩囩▼
        if (clk_cnt < bps_cnt - 1)
            clk_cnt <= clk_cnt + 1'b1;
        else
            clk_cnt <= 16'd0;                   //瀵圭郴缁熸椂閽熻鏁拌揪涓?涓尝鐗圭巼鍛ㄦ湡鍚庢竻闆?
    end
    else                                            
        clk_cnt <= 16'd0;                       //鎺ユ敹杩囩▼缁撴潫锛岃鏁板櫒娓呴浂
end

//杩涘叆鎺ユ敹杩囩▼鍚庯紝鍚姩鎺ユ敹鏁版嵁璁℃暟鍣?
always @(posedge clk or negedge rstn) begin         
    if (!rstn)                             
        rx_cnt  <= 4'd0;
    else if ( rx_flag ) begin                   //澶勪簬鎺ユ敹杩囩▼
        if (clk_cnt == bps_cnt - 1)             //瀵圭郴缁熸椂閽熻鏁拌揪涓?涓尝鐗圭巼鍛ㄦ湡
            rx_cnt <= rx_cnt + 1'b1;            //姝ゆ椂鎺ユ敹鏁版嵁璁℃暟鍣ㄥ姞1
        else
            rx_cnt <= rx_cnt;       
    end
     else
        rx_cnt  <= 4'd0;                        //鎺ユ敹杩囩▼缁撴潫锛岃鏁板櫒娓呴浂
end

//鏍规嵁鎺ユ敹鏁版嵁璁℃暟鍣ㄦ潵瀵勫瓨uart鎺ユ敹绔彛鏁版嵁
always @(posedge clk or negedge rstn) begin 
    if ( !rstn)  
        rxdata <= 8'd0;                                     
    else if(rx_flag)                            //绯荤粺澶勪簬鎺ユ敹杩囩▼
        if (clk_cnt == bps_cnt/2) begin         //鍒ゆ柇绯荤粺鏃堕挓璁℃暟鍣ㄨ鏁板埌鏁版嵁浣嶄腑闂?
            case ( rx_cnt )
             4'd1 : rxdata[0] <= rxd_d1;   //瀵勫瓨鏁版嵁浣嶆渶浣庝綅
             4'd2 : rxdata[1] <= rxd_d1;
             4'd3 : rxdata[2] <= rxd_d1;
             4'd4 : rxdata[3] <= rxd_d1;
             4'd5 : rxdata[4] <= rxd_d1;
             4'd6 : rxdata[5] <= rxd_d1;
             4'd7 : rxdata[6] <= rxd_d1;
             4'd8 : rxdata[7] <= rxd_d1;   //瀵勫瓨鏁版嵁浣嶆渶楂樹綅
             default:;                                    
            endcase
        end
        else 
            rxdata <= rxdata;
    else
        rxdata <= 8'd0;
end

//鏁版嵁鎺ユ敹瀹屾瘯鍚庣粰鍑烘爣蹇椾俊鍙峰苟瀵勫瓨杈撳嚭鎺ユ敹鍒扮殑鏁版嵁
always @(posedge clk or negedge rstn) begin        
    if (!rstn) begin
        data_rx <= 8'd0;                               
        rx_en <= 1'b0;
    end
    else if((rx_cnt == 4'd9) && (clk_cnt == bps_cnt/2) ) begin               //鎺ユ敹鏁版嵁璁℃暟鍣ㄨ鏁板埌鍋滄浣嶆椂           
        data_rx <= rxdata;                    //瀵勫瓨杈撳嚭鎺ユ敹鍒扮殑鏁版嵁
        rx_en <= 1'b1;                      //骞跺皢鎺ユ敹瀹屾垚鏍囧織浣嶆媺楂?
    end
    else begin
        data_rx <= 8'd0;                                   
        rx_en <= 1'b0; 
    end    
end

endmodule   





//////////////////////////////////////////////////////////////////////////
//// Author        : EmbedFire
////Create Date    : 2019/06/12
//// Module Name   : uart_rx
//// Project Name  : rs232
//// Target Devices: Altera EP4CE10F17C8N
//// Tool Versions : Quartus 13.0
//// Description   :
////
//// Revision      : V1.0
//// Additional Comments:
//// 
//// 瀹為獙骞冲彴: 閲庣伀_寰侀?擯ro_FPGA寮?鍙戞澘
//// 鍏徃    : http://www.embedfire.com
//// 璁哄潧    : http://www.firebbs.cn
//// 娣樺疂    : https://fire-stm32.taobao.com
//////////////////////////////////////////////////////////////////////////
//
//module  uart_rx
//#(
//    parameter   UART_BPS    =   'd9600,         //涓插彛娉㈢壒鐜?
//    parameter   CLK_FREQ    =   'd40_000_000    //鏃堕挓棰戠巼
//)
//(
//    input   wire            clk     ,   //绯荤粺鏃堕挓40MHz
//    input   wire            rstn   ,   //鍏ㄥ眬澶嶄綅
//    input   wire            rxd          ,   //涓插彛鎺ユ敹鏁版嵁
//
//    output  reg     [7:0]   data_rx     ,   //涓茶浆骞跺悗鐨?8bit鏁版嵁
//    output  reg             rx_en         //涓茶浆骞跺悗鐨勬暟鎹湁鏁堟爣蹇椾俊鍙?
//);
//
////********************************************************************//
////****************** Parameter and Internal Signal *******************//
////********************************************************************//
////localparam    define
//localparam  BAUD_CNT_MAX    =   CLK_FREQ/UART_BPS   ;
//
////reg   define
//reg         rx_reg1     ;
//reg         rx_reg2     ;
//reg         rx_reg3     ;
//reg         start_nedge ;
//reg         work_en     ;
//reg [12:0]  baud_cnt    ;
//reg         bit_flag    ;
//reg [3:0]   bit_cnt     ;
//reg [7:0]   rx_data     ;
//reg         rx_flag     ;
//
////********************************************************************//
////***************************** Main Code ****************************//
////********************************************************************//
////鎻掑叆涓ょ骇瀵勫瓨鍣ㄨ繘琛屾暟鎹悓姝ワ紝鐢ㄦ潵娑堥櫎浜氱ǔ鎬?
////rx_reg1:绗竴绾у瘎瀛樺櫒锛屽瘎瀛樺櫒绌洪棽鐘舵?佸浣嶄负1
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        rx_reg1 <= 1'b1;
//    else
//        rx_reg1 <= rxd;
//
////rx_reg2:绗簩绾у瘎瀛樺櫒锛屽瘎瀛樺櫒绌洪棽鐘舵?佸浣嶄负1
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        rx_reg2 <= 1'b1;
//    else
//        rx_reg2 <= rx_reg1;
//
////rx_reg3:绗笁绾у瘎瀛樺櫒鍜岀浜岀骇瀵勫瓨鍣ㄥ叡鍚屾瀯鎴愪笅闄嶆部妫?娴?
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        rx_reg3 <= 1'b1;
//    else
//        rx_reg3 <= rx_reg2;
//
////start_nedge:妫?娴嬪埌涓嬮檷娌挎椂start_nedge浜х敓涓?涓椂閽熺殑楂樼數骞?
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        start_nedge <= 1'b0;
//    else    if((~rx_reg2) && (rx_reg3))
//        start_nedge <= 1'b1;
//    else
//        start_nedge <= 1'b0;
//
////work_en:鎺ユ敹鏁版嵁宸ヤ綔浣胯兘淇″彿
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        work_en <= 1'b0;
//    else    if(start_nedge == 1'b1)
//        work_en <= 1'b1;
//    else    if((bit_cnt == 4'd8) && (bit_flag == 1'b1))
//        work_en <= 1'b0;
//
////baud_cnt:娉㈢壒鐜囪鏁板櫒璁℃暟锛屼粠0璁℃暟鍒癇AUD_CNT_MAX - 1
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        baud_cnt <= 13'b0;
//    else    if((baud_cnt == BAUD_CNT_MAX - 1) || (work_en == 1'b0))
//        baud_cnt <= 13'b0;
//    else    if(work_en == 1'b1)
//        baud_cnt <= baud_cnt + 1'b1;
//
////bit_flag:褰揵aud_cnt璁℃暟鍣ㄨ鏁板埌涓棿鏁版椂閲囨牱鐨勬暟鎹渶绋冲畾锛?
////姝ゆ椂鎷夐珮涓?涓爣蹇椾俊鍙疯〃绀烘暟鎹彲浠ヨ鍙栬蛋
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        bit_flag <= 1'b0;
//    else    if(baud_cnt == BAUD_CNT_MAX/2 - 1)
//        bit_flag <= 1'b1;
//    else
//        bit_flag <= 1'b0;
//
////bit_cnt:鏈夋晥鏁版嵁涓暟璁℃暟鍣紝褰?8涓湁鏁堟暟鎹紙涓嶅惈璧峰浣嶅拰鍋滄浣嶏級
////閮芥帴鏀跺畬鎴愬悗璁℃暟鍣ㄦ竻闆?
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        bit_cnt <= 4'b0;
//    else    if((bit_cnt == 4'd8) && (bit_flag == 1'b1))
//        bit_cnt <= 4'b0;
//     else    if(bit_flag ==1'b1)
//         bit_cnt <= bit_cnt + 1'b1;
//
////rx_data:杈撳叆鏁版嵁杩涜绉讳綅
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        rx_data <= 8'b0;
//    else    if((bit_cnt >= 4'd1)&&(bit_cnt <= 4'd8)&&(bit_flag == 1'b1))
//        rx_data <= {rx_reg3, rx_data[7:1]};
//
////rx_flag:杈撳叆鏁版嵁绉讳綅瀹屾垚鏃秗x_flag鎷夐珮涓?涓椂閽熺殑楂樼數骞?
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        rx_flag <= 1'b0;
//    else    if((bit_cnt == 4'd8) && (bit_flag == 1'b1))
//        rx_flag <= 1'b1;
//    else
//        rx_flag <= 1'b0;
//
////data_rx:杈撳嚭瀹屾暣鐨?8浣嶆湁鏁堟暟鎹?
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        data_rx <= 8'b0;
//    else    if(rx_flag == 1'b1)
//        data_rx <= rx_data;
//
////rx_en:杈撳嚭鏁版嵁鏈夋晥鏍囧織锛堟瘮rx_flag寤跺悗涓?涓椂閽熷懆鏈燂紝涓轰簡鍜宲o_data鍚屾锛?
//always@(posedge clk or negedge rstn)
//    if(rstn == 1'b0)
//        rx_en <= 1'b0;
//    else
//        rx_en <= rx_flag;
//
//endmodule

/*
 * @Author: Wang Yuanhao
 * @Date: 2022-10-13 10:25:59
 * @LastEditTime: 2022-10-21 15:46:18
 * @LastEditors: Wang Yuanhao
 * @Description: a bridge between flash_read_ctrl and weight_loader
 * @FilePath: \rtl\data_concat.v
 * Change history:
 */
module data_concat(
    input   wire            sys_clk     ,
    input   wire            sys_rst_n   ,
    input   wire            tx_flag     ,
    input   wire    [7:0]   tx_data     ,   //tx_flag and tx_data come at same cycle!

    output  reg             out_en      ,
    output  reg     [255:0] out_data        //out_data comes one cycle later than out_en!
);

    reg [4:0]   byte_cnt;
    always@(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            byte_cnt <= 'd0;
        end
        else begin
            if(tx_flag) begin
                if(byte_cnt == 5'd31) begin
                    byte_cnt <= 'd0;
                end
                else begin
                    byte_cnt <= byte_cnt + 5'd1;
                end
            end
        end
    end

    reg [255:0] out_data_buffer;
    always@(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            out_data_buffer <= 'd0;
        end
        else begin
            if(tx_flag) begin
                out_data_buffer[8*byte_cnt+:8] <= tx_data;
            end
        end
    end

    reg out_en_buffer;

    always@(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            out_data <= 'd0;
        end
        else begin
            if(out_en_buffer) begin
                out_data <= out_data_buffer;
            end
        end
    end
    
    always@(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            out_en_buffer <= 0;
        end
        else begin
            if(tx_flag && byte_cnt == 5'd31) begin
                out_en_buffer <= 1;
            end
            else begin
                out_en_buffer <= 0;
            end
        end
    end
    
    always@(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            out_en <= 0;
        end
        else begin
            out_en <= out_en_buffer;
        end
    end

endmodule

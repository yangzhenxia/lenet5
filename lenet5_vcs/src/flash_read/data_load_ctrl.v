/*
 * @Author: Wang Yuanhao
 * @Date: 2022-10-13 12:35:05
 * @LastEditTime: 2022-10-21 19:39:04
 * @LastEditors: Wang Yuanhao
 * @Description: Load weights for NPU
 * @FilePath: \rtl\data_load_ctrl.v
 * Change history:
 */
module data_load_ctrl(
    input                   sys_clk,
    input                   sys_rst_n,
    input                   layer_load_start,
    // interact with flash_read_ctrl
    output                  start,
    output          [23:0]  start_addr,
    output          [23:0]  byte_num,
    //interact with NPU
    output  reg             out_en,
    output          [255:0] out_data,
    output  reg             layer_load_finish,
    //interact with data_contact
    input                   load_en,
    input           [255:0] load_data       //load_data comes one cycle later than load_en!
);
    
    assign  start = layer_load_start;
    assign  start_addr = 'd0;
    assign  byte_num = 'd1040 << 5;
    
    assign  out_data = load_data;
    
    always@(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            out_en <= 'd0;
        end
        else begin
            out_en <= load_en;
        end
    end
    
    localparam  IDLE = 'd0,
                READ = 'd1,
                FINISH = 'd2;
    
    reg [1:0]   state;
    reg [1:0]   next_state;
    
    always@(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            state <= IDLE;
        end
        else begin
            state <= next_state;
        end
    end
    
    reg [23:0] byte_cnt;
    always@(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            byte_cnt <= 'd0;
        end
        else begin
            if(load_en) begin
                byte_cnt <= byte_cnt + 'd1;
            end
        end
    end
    
    always@(*) begin
        case(state)
            IDLE: next_state = layer_load_start? READ: IDLE;
            READ: next_state = (byte_cnt == byte_num)? FINISH: READ;
            FINISH: next_state = IDLE;
            default:next_state = IDLE;
        endcase
    end
    
    always@(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            layer_load_finish <= 'd0;
        end
        else begin
            layer_load_finish <= (state == FINISH);
        end
    end

endmodule

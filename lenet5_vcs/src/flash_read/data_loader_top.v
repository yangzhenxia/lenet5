module data_loader_top(
    input                   sys_clk,
    input                   sys_rst_n,
    input                   layer_load_start,
    output                  out_sram_en,
    output  [255:0]         out_sram_data,
    output                  layer_load_finish,

    input                   miso,
    output                  sck,
    output                  cs_n,
    output                  mosi
);

    wire            start;
    wire    [23:0]  start_addr;
    wire    [23:0]  byte_num;
    wire            po_flag;
    wire    [7:0]   po_data;
    wire            load_en;
    wire    [255:0] load_data;


    data_load_ctrl u_data_load_ctrl(
        .sys_clk(sys_clk),
        .sys_rst_n(sys_rst_n),
        .layer_load_start(layer_load_start),
        .start(start),
        .start_addr(start_addr),
        .byte_num(byte_num),
        .out_en(out_sram_en),
        .out_data(out_sram_data),
        .layer_load_finish(layer_load_finish),
        .load_en(load_en),
        .load_data(load_data)
    );

    flash_read_ctrl u_flash_read_ctrl(
        .sys_clk(sys_clk),
        .sys_rst_n(sys_rst_n),
        .start(start),
        .start_addr(start_addr),
        .byte_num(byte_num),
        .miso(miso),

        .sck(sck),
        .cs_n(cs_n),
        .mosi(mosi),
        .po_flag(po_flag),
        .po_data(po_data)
    );

    data_concat u_data_concat(
        .sys_clk(sys_clk),
        .sys_rst_n(sys_rst_n),
        .tx_flag(po_flag),
        .tx_data(po_data),
        .out_en(load_en),
        .out_data(load_data)
    );


endmodule
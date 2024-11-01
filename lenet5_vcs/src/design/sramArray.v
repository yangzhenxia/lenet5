`timescale 1ns/1ps

`define inputWeightWidth 5*5*8
`define inputMapWidth 6*6*8
`define DATA_SIZE 8  

`define mapRam_addra_width 9
`define weightRam_addra_width 11

module sramArray(
    input clk,rst,
    //weights&bias ram
    input weightsBias_sram_ena,weightsBias_sram_wea,
    input [`weightRam_addra_width-1:0]weightsBias_sram_addra,
    output[`inputWeightWidth+`DATA_SIZE-1:0] weightsBias,
    input [`inputWeightWidth+`DATA_SIZE-1:0] weightsBias_load,
    //feature map ram
    input featureMap_sram_ena,featureMap_sram_wea,
    input [`mapRam_addra_width-1:0] featureMap_sram_addra,
    input [`inputMapWidth-1:0] featureMap_sram_din,
    output [`inputMapWidth-1:0] featureMap_sram_douta
);
//weights srams temp param
wire part1weightsBias_sram_ena;
wire [103:0] part1weightsBiasH;
wire [103:0] part1weightsBiasL;

wire part2weightsBias_sram_ena;
wire [103:0] part2weightsBiasH;
wire [103:0] part2weightsBiasL;

wire [10:0]internalweights_sram_addra;

assign weightsBias = (weightsBias_sram_addra < 520)? {part1weightsBiasH,part1weightsBiasL} : {part2weightsBiasH,part2weightsBiasL};
assign part1weightsBias_sram_ena = (weightsBias_sram_addra < 520)?~weightsBias_sram_ena:1;
assign part2weightsBias_sram_ena = (weightsBias_sram_addra < 520)?1:~weightsBias_sram_ena;
assign internalweights_sram_addra = (weightsBias_sram_addra < 520)?weightsBias_sram_addra:weightsBias_sram_addra-520;



//BANK1
sram520x104 u_weight_sram0(
.Q(part1weightsBiasH), //douta
.CLK(clk), 
.CEN(part1weightsBias_sram_ena), //ena
.WEN(!weightsBias_sram_wea), 
.A(internalweights_sram_addra[9:0]), //addra
.D(weightsBias_load[207:104]), //din
.EMA(3'd7), 
.EMAW(2'd3), 
.RET1N(1'b1)
);

sram520x104 u_weight_sram2(
.Q(part1weightsBiasL), //douta
.CLK(clk), 
.CEN(part1weightsBias_sram_ena), //ena
.WEN(!weightsBias_sram_wea), 
.A(internalweights_sram_addra[9:0]), //addra
.D(weightsBias_load[103:0]), //din
.EMA(3'd7), 
.EMAW(2'd3), 
.RET1N(1'b1)
);

sram520x104 u_weight_sram1(
.Q(part2weightsBiasH), //douta
.CLK(clk), 
.CEN(part2weightsBias_sram_ena), //ena
.WEN(!weightsBias_sram_wea), 
.A(internalweights_sram_addra[9:0]), //addra
.D(weightsBias_load[207:104]), //din
.EMA(3'd7), 
.EMAW(2'd3), 
.RET1N(1'b1)
);

sram520x104 u_weight_sram3(
.Q(part2weightsBiasL), //douta
.CLK(clk), 
.CEN(part2weightsBias_sram_ena), //ena
.WEN(!weightsBias_sram_wea), 
.A(internalweights_sram_addra[9:0]), //addra
.D(weightsBias_load[103:0]), //din
.EMA(3'd7), 
.EMAW(2'd3), 
.RET1N(1'b1)
);

wire mapEna = ~featureMap_sram_ena;
wire mapWea = ~featureMap_sram_wea;

//BANK2 featureMap sram
sram464x144 u_featureMap_sram0(
.Q(featureMap_sram_douta[287:144]), //douta
.CLK(clk), 
.CEN(mapEna), //ena
.WEN(mapWea), 
.A(featureMap_sram_addra), //addra
.D(featureMap_sram_din[287:144]), //din
.EMA(3'd7), 
.EMAW(2'd3), 
.RET1N(1'b1)
);

sram464x144 u_featureMap_sram1(
.Q(featureMap_sram_douta[143:0]), //douta
.CLK(clk), 
.CEN(mapEna), //ena
.WEN(mapWea), 
.A(featureMap_sram_addra), //addra
.D(featureMap_sram_din[143:0]), //din
.EMA(3'd7), 
.EMAW(2'd3), 
.RET1N(1'b1)
);


endmodule

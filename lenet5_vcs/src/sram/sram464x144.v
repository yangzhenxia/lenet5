/* verilog_memcomp Version: 4.0.5-EAC1 */
/* common_memcomp Version: 4.0.5-beta22 */
/* lang compiler Version: 4.1.6-beta1 Jul 19 2012 13:55:19 */
//
//       CONFIDENTIAL AND PROPRIETARY SOFTWARE OF ARM PHYSICAL IP, INC.
//      
//       Copyright (c) 1993 - 2022 ARM Physical IP, Inc.  All Rights Reserved.
//      
//       Use of this Software is subject to the terms and conditions of the
//       applicable license agreement with ARM Physical IP, Inc.
//       In addition, this Software is protected by patents, copyright law 
//       and international treaties.
//      
//       The copyright notice(s) in this Software does not indicate actual or
//       intended publication of this Software.
//
//      Verilog model for Synchronous Single-Port Register File
//
//       Instance Name:              sram464x144
//       Words:                      464
//       Bits:                       144
//       Mux:                        2
//       Drive:                      6
//       Write Mask:                 Off
//       Write Thru:                 Off
//       Extra Margin Adjustment:    On
//       Redundant Columns:          0
//       Test Muxes                  Off
//       Power Gating:               Off
//       Retention:                  On
//       Pipeline:                   Off
//       Read Disturb Test:	        Off
//       
//       Creation Date:  Tue Nov 15 11:43:06 2022
//       Version: 	r1p1
//
//      Modeling Assumptions: This model supports full gate level simulation
//          including proper x-handling and timing check behavior.  Unit
//          delay timing is included in the model. Back-annotation of SDF
//          (v3.0 or v2.1) is supported.  SDF can be created utilyzing the delay
//          calculation views provided with this generator and supported
//          delay calculators.  All buses are modeled [MSB:LSB].  All 
//          ports are padded with Verilog primitives.
//
//      Modeling Limitations: None.
//
//      Known Bugs: None.
//
//      Known Work Arounds: N/A
//
`timescale 1 ns/1 ps
`define ARM_MEM_PROP 1.000
`define ARM_MEM_RETAIN 1.000
`define ARM_MEM_PERIOD 3.000
`define ARM_MEM_WIDTH 1.000
`define ARM_MEM_SETUP 1.000
`define ARM_MEM_HOLD 0.500
`define ARM_MEM_COLLISION 3.000
// If ARM_UD_MODEL is defined at Simulator Command Line, it Selects the Fast Functional Model
`ifdef ARM_UD_MODEL

// Following parameter Values can be overridden at Simulator Command Line.

// ARM_UD_DP Defines the delay through Data Paths, for Memory Models it represents BIST MUX output delays.
`ifdef ARM_UD_DP
`else
`define ARM_UD_DP #0.001
`endif
// ARM_UD_CP Defines the delay through Clock Path Cells, for Memory Models it is not used.
`ifdef ARM_UD_CP
`else
`define ARM_UD_CP
`endif
// ARM_UD_SEQ Defines the delay through the Memory, for Memory Models it is used for CLK->Q delays.
`ifdef ARM_UD_SEQ
`else
`define ARM_UD_SEQ #0.01
`endif

`celldefine
// If POWER_PINS is defined at Simulator Command Line, it selects the module definition with Power Ports
`ifdef POWER_PINS
module sram464x144 (VDDCE, VDDPE, VSSE, Q, CLK, CEN, WEN, A, D, EMA, EMAW, RET1N);
`else
module sram464x144 (Q, CLK, CEN, WEN, A, D, EMA, EMAW, RET1N);
`endif

  parameter ASSERT_PREFIX = "";
  parameter BITS = 144;
  parameter WORDS = 464;
  parameter MUX = 2;
  parameter MEM_WIDTH = 288; // redun block size 2, 144 on left, 144 on right
  parameter MEM_HEIGHT = 232;
  parameter WP_SIZE = 144 ;
  parameter UPM_WIDTH = 3;
  parameter UPMW_WIDTH = 2;
  parameter UPMS_WIDTH = 0;

  output [143:0] Q;
  input  CLK;
  input  CEN;
  input  WEN;
  input [8:0] A;
  input [143:0] D;
  input [2:0] EMA;
  input [1:0] EMAW;
  input  RET1N;
`ifdef POWER_PINS
  inout VDDCE;
  inout VDDPE;
  inout VSSE;
`endif

  reg pre_charge_st;
  integer row_address;
  integer mux_address;
  reg [287:0] mem [0:231];
  reg [287:0] row, row_t;
  reg LAST_CLK;
  reg [287:0] row_mask;
  reg [287:0] new_data;
  reg [287:0] data_out;
  reg [143:0] readLatch0;
  reg [143:0] shifted_readLatch0;
  reg [143:0] Q_int;
  reg [143:0] writeEnable;
  reg clk0_int;

  wire [143:0] Q_;
 wire  CLK_;
  wire  CEN_;
  reg  CEN_int;
  reg  CEN_p2;
  wire  WEN_;
  reg  WEN_int;
  wire [8:0] A_;
  reg [8:0] A_int;
  wire [143:0] D_;
  reg [143:0] D_int;
  wire [2:0] EMA_;
  reg [2:0] EMA_int;
  wire [1:0] EMAW_;
  reg [1:0] EMAW_int;
  wire  RET1N_;
  reg  RET1N_int;

  assign Q[0] = Q_[0]; 
  assign Q[1] = Q_[1]; 
  assign Q[2] = Q_[2]; 
  assign Q[3] = Q_[3]; 
  assign Q[4] = Q_[4]; 
  assign Q[5] = Q_[5]; 
  assign Q[6] = Q_[6]; 
  assign Q[7] = Q_[7]; 
  assign Q[8] = Q_[8]; 
  assign Q[9] = Q_[9]; 
  assign Q[10] = Q_[10]; 
  assign Q[11] = Q_[11]; 
  assign Q[12] = Q_[12]; 
  assign Q[13] = Q_[13]; 
  assign Q[14] = Q_[14]; 
  assign Q[15] = Q_[15]; 
  assign Q[16] = Q_[16]; 
  assign Q[17] = Q_[17]; 
  assign Q[18] = Q_[18]; 
  assign Q[19] = Q_[19]; 
  assign Q[20] = Q_[20]; 
  assign Q[21] = Q_[21]; 
  assign Q[22] = Q_[22]; 
  assign Q[23] = Q_[23]; 
  assign Q[24] = Q_[24]; 
  assign Q[25] = Q_[25]; 
  assign Q[26] = Q_[26]; 
  assign Q[27] = Q_[27]; 
  assign Q[28] = Q_[28]; 
  assign Q[29] = Q_[29]; 
  assign Q[30] = Q_[30]; 
  assign Q[31] = Q_[31]; 
  assign Q[32] = Q_[32]; 
  assign Q[33] = Q_[33]; 
  assign Q[34] = Q_[34]; 
  assign Q[35] = Q_[35]; 
  assign Q[36] = Q_[36]; 
  assign Q[37] = Q_[37]; 
  assign Q[38] = Q_[38]; 
  assign Q[39] = Q_[39]; 
  assign Q[40] = Q_[40]; 
  assign Q[41] = Q_[41]; 
  assign Q[42] = Q_[42]; 
  assign Q[43] = Q_[43]; 
  assign Q[44] = Q_[44]; 
  assign Q[45] = Q_[45]; 
  assign Q[46] = Q_[46]; 
  assign Q[47] = Q_[47]; 
  assign Q[48] = Q_[48]; 
  assign Q[49] = Q_[49]; 
  assign Q[50] = Q_[50]; 
  assign Q[51] = Q_[51]; 
  assign Q[52] = Q_[52]; 
  assign Q[53] = Q_[53]; 
  assign Q[54] = Q_[54]; 
  assign Q[55] = Q_[55]; 
  assign Q[56] = Q_[56]; 
  assign Q[57] = Q_[57]; 
  assign Q[58] = Q_[58]; 
  assign Q[59] = Q_[59]; 
  assign Q[60] = Q_[60]; 
  assign Q[61] = Q_[61]; 
  assign Q[62] = Q_[62]; 
  assign Q[63] = Q_[63]; 
  assign Q[64] = Q_[64]; 
  assign Q[65] = Q_[65]; 
  assign Q[66] = Q_[66]; 
  assign Q[67] = Q_[67]; 
  assign Q[68] = Q_[68]; 
  assign Q[69] = Q_[69]; 
  assign Q[70] = Q_[70]; 
  assign Q[71] = Q_[71]; 
  assign Q[72] = Q_[72]; 
  assign Q[73] = Q_[73]; 
  assign Q[74] = Q_[74]; 
  assign Q[75] = Q_[75]; 
  assign Q[76] = Q_[76]; 
  assign Q[77] = Q_[77]; 
  assign Q[78] = Q_[78]; 
  assign Q[79] = Q_[79]; 
  assign Q[80] = Q_[80]; 
  assign Q[81] = Q_[81]; 
  assign Q[82] = Q_[82]; 
  assign Q[83] = Q_[83]; 
  assign Q[84] = Q_[84]; 
  assign Q[85] = Q_[85]; 
  assign Q[86] = Q_[86]; 
  assign Q[87] = Q_[87]; 
  assign Q[88] = Q_[88]; 
  assign Q[89] = Q_[89]; 
  assign Q[90] = Q_[90]; 
  assign Q[91] = Q_[91]; 
  assign Q[92] = Q_[92]; 
  assign Q[93] = Q_[93]; 
  assign Q[94] = Q_[94]; 
  assign Q[95] = Q_[95]; 
  assign Q[96] = Q_[96]; 
  assign Q[97] = Q_[97]; 
  assign Q[98] = Q_[98]; 
  assign Q[99] = Q_[99]; 
  assign Q[100] = Q_[100]; 
  assign Q[101] = Q_[101]; 
  assign Q[102] = Q_[102]; 
  assign Q[103] = Q_[103]; 
  assign Q[104] = Q_[104]; 
  assign Q[105] = Q_[105]; 
  assign Q[106] = Q_[106]; 
  assign Q[107] = Q_[107]; 
  assign Q[108] = Q_[108]; 
  assign Q[109] = Q_[109]; 
  assign Q[110] = Q_[110]; 
  assign Q[111] = Q_[111]; 
  assign Q[112] = Q_[112]; 
  assign Q[113] = Q_[113]; 
  assign Q[114] = Q_[114]; 
  assign Q[115] = Q_[115]; 
  assign Q[116] = Q_[116]; 
  assign Q[117] = Q_[117]; 
  assign Q[118] = Q_[118]; 
  assign Q[119] = Q_[119]; 
  assign Q[120] = Q_[120]; 
  assign Q[121] = Q_[121]; 
  assign Q[122] = Q_[122]; 
  assign Q[123] = Q_[123]; 
  assign Q[124] = Q_[124]; 
  assign Q[125] = Q_[125]; 
  assign Q[126] = Q_[126]; 
  assign Q[127] = Q_[127]; 
  assign Q[128] = Q_[128]; 
  assign Q[129] = Q_[129]; 
  assign Q[130] = Q_[130]; 
  assign Q[131] = Q_[131]; 
  assign Q[132] = Q_[132]; 
  assign Q[133] = Q_[133]; 
  assign Q[134] = Q_[134]; 
  assign Q[135] = Q_[135]; 
  assign Q[136] = Q_[136]; 
  assign Q[137] = Q_[137]; 
  assign Q[138] = Q_[138]; 
  assign Q[139] = Q_[139]; 
  assign Q[140] = Q_[140]; 
  assign Q[141] = Q_[141]; 
  assign Q[142] = Q_[142]; 
  assign Q[143] = Q_[143]; 
  assign CLK_ = CLK;
  assign CEN_ = CEN;
  assign WEN_ = WEN;
  assign A_[0] = A[0];
  assign A_[1] = A[1];
  assign A_[2] = A[2];
  assign A_[3] = A[3];
  assign A_[4] = A[4];
  assign A_[5] = A[5];
  assign A_[6] = A[6];
  assign A_[7] = A[7];
  assign A_[8] = A[8];
  assign D_[0] = D[0];
  assign D_[1] = D[1];
  assign D_[2] = D[2];
  assign D_[3] = D[3];
  assign D_[4] = D[4];
  assign D_[5] = D[5];
  assign D_[6] = D[6];
  assign D_[7] = D[7];
  assign D_[8] = D[8];
  assign D_[9] = D[9];
  assign D_[10] = D[10];
  assign D_[11] = D[11];
  assign D_[12] = D[12];
  assign D_[13] = D[13];
  assign D_[14] = D[14];
  assign D_[15] = D[15];
  assign D_[16] = D[16];
  assign D_[17] = D[17];
  assign D_[18] = D[18];
  assign D_[19] = D[19];
  assign D_[20] = D[20];
  assign D_[21] = D[21];
  assign D_[22] = D[22];
  assign D_[23] = D[23];
  assign D_[24] = D[24];
  assign D_[25] = D[25];
  assign D_[26] = D[26];
  assign D_[27] = D[27];
  assign D_[28] = D[28];
  assign D_[29] = D[29];
  assign D_[30] = D[30];
  assign D_[31] = D[31];
  assign D_[32] = D[32];
  assign D_[33] = D[33];
  assign D_[34] = D[34];
  assign D_[35] = D[35];
  assign D_[36] = D[36];
  assign D_[37] = D[37];
  assign D_[38] = D[38];
  assign D_[39] = D[39];
  assign D_[40] = D[40];
  assign D_[41] = D[41];
  assign D_[42] = D[42];
  assign D_[43] = D[43];
  assign D_[44] = D[44];
  assign D_[45] = D[45];
  assign D_[46] = D[46];
  assign D_[47] = D[47];
  assign D_[48] = D[48];
  assign D_[49] = D[49];
  assign D_[50] = D[50];
  assign D_[51] = D[51];
  assign D_[52] = D[52];
  assign D_[53] = D[53];
  assign D_[54] = D[54];
  assign D_[55] = D[55];
  assign D_[56] = D[56];
  assign D_[57] = D[57];
  assign D_[58] = D[58];
  assign D_[59] = D[59];
  assign D_[60] = D[60];
  assign D_[61] = D[61];
  assign D_[62] = D[62];
  assign D_[63] = D[63];
  assign D_[64] = D[64];
  assign D_[65] = D[65];
  assign D_[66] = D[66];
  assign D_[67] = D[67];
  assign D_[68] = D[68];
  assign D_[69] = D[69];
  assign D_[70] = D[70];
  assign D_[71] = D[71];
  assign D_[72] = D[72];
  assign D_[73] = D[73];
  assign D_[74] = D[74];
  assign D_[75] = D[75];
  assign D_[76] = D[76];
  assign D_[77] = D[77];
  assign D_[78] = D[78];
  assign D_[79] = D[79];
  assign D_[80] = D[80];
  assign D_[81] = D[81];
  assign D_[82] = D[82];
  assign D_[83] = D[83];
  assign D_[84] = D[84];
  assign D_[85] = D[85];
  assign D_[86] = D[86];
  assign D_[87] = D[87];
  assign D_[88] = D[88];
  assign D_[89] = D[89];
  assign D_[90] = D[90];
  assign D_[91] = D[91];
  assign D_[92] = D[92];
  assign D_[93] = D[93];
  assign D_[94] = D[94];
  assign D_[95] = D[95];
  assign D_[96] = D[96];
  assign D_[97] = D[97];
  assign D_[98] = D[98];
  assign D_[99] = D[99];
  assign D_[100] = D[100];
  assign D_[101] = D[101];
  assign D_[102] = D[102];
  assign D_[103] = D[103];
  assign D_[104] = D[104];
  assign D_[105] = D[105];
  assign D_[106] = D[106];
  assign D_[107] = D[107];
  assign D_[108] = D[108];
  assign D_[109] = D[109];
  assign D_[110] = D[110];
  assign D_[111] = D[111];
  assign D_[112] = D[112];
  assign D_[113] = D[113];
  assign D_[114] = D[114];
  assign D_[115] = D[115];
  assign D_[116] = D[116];
  assign D_[117] = D[117];
  assign D_[118] = D[118];
  assign D_[119] = D[119];
  assign D_[120] = D[120];
  assign D_[121] = D[121];
  assign D_[122] = D[122];
  assign D_[123] = D[123];
  assign D_[124] = D[124];
  assign D_[125] = D[125];
  assign D_[126] = D[126];
  assign D_[127] = D[127];
  assign D_[128] = D[128];
  assign D_[129] = D[129];
  assign D_[130] = D[130];
  assign D_[131] = D[131];
  assign D_[132] = D[132];
  assign D_[133] = D[133];
  assign D_[134] = D[134];
  assign D_[135] = D[135];
  assign D_[136] = D[136];
  assign D_[137] = D[137];
  assign D_[138] = D[138];
  assign D_[139] = D[139];
  assign D_[140] = D[140];
  assign D_[141] = D[141];
  assign D_[142] = D[142];
  assign D_[143] = D[143];
  assign EMA_[0] = EMA[0];
  assign EMA_[1] = EMA[1];
  assign EMA_[2] = EMA[2];
  assign EMAW_[0] = EMAW[0];
  assign EMAW_[1] = EMAW[1];
  assign RET1N_ = RET1N;

  assign `ARM_UD_SEQ Q_ = (RET1N_ | pre_charge_st) ? ((Q_int)) : {144{1'bx}};

// If INITIALIZE_MEMORY is defined at Simulator Command Line, it Initializes the Memory with all ZEROS.
`ifdef INITIALIZE_MEMORY
  integer i;
  initial begin
    #0;
    for (i = 0; i < MEM_HEIGHT; i = i + 1)
      mem[i] = {MEM_WIDTH{1'b0}};
  end
`endif
  always @ (EMA_) begin
  	if(EMA_ < 2) 
   	$display("Warning: Set Value for EMA doesn't match Default value 2 in %m at %0t", $time);
  end
  always @ (EMAW_) begin
  	if(EMAW_ < 0) 
   	$display("Warning: Set Value for EMAW doesn't match Default value 0 in %m at %0t", $time);
  end

  task failedWrite;
  input port_f;
  integer i;
  begin
    for (i = 0; i < MEM_HEIGHT; i = i + 1)
      mem[i] = {MEM_WIDTH{1'bx}};
  end
  endtask

  function isBitX;
    input bitval;
    begin
      isBitX = ( bitval===1'bx || bitval==1'bz ) ? 1'b1 : 1'b0;
    end
  endfunction



  task readWrite;
  begin
    if (RET1N_int === 1'bx || RET1N_int === 1'bz) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (RET1N_int === 1'b0 && CEN_int === 1'b0) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (RET1N_int === 1'b0) begin
      // no cycle in retention mode
    end else if (^{CEN_int, EMA_int, EMAW_int, RET1N_int} === 1'bx) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if ((A_int >= WORDS) && (CEN_int === 1'b0)) begin
      Q_int = WEN_int !== 1'b1 ? Q_int : {144{1'bx}};
    end else if (CEN_int === 1'b0 && (^A_int) === 1'bx) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (CEN_int === 1'b0) begin
      mux_address = (A_int & 1'b1);
      row_address = (A_int >> 1);
      if (row_address > 231)
        row = {288{1'bx}};
      else
        row = mem[row_address];
        writeEnable = ~ {144{WEN_int}};
      if (WEN_int !== 1'b1) begin
        row_mask =  ( {1'b0, writeEnable[143], 1'b0, writeEnable[142], 1'b0, writeEnable[141],
          1'b0, writeEnable[140], 1'b0, writeEnable[139], 1'b0, writeEnable[138], 1'b0, writeEnable[137],
          1'b0, writeEnable[136], 1'b0, writeEnable[135], 1'b0, writeEnable[134], 1'b0, writeEnable[133],
          1'b0, writeEnable[132], 1'b0, writeEnable[131], 1'b0, writeEnable[130], 1'b0, writeEnable[129],
          1'b0, writeEnable[128], 1'b0, writeEnable[127], 1'b0, writeEnable[126], 1'b0, writeEnable[125],
          1'b0, writeEnable[124], 1'b0, writeEnable[123], 1'b0, writeEnable[122], 1'b0, writeEnable[121],
          1'b0, writeEnable[120], 1'b0, writeEnable[119], 1'b0, writeEnable[118], 1'b0, writeEnable[117],
          1'b0, writeEnable[116], 1'b0, writeEnable[115], 1'b0, writeEnable[114], 1'b0, writeEnable[113],
          1'b0, writeEnable[112], 1'b0, writeEnable[111], 1'b0, writeEnable[110], 1'b0, writeEnable[109],
          1'b0, writeEnable[108], 1'b0, writeEnable[107], 1'b0, writeEnable[106], 1'b0, writeEnable[105],
          1'b0, writeEnable[104], 1'b0, writeEnable[103], 1'b0, writeEnable[102], 1'b0, writeEnable[101],
          1'b0, writeEnable[100], 1'b0, writeEnable[99], 1'b0, writeEnable[98], 1'b0, writeEnable[97],
          1'b0, writeEnable[96], 1'b0, writeEnable[95], 1'b0, writeEnable[94], 1'b0, writeEnable[93],
          1'b0, writeEnable[92], 1'b0, writeEnable[91], 1'b0, writeEnable[90], 1'b0, writeEnable[89],
          1'b0, writeEnable[88], 1'b0, writeEnable[87], 1'b0, writeEnable[86], 1'b0, writeEnable[85],
          1'b0, writeEnable[84], 1'b0, writeEnable[83], 1'b0, writeEnable[82], 1'b0, writeEnable[81],
          1'b0, writeEnable[80], 1'b0, writeEnable[79], 1'b0, writeEnable[78], 1'b0, writeEnable[77],
          1'b0, writeEnable[76], 1'b0, writeEnable[75], 1'b0, writeEnable[74], 1'b0, writeEnable[73],
          1'b0, writeEnable[72], 1'b0, writeEnable[71], 1'b0, writeEnable[70], 1'b0, writeEnable[69],
          1'b0, writeEnable[68], 1'b0, writeEnable[67], 1'b0, writeEnable[66], 1'b0, writeEnable[65],
          1'b0, writeEnable[64], 1'b0, writeEnable[63], 1'b0, writeEnable[62], 1'b0, writeEnable[61],
          1'b0, writeEnable[60], 1'b0, writeEnable[59], 1'b0, writeEnable[58], 1'b0, writeEnable[57],
          1'b0, writeEnable[56], 1'b0, writeEnable[55], 1'b0, writeEnable[54], 1'b0, writeEnable[53],
          1'b0, writeEnable[52], 1'b0, writeEnable[51], 1'b0, writeEnable[50], 1'b0, writeEnable[49],
          1'b0, writeEnable[48], 1'b0, writeEnable[47], 1'b0, writeEnable[46], 1'b0, writeEnable[45],
          1'b0, writeEnable[44], 1'b0, writeEnable[43], 1'b0, writeEnable[42], 1'b0, writeEnable[41],
          1'b0, writeEnable[40], 1'b0, writeEnable[39], 1'b0, writeEnable[38], 1'b0, writeEnable[37],
          1'b0, writeEnable[36], 1'b0, writeEnable[35], 1'b0, writeEnable[34], 1'b0, writeEnable[33],
          1'b0, writeEnable[32], 1'b0, writeEnable[31], 1'b0, writeEnable[30], 1'b0, writeEnable[29],
          1'b0, writeEnable[28], 1'b0, writeEnable[27], 1'b0, writeEnable[26], 1'b0, writeEnable[25],
          1'b0, writeEnable[24], 1'b0, writeEnable[23], 1'b0, writeEnable[22], 1'b0, writeEnable[21],
          1'b0, writeEnable[20], 1'b0, writeEnable[19], 1'b0, writeEnable[18], 1'b0, writeEnable[17],
          1'b0, writeEnable[16], 1'b0, writeEnable[15], 1'b0, writeEnable[14], 1'b0, writeEnable[13],
          1'b0, writeEnable[12], 1'b0, writeEnable[11], 1'b0, writeEnable[10], 1'b0, writeEnable[9],
          1'b0, writeEnable[8], 1'b0, writeEnable[7], 1'b0, writeEnable[6], 1'b0, writeEnable[5],
          1'b0, writeEnable[4], 1'b0, writeEnable[3], 1'b0, writeEnable[2], 1'b0, writeEnable[1],
          1'b0, writeEnable[0]} << mux_address);
        new_data =  ( {1'b0, D_int[143], 1'b0, D_int[142], 1'b0, D_int[141], 1'b0, D_int[140],
          1'b0, D_int[139], 1'b0, D_int[138], 1'b0, D_int[137], 1'b0, D_int[136], 1'b0, D_int[135],
          1'b0, D_int[134], 1'b0, D_int[133], 1'b0, D_int[132], 1'b0, D_int[131], 1'b0, D_int[130],
          1'b0, D_int[129], 1'b0, D_int[128], 1'b0, D_int[127], 1'b0, D_int[126], 1'b0, D_int[125],
          1'b0, D_int[124], 1'b0, D_int[123], 1'b0, D_int[122], 1'b0, D_int[121], 1'b0, D_int[120],
          1'b0, D_int[119], 1'b0, D_int[118], 1'b0, D_int[117], 1'b0, D_int[116], 1'b0, D_int[115],
          1'b0, D_int[114], 1'b0, D_int[113], 1'b0, D_int[112], 1'b0, D_int[111], 1'b0, D_int[110],
          1'b0, D_int[109], 1'b0, D_int[108], 1'b0, D_int[107], 1'b0, D_int[106], 1'b0, D_int[105],
          1'b0, D_int[104], 1'b0, D_int[103], 1'b0, D_int[102], 1'b0, D_int[101], 1'b0, D_int[100],
          1'b0, D_int[99], 1'b0, D_int[98], 1'b0, D_int[97], 1'b0, D_int[96], 1'b0, D_int[95],
          1'b0, D_int[94], 1'b0, D_int[93], 1'b0, D_int[92], 1'b0, D_int[91], 1'b0, D_int[90],
          1'b0, D_int[89], 1'b0, D_int[88], 1'b0, D_int[87], 1'b0, D_int[86], 1'b0, D_int[85],
          1'b0, D_int[84], 1'b0, D_int[83], 1'b0, D_int[82], 1'b0, D_int[81], 1'b0, D_int[80],
          1'b0, D_int[79], 1'b0, D_int[78], 1'b0, D_int[77], 1'b0, D_int[76], 1'b0, D_int[75],
          1'b0, D_int[74], 1'b0, D_int[73], 1'b0, D_int[72], 1'b0, D_int[71], 1'b0, D_int[70],
          1'b0, D_int[69], 1'b0, D_int[68], 1'b0, D_int[67], 1'b0, D_int[66], 1'b0, D_int[65],
          1'b0, D_int[64], 1'b0, D_int[63], 1'b0, D_int[62], 1'b0, D_int[61], 1'b0, D_int[60],
          1'b0, D_int[59], 1'b0, D_int[58], 1'b0, D_int[57], 1'b0, D_int[56], 1'b0, D_int[55],
          1'b0, D_int[54], 1'b0, D_int[53], 1'b0, D_int[52], 1'b0, D_int[51], 1'b0, D_int[50],
          1'b0, D_int[49], 1'b0, D_int[48], 1'b0, D_int[47], 1'b0, D_int[46], 1'b0, D_int[45],
          1'b0, D_int[44], 1'b0, D_int[43], 1'b0, D_int[42], 1'b0, D_int[41], 1'b0, D_int[40],
          1'b0, D_int[39], 1'b0, D_int[38], 1'b0, D_int[37], 1'b0, D_int[36], 1'b0, D_int[35],
          1'b0, D_int[34], 1'b0, D_int[33], 1'b0, D_int[32], 1'b0, D_int[31], 1'b0, D_int[30],
          1'b0, D_int[29], 1'b0, D_int[28], 1'b0, D_int[27], 1'b0, D_int[26], 1'b0, D_int[25],
          1'b0, D_int[24], 1'b0, D_int[23], 1'b0, D_int[22], 1'b0, D_int[21], 1'b0, D_int[20],
          1'b0, D_int[19], 1'b0, D_int[18], 1'b0, D_int[17], 1'b0, D_int[16], 1'b0, D_int[15],
          1'b0, D_int[14], 1'b0, D_int[13], 1'b0, D_int[12], 1'b0, D_int[11], 1'b0, D_int[10],
          1'b0, D_int[9], 1'b0, D_int[8], 1'b0, D_int[7], 1'b0, D_int[6], 1'b0, D_int[5],
          1'b0, D_int[4], 1'b0, D_int[3], 1'b0, D_int[2], 1'b0, D_int[1], 1'b0, D_int[0]} << mux_address);
        row = (row & ~row_mask) | (row_mask & (~row_mask | new_data));
        mem[row_address] = row;
      end else begin
        data_out = (row >> (mux_address));
        readLatch0 = {data_out[286], data_out[284], data_out[282], data_out[280], data_out[278],
          data_out[276], data_out[274], data_out[272], data_out[270], data_out[268],
          data_out[266], data_out[264], data_out[262], data_out[260], data_out[258],
          data_out[256], data_out[254], data_out[252], data_out[250], data_out[248],
          data_out[246], data_out[244], data_out[242], data_out[240], data_out[238],
          data_out[236], data_out[234], data_out[232], data_out[230], data_out[228],
          data_out[226], data_out[224], data_out[222], data_out[220], data_out[218],
          data_out[216], data_out[214], data_out[212], data_out[210], data_out[208],
          data_out[206], data_out[204], data_out[202], data_out[200], data_out[198],
          data_out[196], data_out[194], data_out[192], data_out[190], data_out[188],
          data_out[186], data_out[184], data_out[182], data_out[180], data_out[178],
          data_out[176], data_out[174], data_out[172], data_out[170], data_out[168],
          data_out[166], data_out[164], data_out[162], data_out[160], data_out[158],
          data_out[156], data_out[154], data_out[152], data_out[150], data_out[148],
          data_out[146], data_out[144], data_out[142], data_out[140], data_out[138],
          data_out[136], data_out[134], data_out[132], data_out[130], data_out[128],
          data_out[126], data_out[124], data_out[122], data_out[120], data_out[118],
          data_out[116], data_out[114], data_out[112], data_out[110], data_out[108],
          data_out[106], data_out[104], data_out[102], data_out[100], data_out[98],
          data_out[96], data_out[94], data_out[92], data_out[90], data_out[88], data_out[86],
          data_out[84], data_out[82], data_out[80], data_out[78], data_out[76], data_out[74],
          data_out[72], data_out[70], data_out[68], data_out[66], data_out[64], data_out[62],
          data_out[60], data_out[58], data_out[56], data_out[54], data_out[52], data_out[50],
          data_out[48], data_out[46], data_out[44], data_out[42], data_out[40], data_out[38],
          data_out[36], data_out[34], data_out[32], data_out[30], data_out[28], data_out[26],
          data_out[24], data_out[22], data_out[20], data_out[18], data_out[16], data_out[14],
          data_out[12], data_out[10], data_out[8], data_out[6], data_out[4], data_out[2],
          data_out[0]};
        shifted_readLatch0 = readLatch0;
        Q_int = {shifted_readLatch0[143], shifted_readLatch0[142], shifted_readLatch0[141],
          shifted_readLatch0[140], shifted_readLatch0[139], shifted_readLatch0[138],
          shifted_readLatch0[137], shifted_readLatch0[136], shifted_readLatch0[135],
          shifted_readLatch0[134], shifted_readLatch0[133], shifted_readLatch0[132],
          shifted_readLatch0[131], shifted_readLatch0[130], shifted_readLatch0[129],
          shifted_readLatch0[128], shifted_readLatch0[127], shifted_readLatch0[126],
          shifted_readLatch0[125], shifted_readLatch0[124], shifted_readLatch0[123],
          shifted_readLatch0[122], shifted_readLatch0[121], shifted_readLatch0[120],
          shifted_readLatch0[119], shifted_readLatch0[118], shifted_readLatch0[117],
          shifted_readLatch0[116], shifted_readLatch0[115], shifted_readLatch0[114],
          shifted_readLatch0[113], shifted_readLatch0[112], shifted_readLatch0[111],
          shifted_readLatch0[110], shifted_readLatch0[109], shifted_readLatch0[108],
          shifted_readLatch0[107], shifted_readLatch0[106], shifted_readLatch0[105],
          shifted_readLatch0[104], shifted_readLatch0[103], shifted_readLatch0[102],
          shifted_readLatch0[101], shifted_readLatch0[100], shifted_readLatch0[99],
          shifted_readLatch0[98], shifted_readLatch0[97], shifted_readLatch0[96], shifted_readLatch0[95],
          shifted_readLatch0[94], shifted_readLatch0[93], shifted_readLatch0[92], shifted_readLatch0[91],
          shifted_readLatch0[90], shifted_readLatch0[89], shifted_readLatch0[88], shifted_readLatch0[87],
          shifted_readLatch0[86], shifted_readLatch0[85], shifted_readLatch0[84], shifted_readLatch0[83],
          shifted_readLatch0[82], shifted_readLatch0[81], shifted_readLatch0[80], shifted_readLatch0[79],
          shifted_readLatch0[78], shifted_readLatch0[77], shifted_readLatch0[76], shifted_readLatch0[75],
          shifted_readLatch0[74], shifted_readLatch0[73], shifted_readLatch0[72], shifted_readLatch0[71],
          shifted_readLatch0[70], shifted_readLatch0[69], shifted_readLatch0[68], shifted_readLatch0[67],
          shifted_readLatch0[66], shifted_readLatch0[65], shifted_readLatch0[64], shifted_readLatch0[63],
          shifted_readLatch0[62], shifted_readLatch0[61], shifted_readLatch0[60], shifted_readLatch0[59],
          shifted_readLatch0[58], shifted_readLatch0[57], shifted_readLatch0[56], shifted_readLatch0[55],
          shifted_readLatch0[54], shifted_readLatch0[53], shifted_readLatch0[52], shifted_readLatch0[51],
          shifted_readLatch0[50], shifted_readLatch0[49], shifted_readLatch0[48], shifted_readLatch0[47],
          shifted_readLatch0[46], shifted_readLatch0[45], shifted_readLatch0[44], shifted_readLatch0[43],
          shifted_readLatch0[42], shifted_readLatch0[41], shifted_readLatch0[40], shifted_readLatch0[39],
          shifted_readLatch0[38], shifted_readLatch0[37], shifted_readLatch0[36], shifted_readLatch0[35],
          shifted_readLatch0[34], shifted_readLatch0[33], shifted_readLatch0[32], shifted_readLatch0[31],
          shifted_readLatch0[30], shifted_readLatch0[29], shifted_readLatch0[28], shifted_readLatch0[27],
          shifted_readLatch0[26], shifted_readLatch0[25], shifted_readLatch0[24], shifted_readLatch0[23],
          shifted_readLatch0[22], shifted_readLatch0[21], shifted_readLatch0[20], shifted_readLatch0[19],
          shifted_readLatch0[18], shifted_readLatch0[17], shifted_readLatch0[16], shifted_readLatch0[15],
          shifted_readLatch0[14], shifted_readLatch0[13], shifted_readLatch0[12], shifted_readLatch0[11],
          shifted_readLatch0[10], shifted_readLatch0[9], shifted_readLatch0[8], shifted_readLatch0[7],
          shifted_readLatch0[6], shifted_readLatch0[5], shifted_readLatch0[4], shifted_readLatch0[3],
          shifted_readLatch0[2], shifted_readLatch0[1], shifted_readLatch0[0]};
      end
      if( isBitX(WEN_int) ) begin
        Q_int = {144{1'bx}};
      end
    end
  end
  endtask
  always @ (CEN_ or CLK_) begin
  	if(CLK_ == 1'b0) begin
  		CEN_p2 = CEN_;
  	end
  end

`ifdef POWER_PINS
  always @ (VDDCE) begin
      if (VDDCE != 1'b1) begin
       if (VDDPE == 1'b1) begin
        $display("VDDCE should be powered down after VDDPE, Illegal power down sequencing in %m at %0t", $time);
       end
        $display("In PowerDown Mode in %m at %0t", $time);
        failedWrite(0);
      end
      if (VDDCE == 1'b1) begin
       if (VDDPE == 1'b1) begin
        $display("VDDPE should be powered up after VDDCE in %m at %0t", $time);
        $display("Illegal power up sequencing in %m at %0t", $time);
       end
        failedWrite(0);
      end
  end
`endif
`ifdef POWER_PINS
  always @ (RET1N_ or VDDPE or VDDCE) begin
`else     
  always @ RET1N_ begin
`endif
`ifdef POWER_PINS
    if (RET1N_ == 1'b1 && RET1N_int == 1'b1 && VDDCE == 1'b1 && VDDPE == 1'b1 && pre_charge_st == 1'b1 && (CEN_ === 1'bx || CLK_ === 1'bx)) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end
`else     
`endif
`ifdef POWER_PINS
`else     
      pre_charge_st = 0;
`endif
    if (RET1N_ === 1'bx || RET1N_ === 1'bz) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (RET1N_ === 1'b0 && CEN_p2 === 1'b0 ) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (RET1N_ === 1'b1 && CEN_p2 === 1'b0 ) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end
`ifdef POWER_PINS
    if (RET1N_ == 1'b0 && VDDCE == 1'b1 && VDDPE == 1'b1) begin
      pre_charge_st = 1;
    end else if (RET1N_ == 1'b0 && VDDPE == 1'b0) begin
      pre_charge_st = 0;
      if (VDDCE != 1'b1) begin
        failedWrite(0);
      end
`else     
    if (RET1N_ == 1'b0) begin
`endif
      Q_int = {144{1'bx}};
      CEN_int = 1'bx;
      WEN_int = 1'bx;
      A_int = {9{1'bx}};
      D_int = {144{1'bx}};
      EMA_int = {3{1'bx}};
      EMAW_int = {2{1'bx}};
      RET1N_int = 1'bx;
`ifdef POWER_PINS
    end else if (RET1N_ == 1'b1 && VDDCE == 1'b1 && VDDPE == 1'b1 &&  pre_charge_st == 1'b1) begin
      pre_charge_st = 0;
    end else begin
      pre_charge_st = 0;
`else     
    end else begin
`endif
        Q_int = {144{1'bx}};
      CEN_int = 1'bx;
      WEN_int = 1'bx;
      A_int = {9{1'bx}};
      D_int = {144{1'bx}};
      EMA_int = {3{1'bx}};
      EMAW_int = {2{1'bx}};
      RET1N_int = 1'bx;
    end
    RET1N_int = RET1N_;
  end


  always @ CLK_ begin
// If POWER_PINS is defined at Simulator Command Line, it selects the module definition with Power Ports
`ifdef POWER_PINS
    if (VDDCE === 1'bx || VDDCE === 1'bz)
      $display("Warning: Unknown value for VDDCE %b in %m at %0t", VDDCE, $time);
    if (VDDPE === 1'bx || VDDPE === 1'bz)
      $display("Warning: Unknown value for VDDPE %b in %m at %0t", VDDPE, $time);
    if (VSSE === 1'bx || VSSE === 1'bz)
      $display("Warning: Unknown value for VSSE %b in %m at %0t", VSSE, $time);
`endif
`ifdef POWER_PINS
  if (RET1N_ == 1'b0) begin
`else     
  if (RET1N_ == 1'b0) begin
`endif
      // no cycle in retention mode
  end else begin
    if ((CLK_ === 1'bx || CLK_ === 1'bz) && RET1N_ !== 1'b0) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (CLK_ === 1'b1 && LAST_CLK === 1'b0) begin
      CEN_int = CEN_;
      EMA_int = EMA_;
      EMAW_int = EMAW_;
      RET1N_int = RET1N_;
      if (CEN_int != 1'b1) begin
        WEN_int = WEN_;
        A_int = A_;
        D_int = D_;
      end
      clk0_int = 1'b0;
      CEN_int = CEN_;
      EMA_int = EMA_;
      EMAW_int = EMAW_;
      RET1N_int = RET1N_;
      if (CEN_int != 1'b1) begin
        WEN_int = WEN_;
        A_int = A_;
        D_int = D_;
      end
      clk0_int = 1'b0;
    readWrite;
    end else if (CLK_ === 1'b0 && LAST_CLK === 1'b1) begin
    end
  end
    LAST_CLK = CLK_;
  end
// If POWER_PINS is defined at Simulator Command Line, it selects the module definition with Power Ports
`ifdef POWER_PINS
 always @ (VDDCE or VDDPE or VSSE) begin
    if (VDDCE === 1'bx || VDDCE === 1'bz)
      $display("Warning: Unknown value for VDDCE %b in %m at %0t", VDDCE, $time);
    if (VDDPE === 1'bx || VDDPE === 1'bz)
      $display("Warning: Unknown value for VDDPE %b in %m at %0t", VDDPE, $time);
    if (VSSE === 1'bx || VSSE === 1'bz)
      $display("Warning: Unknown value for VSSE %b in %m at %0t", VSSE, $time);
 end
`endif

endmodule
`endcelldefine
`else
`celldefine
// If POWER_PINS is defined at Simulator Command Line, it selects the module definition with Power Ports
`ifdef POWER_PINS
module sram464x144 (VDDCE, VDDPE, VSSE, Q, CLK, CEN, WEN, A, D, EMA, EMAW, RET1N);
`else
module sram464x144 (Q, CLK, CEN, WEN, A, D, EMA, EMAW, RET1N);
`endif

  parameter ASSERT_PREFIX = "";
  parameter BITS = 144;
  parameter WORDS = 464;
  parameter MUX = 2;
  parameter MEM_WIDTH = 288; // redun block size 2, 144 on left, 144 on right
  parameter MEM_HEIGHT = 232;
  parameter WP_SIZE = 144 ;
  parameter UPM_WIDTH = 3;
  parameter UPMW_WIDTH = 2;
  parameter UPMS_WIDTH = 0;

  output [143:0] Q;
  input  CLK;
  input  CEN;
  input  WEN;
  input [8:0] A;
  input [143:0] D;
  input [2:0] EMA;
  input [1:0] EMAW;
  input  RET1N;
`ifdef POWER_PINS
  inout VDDCE;
  inout VDDPE;
  inout VSSE;
`endif

  reg pre_charge_st;
  integer row_address;
  integer mux_address;
  reg [287:0] mem [0:231];
  reg [287:0] row, row_t;
  reg LAST_CLK;
  reg [287:0] row_mask;
  reg [287:0] new_data;
  reg [287:0] data_out;
  reg [143:0] readLatch0;
  reg [143:0] shifted_readLatch0;
  reg [143:0] Q_int;
  reg [143:0] writeEnable;

  reg NOT_CEN, NOT_WEN, NOT_A8, NOT_A7, NOT_A6, NOT_A5, NOT_A4, NOT_A3, NOT_A2, NOT_A1;
  reg NOT_A0, NOT_D143, NOT_D142, NOT_D141, NOT_D140, NOT_D139, NOT_D138, NOT_D137;
  reg NOT_D136, NOT_D135, NOT_D134, NOT_D133, NOT_D132, NOT_D131, NOT_D130, NOT_D129;
  reg NOT_D128, NOT_D127, NOT_D126, NOT_D125, NOT_D124, NOT_D123, NOT_D122, NOT_D121;
  reg NOT_D120, NOT_D119, NOT_D118, NOT_D117, NOT_D116, NOT_D115, NOT_D114, NOT_D113;
  reg NOT_D112, NOT_D111, NOT_D110, NOT_D109, NOT_D108, NOT_D107, NOT_D106, NOT_D105;
  reg NOT_D104, NOT_D103, NOT_D102, NOT_D101, NOT_D100, NOT_D99, NOT_D98, NOT_D97;
  reg NOT_D96, NOT_D95, NOT_D94, NOT_D93, NOT_D92, NOT_D91, NOT_D90, NOT_D89, NOT_D88;
  reg NOT_D87, NOT_D86, NOT_D85, NOT_D84, NOT_D83, NOT_D82, NOT_D81, NOT_D80, NOT_D79;
  reg NOT_D78, NOT_D77, NOT_D76, NOT_D75, NOT_D74, NOT_D73, NOT_D72, NOT_D71, NOT_D70;
  reg NOT_D69, NOT_D68, NOT_D67, NOT_D66, NOT_D65, NOT_D64, NOT_D63, NOT_D62, NOT_D61;
  reg NOT_D60, NOT_D59, NOT_D58, NOT_D57, NOT_D56, NOT_D55, NOT_D54, NOT_D53, NOT_D52;
  reg NOT_D51, NOT_D50, NOT_D49, NOT_D48, NOT_D47, NOT_D46, NOT_D45, NOT_D44, NOT_D43;
  reg NOT_D42, NOT_D41, NOT_D40, NOT_D39, NOT_D38, NOT_D37, NOT_D36, NOT_D35, NOT_D34;
  reg NOT_D33, NOT_D32, NOT_D31, NOT_D30, NOT_D29, NOT_D28, NOT_D27, NOT_D26, NOT_D25;
  reg NOT_D24, NOT_D23, NOT_D22, NOT_D21, NOT_D20, NOT_D19, NOT_D18, NOT_D17, NOT_D16;
  reg NOT_D15, NOT_D14, NOT_D13, NOT_D12, NOT_D11, NOT_D10, NOT_D9, NOT_D8, NOT_D7;
  reg NOT_D6, NOT_D5, NOT_D4, NOT_D3, NOT_D2, NOT_D1, NOT_D0, NOT_EMA2, NOT_EMA1, NOT_EMA0;
  reg NOT_EMAW1, NOT_EMAW0, NOT_RET1N;
  reg NOT_CLK_PER, NOT_CLK_MINH, NOT_CLK_MINL;
  reg clk0_int;

  wire [143:0] Q_;
 wire  CLK_;
  wire  CEN_;
  reg  CEN_int;
  reg  CEN_p2;
  wire  WEN_;
  reg  WEN_int;
  wire [8:0] A_;
  reg [8:0] A_int;
  wire [143:0] D_;
  reg [143:0] D_int;
  wire [2:0] EMA_;
  reg [2:0] EMA_int;
  wire [1:0] EMAW_;
  reg [1:0] EMAW_int;
  wire  RET1N_;
  reg  RET1N_int;

  buf B0(Q[0], Q_[0]);
  buf B1(Q[1], Q_[1]);
  buf B2(Q[2], Q_[2]);
  buf B3(Q[3], Q_[3]);
  buf B4(Q[4], Q_[4]);
  buf B5(Q[5], Q_[5]);
  buf B6(Q[6], Q_[6]);
  buf B7(Q[7], Q_[7]);
  buf B8(Q[8], Q_[8]);
  buf B9(Q[9], Q_[9]);
  buf B10(Q[10], Q_[10]);
  buf B11(Q[11], Q_[11]);
  buf B12(Q[12], Q_[12]);
  buf B13(Q[13], Q_[13]);
  buf B14(Q[14], Q_[14]);
  buf B15(Q[15], Q_[15]);
  buf B16(Q[16], Q_[16]);
  buf B17(Q[17], Q_[17]);
  buf B18(Q[18], Q_[18]);
  buf B19(Q[19], Q_[19]);
  buf B20(Q[20], Q_[20]);
  buf B21(Q[21], Q_[21]);
  buf B22(Q[22], Q_[22]);
  buf B23(Q[23], Q_[23]);
  buf B24(Q[24], Q_[24]);
  buf B25(Q[25], Q_[25]);
  buf B26(Q[26], Q_[26]);
  buf B27(Q[27], Q_[27]);
  buf B28(Q[28], Q_[28]);
  buf B29(Q[29], Q_[29]);
  buf B30(Q[30], Q_[30]);
  buf B31(Q[31], Q_[31]);
  buf B32(Q[32], Q_[32]);
  buf B33(Q[33], Q_[33]);
  buf B34(Q[34], Q_[34]);
  buf B35(Q[35], Q_[35]);
  buf B36(Q[36], Q_[36]);
  buf B37(Q[37], Q_[37]);
  buf B38(Q[38], Q_[38]);
  buf B39(Q[39], Q_[39]);
  buf B40(Q[40], Q_[40]);
  buf B41(Q[41], Q_[41]);
  buf B42(Q[42], Q_[42]);
  buf B43(Q[43], Q_[43]);
  buf B44(Q[44], Q_[44]);
  buf B45(Q[45], Q_[45]);
  buf B46(Q[46], Q_[46]);
  buf B47(Q[47], Q_[47]);
  buf B48(Q[48], Q_[48]);
  buf B49(Q[49], Q_[49]);
  buf B50(Q[50], Q_[50]);
  buf B51(Q[51], Q_[51]);
  buf B52(Q[52], Q_[52]);
  buf B53(Q[53], Q_[53]);
  buf B54(Q[54], Q_[54]);
  buf B55(Q[55], Q_[55]);
  buf B56(Q[56], Q_[56]);
  buf B57(Q[57], Q_[57]);
  buf B58(Q[58], Q_[58]);
  buf B59(Q[59], Q_[59]);
  buf B60(Q[60], Q_[60]);
  buf B61(Q[61], Q_[61]);
  buf B62(Q[62], Q_[62]);
  buf B63(Q[63], Q_[63]);
  buf B64(Q[64], Q_[64]);
  buf B65(Q[65], Q_[65]);
  buf B66(Q[66], Q_[66]);
  buf B67(Q[67], Q_[67]);
  buf B68(Q[68], Q_[68]);
  buf B69(Q[69], Q_[69]);
  buf B70(Q[70], Q_[70]);
  buf B71(Q[71], Q_[71]);
  buf B72(Q[72], Q_[72]);
  buf B73(Q[73], Q_[73]);
  buf B74(Q[74], Q_[74]);
  buf B75(Q[75], Q_[75]);
  buf B76(Q[76], Q_[76]);
  buf B77(Q[77], Q_[77]);
  buf B78(Q[78], Q_[78]);
  buf B79(Q[79], Q_[79]);
  buf B80(Q[80], Q_[80]);
  buf B81(Q[81], Q_[81]);
  buf B82(Q[82], Q_[82]);
  buf B83(Q[83], Q_[83]);
  buf B84(Q[84], Q_[84]);
  buf B85(Q[85], Q_[85]);
  buf B86(Q[86], Q_[86]);
  buf B87(Q[87], Q_[87]);
  buf B88(Q[88], Q_[88]);
  buf B89(Q[89], Q_[89]);
  buf B90(Q[90], Q_[90]);
  buf B91(Q[91], Q_[91]);
  buf B92(Q[92], Q_[92]);
  buf B93(Q[93], Q_[93]);
  buf B94(Q[94], Q_[94]);
  buf B95(Q[95], Q_[95]);
  buf B96(Q[96], Q_[96]);
  buf B97(Q[97], Q_[97]);
  buf B98(Q[98], Q_[98]);
  buf B99(Q[99], Q_[99]);
  buf B100(Q[100], Q_[100]);
  buf B101(Q[101], Q_[101]);
  buf B102(Q[102], Q_[102]);
  buf B103(Q[103], Q_[103]);
  buf B104(Q[104], Q_[104]);
  buf B105(Q[105], Q_[105]);
  buf B106(Q[106], Q_[106]);
  buf B107(Q[107], Q_[107]);
  buf B108(Q[108], Q_[108]);
  buf B109(Q[109], Q_[109]);
  buf B110(Q[110], Q_[110]);
  buf B111(Q[111], Q_[111]);
  buf B112(Q[112], Q_[112]);
  buf B113(Q[113], Q_[113]);
  buf B114(Q[114], Q_[114]);
  buf B115(Q[115], Q_[115]);
  buf B116(Q[116], Q_[116]);
  buf B117(Q[117], Q_[117]);
  buf B118(Q[118], Q_[118]);
  buf B119(Q[119], Q_[119]);
  buf B120(Q[120], Q_[120]);
  buf B121(Q[121], Q_[121]);
  buf B122(Q[122], Q_[122]);
  buf B123(Q[123], Q_[123]);
  buf B124(Q[124], Q_[124]);
  buf B125(Q[125], Q_[125]);
  buf B126(Q[126], Q_[126]);
  buf B127(Q[127], Q_[127]);
  buf B128(Q[128], Q_[128]);
  buf B129(Q[129], Q_[129]);
  buf B130(Q[130], Q_[130]);
  buf B131(Q[131], Q_[131]);
  buf B132(Q[132], Q_[132]);
  buf B133(Q[133], Q_[133]);
  buf B134(Q[134], Q_[134]);
  buf B135(Q[135], Q_[135]);
  buf B136(Q[136], Q_[136]);
  buf B137(Q[137], Q_[137]);
  buf B138(Q[138], Q_[138]);
  buf B139(Q[139], Q_[139]);
  buf B140(Q[140], Q_[140]);
  buf B141(Q[141], Q_[141]);
  buf B142(Q[142], Q_[142]);
  buf B143(Q[143], Q_[143]);
  buf B144(CLK_, CLK);
  buf B145(CEN_, CEN);
  buf B146(WEN_, WEN);
  buf B147(A_[0], A[0]);
  buf B148(A_[1], A[1]);
  buf B149(A_[2], A[2]);
  buf B150(A_[3], A[3]);
  buf B151(A_[4], A[4]);
  buf B152(A_[5], A[5]);
  buf B153(A_[6], A[6]);
  buf B154(A_[7], A[7]);
  buf B155(A_[8], A[8]);
  buf B156(D_[0], D[0]);
  buf B157(D_[1], D[1]);
  buf B158(D_[2], D[2]);
  buf B159(D_[3], D[3]);
  buf B160(D_[4], D[4]);
  buf B161(D_[5], D[5]);
  buf B162(D_[6], D[6]);
  buf B163(D_[7], D[7]);
  buf B164(D_[8], D[8]);
  buf B165(D_[9], D[9]);
  buf B166(D_[10], D[10]);
  buf B167(D_[11], D[11]);
  buf B168(D_[12], D[12]);
  buf B169(D_[13], D[13]);
  buf B170(D_[14], D[14]);
  buf B171(D_[15], D[15]);
  buf B172(D_[16], D[16]);
  buf B173(D_[17], D[17]);
  buf B174(D_[18], D[18]);
  buf B175(D_[19], D[19]);
  buf B176(D_[20], D[20]);
  buf B177(D_[21], D[21]);
  buf B178(D_[22], D[22]);
  buf B179(D_[23], D[23]);
  buf B180(D_[24], D[24]);
  buf B181(D_[25], D[25]);
  buf B182(D_[26], D[26]);
  buf B183(D_[27], D[27]);
  buf B184(D_[28], D[28]);
  buf B185(D_[29], D[29]);
  buf B186(D_[30], D[30]);
  buf B187(D_[31], D[31]);
  buf B188(D_[32], D[32]);
  buf B189(D_[33], D[33]);
  buf B190(D_[34], D[34]);
  buf B191(D_[35], D[35]);
  buf B192(D_[36], D[36]);
  buf B193(D_[37], D[37]);
  buf B194(D_[38], D[38]);
  buf B195(D_[39], D[39]);
  buf B196(D_[40], D[40]);
  buf B197(D_[41], D[41]);
  buf B198(D_[42], D[42]);
  buf B199(D_[43], D[43]);
  buf B200(D_[44], D[44]);
  buf B201(D_[45], D[45]);
  buf B202(D_[46], D[46]);
  buf B203(D_[47], D[47]);
  buf B204(D_[48], D[48]);
  buf B205(D_[49], D[49]);
  buf B206(D_[50], D[50]);
  buf B207(D_[51], D[51]);
  buf B208(D_[52], D[52]);
  buf B209(D_[53], D[53]);
  buf B210(D_[54], D[54]);
  buf B211(D_[55], D[55]);
  buf B212(D_[56], D[56]);
  buf B213(D_[57], D[57]);
  buf B214(D_[58], D[58]);
  buf B215(D_[59], D[59]);
  buf B216(D_[60], D[60]);
  buf B217(D_[61], D[61]);
  buf B218(D_[62], D[62]);
  buf B219(D_[63], D[63]);
  buf B220(D_[64], D[64]);
  buf B221(D_[65], D[65]);
  buf B222(D_[66], D[66]);
  buf B223(D_[67], D[67]);
  buf B224(D_[68], D[68]);
  buf B225(D_[69], D[69]);
  buf B226(D_[70], D[70]);
  buf B227(D_[71], D[71]);
  buf B228(D_[72], D[72]);
  buf B229(D_[73], D[73]);
  buf B230(D_[74], D[74]);
  buf B231(D_[75], D[75]);
  buf B232(D_[76], D[76]);
  buf B233(D_[77], D[77]);
  buf B234(D_[78], D[78]);
  buf B235(D_[79], D[79]);
  buf B236(D_[80], D[80]);
  buf B237(D_[81], D[81]);
  buf B238(D_[82], D[82]);
  buf B239(D_[83], D[83]);
  buf B240(D_[84], D[84]);
  buf B241(D_[85], D[85]);
  buf B242(D_[86], D[86]);
  buf B243(D_[87], D[87]);
  buf B244(D_[88], D[88]);
  buf B245(D_[89], D[89]);
  buf B246(D_[90], D[90]);
  buf B247(D_[91], D[91]);
  buf B248(D_[92], D[92]);
  buf B249(D_[93], D[93]);
  buf B250(D_[94], D[94]);
  buf B251(D_[95], D[95]);
  buf B252(D_[96], D[96]);
  buf B253(D_[97], D[97]);
  buf B254(D_[98], D[98]);
  buf B255(D_[99], D[99]);
  buf B256(D_[100], D[100]);
  buf B257(D_[101], D[101]);
  buf B258(D_[102], D[102]);
  buf B259(D_[103], D[103]);
  buf B260(D_[104], D[104]);
  buf B261(D_[105], D[105]);
  buf B262(D_[106], D[106]);
  buf B263(D_[107], D[107]);
  buf B264(D_[108], D[108]);
  buf B265(D_[109], D[109]);
  buf B266(D_[110], D[110]);
  buf B267(D_[111], D[111]);
  buf B268(D_[112], D[112]);
  buf B269(D_[113], D[113]);
  buf B270(D_[114], D[114]);
  buf B271(D_[115], D[115]);
  buf B272(D_[116], D[116]);
  buf B273(D_[117], D[117]);
  buf B274(D_[118], D[118]);
  buf B275(D_[119], D[119]);
  buf B276(D_[120], D[120]);
  buf B277(D_[121], D[121]);
  buf B278(D_[122], D[122]);
  buf B279(D_[123], D[123]);
  buf B280(D_[124], D[124]);
  buf B281(D_[125], D[125]);
  buf B282(D_[126], D[126]);
  buf B283(D_[127], D[127]);
  buf B284(D_[128], D[128]);
  buf B285(D_[129], D[129]);
  buf B286(D_[130], D[130]);
  buf B287(D_[131], D[131]);
  buf B288(D_[132], D[132]);
  buf B289(D_[133], D[133]);
  buf B290(D_[134], D[134]);
  buf B291(D_[135], D[135]);
  buf B292(D_[136], D[136]);
  buf B293(D_[137], D[137]);
  buf B294(D_[138], D[138]);
  buf B295(D_[139], D[139]);
  buf B296(D_[140], D[140]);
  buf B297(D_[141], D[141]);
  buf B298(D_[142], D[142]);
  buf B299(D_[143], D[143]);
  buf B300(EMA_[0], EMA[0]);
  buf B301(EMA_[1], EMA[1]);
  buf B302(EMA_[2], EMA[2]);
  buf B303(EMAW_[0], EMAW[0]);
  buf B304(EMAW_[1], EMAW[1]);
  buf B305(RET1N_, RET1N);

   `ifdef ARM_FAULT_MODELING
     sram464x144_error_injection u1(.CLK(CLK_), .Q_out(Q_), .A(A_int), .CEN(CEN_int), .WEN(WEN_int), .Q_in(Q_int));
  `else
  assign Q_ = (RET1N_ | pre_charge_st) ? ((Q_int)) : {144{1'bx}};
  `endif

// If INITIALIZE_MEMORY is defined at Simulator Command Line, it Initializes the Memory with all ZEROS.
`ifdef INITIALIZE_MEMORY
  integer i;
  initial begin
    #0;
    for (i = 0; i < MEM_HEIGHT; i = i + 1)
      mem[i] = {MEM_WIDTH{1'b0}};
  end
`endif
  always @ (EMA_) begin
  	if(EMA_ < 2) 
   	$display("Warning: Set Value for EMA doesn't match Default value 2 in %m at %0t", $time);
  end
  always @ (EMAW_) begin
  	if(EMAW_ < 0) 
   	$display("Warning: Set Value for EMAW doesn't match Default value 0 in %m at %0t", $time);
  end

  task failedWrite;
  input port_f;
  integer i;
  begin
    for (i = 0; i < MEM_HEIGHT; i = i + 1)
      mem[i] = {MEM_WIDTH{1'bx}};
  end
  endtask

  function isBitX;
    input bitval;
    begin
      isBitX = ( bitval===1'bx || bitval==1'bz ) ? 1'b1 : 1'b0;
    end
  endfunction



  task readWrite;
  begin
    if (RET1N_int === 1'bx || RET1N_int === 1'bz) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (RET1N_int === 1'b0 && CEN_int === 1'b0) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (RET1N_int === 1'b0) begin
      // no cycle in retention mode
    end else if (^{CEN_int, EMA_int, EMAW_int, RET1N_int} === 1'bx) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if ((A_int >= WORDS) && (CEN_int === 1'b0)) begin
      Q_int = WEN_int !== 1'b1 ? Q_int : {144{1'bx}};
    end else if (CEN_int === 1'b0 && (^A_int) === 1'bx) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (CEN_int === 1'b0) begin
      mux_address = (A_int & 1'b1);
      row_address = (A_int >> 1);
      if (row_address > 231)
        row = {288{1'bx}};
      else
        row = mem[row_address];
        writeEnable = ~ {144{WEN_int}};
      if (WEN_int !== 1'b1) begin
        row_mask =  ( {1'b0, writeEnable[143], 1'b0, writeEnable[142], 1'b0, writeEnable[141],
          1'b0, writeEnable[140], 1'b0, writeEnable[139], 1'b0, writeEnable[138], 1'b0, writeEnable[137],
          1'b0, writeEnable[136], 1'b0, writeEnable[135], 1'b0, writeEnable[134], 1'b0, writeEnable[133],
          1'b0, writeEnable[132], 1'b0, writeEnable[131], 1'b0, writeEnable[130], 1'b0, writeEnable[129],
          1'b0, writeEnable[128], 1'b0, writeEnable[127], 1'b0, writeEnable[126], 1'b0, writeEnable[125],
          1'b0, writeEnable[124], 1'b0, writeEnable[123], 1'b0, writeEnable[122], 1'b0, writeEnable[121],
          1'b0, writeEnable[120], 1'b0, writeEnable[119], 1'b0, writeEnable[118], 1'b0, writeEnable[117],
          1'b0, writeEnable[116], 1'b0, writeEnable[115], 1'b0, writeEnable[114], 1'b0, writeEnable[113],
          1'b0, writeEnable[112], 1'b0, writeEnable[111], 1'b0, writeEnable[110], 1'b0, writeEnable[109],
          1'b0, writeEnable[108], 1'b0, writeEnable[107], 1'b0, writeEnable[106], 1'b0, writeEnable[105],
          1'b0, writeEnable[104], 1'b0, writeEnable[103], 1'b0, writeEnable[102], 1'b0, writeEnable[101],
          1'b0, writeEnable[100], 1'b0, writeEnable[99], 1'b0, writeEnable[98], 1'b0, writeEnable[97],
          1'b0, writeEnable[96], 1'b0, writeEnable[95], 1'b0, writeEnable[94], 1'b0, writeEnable[93],
          1'b0, writeEnable[92], 1'b0, writeEnable[91], 1'b0, writeEnable[90], 1'b0, writeEnable[89],
          1'b0, writeEnable[88], 1'b0, writeEnable[87], 1'b0, writeEnable[86], 1'b0, writeEnable[85],
          1'b0, writeEnable[84], 1'b0, writeEnable[83], 1'b0, writeEnable[82], 1'b0, writeEnable[81],
          1'b0, writeEnable[80], 1'b0, writeEnable[79], 1'b0, writeEnable[78], 1'b0, writeEnable[77],
          1'b0, writeEnable[76], 1'b0, writeEnable[75], 1'b0, writeEnable[74], 1'b0, writeEnable[73],
          1'b0, writeEnable[72], 1'b0, writeEnable[71], 1'b0, writeEnable[70], 1'b0, writeEnable[69],
          1'b0, writeEnable[68], 1'b0, writeEnable[67], 1'b0, writeEnable[66], 1'b0, writeEnable[65],
          1'b0, writeEnable[64], 1'b0, writeEnable[63], 1'b0, writeEnable[62], 1'b0, writeEnable[61],
          1'b0, writeEnable[60], 1'b0, writeEnable[59], 1'b0, writeEnable[58], 1'b0, writeEnable[57],
          1'b0, writeEnable[56], 1'b0, writeEnable[55], 1'b0, writeEnable[54], 1'b0, writeEnable[53],
          1'b0, writeEnable[52], 1'b0, writeEnable[51], 1'b0, writeEnable[50], 1'b0, writeEnable[49],
          1'b0, writeEnable[48], 1'b0, writeEnable[47], 1'b0, writeEnable[46], 1'b0, writeEnable[45],
          1'b0, writeEnable[44], 1'b0, writeEnable[43], 1'b0, writeEnable[42], 1'b0, writeEnable[41],
          1'b0, writeEnable[40], 1'b0, writeEnable[39], 1'b0, writeEnable[38], 1'b0, writeEnable[37],
          1'b0, writeEnable[36], 1'b0, writeEnable[35], 1'b0, writeEnable[34], 1'b0, writeEnable[33],
          1'b0, writeEnable[32], 1'b0, writeEnable[31], 1'b0, writeEnable[30], 1'b0, writeEnable[29],
          1'b0, writeEnable[28], 1'b0, writeEnable[27], 1'b0, writeEnable[26], 1'b0, writeEnable[25],
          1'b0, writeEnable[24], 1'b0, writeEnable[23], 1'b0, writeEnable[22], 1'b0, writeEnable[21],
          1'b0, writeEnable[20], 1'b0, writeEnable[19], 1'b0, writeEnable[18], 1'b0, writeEnable[17],
          1'b0, writeEnable[16], 1'b0, writeEnable[15], 1'b0, writeEnable[14], 1'b0, writeEnable[13],
          1'b0, writeEnable[12], 1'b0, writeEnable[11], 1'b0, writeEnable[10], 1'b0, writeEnable[9],
          1'b0, writeEnable[8], 1'b0, writeEnable[7], 1'b0, writeEnable[6], 1'b0, writeEnable[5],
          1'b0, writeEnable[4], 1'b0, writeEnable[3], 1'b0, writeEnable[2], 1'b0, writeEnable[1],
          1'b0, writeEnable[0]} << mux_address);
        new_data =  ( {1'b0, D_int[143], 1'b0, D_int[142], 1'b0, D_int[141], 1'b0, D_int[140],
          1'b0, D_int[139], 1'b0, D_int[138], 1'b0, D_int[137], 1'b0, D_int[136], 1'b0, D_int[135],
          1'b0, D_int[134], 1'b0, D_int[133], 1'b0, D_int[132], 1'b0, D_int[131], 1'b0, D_int[130],
          1'b0, D_int[129], 1'b0, D_int[128], 1'b0, D_int[127], 1'b0, D_int[126], 1'b0, D_int[125],
          1'b0, D_int[124], 1'b0, D_int[123], 1'b0, D_int[122], 1'b0, D_int[121], 1'b0, D_int[120],
          1'b0, D_int[119], 1'b0, D_int[118], 1'b0, D_int[117], 1'b0, D_int[116], 1'b0, D_int[115],
          1'b0, D_int[114], 1'b0, D_int[113], 1'b0, D_int[112], 1'b0, D_int[111], 1'b0, D_int[110],
          1'b0, D_int[109], 1'b0, D_int[108], 1'b0, D_int[107], 1'b0, D_int[106], 1'b0, D_int[105],
          1'b0, D_int[104], 1'b0, D_int[103], 1'b0, D_int[102], 1'b0, D_int[101], 1'b0, D_int[100],
          1'b0, D_int[99], 1'b0, D_int[98], 1'b0, D_int[97], 1'b0, D_int[96], 1'b0, D_int[95],
          1'b0, D_int[94], 1'b0, D_int[93], 1'b0, D_int[92], 1'b0, D_int[91], 1'b0, D_int[90],
          1'b0, D_int[89], 1'b0, D_int[88], 1'b0, D_int[87], 1'b0, D_int[86], 1'b0, D_int[85],
          1'b0, D_int[84], 1'b0, D_int[83], 1'b0, D_int[82], 1'b0, D_int[81], 1'b0, D_int[80],
          1'b0, D_int[79], 1'b0, D_int[78], 1'b0, D_int[77], 1'b0, D_int[76], 1'b0, D_int[75],
          1'b0, D_int[74], 1'b0, D_int[73], 1'b0, D_int[72], 1'b0, D_int[71], 1'b0, D_int[70],
          1'b0, D_int[69], 1'b0, D_int[68], 1'b0, D_int[67], 1'b0, D_int[66], 1'b0, D_int[65],
          1'b0, D_int[64], 1'b0, D_int[63], 1'b0, D_int[62], 1'b0, D_int[61], 1'b0, D_int[60],
          1'b0, D_int[59], 1'b0, D_int[58], 1'b0, D_int[57], 1'b0, D_int[56], 1'b0, D_int[55],
          1'b0, D_int[54], 1'b0, D_int[53], 1'b0, D_int[52], 1'b0, D_int[51], 1'b0, D_int[50],
          1'b0, D_int[49], 1'b0, D_int[48], 1'b0, D_int[47], 1'b0, D_int[46], 1'b0, D_int[45],
          1'b0, D_int[44], 1'b0, D_int[43], 1'b0, D_int[42], 1'b0, D_int[41], 1'b0, D_int[40],
          1'b0, D_int[39], 1'b0, D_int[38], 1'b0, D_int[37], 1'b0, D_int[36], 1'b0, D_int[35],
          1'b0, D_int[34], 1'b0, D_int[33], 1'b0, D_int[32], 1'b0, D_int[31], 1'b0, D_int[30],
          1'b0, D_int[29], 1'b0, D_int[28], 1'b0, D_int[27], 1'b0, D_int[26], 1'b0, D_int[25],
          1'b0, D_int[24], 1'b0, D_int[23], 1'b0, D_int[22], 1'b0, D_int[21], 1'b0, D_int[20],
          1'b0, D_int[19], 1'b0, D_int[18], 1'b0, D_int[17], 1'b0, D_int[16], 1'b0, D_int[15],
          1'b0, D_int[14], 1'b0, D_int[13], 1'b0, D_int[12], 1'b0, D_int[11], 1'b0, D_int[10],
          1'b0, D_int[9], 1'b0, D_int[8], 1'b0, D_int[7], 1'b0, D_int[6], 1'b0, D_int[5],
          1'b0, D_int[4], 1'b0, D_int[3], 1'b0, D_int[2], 1'b0, D_int[1], 1'b0, D_int[0]} << mux_address);
        row = (row & ~row_mask) | (row_mask & (~row_mask | new_data));
        mem[row_address] = row;
      end else begin
        data_out = (row >> (mux_address));
        readLatch0 = {data_out[286], data_out[284], data_out[282], data_out[280], data_out[278],
          data_out[276], data_out[274], data_out[272], data_out[270], data_out[268],
          data_out[266], data_out[264], data_out[262], data_out[260], data_out[258],
          data_out[256], data_out[254], data_out[252], data_out[250], data_out[248],
          data_out[246], data_out[244], data_out[242], data_out[240], data_out[238],
          data_out[236], data_out[234], data_out[232], data_out[230], data_out[228],
          data_out[226], data_out[224], data_out[222], data_out[220], data_out[218],
          data_out[216], data_out[214], data_out[212], data_out[210], data_out[208],
          data_out[206], data_out[204], data_out[202], data_out[200], data_out[198],
          data_out[196], data_out[194], data_out[192], data_out[190], data_out[188],
          data_out[186], data_out[184], data_out[182], data_out[180], data_out[178],
          data_out[176], data_out[174], data_out[172], data_out[170], data_out[168],
          data_out[166], data_out[164], data_out[162], data_out[160], data_out[158],
          data_out[156], data_out[154], data_out[152], data_out[150], data_out[148],
          data_out[146], data_out[144], data_out[142], data_out[140], data_out[138],
          data_out[136], data_out[134], data_out[132], data_out[130], data_out[128],
          data_out[126], data_out[124], data_out[122], data_out[120], data_out[118],
          data_out[116], data_out[114], data_out[112], data_out[110], data_out[108],
          data_out[106], data_out[104], data_out[102], data_out[100], data_out[98],
          data_out[96], data_out[94], data_out[92], data_out[90], data_out[88], data_out[86],
          data_out[84], data_out[82], data_out[80], data_out[78], data_out[76], data_out[74],
          data_out[72], data_out[70], data_out[68], data_out[66], data_out[64], data_out[62],
          data_out[60], data_out[58], data_out[56], data_out[54], data_out[52], data_out[50],
          data_out[48], data_out[46], data_out[44], data_out[42], data_out[40], data_out[38],
          data_out[36], data_out[34], data_out[32], data_out[30], data_out[28], data_out[26],
          data_out[24], data_out[22], data_out[20], data_out[18], data_out[16], data_out[14],
          data_out[12], data_out[10], data_out[8], data_out[6], data_out[4], data_out[2],
          data_out[0]};
        shifted_readLatch0 = readLatch0;
        Q_int = {shifted_readLatch0[143], shifted_readLatch0[142], shifted_readLatch0[141],
          shifted_readLatch0[140], shifted_readLatch0[139], shifted_readLatch0[138],
          shifted_readLatch0[137], shifted_readLatch0[136], shifted_readLatch0[135],
          shifted_readLatch0[134], shifted_readLatch0[133], shifted_readLatch0[132],
          shifted_readLatch0[131], shifted_readLatch0[130], shifted_readLatch0[129],
          shifted_readLatch0[128], shifted_readLatch0[127], shifted_readLatch0[126],
          shifted_readLatch0[125], shifted_readLatch0[124], shifted_readLatch0[123],
          shifted_readLatch0[122], shifted_readLatch0[121], shifted_readLatch0[120],
          shifted_readLatch0[119], shifted_readLatch0[118], shifted_readLatch0[117],
          shifted_readLatch0[116], shifted_readLatch0[115], shifted_readLatch0[114],
          shifted_readLatch0[113], shifted_readLatch0[112], shifted_readLatch0[111],
          shifted_readLatch0[110], shifted_readLatch0[109], shifted_readLatch0[108],
          shifted_readLatch0[107], shifted_readLatch0[106], shifted_readLatch0[105],
          shifted_readLatch0[104], shifted_readLatch0[103], shifted_readLatch0[102],
          shifted_readLatch0[101], shifted_readLatch0[100], shifted_readLatch0[99],
          shifted_readLatch0[98], shifted_readLatch0[97], shifted_readLatch0[96], shifted_readLatch0[95],
          shifted_readLatch0[94], shifted_readLatch0[93], shifted_readLatch0[92], shifted_readLatch0[91],
          shifted_readLatch0[90], shifted_readLatch0[89], shifted_readLatch0[88], shifted_readLatch0[87],
          shifted_readLatch0[86], shifted_readLatch0[85], shifted_readLatch0[84], shifted_readLatch0[83],
          shifted_readLatch0[82], shifted_readLatch0[81], shifted_readLatch0[80], shifted_readLatch0[79],
          shifted_readLatch0[78], shifted_readLatch0[77], shifted_readLatch0[76], shifted_readLatch0[75],
          shifted_readLatch0[74], shifted_readLatch0[73], shifted_readLatch0[72], shifted_readLatch0[71],
          shifted_readLatch0[70], shifted_readLatch0[69], shifted_readLatch0[68], shifted_readLatch0[67],
          shifted_readLatch0[66], shifted_readLatch0[65], shifted_readLatch0[64], shifted_readLatch0[63],
          shifted_readLatch0[62], shifted_readLatch0[61], shifted_readLatch0[60], shifted_readLatch0[59],
          shifted_readLatch0[58], shifted_readLatch0[57], shifted_readLatch0[56], shifted_readLatch0[55],
          shifted_readLatch0[54], shifted_readLatch0[53], shifted_readLatch0[52], shifted_readLatch0[51],
          shifted_readLatch0[50], shifted_readLatch0[49], shifted_readLatch0[48], shifted_readLatch0[47],
          shifted_readLatch0[46], shifted_readLatch0[45], shifted_readLatch0[44], shifted_readLatch0[43],
          shifted_readLatch0[42], shifted_readLatch0[41], shifted_readLatch0[40], shifted_readLatch0[39],
          shifted_readLatch0[38], shifted_readLatch0[37], shifted_readLatch0[36], shifted_readLatch0[35],
          shifted_readLatch0[34], shifted_readLatch0[33], shifted_readLatch0[32], shifted_readLatch0[31],
          shifted_readLatch0[30], shifted_readLatch0[29], shifted_readLatch0[28], shifted_readLatch0[27],
          shifted_readLatch0[26], shifted_readLatch0[25], shifted_readLatch0[24], shifted_readLatch0[23],
          shifted_readLatch0[22], shifted_readLatch0[21], shifted_readLatch0[20], shifted_readLatch0[19],
          shifted_readLatch0[18], shifted_readLatch0[17], shifted_readLatch0[16], shifted_readLatch0[15],
          shifted_readLatch0[14], shifted_readLatch0[13], shifted_readLatch0[12], shifted_readLatch0[11],
          shifted_readLatch0[10], shifted_readLatch0[9], shifted_readLatch0[8], shifted_readLatch0[7],
          shifted_readLatch0[6], shifted_readLatch0[5], shifted_readLatch0[4], shifted_readLatch0[3],
          shifted_readLatch0[2], shifted_readLatch0[1], shifted_readLatch0[0]};
      end
      if( isBitX(WEN_int) ) begin
        Q_int = {144{1'bx}};
      end
    end
  end
  endtask
  always @ (CEN_ or CLK_) begin
  	if(CLK_ == 1'b0) begin
  		CEN_p2 = CEN_;
  	end
  end

`ifdef POWER_PINS
  always @ (VDDCE) begin
      if (VDDCE != 1'b1) begin
       if (VDDPE == 1'b1) begin
        $display("VDDCE should be powered down after VDDPE, Illegal power down sequencing in %m at %0t", $time);
       end
        $display("In PowerDown Mode in %m at %0t", $time);
        failedWrite(0);
      end
      if (VDDCE == 1'b1) begin
       if (VDDPE == 1'b1) begin
        $display("VDDPE should be powered up after VDDCE in %m at %0t", $time);
        $display("Illegal power up sequencing in %m at %0t", $time);
       end
        failedWrite(0);
      end
  end
`endif
`ifdef POWER_PINS
  always @ (RET1N_ or VDDPE or VDDCE) begin
`else     
  always @ RET1N_ begin
`endif
`ifdef POWER_PINS
    if (RET1N_ == 1'b1 && RET1N_int == 1'b1 && VDDCE == 1'b1 && VDDPE == 1'b1 && pre_charge_st == 1'b1 && (CEN_ === 1'bx || CLK_ === 1'bx)) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end
`else     
`endif
`ifdef POWER_PINS
`else     
      pre_charge_st = 0;
`endif
    if (RET1N_ === 1'bx || RET1N_ === 1'bz) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (RET1N_ === 1'b0 && CEN_p2 === 1'b0 ) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (RET1N_ === 1'b1 && CEN_p2 === 1'b0 ) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end
`ifdef POWER_PINS
    if (RET1N_ == 1'b0 && VDDCE == 1'b1 && VDDPE == 1'b1) begin
      pre_charge_st = 1;
    end else if (RET1N_ == 1'b0 && VDDPE == 1'b0) begin
      pre_charge_st = 0;
      if (VDDCE != 1'b1) begin
        failedWrite(0);
      end
`else     
    if (RET1N_ == 1'b0) begin
`endif
      Q_int = {144{1'bx}};
      CEN_int = 1'bx;
      WEN_int = 1'bx;
      A_int = {9{1'bx}};
      D_int = {144{1'bx}};
      EMA_int = {3{1'bx}};
      EMAW_int = {2{1'bx}};
      RET1N_int = 1'bx;
`ifdef POWER_PINS
    end else if (RET1N_ == 1'b1 && VDDCE == 1'b1 && VDDPE == 1'b1 &&  pre_charge_st == 1'b1) begin
      pre_charge_st = 0;
    end else begin
      pre_charge_st = 0;
`else     
    end else begin
`endif
        Q_int = {144{1'bx}};
      CEN_int = 1'bx;
      WEN_int = 1'bx;
      A_int = {9{1'bx}};
      D_int = {144{1'bx}};
      EMA_int = {3{1'bx}};
      EMAW_int = {2{1'bx}};
      RET1N_int = 1'bx;
    end
    RET1N_int = RET1N_;
  end


  always @ CLK_ begin
// If POWER_PINS is defined at Simulator Command Line, it selects the module definition with Power Ports
`ifdef POWER_PINS
    if (VDDCE === 1'bx || VDDCE === 1'bz)
      $display("Warning: Unknown value for VDDCE %b in %m at %0t", VDDCE, $time);
    if (VDDPE === 1'bx || VDDPE === 1'bz)
      $display("Warning: Unknown value for VDDPE %b in %m at %0t", VDDPE, $time);
    if (VSSE === 1'bx || VSSE === 1'bz)
      $display("Warning: Unknown value for VSSE %b in %m at %0t", VSSE, $time);
`endif
`ifdef POWER_PINS
  if (RET1N_ == 1'b0) begin
`else     
  if (RET1N_ == 1'b0) begin
`endif
      // no cycle in retention mode
  end else begin
    if ((CLK_ === 1'bx || CLK_ === 1'bz) && RET1N_ !== 1'b0) begin
      failedWrite(0);
        Q_int = {144{1'bx}};
    end else if (CLK_ === 1'b1 && LAST_CLK === 1'b0) begin
      CEN_int = CEN_;
      EMA_int = EMA_;
      EMAW_int = EMAW_;
      RET1N_int = RET1N_;
      if (CEN_int != 1'b1) begin
        WEN_int = WEN_;
        A_int = A_;
        D_int = D_;
      end
      clk0_int = 1'b0;
      CEN_int = CEN_;
      EMA_int = EMA_;
      EMAW_int = EMAW_;
      RET1N_int = RET1N_;
      if (CEN_int != 1'b1) begin
        WEN_int = WEN_;
        A_int = A_;
        D_int = D_;
      end
      clk0_int = 1'b0;
    readWrite;
    end else if (CLK_ === 1'b0 && LAST_CLK === 1'b1) begin
    end
  end
    LAST_CLK = CLK_;
  end

  reg globalNotifier0;
  initial globalNotifier0 = 1'b0;

  always @ globalNotifier0 begin
    if ($realtime == 0) begin
    end else if (CEN_int === 1'bx || EMAW_int[0] === 1'bx || EMAW_int[1] === 1'bx || 
      EMA_int[0] === 1'bx || EMA_int[1] === 1'bx || EMA_int[2] === 1'bx || RET1N_int === 1'bx || 
      clk0_int === 1'bx) begin
        Q_int = {144{1'bx}};
      failedWrite(0);
    end else begin
      #0;
      readWrite;
   end
    globalNotifier0 = 1'b0;
  end
// If POWER_PINS is defined at Simulator Command Line, it selects the module definition with Power Ports
`ifdef POWER_PINS
 always @ (VDDCE or VDDPE or VSSE) begin
    if (VDDCE === 1'bx || VDDCE === 1'bz)
      $display("Warning: Unknown value for VDDCE %b in %m at %0t", VDDCE, $time);
    if (VDDPE === 1'bx || VDDPE === 1'bz)
      $display("Warning: Unknown value for VDDPE %b in %m at %0t", VDDPE, $time);
    if (VSSE === 1'bx || VSSE === 1'bz)
      $display("Warning: Unknown value for VSSE %b in %m at %0t", VSSE, $time);
 end
`endif

  always @ NOT_CEN begin
    CEN_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_WEN begin
    WEN_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A8 begin
    A_int[8] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A7 begin
    A_int[7] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A6 begin
    A_int[6] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A5 begin
    A_int[5] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A4 begin
    A_int[4] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A3 begin
    A_int[3] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A2 begin
    A_int[2] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A1 begin
    A_int[1] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A0 begin
    A_int[0] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D143 begin
    D_int[143] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D142 begin
    D_int[142] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D141 begin
    D_int[141] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D140 begin
    D_int[140] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D139 begin
    D_int[139] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D138 begin
    D_int[138] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D137 begin
    D_int[137] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D136 begin
    D_int[136] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D135 begin
    D_int[135] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D134 begin
    D_int[134] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D133 begin
    D_int[133] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D132 begin
    D_int[132] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D131 begin
    D_int[131] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D130 begin
    D_int[130] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D129 begin
    D_int[129] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D128 begin
    D_int[128] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D127 begin
    D_int[127] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D126 begin
    D_int[126] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D125 begin
    D_int[125] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D124 begin
    D_int[124] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D123 begin
    D_int[123] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D122 begin
    D_int[122] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D121 begin
    D_int[121] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D120 begin
    D_int[120] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D119 begin
    D_int[119] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D118 begin
    D_int[118] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D117 begin
    D_int[117] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D116 begin
    D_int[116] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D115 begin
    D_int[115] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D114 begin
    D_int[114] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D113 begin
    D_int[113] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D112 begin
    D_int[112] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D111 begin
    D_int[111] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D110 begin
    D_int[110] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D109 begin
    D_int[109] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D108 begin
    D_int[108] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D107 begin
    D_int[107] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D106 begin
    D_int[106] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D105 begin
    D_int[105] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D104 begin
    D_int[104] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D103 begin
    D_int[103] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D102 begin
    D_int[102] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D101 begin
    D_int[101] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D100 begin
    D_int[100] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D99 begin
    D_int[99] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D98 begin
    D_int[98] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D97 begin
    D_int[97] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D96 begin
    D_int[96] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D95 begin
    D_int[95] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D94 begin
    D_int[94] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D93 begin
    D_int[93] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D92 begin
    D_int[92] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D91 begin
    D_int[91] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D90 begin
    D_int[90] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D89 begin
    D_int[89] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D88 begin
    D_int[88] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D87 begin
    D_int[87] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D86 begin
    D_int[86] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D85 begin
    D_int[85] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D84 begin
    D_int[84] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D83 begin
    D_int[83] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D82 begin
    D_int[82] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D81 begin
    D_int[81] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D80 begin
    D_int[80] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D79 begin
    D_int[79] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D78 begin
    D_int[78] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D77 begin
    D_int[77] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D76 begin
    D_int[76] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D75 begin
    D_int[75] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D74 begin
    D_int[74] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D73 begin
    D_int[73] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D72 begin
    D_int[72] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D71 begin
    D_int[71] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D70 begin
    D_int[70] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D69 begin
    D_int[69] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D68 begin
    D_int[68] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D67 begin
    D_int[67] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D66 begin
    D_int[66] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D65 begin
    D_int[65] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D64 begin
    D_int[64] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D63 begin
    D_int[63] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D62 begin
    D_int[62] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D61 begin
    D_int[61] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D60 begin
    D_int[60] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D59 begin
    D_int[59] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D58 begin
    D_int[58] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D57 begin
    D_int[57] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D56 begin
    D_int[56] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D55 begin
    D_int[55] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D54 begin
    D_int[54] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D53 begin
    D_int[53] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D52 begin
    D_int[52] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D51 begin
    D_int[51] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D50 begin
    D_int[50] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D49 begin
    D_int[49] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D48 begin
    D_int[48] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D47 begin
    D_int[47] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D46 begin
    D_int[46] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D45 begin
    D_int[45] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D44 begin
    D_int[44] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D43 begin
    D_int[43] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D42 begin
    D_int[42] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D41 begin
    D_int[41] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D40 begin
    D_int[40] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D39 begin
    D_int[39] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D38 begin
    D_int[38] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D37 begin
    D_int[37] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D36 begin
    D_int[36] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D35 begin
    D_int[35] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D34 begin
    D_int[34] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D33 begin
    D_int[33] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D32 begin
    D_int[32] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D31 begin
    D_int[31] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D30 begin
    D_int[30] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D29 begin
    D_int[29] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D28 begin
    D_int[28] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D27 begin
    D_int[27] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D26 begin
    D_int[26] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D25 begin
    D_int[25] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D24 begin
    D_int[24] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D23 begin
    D_int[23] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D22 begin
    D_int[22] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D21 begin
    D_int[21] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D20 begin
    D_int[20] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D19 begin
    D_int[19] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D18 begin
    D_int[18] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D17 begin
    D_int[17] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D16 begin
    D_int[16] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D15 begin
    D_int[15] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D14 begin
    D_int[14] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D13 begin
    D_int[13] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D12 begin
    D_int[12] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D11 begin
    D_int[11] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D10 begin
    D_int[10] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D9 begin
    D_int[9] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D8 begin
    D_int[8] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D7 begin
    D_int[7] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D6 begin
    D_int[6] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D5 begin
    D_int[5] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D4 begin
    D_int[4] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D3 begin
    D_int[3] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D2 begin
    D_int[2] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D1 begin
    D_int[1] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D0 begin
    D_int[0] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_EMA2 begin
    EMA_int[2] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_EMA1 begin
    EMA_int[1] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_EMA0 begin
    EMA_int[0] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_EMAW1 begin
    EMAW_int[1] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_EMAW0 begin
    EMAW_int[0] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_RET1N begin
    RET1N_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end

  always @ NOT_CLK_PER begin
    clk0_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_CLK_MINH begin
    clk0_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_CLK_MINL begin
    clk0_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end



  wire RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0;
  wire RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0, RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0;
  wire RET1Neq1, RET1Neq1aCENeq0aWENeq0, RET1Neq1aCENeq0;


  assign RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0 = RET1N && !EMA[2] && !EMA[1] && !EMA[0] && !EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0 = RET1N && !EMA[2] && !EMA[1] && !EMA[0] && !EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0 = RET1N && !EMA[2] && !EMA[1] && !EMA[0] && EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0 = RET1N && !EMA[2] && !EMA[1] && !EMA[0] && EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0 = RET1N && !EMA[2] && !EMA[1] && EMA[0] && !EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0 = RET1N && !EMA[2] && !EMA[1] && EMA[0] && !EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0 = RET1N && !EMA[2] && !EMA[1] && EMA[0] && EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0 = RET1N && !EMA[2] && !EMA[1] && EMA[0] && EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0 = RET1N && !EMA[2] && EMA[1] && !EMA[0] && !EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0 = RET1N && !EMA[2] && EMA[1] && !EMA[0] && !EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0 = RET1N && !EMA[2] && EMA[1] && !EMA[0] && EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0 = RET1N && !EMA[2] && EMA[1] && !EMA[0] && EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0 = RET1N && !EMA[2] && EMA[1] && EMA[0] && !EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0 = RET1N && !EMA[2] && EMA[1] && EMA[0] && !EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0 = RET1N && !EMA[2] && EMA[1] && EMA[0] && EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0 = RET1N && !EMA[2] && EMA[1] && EMA[0] && EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0 = RET1N && EMA[2] && !EMA[1] && !EMA[0] && !EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0 = RET1N && EMA[2] && !EMA[1] && !EMA[0] && !EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0 = RET1N && EMA[2] && !EMA[1] && !EMA[0] && EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0 = RET1N && EMA[2] && !EMA[1] && !EMA[0] && EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0 = RET1N && EMA[2] && !EMA[1] && EMA[0] && !EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0 = RET1N && EMA[2] && !EMA[1] && EMA[0] && !EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0 = RET1N && EMA[2] && !EMA[1] && EMA[0] && EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0 = RET1N && EMA[2] && !EMA[1] && EMA[0] && EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0 = RET1N && EMA[2] && EMA[1] && !EMA[0] && !EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0 = RET1N && EMA[2] && EMA[1] && !EMA[0] && !EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0 = RET1N && EMA[2] && EMA[1] && !EMA[0] && EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0 = RET1N && EMA[2] && EMA[1] && !EMA[0] && EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0 = RET1N && EMA[2] && EMA[1] && EMA[0] && !EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0 = RET1N && EMA[2] && EMA[1] && EMA[0] && !EMAW[1] && EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0 = RET1N && EMA[2] && EMA[1] && EMA[0] && EMAW[1] && !EMAW[0] && !CEN;
  assign RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0 = RET1N && EMA[2] && EMA[1] && EMA[0] && EMAW[1] && EMAW[0] && !CEN;

  assign RET1Neq1aCENeq0aWENeq0 = RET1N && !CEN && !WEN;

  assign RET1Neq1 = RET1N;
  assign RET1Neq1aCENeq0 = RET1N && !CEN;

  specify

    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[143] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[142] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[141] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[140] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[139] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[138] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[137] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[136] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[135] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[134] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[133] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[132] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[131] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[130] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[129] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[128] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[127] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[126] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[125] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[124] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[123] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[122] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[121] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[120] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[119] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[118] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[117] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[116] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[115] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[114] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[113] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[112] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[111] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[110] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[109] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[108] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[107] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[106] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[105] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[104] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[103] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[102] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[101] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[100] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[99] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[98] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[97] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[96] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[95] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[94] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[93] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[92] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[91] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[90] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[89] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[88] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[87] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[86] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[85] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[84] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[83] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[82] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[81] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[80] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[79] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[78] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[77] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[76] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[75] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[74] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[73] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[72] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[71] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[70] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[69] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[68] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[67] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[66] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[65] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[64] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[63] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[62] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[61] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[60] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[59] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[58] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[57] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[56] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[55] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[54] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[53] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[52] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[51] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[50] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[49] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[48] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[47] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[46] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[45] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[44] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[43] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[42] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[41] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[40] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[39] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[38] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[37] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[36] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[35] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[34] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[33] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[32] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[31] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[30] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[29] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[28] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[27] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[26] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[25] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[24] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[23] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[22] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[21] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[20] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[19] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[18] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[17] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[16] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[15] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[14] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[13] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[12] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[11] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[10] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[9] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[8] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[7] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[6] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[5] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[4] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[3] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[2] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[1] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[0] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[143] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[142] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[141] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[140] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[139] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[138] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[137] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[136] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[135] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[134] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[133] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[132] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[131] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[130] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[129] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[128] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[127] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[126] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[125] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[124] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[123] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[122] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[121] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[120] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[119] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[118] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[117] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[116] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[115] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[114] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[113] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[112] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[111] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[110] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[109] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[108] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[107] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[106] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[105] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[104] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[103] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[102] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[101] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[100] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[99] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[98] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[97] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[96] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[95] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[94] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[93] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[92] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[91] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[90] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[89] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[88] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[87] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[86] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[85] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[84] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[83] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[82] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[81] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[80] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[79] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[78] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[77] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[76] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[75] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[74] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[73] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[72] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[71] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[70] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[69] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[68] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[67] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[66] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[65] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[64] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[63] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[62] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[61] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[60] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[59] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[58] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[57] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[56] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[55] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[54] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[53] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[52] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[51] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[50] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[49] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[48] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[47] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[46] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[45] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[44] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[43] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[42] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[41] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[40] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[39] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[38] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[37] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[36] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[35] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[34] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[33] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[32] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[31] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[30] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[29] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[28] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[27] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[26] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[25] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[24] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[23] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[22] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[21] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[20] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[19] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[18] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[17] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[16] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[15] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[14] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[13] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[12] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[11] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[10] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[9] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[8] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[7] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[6] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[5] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[4] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[3] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[2] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[1] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[0] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[143] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[142] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[141] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[140] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[139] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[138] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[137] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[136] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[135] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[134] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[133] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[132] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[131] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[130] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[129] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[128] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[127] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[126] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[125] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[124] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[123] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[122] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[121] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[120] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[119] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[118] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[117] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[116] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[115] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[114] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[113] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[112] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[111] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[110] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[109] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[108] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[107] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[106] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[105] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[104] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[103] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[102] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[101] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[100] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[99] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[98] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[97] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[96] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[95] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[94] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[93] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[92] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[91] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[90] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[89] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[88] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[87] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[86] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[85] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[84] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[83] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[82] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[81] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[80] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[79] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[78] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[77] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[76] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[75] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[74] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[73] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[72] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[71] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[70] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[69] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[68] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[67] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[66] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[65] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[64] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[63] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[62] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[61] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[60] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[59] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[58] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[57] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[56] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[55] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[54] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[53] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[52] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[51] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[50] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[49] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[48] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[47] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[46] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[45] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[44] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[43] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[42] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[41] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[40] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[39] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[38] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[37] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[36] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[35] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[34] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[33] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[32] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[31] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[30] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[29] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[28] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[27] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[26] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[25] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[24] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[23] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[22] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[21] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[20] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[19] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[18] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[17] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[16] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[15] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[14] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[13] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[12] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[11] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[10] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[9] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[8] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[7] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[6] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[5] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[4] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[3] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[2] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[1] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[0] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[143] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[142] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[141] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[140] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[139] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[138] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[137] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[136] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[135] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[134] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[133] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[132] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[131] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[130] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[129] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[128] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[127] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[126] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[125] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[124] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[123] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[122] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[121] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[120] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[119] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[118] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[117] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[116] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[115] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[114] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[113] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[112] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[111] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[110] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[109] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[108] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[107] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[106] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[105] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[104] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[103] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[102] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[101] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[100] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[99] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[98] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[97] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[96] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[95] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[94] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[93] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[92] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[91] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[90] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[89] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[88] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[87] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[86] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[85] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[84] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[83] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[82] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[81] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[80] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[79] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[78] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[77] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[76] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[75] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[74] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[73] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[72] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[71] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[70] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[69] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[68] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[67] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[66] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[65] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[64] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[63] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[62] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[61] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[60] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[59] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[58] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[57] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[56] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[55] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[54] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[53] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[52] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[51] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[50] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[49] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[48] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[47] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[46] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[45] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[44] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[43] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[42] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[41] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[40] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[39] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[38] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[37] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[36] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[35] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[34] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[33] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[32] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[31] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[30] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[29] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[28] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[27] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[26] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[25] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[24] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[23] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[22] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[21] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[20] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[19] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[18] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[17] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[16] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[15] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[14] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[13] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[12] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[11] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[10] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[9] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[8] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[7] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[6] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[5] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[4] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[3] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[2] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[1] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b0 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[0] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[143] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[142] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[141] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[140] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[139] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[138] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[137] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[136] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[135] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[134] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[133] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[132] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[131] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[130] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[129] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[128] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[127] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[126] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[125] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[124] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[123] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[122] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[121] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[120] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[119] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[118] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[117] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[116] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[115] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[114] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[113] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[112] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[111] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[110] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[109] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[108] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[107] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[106] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[105] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[104] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[103] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[102] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[101] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[100] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[99] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[98] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[97] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[96] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[95] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[94] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[93] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[92] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[91] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[90] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[89] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[88] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[87] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[86] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[85] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[84] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[83] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[82] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[81] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[80] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[79] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[78] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[77] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[76] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[75] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[74] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[73] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[72] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[71] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[70] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[69] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[68] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[67] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[66] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[65] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[64] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[63] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[62] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[61] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[60] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[59] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[58] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[57] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[56] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[55] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[54] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[53] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[52] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[51] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[50] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[49] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[48] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[47] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[46] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[45] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[44] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[43] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[42] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[41] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[40] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[39] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[38] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[37] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[36] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[35] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[34] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[33] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[32] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[31] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[30] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[29] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[28] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[27] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[26] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[25] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[24] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[23] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[22] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[21] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[20] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[19] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[18] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[17] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[16] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[15] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[14] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[13] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[12] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[11] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[10] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[9] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[8] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[7] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[6] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[5] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[4] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[3] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[2] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[1] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[0] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[143] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[142] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[141] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[140] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[139] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[138] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[137] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[136] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[135] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[134] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[133] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[132] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[131] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[130] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[129] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[128] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[127] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[126] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[125] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[124] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[123] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[122] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[121] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[120] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[119] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[118] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[117] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[116] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[115] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[114] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[113] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[112] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[111] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[110] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[109] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[108] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[107] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[106] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[105] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[104] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[103] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[102] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[101] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[100] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[99] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[98] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[97] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[96] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[95] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[94] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[93] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[92] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[91] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[90] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[89] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[88] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[87] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[86] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[85] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[84] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[83] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[82] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[81] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[80] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[79] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[78] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[77] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[76] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[75] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[74] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[73] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[72] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[71] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[70] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[69] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[68] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[67] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[66] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[65] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[64] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[63] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[62] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[61] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[60] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[59] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[58] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[57] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[56] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[55] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[54] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[53] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[52] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[51] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[50] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[49] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[48] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[47] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[46] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[45] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[44] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[43] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[42] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[41] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[40] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[39] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[38] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[37] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[36] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[35] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[34] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[33] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[32] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[31] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[30] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[29] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[28] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[27] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[26] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[25] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[24] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[23] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[22] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[21] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[20] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[19] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[18] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[17] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[16] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[15] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[14] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[13] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[12] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[11] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[10] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[9] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[8] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[7] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[6] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[5] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[4] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[3] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[2] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[1] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b0 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[0] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[143] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[142] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[141] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[140] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[139] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[138] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[137] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[136] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[135] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[134] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[133] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[132] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[131] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[130] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[129] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[128] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[127] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[126] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[125] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[124] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[123] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[122] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[121] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[120] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[119] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[118] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[117] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[116] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[115] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[114] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[113] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[112] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[111] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[110] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[109] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[108] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[107] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[106] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[105] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[104] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[103] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[102] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[101] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[100] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[99] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[98] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[97] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[96] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[95] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[94] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[93] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[92] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[91] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[90] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[89] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[88] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[87] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[86] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[85] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[84] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[83] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[82] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[81] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[80] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[79] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[78] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[77] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[76] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[75] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[74] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[73] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[72] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[71] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[70] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[69] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[68] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[67] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[66] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[65] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[64] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[63] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[62] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[61] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[60] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[59] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[58] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[57] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[56] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[55] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[54] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[53] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[52] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[51] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[50] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[49] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[48] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[47] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[46] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[45] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[44] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[43] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[42] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[41] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[40] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[39] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[38] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[37] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[36] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[35] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[34] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[33] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[32] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[31] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[30] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[29] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[28] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[27] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[26] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[25] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[24] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[23] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[22] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[21] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[20] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[19] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[18] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[17] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[16] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[15] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[14] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[13] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[12] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[11] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[10] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[9] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[8] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[7] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[6] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[5] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[4] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[3] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[2] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[1] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b0 && WEN == 1'b1)
       (posedge CLK => (Q[0] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[143] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[142] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[141] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[140] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[139] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[138] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[137] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[136] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[135] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[134] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[133] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[132] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[131] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[130] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[129] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[128] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[127] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[126] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[125] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[124] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[123] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[122] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[121] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[120] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[119] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[118] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[117] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[116] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[115] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[114] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[113] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[112] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[111] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[110] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[109] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[108] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[107] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[106] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[105] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[104] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[103] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[102] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[101] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[100] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[99] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[98] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[97] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[96] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[95] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[94] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[93] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[92] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[91] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[90] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[89] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[88] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[87] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[86] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[85] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[84] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[83] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[82] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[81] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[80] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[79] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[78] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[77] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[76] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[75] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[74] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[73] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[72] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[71] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[70] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[69] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[68] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[67] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[66] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[65] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[64] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[63] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[62] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[61] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[60] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[59] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[58] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[57] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[56] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[55] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[54] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[53] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[52] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[51] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[50] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[49] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[48] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[47] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[46] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[45] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[44] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[43] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[42] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[41] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[40] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[39] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[38] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[37] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[36] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[35] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[34] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[33] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[32] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[31] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[30] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[29] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[28] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[27] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[26] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[25] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[24] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[23] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[22] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[21] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[20] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[19] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[18] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[17] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[16] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[15] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[14] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[13] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[12] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[11] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[10] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[9] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[8] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[7] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[6] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[5] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[4] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[3] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[2] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[1] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);
    if (RET1N == 1'b1 && EMA[2] == 1'b1 && EMA[1] == 1'b1 && EMA[0] == 1'b1 && WEN == 1'b1)
       (posedge CLK => (Q[0] : 1'b0)) = (`ARM_MEM_PROP, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP, `ARM_MEM_RETAIN, `ARM_MEM_PROP);


   // Define SDTC only if back-annotating SDF file generated by Design Compiler
   `ifdef NO_SDTC
       $period(posedge CLK, `ARM_MEM_PERIOD, NOT_CLK_PER);
   `else
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq0aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq0aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq0aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq0aEMAW1eq1aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq0aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq0aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
       $period(posedge CLK &&& RET1Neq1aEMA2eq1aEMA1eq1aEMA0eq1aEMAW1eq1aEMAW0eq1aCENeq0, `ARM_MEM_PERIOD, NOT_CLK_PER);
   `endif


   // Define SDTC only if back-annotating SDF file generated by Design Compiler
   `ifdef NO_SDTC
       $width(posedge CLK, `ARM_MEM_WIDTH, 0, NOT_CLK_MINH);
       $width(negedge CLK, `ARM_MEM_WIDTH, 0, NOT_CLK_MINL);
   `else
       $width(posedge CLK &&& RET1Neq1, `ARM_MEM_WIDTH, 0, NOT_CLK_MINH);
       $width(negedge CLK &&& RET1Neq1, `ARM_MEM_WIDTH, 0, NOT_CLK_MINL);
   `endif

    $setuphold(posedge CLK &&& RET1Neq1, posedge CEN, `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_CEN);
    $setuphold(posedge CLK &&& RET1Neq1, negedge CEN, `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_CEN);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge WEN, `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_WEN);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge WEN, `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_WEN);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge A[8], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A8);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge A[7], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A7);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge A[6], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A6);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge A[5], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A5);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge A[4], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A4);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge A[3], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A3);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge A[2], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A2);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge A[1], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A1);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge A[0], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A0);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge A[8], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A8);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge A[7], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A7);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge A[6], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A6);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge A[5], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A5);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge A[4], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A4);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge A[3], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A3);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge A[2], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A2);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge A[1], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A1);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge A[0], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_A0);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[143], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D143);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[142], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D142);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[141], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D141);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[140], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D140);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[139], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D139);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[138], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D138);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[137], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D137);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[136], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D136);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[135], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D135);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[134], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D134);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[133], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D133);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[132], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D132);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[131], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D131);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[130], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D130);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[129], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D129);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[128], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D128);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[127], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D127);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[126], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D126);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[125], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D125);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[124], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D124);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[123], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D123);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[122], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D122);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[121], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D121);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[120], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D120);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[119], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D119);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[118], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D118);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[117], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D117);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[116], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D116);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[115], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D115);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[114], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D114);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[113], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D113);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[112], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D112);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[111], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D111);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[110], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D110);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[109], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D109);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[108], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D108);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[107], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D107);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[106], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D106);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[105], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D105);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[104], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D104);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[103], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D103);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[102], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D102);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[101], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D101);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[100], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D100);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[99], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D99);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[98], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D98);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[97], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D97);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[96], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D96);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[95], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D95);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[94], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D94);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[93], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D93);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[92], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D92);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[91], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D91);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[90], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D90);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[89], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D89);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[88], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D88);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[87], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D87);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[86], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D86);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[85], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D85);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[84], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D84);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[83], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D83);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[82], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D82);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[81], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D81);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[80], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D80);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[79], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D79);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[78], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D78);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[77], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D77);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[76], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D76);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[75], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D75);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[74], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D74);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[73], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D73);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[72], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D72);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[71], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D71);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[70], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D70);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[69], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D69);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[68], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D68);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[67], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D67);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[66], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D66);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[65], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D65);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[64], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D64);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[63], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D63);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[62], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D62);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[61], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D61);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[60], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D60);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[59], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D59);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[58], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D58);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[57], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D57);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[56], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D56);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[55], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D55);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[54], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D54);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[53], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D53);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[52], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D52);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[51], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D51);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[50], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D50);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[49], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D49);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[48], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D48);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[47], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D47);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[46], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D46);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[45], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D45);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[44], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D44);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[43], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D43);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[42], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D42);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[41], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D41);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[40], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D40);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[39], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D39);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[38], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D38);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[37], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D37);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[36], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D36);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[35], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D35);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[34], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D34);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[33], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D33);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[32], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D32);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[31], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D31);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[30], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D30);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[29], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D29);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[28], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D28);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[27], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D27);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[26], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D26);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[25], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D25);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[24], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D24);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[23], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D23);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[22], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D22);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[21], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D21);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[20], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D20);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[19], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D19);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[18], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D18);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[17], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D17);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[16], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D16);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[15], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D15);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[14], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D14);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[13], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D13);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[12], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D12);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[11], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D11);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[10], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D10);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[9], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D9);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[8], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D8);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[7], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D7);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[6], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D6);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[5], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D5);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[4], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D4);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[3], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D3);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[2], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D2);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[1], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D1);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, posedge D[0], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D0);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[143], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D143);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[142], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D142);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[141], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D141);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[140], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D140);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[139], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D139);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[138], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D138);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[137], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D137);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[136], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D136);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[135], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D135);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[134], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D134);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[133], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D133);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[132], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D132);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[131], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D131);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[130], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D130);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[129], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D129);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[128], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D128);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[127], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D127);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[126], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D126);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[125], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D125);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[124], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D124);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[123], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D123);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[122], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D122);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[121], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D121);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[120], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D120);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[119], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D119);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[118], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D118);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[117], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D117);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[116], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D116);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[115], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D115);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[114], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D114);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[113], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D113);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[112], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D112);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[111], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D111);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[110], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D110);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[109], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D109);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[108], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D108);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[107], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D107);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[106], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D106);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[105], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D105);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[104], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D104);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[103], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D103);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[102], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D102);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[101], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D101);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[100], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D100);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[99], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D99);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[98], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D98);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[97], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D97);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[96], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D96);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[95], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D95);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[94], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D94);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[93], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D93);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[92], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D92);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[91], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D91);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[90], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D90);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[89], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D89);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[88], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D88);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[87], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D87);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[86], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D86);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[85], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D85);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[84], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D84);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[83], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D83);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[82], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D82);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[81], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D81);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[80], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D80);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[79], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D79);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[78], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D78);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[77], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D77);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[76], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D76);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[75], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D75);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[74], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D74);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[73], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D73);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[72], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D72);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[71], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D71);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[70], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D70);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[69], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D69);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[68], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D68);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[67], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D67);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[66], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D66);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[65], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D65);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[64], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D64);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[63], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D63);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[62], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D62);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[61], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D61);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[60], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D60);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[59], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D59);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[58], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D58);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[57], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D57);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[56], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D56);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[55], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D55);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[54], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D54);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[53], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D53);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[52], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D52);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[51], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D51);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[50], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D50);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[49], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D49);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[48], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D48);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[47], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D47);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[46], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D46);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[45], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D45);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[44], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D44);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[43], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D43);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[42], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D42);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[41], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D41);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[40], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D40);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[39], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D39);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[38], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D38);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[37], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D37);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[36], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D36);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[35], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D35);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[34], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D34);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[33], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D33);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[32], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D32);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[31], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D31);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[30], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D30);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[29], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D29);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[28], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D28);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[27], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D27);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[26], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D26);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[25], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D25);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[24], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D24);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[23], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D23);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[22], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D22);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[21], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D21);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[20], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D20);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[19], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D19);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[18], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D18);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[17], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D17);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[16], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D16);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[15], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D15);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[14], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D14);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[13], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D13);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[12], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D12);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[11], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D11);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[10], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D10);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[9], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D9);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[8], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D8);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[7], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D7);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[6], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D6);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[5], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D5);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[4], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D4);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[3], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D3);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[2], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D2);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[1], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D1);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0aWENeq0, negedge D[0], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_D0);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge EMA[2], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_EMA2);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge EMA[1], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_EMA1);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge EMA[0], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_EMA0);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge EMA[2], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_EMA2);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge EMA[1], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_EMA1);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge EMA[0], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_EMA0);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge EMAW[1], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_EMAW1);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, posedge EMAW[0], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_EMAW0);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge EMAW[1], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_EMAW1);
    $setuphold(posedge CLK &&& RET1Neq1aCENeq0, negedge EMAW[0], `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_EMAW0);
    $setuphold(posedge CLK, posedge RET1N, `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_RET1N);
    $setuphold(posedge CLK, negedge RET1N, `ARM_MEM_SETUP, `ARM_MEM_HOLD, NOT_RET1N);
    $setuphold(negedge RET1N, negedge CEN, 0.000, `ARM_MEM_HOLD, NOT_RET1N);
    $setuphold(posedge RET1N, negedge CEN, 0.000, `ARM_MEM_HOLD, NOT_RET1N);
    $setuphold(posedge CEN, negedge RET1N, 0.000, `ARM_MEM_HOLD, NOT_RET1N);
    $setuphold(posedge CEN, posedge RET1N, 0.000, `ARM_MEM_HOLD, NOT_RET1N);
  endspecify


endmodule
`endcelldefine
`endif
`timescale 1ns/1ps
module sram464x144_error_injection (Q_out, Q_in, CLK, A, CEN, WEN);
   output [143:0] Q_out;
   input [143:0] Q_in;
   input CLK;
   input [8:0] A;
   input CEN;
   input WEN;
   parameter LEFT_RED_COLUMN_FAULT = 2'd1;
   parameter RIGHT_RED_COLUMN_FAULT = 2'd2;
   parameter NO_RED_FAULT = 2'd0;
   reg [143:0] Q_out;
   reg entry_found;
   reg list_complete;
   reg [21:0] fault_table [231:0];
   reg [21:0] fault_entry;
initial
begin
   `ifdef DUT
      `define pre_pend_path TB.DUT_inst.CHIP
   `else
       `define pre_pend_path TB.CHIP
   `endif
   `ifdef ARM_NONREPAIRABLE_FAULT
      `pre_pend_path.SMARCHCHKBVCD_LVISION_MBISTPG_ASSEMBLY_UNDER_TEST_INST.MEM0_MEM_INST.u1.add_fault(9'd144,8'd70,2'd1,2'd0);
   `endif
end
   task add_fault;
   //This task injects fault in memory
      input [8:0] address;
      input [7:0] bitPlace;
      input [1:0] fault_type;
      input [1:0] red_fault;
 
      integer i;
      reg done;
   begin
      done = 1'b0;
      i = 0;
      while ((!done) && i < 231)
      begin
         fault_entry = fault_table[i];
         if (fault_entry[0] === 1'b0 || fault_entry[0] === 1'bx)
         begin
            fault_entry[0] = 1'b1;
            fault_entry[2:1] = red_fault;
            fault_entry[4:3] = fault_type;
            fault_entry[12:5] = bitPlace;
            fault_entry[21:13] = address;
            fault_table[i] = fault_entry;
            done = 1'b1;
         end
         i = i+1;
      end
   end
   endtask
//This task removes all fault entries injected by user
task remove_all_faults;
   integer i;
begin
   for (i = 0; i < 232; i=i+1)
   begin
      fault_entry = fault_table[i];
      fault_entry[0] = 1'b0;
      fault_table[i] = fault_entry;
   end
end
endtask
task bit_error;
// This task is used to inject error in memory and should be called
// only from current module.
//
// This task injects error depending upon fault type to particular bit
// of the output
   inout [143:0] q_int;
   input [1:0] fault_type;
   input [7:0] bitLoc;
begin
   if (fault_type === 2'd0)
      q_int[bitLoc] = 1'b0;
   else if (fault_type === 2'd1)
      q_int[bitLoc] = 1'b1;
   else
      q_int[bitLoc] = ~q_int[bitLoc];
end
endtask
task error_injection_on_output;
// This function goes through error injection table for every
// read cycle and corrupts Q output if fault for the particular
// address is present in fault table
//
// If fault is redundant column is detected, this task corrupts
// Q output in read cycle
//
// If fault is repaired using repair bus, this task does not
// courrpt Q output in read cycle
//
   output [143:0] Q_output;
   reg list_complete;
   integer i;
   reg [7:0] row_address;
   reg [0:0] column_address;
   reg [7:0] bitPlace;
   reg [1:0] fault_type;
   reg [1:0] red_fault;
   reg valid;
   reg [6:0] msb_bit_calc;
begin
   entry_found = 1'b0;
   list_complete = 1'b0;
   i = 0;
   Q_output = Q_in;
   while(!list_complete)
   begin
      fault_entry = fault_table[i];
      {row_address, column_address, bitPlace, fault_type, red_fault, valid} = fault_entry;
      i = i + 1;
      if (valid == 1'b1)
      begin
         if (red_fault === NO_RED_FAULT)
         begin
            if (row_address == A[8:1] && column_address == A[0:0])
            begin
               if (bitPlace < 72)
                  bit_error(Q_output,fault_type, bitPlace);
               else if (bitPlace >= 72 )
                  bit_error(Q_output,fault_type, bitPlace);
            end
         end
      end
      else
         list_complete = 1'b1;
      end
   end
   endtask
   always @ (Q_in or CLK or A or CEN or WEN)
   begin
   if (CEN === 1'b0 && &WEN === 1'b1)
      error_injection_on_output(Q_out);
   else
      Q_out = Q_in;
   end
endmodule

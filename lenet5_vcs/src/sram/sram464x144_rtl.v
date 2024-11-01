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
//       Repair Verilog RTL for Synchronous Single-Port Register File
//
//       Instance Name:              sram464x144_rtl_top
//       Words:                      464
//       User Bits:                  144
//       Mux:                        2
//       Drive:                      6
//       Write Mask:                 Off
//       Extra Margin Adjustment:    On
//       Redundancy:                 off
//       Redundant Rows:             0
//       Redundant Columns:          0
//       Test Muxes                  Off
//       Ser:                        none
//       Retention:                  on
//       Power Gating:               off
//
//       Creation Date:  Tue Nov 15 11:43:09 2022
//       Version:      r1p1
//
//       Verified
//
//       Known Bugs: None.
//
//       Known Work Arounds: N/A
//
`timescale 1ns/1ps

module sram464x144_rtl_top (
          Q, 
          CLK, 
          CEN, 
          WEN, 
          A, 
          D, 
          EMA, 
          EMAW, 
          RET1N
   );

   output [143:0]           Q;
   input                    CLK;
   input                    CEN;
   input                    WEN;
   input [8:0]              A;
   input [143:0]            D;
   input [2:0]              EMA;
   input [1:0]              EMAW;
   input                    RET1N;
   wire [143:0]             DI;
   wire [143:0]             QO;

   assign Q = QO;
   assign DI = D;
   sram464x144_fr_top u0 (
         .QO(QO),
         .CLK(CLK),
         .CEN(CEN),
         .WEN(WEN),
         .A(A),
         .DI(DI),
         .EMA(EMA),
         .EMAW(EMAW),
         .RET1N(RET1N)
);

endmodule

module sram464x144_fr_top (
          QO, 
          CLK, 
          CEN, 
          WEN, 
          A, 
          DI, 
          EMA, 
          EMAW, 
          RET1N
   );

   output [143:0]           QO;
   input                    CLK;
   input                    CEN;
   input                    WEN;
   input [8:0]              A;
   input [143:0]            DI;
   input [2:0]              EMA;
   input [1:0]              EMAW;
   input                    RET1N;

   wire [143:0]    D;
   wire [143:0]    Q;

   assign D=DI;
   assign QO=Q;
   sram464x144 u0 (
         .Q(Q),
         .CLK(CLK),
         .CEN(CEN),
         .WEN(WEN),
         .A(A),
         .D(D),
         .EMA(EMA),
         .EMAW(EMAW),
         .RET1N(RET1N)
   );

endmodule // sram464x144_fr_top


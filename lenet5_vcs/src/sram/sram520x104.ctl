/* ctl_memcomp Version: 4.0.5-EAC3 */
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
//      CTL model for Synchronous Single-Port Register File
//
//       Instance Name:              sram520x104
//       Words:                      520
//       Bits:                       104
//       Mux:                        4
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
//       Creation Date:  Tue Nov 15 11:45:11 2022
//       Version: 	r1p1
STIL 1.0 {
   CTL P2001.10;
   Design P2001.01;
}
Header {
   Title "CTL model for `sram520x104";
}
Signals {
   "Q[103]" Out;
   "Q[102]" Out;
   "Q[101]" Out;
   "Q[100]" Out;
   "Q[99]" Out;
   "Q[98]" Out;
   "Q[97]" Out;
   "Q[96]" Out;
   "Q[95]" Out;
   "Q[94]" Out;
   "Q[93]" Out;
   "Q[92]" Out;
   "Q[91]" Out;
   "Q[90]" Out;
   "Q[89]" Out;
   "Q[88]" Out;
   "Q[87]" Out;
   "Q[86]" Out;
   "Q[85]" Out;
   "Q[84]" Out;
   "Q[83]" Out;
   "Q[82]" Out;
   "Q[81]" Out;
   "Q[80]" Out;
   "Q[79]" Out;
   "Q[78]" Out;
   "Q[77]" Out;
   "Q[76]" Out;
   "Q[75]" Out;
   "Q[74]" Out;
   "Q[73]" Out;
   "Q[72]" Out;
   "Q[71]" Out;
   "Q[70]" Out;
   "Q[69]" Out;
   "Q[68]" Out;
   "Q[67]" Out;
   "Q[66]" Out;
   "Q[65]" Out;
   "Q[64]" Out;
   "Q[63]" Out;
   "Q[62]" Out;
   "Q[61]" Out;
   "Q[60]" Out;
   "Q[59]" Out;
   "Q[58]" Out;
   "Q[57]" Out;
   "Q[56]" Out;
   "Q[55]" Out;
   "Q[54]" Out;
   "Q[53]" Out;
   "Q[52]" Out;
   "Q[51]" Out;
   "Q[50]" Out;
   "Q[49]" Out;
   "Q[48]" Out;
   "Q[47]" Out;
   "Q[46]" Out;
   "Q[45]" Out;
   "Q[44]" Out;
   "Q[43]" Out;
   "Q[42]" Out;
   "Q[41]" Out;
   "Q[40]" Out;
   "Q[39]" Out;
   "Q[38]" Out;
   "Q[37]" Out;
   "Q[36]" Out;
   "Q[35]" Out;
   "Q[34]" Out;
   "Q[33]" Out;
   "Q[32]" Out;
   "Q[31]" Out;
   "Q[30]" Out;
   "Q[29]" Out;
   "Q[28]" Out;
   "Q[27]" Out;
   "Q[26]" Out;
   "Q[25]" Out;
   "Q[24]" Out;
   "Q[23]" Out;
   "Q[22]" Out;
   "Q[21]" Out;
   "Q[20]" Out;
   "Q[19]" Out;
   "Q[18]" Out;
   "Q[17]" Out;
   "Q[16]" Out;
   "Q[15]" Out;
   "Q[14]" Out;
   "Q[13]" Out;
   "Q[12]" Out;
   "Q[11]" Out;
   "Q[10]" Out;
   "Q[9]" Out;
   "Q[8]" Out;
   "Q[7]" Out;
   "Q[6]" Out;
   "Q[5]" Out;
   "Q[4]" Out;
   "Q[3]" Out;
   "Q[2]" Out;
   "Q[1]" Out;
   "Q[0]" Out;
   "CLK" In;
   "CEN" In;
   "WEN" In;
   "A[9]" In;
   "A[8]" In;
   "A[7]" In;
   "A[6]" In;
   "A[5]" In;
   "A[4]" In;
   "A[3]" In;
   "A[2]" In;
   "A[1]" In;
   "A[0]" In;
   "D[103]" In;
   "D[102]" In;
   "D[101]" In;
   "D[100]" In;
   "D[99]" In;
   "D[98]" In;
   "D[97]" In;
   "D[96]" In;
   "D[95]" In;
   "D[94]" In;
   "D[93]" In;
   "D[92]" In;
   "D[91]" In;
   "D[90]" In;
   "D[89]" In;
   "D[88]" In;
   "D[87]" In;
   "D[86]" In;
   "D[85]" In;
   "D[84]" In;
   "D[83]" In;
   "D[82]" In;
   "D[81]" In;
   "D[80]" In;
   "D[79]" In;
   "D[78]" In;
   "D[77]" In;
   "D[76]" In;
   "D[75]" In;
   "D[74]" In;
   "D[73]" In;
   "D[72]" In;
   "D[71]" In;
   "D[70]" In;
   "D[69]" In;
   "D[68]" In;
   "D[67]" In;
   "D[66]" In;
   "D[65]" In;
   "D[64]" In;
   "D[63]" In;
   "D[62]" In;
   "D[61]" In;
   "D[60]" In;
   "D[59]" In;
   "D[58]" In;
   "D[57]" In;
   "D[56]" In;
   "D[55]" In;
   "D[54]" In;
   "D[53]" In;
   "D[52]" In;
   "D[51]" In;
   "D[50]" In;
   "D[49]" In;
   "D[48]" In;
   "D[47]" In;
   "D[46]" In;
   "D[45]" In;
   "D[44]" In;
   "D[43]" In;
   "D[42]" In;
   "D[41]" In;
   "D[40]" In;
   "D[39]" In;
   "D[38]" In;
   "D[37]" In;
   "D[36]" In;
   "D[35]" In;
   "D[34]" In;
   "D[33]" In;
   "D[32]" In;
   "D[31]" In;
   "D[30]" In;
   "D[29]" In;
   "D[28]" In;
   "D[27]" In;
   "D[26]" In;
   "D[25]" In;
   "D[24]" In;
   "D[23]" In;
   "D[22]" In;
   "D[21]" In;
   "D[20]" In;
   "D[19]" In;
   "D[18]" In;
   "D[17]" In;
   "D[16]" In;
   "D[15]" In;
   "D[14]" In;
   "D[13]" In;
   "D[12]" In;
   "D[11]" In;
   "D[10]" In;
   "D[9]" In;
   "D[8]" In;
   "D[7]" In;
   "D[6]" In;
   "D[5]" In;
   "D[4]" In;
   "D[3]" In;
   "D[2]" In;
   "D[1]" In;
   "D[0]" In;
   "EMA[2]" In;
   "EMA[1]" In;
   "EMA[0]" In;
   "EMAW[1]" In;
   "EMAW[0]" In;
   "RET1N" In;
}
SignalGroups {
   "all_inputs" = '"CLK" + "CEN" + "WEN" + "A[9]" + "A[8]" + "A[7]" + "A[6]" + "A[5]" + 
   "A[4]" + "A[3]" + "A[2]" + "A[1]" + "A[0]" + "D[103]" + "D[102]" + "D[101]" + 
   "D[100]" + "D[99]" + "D[98]" + "D[97]" + "D[96]" + "D[95]" + "D[94]" + "D[93]" + 
   "D[92]" + "D[91]" + "D[90]" + "D[89]" + "D[88]" + "D[87]" + "D[86]" + "D[85]" + 
   "D[84]" + "D[83]" + "D[82]" + "D[81]" + "D[80]" + "D[79]" + "D[78]" + "D[77]" + 
   "D[76]" + "D[75]" + "D[74]" + "D[73]" + "D[72]" + "D[71]" + "D[70]" + "D[69]" + 
   "D[68]" + "D[67]" + "D[66]" + "D[65]" + "D[64]" + "D[63]" + "D[62]" + "D[61]" + 
   "D[60]" + "D[59]" + "D[58]" + "D[57]" + "D[56]" + "D[55]" + "D[54]" + "D[53]" + 
   "D[52]" + "D[51]" + "D[50]" + "D[49]" + "D[48]" + "D[47]" + "D[46]" + "D[45]" + 
   "D[44]" + "D[43]" + "D[42]" + "D[41]" + "D[40]" + "D[39]" + "D[38]" + "D[37]" + 
   "D[36]" + "D[35]" + "D[34]" + "D[33]" + "D[32]" + "D[31]" + "D[30]" + "D[29]" + 
   "D[28]" + "D[27]" + "D[26]" + "D[25]" + "D[24]" + "D[23]" + "D[22]" + "D[21]" + 
   "D[20]" + "D[19]" + "D[18]" + "D[17]" + "D[16]" + "D[15]" + "D[14]" + "D[13]" + 
   "D[12]" + "D[11]" + "D[10]" + "D[9]" + "D[8]" + "D[7]" + "D[6]" + "D[5]" + "D[4]" + 
   "D[3]" + "D[2]" + "D[1]" + "D[0]" + "EMA[2]" + "EMA[1]" + "EMA[0]" + "EMAW[1]" + 
   "EMAW[0]" + "RET1N"';
   "all_outputs" = '"Q[103]" + "Q[102]" + "Q[101]" + "Q[100]" + "Q[99]" + "Q[98]" + 
   "Q[97]" + "Q[96]" + "Q[95]" + "Q[94]" + "Q[93]" + "Q[92]" + "Q[91]" + "Q[90]" + 
   "Q[89]" + "Q[88]" + "Q[87]" + "Q[86]" + "Q[85]" + "Q[84]" + "Q[83]" + "Q[82]" + 
   "Q[81]" + "Q[80]" + "Q[79]" + "Q[78]" + "Q[77]" + "Q[76]" + "Q[75]" + "Q[74]" + 
   "Q[73]" + "Q[72]" + "Q[71]" + "Q[70]" + "Q[69]" + "Q[68]" + "Q[67]" + "Q[66]" + 
   "Q[65]" + "Q[64]" + "Q[63]" + "Q[62]" + "Q[61]" + "Q[60]" + "Q[59]" + "Q[58]" + 
   "Q[57]" + "Q[56]" + "Q[55]" + "Q[54]" + "Q[53]" + "Q[52]" + "Q[51]" + "Q[50]" + 
   "Q[49]" + "Q[48]" + "Q[47]" + "Q[46]" + "Q[45]" + "Q[44]" + "Q[43]" + "Q[42]" + 
   "Q[41]" + "Q[40]" + "Q[39]" + "Q[38]" + "Q[37]" + "Q[36]" + "Q[35]" + "Q[34]" + 
   "Q[33]" + "Q[32]" + "Q[31]" + "Q[30]" + "Q[29]" + "Q[28]" + "Q[27]" + "Q[26]" + 
   "Q[25]" + "Q[24]" + "Q[23]" + "Q[22]" + "Q[21]" + "Q[20]" + "Q[19]" + "Q[18]" + 
   "Q[17]" + "Q[16]" + "Q[15]" + "Q[14]" + "Q[13]" + "Q[12]" + "Q[11]" + "Q[10]" + 
   "Q[9]" + "Q[8]" + "Q[7]" + "Q[6]" + "Q[5]" + "Q[4]" + "Q[3]" + "Q[2]" + "Q[1]" + 
   "Q[0]"';
   "all_ports" = '"all_inputs" + "all_outputs"';
   "_pi" = '"CLK" + "CEN" + "WEN" + "A[9]" + "A[8]" + "A[7]" + "A[6]" + "A[5]" + 
   "A[4]" + "A[3]" + "A[2]" + "A[1]" + "A[0]" + "D[103]" + "D[102]" + "D[101]" + 
   "D[100]" + "D[99]" + "D[98]" + "D[97]" + "D[96]" + "D[95]" + "D[94]" + "D[93]" + 
   "D[92]" + "D[91]" + "D[90]" + "D[89]" + "D[88]" + "D[87]" + "D[86]" + "D[85]" + 
   "D[84]" + "D[83]" + "D[82]" + "D[81]" + "D[80]" + "D[79]" + "D[78]" + "D[77]" + 
   "D[76]" + "D[75]" + "D[74]" + "D[73]" + "D[72]" + "D[71]" + "D[70]" + "D[69]" + 
   "D[68]" + "D[67]" + "D[66]" + "D[65]" + "D[64]" + "D[63]" + "D[62]" + "D[61]" + 
   "D[60]" + "D[59]" + "D[58]" + "D[57]" + "D[56]" + "D[55]" + "D[54]" + "D[53]" + 
   "D[52]" + "D[51]" + "D[50]" + "D[49]" + "D[48]" + "D[47]" + "D[46]" + "D[45]" + 
   "D[44]" + "D[43]" + "D[42]" + "D[41]" + "D[40]" + "D[39]" + "D[38]" + "D[37]" + 
   "D[36]" + "D[35]" + "D[34]" + "D[33]" + "D[32]" + "D[31]" + "D[30]" + "D[29]" + 
   "D[28]" + "D[27]" + "D[26]" + "D[25]" + "D[24]" + "D[23]" + "D[22]" + "D[21]" + 
   "D[20]" + "D[19]" + "D[18]" + "D[17]" + "D[16]" + "D[15]" + "D[14]" + "D[13]" + 
   "D[12]" + "D[11]" + "D[10]" + "D[9]" + "D[8]" + "D[7]" + "D[6]" + "D[5]" + "D[4]" + 
   "D[3]" + "D[2]" + "D[1]" + "D[0]" + "EMA[2]" + "EMA[1]" + "EMA[0]" + "EMAW[1]" + 
   "EMAW[0]" + "RET1N"';
   "_po" = '"Q[103]" + "Q[102]" + "Q[101]" + "Q[100]" + "Q[99]" + "Q[98]" + "Q[97]" + 
   "Q[96]" + "Q[95]" + "Q[94]" + "Q[93]" + "Q[92]" + "Q[91]" + "Q[90]" + "Q[89]" + 
   "Q[88]" + "Q[87]" + "Q[86]" + "Q[85]" + "Q[84]" + "Q[83]" + "Q[82]" + "Q[81]" + 
   "Q[80]" + "Q[79]" + "Q[78]" + "Q[77]" + "Q[76]" + "Q[75]" + "Q[74]" + "Q[73]" + 
   "Q[72]" + "Q[71]" + "Q[70]" + "Q[69]" + "Q[68]" + "Q[67]" + "Q[66]" + "Q[65]" + 
   "Q[64]" + "Q[63]" + "Q[62]" + "Q[61]" + "Q[60]" + "Q[59]" + "Q[58]" + "Q[57]" + 
   "Q[56]" + "Q[55]" + "Q[54]" + "Q[53]" + "Q[52]" + "Q[51]" + "Q[50]" + "Q[49]" + 
   "Q[48]" + "Q[47]" + "Q[46]" + "Q[45]" + "Q[44]" + "Q[43]" + "Q[42]" + "Q[41]" + 
   "Q[40]" + "Q[39]" + "Q[38]" + "Q[37]" + "Q[36]" + "Q[35]" + "Q[34]" + "Q[33]" + 
   "Q[32]" + "Q[31]" + "Q[30]" + "Q[29]" + "Q[28]" + "Q[27]" + "Q[26]" + "Q[25]" + 
   "Q[24]" + "Q[23]" + "Q[22]" + "Q[21]" + "Q[20]" + "Q[19]" + "Q[18]" + "Q[17]" + 
   "Q[16]" + "Q[15]" + "Q[14]" + "Q[13]" + "Q[12]" + "Q[11]" + "Q[10]" + "Q[9]" + 
   "Q[8]" + "Q[7]" + "Q[6]" + "Q[5]" + "Q[4]" + "Q[3]" + "Q[2]" + "Q[1]" + "Q[0]"';
}
ScanStructures {
}
Timing {
   WaveformTable "_default_WFT_" {
      Period '100ns';
      Waveforms {
         "all_inputs" {
            01ZN { '0ns' D/U/Z/N; }
         }
         "all_outputs" {
            XHTL { '40ns' X/H/T/L; }
         }
         "CLK" {
            P { '0ns' D; '45ns' U; '55ns' D; }
         }
      }
   }
}
Procedures {
   "capture" {
      W "_default_WFT_";
      V { "_pi" = #; "_po" = #; }
   }
   "capture_CLK" {
      W "_default_WFT_";
      V {"_pi" = #; "_po" = #;"CLK" = P; }
   }
   "load_unload" {
      W "_default_WFT_";
      V { "CLK" = 0; }
      Shift {
         V { "CLK" = P; }
      }
   }
}
MacroDefs {
   "test_setup" {
      W "_default_WFT_";
      C {"all_inputs" = \r60 N; "all_outputs" = \r34 X; }
      V { "CLK" = P; }
   }
}
Environment "sram520x104" {
   CTL {
   }
   CTL Internal_scan {
      TestMode InternalTest;
      Focus Top {
      }
      Internal {
      }
   }
}
Environment dftSpec {
   CTL {
   }
   CTL all_dft {
      TestMode ForInheritOnly;
   }
}

sidCmdLineBehaviorAnalysisOpt -incr -clockSkew 0 -loopUnroll 0 -bboxEmptyModule 0  -cellModel 0 -bboxIgnoreProtected 0 
debImport "-sv" "-f" "./filelist.f" "-top" "uart_rx_tb"
debLoadSimResult \
           /home/ICer/Desktop/Projects/Lenet_Fukami_competition/sim/uart_rx_tb.fsdb
wvCreateWindow
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart" -win $_nTrace1
srcHBSelect "uart_rx_tb.u_W25Q128JVxIM" -win $_nTrace1
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart" -win $_nTrace1
srcHBSelect "uart_rx_tb.u_W25Q128JVxIM" -win $_nTrace1
srcHBSelect "uart_rx_tb" -win $_nTrace1
srcSetScope -win $_nTrace1 "uart_rx_tb" -delim "."
srcHBDrag -win $_nTrace1
wvRenameGroup -win $_nWave2 {G1} {uart_rx_tb}
srcHBDrag -win $_nTrace1
wvRenameGroup -win $_nWave2 {uart_rx_tb} {uart_rx_tb#1}
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart" -win $_nTrace1
srcHBDrag -win $_nTrace1
wvRenameGroup -win $_nWave2 {uart_rx_tb#1} {u_fpga_top_with_uart}
wvAddSignal -win $_nWave2 "/uart_rx_tb/u_fpga_top_with_uart/clk200M" \
           "/uart_rx_tb/u_fpga_top_with_uart/rstn" \
           "/uart_rx_tb/u_fpga_top_with_uart/key" \
           "/uart_rx_tb/u_fpga_top_with_uart/rxd" \
           "/uart_rx_tb/u_fpga_top_with_uart/txd" \
           "/uart_rx_tb/u_fpga_top_with_uart/free" \
           "/uart_rx_tb/u_fpga_top_with_uart/init_done_led" \
           "/uart_rx_tb/u_fpga_top_with_uart/uart_done_led" \
           "/uart_rx_tb/u_fpga_top_with_uart/miso" \
           "/uart_rx_tb/u_fpga_top_with_uart/sck" \
           "/uart_rx_tb/u_fpga_top_with_uart/cs_n" \
           "/uart_rx_tb/u_fpga_top_with_uart/mosi"
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 0)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 12)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 12)}
wvZoomAll -win $_nWave2
wvZoomAll -win $_nWave2
wvZoomAll -win $_nWave2
wvZoomAll -win $_nWave2
srcHBSelect "uart_rx_tb.u_W25Q128JVxIM" -win $_nTrace1
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart" -win $_nTrace1
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart.u_uart_codec_top" -win $_nTrace1
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart.u_controller" -win $_nTrace1
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart" -win $_nTrace1
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart" -win $_nTrace1
srcSetScope -win $_nTrace1 "uart_rx_tb.u_fpga_top_with_uart" -delim "."
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart" -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "input_flash_data" -line 81 -pos 2 -win $_nTrace1
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 4)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 6)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 9)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 10)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 11)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 0)}
wvSetPosition -win $_nWave2 {("G2" 0)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 12)}
wvSetPosition -win $_nWave2 {("G2" 0)}
wvAddSignal -win $_nWave2 \
           "/uart_rx_tb/u_fpga_top_with_uart/input_flash_data\[207:0\]"
wvSetPosition -win $_nWave2 {("G2" 0)}
wvSetPosition -win $_nWave2 {("G2" 1)}
wvSetPosition -win $_nWave2 {("G2" 1)}
wvZoomAll -win $_nWave2
srcDeselectAll -win $_nTrace1
srcSelect -signal "controller2Uart_data" -line 74 -pos 2 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "controller2Uart_ena" -line 73 -pos 2 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "controller2Uart_data" -line 74 -pos 2 -win $_nTrace1
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 0)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 1)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 4)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 7)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 8)}
wvSetPosition -win $_nWave2 {("u_fpga_top_with_uart" 0)}
wvSetPosition -win $_nWave2 {("G3" 0)}
wvAddSignal -win $_nWave2 \
           "/uart_rx_tb/u_fpga_top_with_uart/controller2Uart_data\[7:0\]"
wvSetPosition -win $_nWave2 {("G3" 0)}
wvSetPosition -win $_nWave2 {("G3" 1)}
wvSetPosition -win $_nWave2 {("G3" 1)}
wvZoomAll -win $_nWave2
wvZoom -win $_nWave2 144375321573.604065 158858687166.384644
wvZoomAll -win $_nWave2
wvSelectSignal -win $_nWave2 {( "G2" 1 )} 
srcHBSelect "uart_rx_tb.u_W25Q128JVxIM" -win $_nTrace1
srcHBSelect "uart_rx_tb" -win $_nTrace1
srcSetScope -win $_nTrace1 "uart_rx_tb" -delim "."
srcHBSelect "uart_rx_tb" -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "clk_freq" -line 26 -pos 1 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "clk200M" -line 36 -pos 2 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "rstn" -line 38 -pos 2 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "rstn" -line 38 -pos 2 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "key" -line 39 -pos 2 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "rxd" -line 42 -pos 2 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "txd" -line 43 -pos 2 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "uart_done_led" -line 48 -pos 2 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "txd" -line 43 -pos 2 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "txd" -line 43 -pos 2 -win $_nTrace1
srcAction -pos 42 4 1 -win $_nTrace1 -name "txd" -ctrlKey off
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart" -win $_nTrace1
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart" -win $_nTrace1
srcSetScope -win $_nTrace1 "uart_rx_tb.u_fpga_top_with_uart" -delim "."
srcHBSelect "uart_rx_tb.u_fpga_top_with_uart" -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcHBSelect "uart_rx_tb.u_W25Q128JVxIM" -win $_nTrace1
srcSetScope -win $_nTrace1 "uart_rx_tb.u_W25Q128JVxIM" -delim "."
srcHBSelect "uart_rx_tb.u_W25Q128JVxIM" -win $_nTrace1
srcHBSelect "uart_rx_tb.u_W25Q128JVxIM" -win $_nTrace1
srcSetScope -win $_nTrace1 "uart_rx_tb.u_W25Q128JVxIM" -delim "."
srcHBSelect "uart_rx_tb.u_W25Q128JVxIM" -win $_nTrace1
srcHBSelect "uart_rx_tb" -win $_nTrace1
srcSetScope -win $_nTrace1 "uart_rx_tb" -delim "."
srcHBSelect "uart_rx_tb" -win $_nTrace1
wvZoomAll -win $_nWave2
debExit

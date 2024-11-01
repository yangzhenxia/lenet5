.PHONY:com sim debug cov clean

export OUTPUT = uart_rx_tb
# ALL_DEFINE = +define+DUMP_VPD
PLATFORM = linux64
WAVEFORM = ${OUTPUT}.fsdb
FILELIST = ./filelist.f



#code coverage command
CM = -cm line+cond+fsm+branch+tgl
CM_NAME = -cm_name ${OUTPUT}
CM_DIR = -cm_dir ./${OUTPUT}.vdb


# VPD_NAME = +vpdfile+${OUTPUT}.vpd    disabled

VCS = vcs -full64 -sverilog +v2k -timescale=1ns/1ns		    \
	  -o ${OUTPUT}											\
	  -l compile.log										\
	  ${VPD_NAME}											\
	  ${ALL_DEFINE}											\
	  ${CM}													\
	  ${CM_NAME}											\
	  ${CM_DIR}												\
	  -debug_pp 											\
	  -Mupdate												\
	  -LDFLAGS												\
	  -rdynamic												\
	  -P ${VERDI_HOME}/share/PLI/VCS/${PLATFORM}/novas.tab	\
	  ${VERDI_HOME}/share/PLI/VCS/${PLATFORM}/pli.a			\
	  +notimingheck											\
	  +nospecify											\


SIM = ./${OUTPUT}											\
	  -ucli -i ./scripts/dump_fsdb_vcs.tcl					\
	  +fsdb+autoflush										\
	  ${CM}													\
	  ${CM_NAME}											\
	  ${CM_DIR}												\
      -l ${OUTPUT}.log										\

LOAD_WAVE = 

com:
	${VCS} -f ${FILELIST}

sim:
	${SIM}


com_questa:
	vlib work
	vmap work work
	vlog \
	-64 \
	-sv \
	+acc \
	-f filelist.f \
	-l compile.log

sim_questa:
	vsim \
	-64 \
	-batch \
	-pli ${VERDI_HOME}/share/PLI/MODELSIM/${PLATFORM}/novas_fli.so \
	work.addertb \
	-do ../scripts/dump_fsdb_questa.tcl \
	+fsdb+flush \
	-l ${OUTPUT}.log


#show the coverage
cov:
	dve -covdir *.vdb &

dbg:
	verdi \
	-sv \
	-f ${FILELIST} \
	-top ${OUTPUT} \
	-ssf ${WAVEFORM}
	

	# dve -vpd ${OUTPUT}.vpd &  we dont use dve
	
clean:
	rm -rf ./csrc *.daidir *.log simv* *.key *.vpd ./DVEfiles ${OUTPUT} *.vdb *.fsdb
	rm -rf novas.* ./verdiLog
	rm -rf *.ini ./work

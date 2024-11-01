global env
fsdbDumpfile "$env(OUTPUT).fsdb"
fsdbDumpvars 0 "addertb"
run -all
quit -sim

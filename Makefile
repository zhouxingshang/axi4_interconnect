all: clean com sim verdi

TOP=tb_top
FILELIST=./UVM_AXI_TB/flist.f

VCS=vcs
VCS_FLAG=-full64 -sverilog -f $(FILELIST) -R +v2k -debug_access+all \
         -timescale=1ns/1ps -fsdb +define+FSDB -l com.log +lint=TFIPC-L +lint=PCWM \
         -ntb_opts uvm-1.2

VERDI=verdi
VERDI_FLAGS=-f $(FILELIST) -ssf $(TOP).fsdb -nologo -sswr \
            -uvmdebug

clean:
    rm -rf csrc *.fsdb *.log *.daidir verdiLog *.conf simv *.key vfastLog *.h *.rc

com:
    $(VCS) $(VCS_FLAG)

sim:
    ./simv -l sim.log

verdi:
    $(VERDI) $(VERDI_FLAGS) &
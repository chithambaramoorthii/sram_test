# Design
set ::env(DESIGN_NAME) "SRAM_Wrapper_top"

set ::env(PDK) {sky130B}
set ::env(VERILOG_FILES) [glob  ./designs/SRAM_16x16_1/src/*.v ]

set ::env(CLOCK_PERIOD) "10.000"
set ::env(CLOCK_PORT) "clk"
#set ::env(CELL_PAD) 4

set ::env(EXTRA_LIBS) "./designs/SRAM_16x16_1/Integrated_bitcell_with_dummy_cells.lib"

set ::env(EXTRA_LEFS) { ./designs/SRAM_16x16_1/Integrated_bitcell_with_dummy_cells.lef}

set ::env(DIE_AREA) {0.0 0.0 2500 2500  }

set ::env(FP_SIZING) "absolute"

#set ::env(TAP_DECAP_INSERTION) {0}

#set ::env(FILL_INSERTION) {0}
#set ::env(QUIT_ON_TR_DRC) {0}
set ::env(PL_MACRO_HALO) {100 100}

#set ::env(FP_PDN_ENABLE_MACROS_GRID) {0}
set ::env(SYNTH_NO_FLAT) {0}

#set ::env(ROUTING_OPT_ITERS) {10}
set ::env(EXTRA_GDS_FILES)  { ./designs/SRAM_16x16_1/Integrated_bitcell_with_dummy_cells.gds} 


#set ::env(FP_PIN_ORDER_CFG) $::env(OPENLANE_ROOT)/designs/rram_wrapper_16x16/pin_order.cfg

set ::env(MACRO_PLACEMENT_CFG) [glob $::env(DESIGN_DIR)/macro.cfg]

set ::env(SYNTH_BUFFERING) {0}


set ::env(PL_BASIC_PLACEMENT) {1}

#set ::env(TAP_DECAP_INSERTION) {0}

set filename ./designs/$::env(DESIGN_NAME)/$::env(PDK)_$::env(STD_CELL_LIBRARY)_config.tcl
if { [file exists $filename] == 1} {
	source $filename
}

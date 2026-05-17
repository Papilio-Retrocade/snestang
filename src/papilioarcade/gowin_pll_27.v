// gowin_pll_27.v - Stub module for GW2A-18C (Papilio Retrocade)
// This module is NOT instantiated in the NANO clock path.
// It was used in the PRIMER path (50MHz->27MHz), which is not needed here
// since the Tang Primer 20K already has a 27MHz oscillator.
// Kept as a valid Verilog module to avoid compile errors from the .gprj file list.

module gowin_pll_27 (clkout0, clkin);

output clkout0;
input clkin;

// Passthrough - not used in NANO path
assign clkout0 = clkin;

endmodule //gowin_pll_27

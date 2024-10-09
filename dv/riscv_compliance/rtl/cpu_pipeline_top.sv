/* verilator lint_off EOFNEWLINE */
module cpu_pipeline_top import ibex_pkg::*; #(
    /* verilator lint_off UNUSED */
                        parameter bit          PMPEnable        = 1'b0,
                        parameter int unsigned PMPGranularity   = 0,
                        parameter int unsigned PMPNumRegions    = 4,
                        parameter int unsigned MHPMCounterNum   = 0,
                        parameter int unsigned MHPMCounterWidth = 40,
                        parameter bit          RV32E            = 1'b0,
                        parameter rv32m_e      RV32M            = RV32MFast,
                        parameter rv32b_e      RV32B            = RV32BNone,
                        parameter regfile_e    RegFile          = RegFileFF,
                        parameter bit          BranchTargetALU  = 1'b0,
                        parameter bit          WritebackStage   = 1'b0,
                        parameter bit          ICache           = 1'b0,
                        parameter bit          ICacheECC        = 1'b0,
                        parameter bit          BranchPredictor  = 1'b0,
                        parameter bit          DbgTriggerEn     = 1'b0,
                        parameter int unsigned DbgHwBreakNum    = 1,
                        parameter bit          SecureIbex       = 1'b0,
                        parameter bit          ICacheScramble   = 1'b0,
                        parameter lfsr_seed_t  RndCnstLfsrSeed  = RndCnstLfsrSeedDefault,
                        parameter lfsr_perm_t  RndCnstLfsrPerm  = RndCnstLfsrPermDefault,
                        parameter int unsigned DmHaltAddr       = 32'h1A110800,
                        parameter int unsigned DmExceptionAddr  = 32'h1A110808
                        /* verilator lint_on UNUSED */
                        )(CLK, RST_N, test_en_i, scan_rst_ni, ram_cfg_i, hart_id_i, boot_addr_i, 
                        instr_req_o, instr_gnt_i, instr_rvalid_i, instr_addr_o, instr_rdata_i,
                        instr_rdata_intg_i, instr_err_i, data_req_o, data_gnt_i,
                        data_rvalid_i, data_we_o, data_be_o, data_addr_o, data_wdata_o,
                        data_wdata_intg_o, data_rdata_i, data_rdata_intg_i, data_err_i,
                        irq_software_i, irq_timer_i, irq_external_i, irq_fast_i,
                        irq_nm_i, scramble_key_valid_i, scramble_key_i, scramble_nonce_i, scramble_req_o, 
                        debug_req_i, crash_dump_o, double_fault_seen_o, fetch_enable_i, alert_minor_o, 
                        alert_major_internal_o, alert_major_bus_o, core_sleep_o,
                        rvfi_valid, rvfi_order, rvfi_insn, rvfi_trap, rvfi_halt, rvfi_intr,
                        rvfi_mode, rvfi_ixl, rvfi_rs1_addr, rvfi_rs2_addr, rvfi_rs3_addr, 
                        rvfi_rs1_rdata, rvfi_rs2_rdata, rvfi_rs3_rdata, rvfi_rd_addr, rvfi_rd_wdata, 
                        rvfi_pc_rdata, rvfi_pc_wdata, rvfi_mem_addr, rvfi_mem_rmask, rvfi_mem_wmask, 
                        rvfi_mem_rdata, rvfi_mem_wdata, rvfi_ext_pre_mip, rvfi_ext_post_mip, rvfi_ext_nmi, 
                        rvfi_ext_nmi_int, rvfi_ext_debug_req, rvfi_ext_debug_mode, rvfi_ext_rf_wr_suppress,
                        rvfi_ext_mcycle, 
                        rvfi_ext_ic_scr_key_valid, rvfi_ext_irq_valid);

input CLK, RST_N;
input test_en_i; 
input prim_ram_1p_pkg::ram_1p_cfg_t ram_cfg_i;
input [31:0] hart_id_i, boot_addr_i;

input instr_gnt_i, instr_rvalid_i, instr_err_i;
input [31:0] instr_rdata_i;
input [6:0] instr_rdata_intg_i;
output instr_req_o;
output [31:0] instr_addr_o;

input data_gnt_i, data_rvalid_i, data_err_i;
input [31:0] data_rdata_i;
input [6:0] data_rdata_intg_i;
output data_req_o, data_we_o;
output [3:0] data_be_o;
output [31:0] data_addr_o;
output [31:0] data_wdata_o;
output [6:0] data_wdata_intg_o;

output        rvfi_valid;
output [63:0] rvfi_order;
output [31:0] rvfi_insn;
output        rvfi_trap;
output        rvfi_halt;
output        rvfi_intr;
output [ 1:0] rvfi_mode;
output [ 1:0] rvfi_ixl;
output [ 4:0] rvfi_rs1_addr;
output [ 4:0] rvfi_rs2_addr;
output [ 4:0] rvfi_rs3_addr;
output [31:0] rvfi_rs1_rdata;
output [31:0] rvfi_rs2_rdata;
output [31:0] rvfi_rs3_rdata;
output [ 4:0] rvfi_rd_addr;
output [31:0] rvfi_rd_wdata;
output [31:0] rvfi_pc_rdata;
output [31:0] rvfi_pc_wdata;
output [31:0] rvfi_mem_addr;
output [ 3:0] rvfi_mem_rmask;
output [ 3:0] rvfi_mem_wmask;
output [31:0] rvfi_mem_rdata;
output [31:0] rvfi_mem_wdata;
output [31:0] rvfi_ext_pre_mip;
output [31:0] rvfi_ext_post_mip;
output        rvfi_ext_nmi;
output        rvfi_ext_nmi_int;
output        rvfi_ext_debug_req;
output        rvfi_ext_debug_mode;
output        rvfi_ext_rf_wr_suppress;
output [63:0] rvfi_ext_mcycle;
//output [31:0] rvfi_ext_mhpmcounters [10];
//output [31:0] rvfi_ext_mhpmcountersh [10];
output        rvfi_ext_ic_scr_key_valid;
output        rvfi_ext_irq_valid;

input scan_rst_ni;
// Interrupt inputs
input  logic                         irq_software_i;
input  logic                         irq_timer_i;
input  logic                         irq_external_i;
input  logic [14:0]                  irq_fast_i;
input  logic                         irq_nm_i;     // non-maskeable interrupt

// Scrambling Interface
input  logic                         scramble_key_valid_i;
input  logic [SCRAMBLE_KEY_W-1:0]      scramble_key_i;
input  logic [SCRAMBLE_NONCE_W-1:0]                         scramble_nonce_i;
output logic                         scramble_req_o;

// Debug Interface
input  logic                         debug_req_i;
output crash_dump_t                 crash_dump_o;
output logic                         double_fault_seen_o;

// CPU Control Signals
input  ibex_mubi_t                   fetch_enable_i;
output logic                         alert_minor_o;
output logic                         alert_major_internal_o;
output logic                         alert_major_bus_o;
output logic                         core_sleep_o;

logic        unused_scan_rst_ni = scan_rst_ni;
logic        unused_irq_software_i = irq_software_i;
logic        unused_irq_timer_i = irq_timer_i;
logic        unused_irq_external_i = irq_external_i;
logic   [14:0]     unused_irq_fast_i = irq_fast_i;
logic        unused_irq_nm_i = irq_nm_i;
logic        unused_scramble_key_valid_i = scramble_key_valid_i;
logic   [SCRAMBLE_KEY_W-1:0]      unused_scramble_key_i = scramble_key_i;
logic    [SCRAMBLE_NONCE_W-1:0]    unused_scramble_nonce_i = scramble_nonce_i;

logic        unused_debug_req_i = debug_req_i;


ibex_mubi_t        unused_fetch_enable_i = fetch_enable_i;

assign scramble_req_o = 0;
//assign crash_dump_o = '0;
assign double_fault_seen_o = 0;
assign  alert_minor_o =0;
assign    alert_major_internal_o = 0;
assign   alert_major_bus_o = 0;
assign  core_sleep_o = 0;


wire [31:0] WRITE_DATA, DATA1, DATA2;
wire [4:0] WRITE_ADDRESS, DATA1_ADDRESS, DATA2_ADDRESS; 
wire WRITE_ENABLE;

cpu_pipeline my_cpu_pipeline (
    .CLK (CLK), 
    .RST_N (RST_N), 
    .test_en_i (test_en_i), 
    .ram_cfg_i (ram_cfg_i), 
    .hart_id_i (hart_id_i), 
    .boot_addr_i(boot_addr_i), 
    .instr_req_o (instr_req_o), 
    .instr_gnt_i (instr_gnt_i), 
    .instr_rvalid_i (instr_rvalid_i), 
    .instr_addr_o (instr_addr_o), 
    .instr_rdata_i (instr_rdata_i),
    .instr_rdata_intg_i (instr_rdata_intg_i), 
    .instr_err_i (instr_err_i), 
    .data_req_o (data_req_o), 
    .data_gnt_i (data_gnt_i),
    .data_rvalid_i (data_rvalid_i), 
    .data_we_o (data_we_o), 
    .data_be_o (data_be_o), 
    .data_addr_o (data_addr_o), 
    .data_wdata_o (data_wdata_o),
    .data_wdata_intg_o (data_wdata_intg_o), 
    .data_rdata_i (data_rdata_i), 
    .data_rdata_intg_i (data_rdata_intg_i), 
    .data_err_i (data_err_i),
    .WB_MUX_OUT (WRITE_DATA), 
    .REG_FILE_OUT1 (DATA1), 
    .REG_FILE_OUT2 (DATA2), 
    .WRITE_ADDRESS_WB (WRITE_ADDRESS), 
    .DATA1_ADDRESS (DATA1_ADDRESS), 
    .DATA2_ADDRESS (DATA2_ADDRESS), 
    .REG_WRITE_EN_WB (WRITE_ENABLE),
    .rvfi_valid, 
    .rvfi_order, 
    .rvfi_insn, 
    .rvfi_trap, 
    .rvfi_halt, 
    .rvfi_intr,
    .rvfi_mode, 
    .rvfi_ixl, 
    .rvfi_rs1_addr, 
    .rvfi_rs2_addr, 
    .rvfi_rs3_addr, 
    .rvfi_rs1_rdata, 
    .rvfi_rs2_rdata, 
    .rvfi_rs3_rdata, 
    .rvfi_rd_addr, 
    .rvfi_rd_wdata, 
    .rvfi_pc_rdata, 
    .rvfi_pc_wdata, 
    .rvfi_mem_addr, 
    .rvfi_mem_rmask, 
    .rvfi_mem_wmask, 
    .rvfi_mem_rdata, 
    .rvfi_mem_wdata, 
    .rvfi_ext_pre_mip, 
    .rvfi_ext_post_mip, 
    .rvfi_ext_nmi, 
    .rvfi_ext_nmi_int, 
    .rvfi_ext_debug_req, 
    .rvfi_ext_debug_mode, 
    .rvfi_ext_rf_wr_suppress,
    .rvfi_ext_mcycle, 
    //.rvfi_ext_mhpmcounters, 
    //.rvfi_ext_mhpmcountersh, 
    .rvfi_ext_ic_scr_key_valid, 
    .rvfi_ext_irq_valid,
    .crash_dump_o
);

reg_file my_reg_file (
    .WRITE_DATA (WRITE_DATA), 
    .DATA1 (DATA1), 
    .DATA2 (DATA2), 
    .WRITE_ADDRESS (WRITE_ADDRESS), 
    .DATA1_ADDRESS (DATA1_ADDRESS), 
    .DATA2_ADDRESS (DATA2_ADDRESS), 
    .WRITE_ENABLE (WRITE_ENABLE), 
    .CLK (CLK), 
    .RST_N (RST_N) 
);

endmodule
/* verilator lint_on EOFNEWLINE */

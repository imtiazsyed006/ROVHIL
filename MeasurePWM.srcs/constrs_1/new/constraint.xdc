#===========================================================
# ZedBoard (PL) minimal constraints for UART on PMOD JA
# VADJ jumper set to 3.3V (JA is 3.3V bank)
# Clock: Y9 is the on-board 100 MHz oscillator into PL
#===========================================================

#---------------------------
# Clock (PL) - 100 MHz
#---------------------------
set_property PACKAGE_PIN Y9 [get_ports {clk_100m}]
set_property IOSTANDARD LVCMOS33 [get_ports {clk_100m}]
create_clock -name clk_100m -period 10.000 [get_ports {clk_100m}]  ;# 100 MHz

#---------------------------
# Reset (SW0) - active low
# (commented out: 'resetn'/'enable_sw' ports are not present on the current
#  top module 'main'; the design uses an internal power-on reset instead.
#  Uncomment if you re-add these ports.)
#---------------------------
# set_property PACKAGE_PIN F22 [get_ports {resetn}]
# set_property IOSTANDARD LVCMOS18 [get_ports {resetn}]
# set_property PULLUP true [get_ports {resetn}]

#---------------------------
# UART on PMOD JA (PL)
#---------------------------
# uart_tx -> JA1
set_property PACKAGE_PIN Y11 [get_ports {uart_tx}]
set_property IOSTANDARD LVCMOS33 [get_ports {uart_tx}]

# uart_rx -> JA2
set_property PACKAGE_PIN AA11 [get_ports {uart_rx}]
set_property IOSTANDARD LVCMOS33 [get_ports {uart_rx}]

#---------------------------
# Debug UART on PMOD JA (PL) — JA3, JA4
#---------------------------
# dbg_uart_tx -> JA3
set_property PACKAGE_PIN Y10 [get_ports {dbg_uart_tx}]
set_property IOSTANDARD LVCMOS33 [get_ports {dbg_uart_tx}]

# dbg_uart_rx -> JA4
set_property PACKAGE_PIN AA9 [get_ports {dbg_uart_rx}]
set_property IOSTANDARD LVCMOS33 [get_ports {dbg_uart_rx}]
#---------------------------
# Status LEDs LD0-LD7 (PL)
#   LD0 heartbeat | LD1 Mx-present | LD2 My-present | LD3 Mz-present
#   LD4 Mx-sign(+) | LD5 My-sign(+) | LD6 Mz-sign(+) | LD7 any-PWM-active
#---------------------------
set_property PACKAGE_PIN T22 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN T21 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property PACKAGE_PIN U22 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property PACKAGE_PIN U21 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property PACKAGE_PIN V22 [get_ports {led[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[4]}]
set_property PACKAGE_PIN W22 [get_ports {led[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[5]}]
set_property PACKAGE_PIN U19 [get_ports {led[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[6]}]
set_property PACKAGE_PIN U14 [get_ports {led[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[7]}]
# enable_sw not present on current top module (commented out to avoid warnings)
# set_property PACKAGE_PIN G22 [get_ports {enable_sw}]
# set_property IOSTANDARD LVCMOS18 [get_ports {enable_sw}]

#===========================================================
# 8 PWM inputs on PMOD JB (PL, Bank 13, 3.3V)
# Top row:  JB1=W12  JB2=W11  JB3=V10  JB4=W8
# Bot row:  JB7=V12  JB8=W10  JB9=V9   JB10=V8
# Channel order (matches pwm_in[7:0] in main.v):
#   pwm_in[0] FR_Top  -> JB1   pwm_in[4] FR_Bot -> JB7
#   pwm_in[1] FL_Top  -> JB2   pwm_in[5] FL_Bot -> JB8
#   pwm_in[2] RR_Top  -> JB3   pwm_in[6] RR_Bot -> JB9
#   pwm_in[3] RL_Top  -> JB4   pwm_in[7] RL_Bot -> JB10
#===========================================================
set_property PACKAGE_PIN W12 [get_ports {pwm_in[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pwm_in[0]}]
set_property PACKAGE_PIN W11 [get_ports {pwm_in[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pwm_in[1]}]
set_property PACKAGE_PIN V10 [get_ports {pwm_in[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pwm_in[2]}]
set_property PACKAGE_PIN W8 [get_ports {pwm_in[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pwm_in[3]}]
set_property PACKAGE_PIN V12 [get_ports {pwm_in[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pwm_in[4]}]
set_property PACKAGE_PIN W10 [get_ports {pwm_in[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pwm_in[5]}]
set_property PACKAGE_PIN V9 [get_ports {pwm_in[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pwm_in[6]}]
set_property PACKAGE_PIN V8 [get_ports {pwm_in[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pwm_in[7]}]

#===========================================================
# Timing exceptions for off-chip I/O
#
# All I/O on this top level is asynchronous to the 100 MHz PL
# clock and has no external launch/capture clock:
#   - pwm_in, uart_rx, dbg_uart_rx : async inputs, each handled
#     by a 2-FF synchronizer inside the design
#   - led, uart_tx, dbg_uart_tx     : slow outputs (LEDs ~1 Hz,
#     UART 115200 baud) with no setup relationship to sample
# Marking them false_path tells timing analysis not to expect a
# constrained I/O relationship, which clears the TIMING-18
# ("Missing input or output delay") warnings.
#===========================================================
set_false_path -from [get_ports {pwm_in[*]}]
set_false_path -from [get_ports {uart_rx}]
set_false_path -from [get_ports {dbg_uart_rx}]
set_false_path -to   [get_ports {led[*]}]
set_false_path -to   [get_ports {uart_tx}]
set_false_path -to   [get_ports {dbg_uart_tx}]

#===========================================================
# Multicycle paths for 32x32 Q16.16 multiplies
#
# A full 32x32 needs a multi-DSP48 cascade that cannot meet a single
# 10 ns period on the -1 speed grade. The FSMs in rov_model and um7_tx
# hold every pure-fmul state for 2 clocks (mul_hold) before the result
# is consumed, so setup may use 2 cycles and hold uses 1.
#
# Filters require IS_SEQUENTIAL so LUT helper cells named "*_reg*_i_*"
# are not matched (those are not valid endpoints).
#===========================================================
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r1_*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r1_*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r2_dq*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r2_dq*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r2_g*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r2_g*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r3_cw*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r3_cw*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r3b_eul*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r3b_eul*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r3b_t*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r3b_t*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r5_*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r5_*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r6_n*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/r6_n*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/kprod_*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/kprod_*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/up_*_dt_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_model/up_*_dt_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_um7/m_*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_um7/m_*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_um7/q2_*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_um7/q2_*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_um7/qs_*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_um7/qs_*_reg*}]
set_multicycle_path 2 -setup -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_um7/deg_*_reg*}]
set_multicycle_path 1 -hold  -to [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ *u_um7/deg_*_reg*}]
# Internal DSP48 cascade stages of the 32x32 fmul also need 2 cycles
# (source/dest are DSP A/B/P registers, not the fabric r1_* flops).
# Scoped to u_model / u_um7 only — tau_alloc keeps single-cycle DSP stages.
set_multicycle_path 2 -setup \
    -from [get_cells -hier -filter {PRIMITIVE_SUBGROUP == DSP && NAME =~ *u_model*}] \
    -to   [get_cells -hier -filter {PRIMITIVE_SUBGROUP == DSP && NAME =~ *u_model*}]
set_multicycle_path 1 -hold  \
    -from [get_cells -hier -filter {PRIMITIVE_SUBGROUP == DSP && NAME =~ *u_model*}] \
    -to   [get_cells -hier -filter {PRIMITIVE_SUBGROUP == DSP && NAME =~ *u_model*}]
set_multicycle_path 2 -setup \
    -from [get_cells -hier -filter {PRIMITIVE_SUBGROUP == DSP && NAME =~ *u_um7*}] \
    -to   [get_cells -hier -filter {PRIMITIVE_SUBGROUP == DSP && NAME =~ *u_um7*}]
set_multicycle_path 1 -hold  \
    -from [get_cells -hier -filter {PRIMITIVE_SUBGROUP == DSP && NAME =~ *u_um7*}] \
    -to   [get_cells -hier -filter {PRIMITIVE_SUBGROUP == DSP && NAME =~ *u_um7*}]

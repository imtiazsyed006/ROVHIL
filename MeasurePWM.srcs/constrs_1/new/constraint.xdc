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

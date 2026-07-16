# Rebuild synth+impl and dump timing summary
open_project C:/VivadoWork/MeasurePWM/MeasurePWM.xpr
update_compile_order -fileset sources_1

# Ensure q16_mul is in sources
set q16 C:/VivadoWork/MeasurePWM/MeasurePWM.srcs/sources_1/new/q16_mul.v
if {[llength [get_files -quiet $q16]] == 0} {
  add_files -fileset sources_1 $q16
}
update_compile_order -fileset sources_1

reset_run synth_1
launch_runs synth_1 -jobs 16
wait_on_run synth_1

set synth_status [get_property STATUS [get_runs synth_1]]
puts "SYNTH STATUS: $synth_status"
if {[string match "*ERROR*" $synth_status] || [string match "*Failed*" $synth_status]} {
  puts "SYNTH FAILED"
  exit 1
}

reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 16
wait_on_run impl_1

set impl_status [get_property STATUS [get_runs impl_1]]
puts "IMPL STATUS: $impl_status"

open_run impl_1
report_timing_summary -max_paths 20 -file C:/VivadoWork/MeasurePWM/fpga_verify/timing_summary_postfix.rpt
report_utilization -file C:/VivadoWork/MeasurePWM/fpga_verify/utilization_postfix.rpt

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "========================================"
puts "WNS = $wns ns"
puts "========================================"

# Print top 10 paths briefly
set paths [get_timing_paths -max_paths 10 -nworst 10 -setup]
foreach p $paths {
  set slack [get_property SLACK $p]
  set startpoint [get_property STARTPOINT_PIN $p]
  set endpoint [get_property ENDPOINT_PIN $p]
  puts [format "  slack=%7.3f  %s -> %s" $slack $startpoint $endpoint]
}

if {$wns < 0} {
  puts "TIMING STILL FAILING"
  exit 2
}
puts "TIMING MET"
exit 0

open_project C:/VivadoWork/MeasurePWM/MeasurePWM.xpr
update_compile_order -fileset sources_1
reset_run synth_1
launch_runs impl_1 -jobs 8
wait_on_run impl_1
open_run impl_1
report_timing_summary -file C:/VivadoWork/MeasurePWM/fpga_verify/timing_summary.rpt
report_utilization -file C:/VivadoWork/MeasurePWM/fpga_verify/utilization.rpt
exit

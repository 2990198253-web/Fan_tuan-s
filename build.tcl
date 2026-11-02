# T20 (key_sw[3]) is shared with SSPI and must be available as GPIO.
set project_dir [file dirname [file normalize [info script]]]
open_project [file join $project_dir swing_lab.gprj]
set_option -top_module swing_lab_top
set_option -use_sspi_as_gpio 1
run all

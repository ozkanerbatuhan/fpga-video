# Prints the worst setup paths (start, end, levels, cells on the path).
#   vivado -mode batch -source hw/timing_paths.tcl -tclargs <routed.dcp> [n]
open_checkpoint [lindex $argv 0]
set n [expr {[llength $argv] > 1 ? [lindex $argv 1] : 20}]
foreach p [get_timing_paths -setup -max_paths $n -nworst 1 -unique_pins] {
    puts [format "PATH %7.3f LL%2d %s -> %s" [get_property SLACK $p] [get_property LOGIC_LEVELS $p] \
        [get_property STARTPOINT_PIN $p] [get_property ENDPOINT_PIN $p]]
    set cells {}
    foreach c [get_cells -of_objects $p] { lappend cells "[get_property REF_NAME $c]:[regsub {.*/u_top/} $c {}]" }
    puts "     [join $cells { > }]"
}

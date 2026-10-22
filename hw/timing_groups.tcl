# Groups the failing setup paths of a routed checkpoint by source/destination module.
#   vivado -mode batch -source hw/timing_groups.tcl -tclargs <routed.dcp>
open_checkpoint [lindex $argv 0]
set paths [get_timing_paths -setup -max_paths 3000 -nworst 1 -slack_lesser_than 0]
array set grp {}
foreach p $paths {
    set s [get_property STARTPOINT_PIN $p]
    set d [get_property ENDPOINT_PIN $p]
    regsub {/[^/]*$} [regsub {\[.*} $s ""] "" sm
    regsub {/[^/]*$} [regsub {\[.*} $d ""] "" dm
    set k "$sm -> $dm"
    if {![info exists grp($k)]} { set grp($k) [list 0 0 [get_property LOGIC_LEVELS $p]] }
    lassign $grp($k) n w ll
    set sl [get_property SLACK $p]
    set grp($k) [list [expr {$n + 1}] [expr {min($w, $sl)}] $ll]
}
foreach k [array names grp] {
    lassign $grp($k) n w ll
    puts [format "GRP %7.3f %5d LL%2d  %s" $w $n $ll $k]
}

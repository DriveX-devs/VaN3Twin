$node_(0) set X_ 1 # line OK 
$node_(0) set Y_ 2 # line OK 
$node_(0) set Z_ 3 # line OK 
$node_(-22) set Y_ 3 # node id not correct
$node_(3.3) set Y_ 1111 # node id not correct
$ns_ at sss "$node_(0) setdest 5 5 5" # time is not a number
$ns_ at 1 "$node_(0) setdest 2 2 1" # line OK 
$ns_ at 1 "$node_(0) setdest 2 2 -1" # negative speed is not correct
$ns_ at 1 "$node_(0) setdest 2 2 sdfs"    # speed is not a number
$ns_ at 1 "$node_(0) setdest 2 2 s232dfs" # speed is not a number
$ns_ at 1 "$node_(0) setdest 233 2.. s232dfs"   # more than one non numbers
$ns_ at -12 "$node_(0) setdest 11 22 33" # time should not be negative

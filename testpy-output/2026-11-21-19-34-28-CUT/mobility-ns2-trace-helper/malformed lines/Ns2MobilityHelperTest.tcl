$node() set X_ 1 # node id is not present
$node # incoplete line"
$node this line is not correct
$node_(0) set X_ 1 # line OK 
$node_(0) set Y_ 2 # line OK 
$node_(0) set Z_ 3 # line OK 
$ns_ at  "$node_(0) setdest 4 4 4" # time not present
$ns_ at 1 "$node_(0) setdest 2 2 1   " # line OK 

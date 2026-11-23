$node_(0) set X_ 1.0
$node_(0) set Y_ 2.0
$node_(0) set Z_ 3.0
$ns_ at 1.0 "$node_(1) setdest 25 0 5"
$node_(2) set X_ 0.0
$node_(2) set Y_ 0.0
$ns_ at 1.0 "$node_(2) setdest 5  0  5"
$ns_ at 2.0 "$node_(2) setdest 5  5  5"
$ns_ at 3.0 "$node_(2) setdest 0  5  5"
$ns_ at 4.0 "$node_(2) setdest 0  0  5"

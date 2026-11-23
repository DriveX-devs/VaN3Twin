#!/bin/bash
# Run one v2p-TIP-80211p scenario and store every log/pcap in TIP-logs/<set>/<scenario-name>/
# Usage: run_scenario.sh <set: sync|desync> <scenario-name> <tip-mode: none|single|all> [tip-ped] [sim-time]
#   sync:   all the stations check the CAM/VAM conditions at the same instants (--desync=false, original behaviour)
#   desync: the CAM/VAM dissemination of each station starts after a random time between 0 and 1 s (--desync=true)
set -u
NS3_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
SET="$1"; NAME="$2"; MODE="$3"; PED="${4:-1003}"; SIMTIME="${5:-100}"
OUT="$NS3_DIR/TIP-logs/$SET/$NAME"
[ "$SET" = "desync" ] && DESYNC=true || DESYNC=false

rm -rf "$OUT"
mkdir -p "$OUT/VAM_triggering" "$OUT/CAM_triggering" "$OUT/pcap" "$OUT/sumo"

cd "$NS3_DIR"
ARGS="--sumo-gui=false --sim-time=$SIMTIME --met-sup=true --desync=$DESYNC --tip-mode=$MODE --tip-ped=$PED --log-dir=$OUT --csv-log=$OUT/VAM_metrics --csv-log-cumulative=$OUT/PRR_latency_CBR_summary"
echo "v2p-TIP-80211p $ARGS" > "$OUT/command.txt"
START=$(date +%s)
./ns3 run --no-build "v2p-TIP-80211p $ARGS" > "$OUT/ns3_stdout.log" 2> "$OUT/ns3_stderr.log"
RC=$?
echo "exit_code=$RC wall_clock_s=$(( $(date +%s) - START ))" >> "$OUT/command.txt"
exit $RC

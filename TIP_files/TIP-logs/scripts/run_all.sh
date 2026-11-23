#!/bin/bash
# Run the four TIP scenarios (100 s each, the default of run_scenario.sh), with desynchronised transmissions, then generate all the plots
cd "$(dirname "$0")"
for SET in desync; do
  ./run_scenario.sh $SET 0_no_TIP        none   1003 || exit 1   # baseline: TIP disabled for every VRU
  ./run_scenario.sh $SET 1_TIP_ped1003   single 1003 || exit 1   # TIP enabled only for ped1003 (scenario default)
  ./run_scenario.sh $SET 2_TIP_ped1014   single 1014 || exit 1   # TIP enabled only for ped1014 (hit by veh12 at t=49.64 s)
  ./run_scenario.sh $SET 3_TIP_all_VRUs  all    1003 || exit 1   # TIP enabled for all the VRUs
done
python3 plot_results.py
python3 plot_vam_map.py   # SUMO map with the VAMs sent by ped1014 (pass another station ID to change pedestrian)

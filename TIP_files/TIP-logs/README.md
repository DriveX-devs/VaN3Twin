# TIP-logs — v2p-TIP-80211p, with and without TIP-based VAM triggering

Scenario: `src/automotive/examples/v2p-TIP-80211p.cc` (802.11p, 12 Mbit/s, 23 dBm), SUMO map
`sumo_files_v2p_tip`, 100 s, no SUMO GUI. The SUMO configuration file ends at 55 s, so the example now passes
`--end <sim-time>` to SUMO (only when `--log-dir` is used).
45 vehicles and 25 pedestrians are defined; 30 vehicles and all 25 pedestrians enter within 100 s.
The runs are deterministic (same SUMO seed and ns-3 seed), so the mobility is identical in all scenarios.

## Runs

All the runs are in `desync/`: each station starts its CAM/VAM dissemination after a random time between 0 and
1 s, as in the other VaN3Twin examples (`--desync=true`, now the default of the scenario).

## Scenarios (inside `desync/`)

| Folder | TIP | Command-line switch |
|---|---|---|
| `0_no_TIP` | disabled for every VRU (baseline) | `--tip-mode=none` |
| `1_TIP_ped1003` | only ped1003 (the scenario default) | `--tip-mode=single --tip-ped=1003` |
| `2_TIP_ped1014` | only ped1014 | `--tip-mode=single --tip-ped=1014` |
| `3_TIP_all_VRUs` | every VRU | `--tip-mode=all` |

ped1014 was chosen as the second VRU because SUMO records a real collision for it
(`sumo/collisions.xml`: veh12 hits ped1014 at t = 49.64 s) and it is among the VRUs sending the most
TIP-triggered VAMs when TIP is enabled for everybody. TIP parameters are those already in the scenario
(exp modality, TTCmax 4 s, TTCmin 1 s, k 0.27, STCmin 5 m, threshold 0.1).

## Content of each scenario folder

| Path | Content |
|---|---|
| `VAM_metrics.csv` | one row per VAM sent by any VRU: time, time since the previous VAM, triggering condition (0 start, 1 time, 2 heading, 3 position, 4 speed, 5 safe distance/proximity, 6 TIP), position, speed |
| `TIP_metrics.csv` | (TIP scenarios) one row per CAM/VAM received by a TIP-enabled VRU: the TTC (T2C), STC and TIP it computed. The three values are -1 when no collision is predicted (not approaching, TTC >= 4 s or STC > 5 m) |
| `VAM_triggering/pedXXXX.log` | verbose log of every 100 ms VAM condition check of each VRU |
| `CAM_triggering/vehX.log` | verbose log of the CAM condition checks of each vehicle |
| `CBR.csv` | CBR of every node for every 100 ms window (fraction of the window in which the PHY is in TX, RX or CCA_BUSY state). Rows exist for the whole run; the plots only use the windows in which the station is in the SUMO simulation |
| `CBR_MetricSupervisor.csv` | CBR as logged by the framework's Metric Supervisor. **Do not use for analysis**: windows with no PHY activity are skipped and the previous value is repeated, which overestimates the CBR in a lightly loaded channel like this one |
| `PRR_latency_CBR_summary.csv` | Metric Supervisor averages (PRR with 150 m baseline, latency) and message counters |
| `pcap/v2p-vam-80211p-N-0.pcap` | all frames transmitted and received by node N. N = 0..44 is vehN; N = 45..69 is ped(1000 + N - 45) |
| `sumo/` | SUMO FCD mobility trace (0.1 s period), collisions, trip information, error log |
| `ns3_stdout.log`, `ns3_stderr.log`, `command.txt` | simulator output, exact command line, exit code |
| `plots/` | figures of the scenario (see below) |

The line `Error: tcpip::Socket::recvAndCheck @ recv: peer shutdown` in `ns3_stderr.log` is printed by
SUMO when ns-3 closes the TraCI connection at the end of the run; all runs exited with code 0.

## Plots

Per scenario (`desync/<scenario>/plots/`), in the style of thesis Figures 5.6 and 5.8:

- `VAM_dissemination_{AllPed,ped1003,ped1014}.png` — share of VAMs per triggering cause (Fig. 5.6 style);
- `VAM_occurrences_{AllPed,ped1003,ped1014}.png` — occurrences vs. VAM periodicity (Fig. 5.8 style);
- `CBR_over_time.png` — CBR of the scenario over time;
- `VAM_map_ped1014.png` — SUMO map with one dot for each VAM sent by ped1014, coloured by triggering cause
  (drawn by `scripts/plot_vam_map.py`). The first VAM (dissemination start) is included, as a white dot. VAMs
  sent too close to each other to fit along the path are stacked sideways from it. Each label names another road
  user and is linked to the dots about it: for the Proximity dots, the road user which came inside the safe
  distances (`stationID_safe_dist` in `VAM_metrics.csv`); for the TIP dots, the station(s) whose TIP required the
  VAM (the `[TIP] TIPs found ...: StationID=... TIP=...;` line of `VAM_triggering/pedXXXX.log`, which now lists them);
- `TTC_TIP_evolution_pedXXXX.png` (TIP scenarios) — TTC and TIP of the TIP-enabled VRU over time towards the
  stations on a collision course, and the VAMs it sends. Dots are the values computed by the VRU from the
  received messages; lines are the same formula applied offline to the SUMO positions every 100 ms.

Cross-scenario (`desync/comparison/`): the four donuts and the four histograms side by side, the VAM counts per
cause (`.png` and `.csv`), the CBR over time (all stations, ped1003, ped1014) and the CBR summary (`.png` and `.csv`).
The donuts, the histograms and the VAM counts also exist for the reference VRU only (files ending in `_ped1003`).
`VAM_map_ped1014_all_scenarios.png` puts the four VAM maps of ped1014 side by side.

Differences from the thesis figures:

- the first VAM of each VRU (dissemination start) is not considered in any plot or table, because it is
  sent automatically and not triggered by a condition (the thesis plots count it as "Time"). It is still
  present in `VAM_metrics.csv` with triggering condition 0;
- there is no "Hazardous situations / PotentialCollision" sub-category.

## Why the transmissions are desynchronised

With `--desync=false` (the behaviour of the scenario before the desync was added) all CAM and VAM generation
checks run on the same 100 ms grid, so most frames are sent while another station is transmitting. Stations
cannot receive while they transmit and concurrent frames collide. Since the TIP is only evaluated when a CAM/VAM
is received, this limits the number of TIP-triggered VAMs. A set of runs made with `--desync=false` (no longer
kept in this folder; 55 s runs) had an average PRR (150 m baseline) of about 0.18 instead of 0.48, and the
pedestrians received about 1750 CAMs instead of 6100.

The desync seed is the simulation time plus the station ID (the other examples use the simulation time
only), so that stations entering at the same instant, such as veh0 and ped1000, get different offsets.

## Proximity (safe distance) trigger: ported from OScar

All the results in this folder use the safe-distance computation of OScar (`src/VRUBasicService.cpp`,
`get_min_distance()`), ported to `VRUdp::get_min_distance()`. Before the port the function passed the heading in
degrees to `cos()`, used the wrong slots when looking for the nearest pedestrian/vehicle (vehicles could never
trigger a proximity VAM) and did not project the distance on the heading. What changed:

- longitudinal/lateral distance = projection of the distance along/across the heading of the *other* station
  (ETSI TS 103 300-2), using the ego heading if the other one is unavailable;
- correct pedestrian and vehicle slots, ego station skipped;
- trigger block as in OScar (`vamredmit_verified` handling, `m_safedist_sent` counted for pedestrians too);
- one deviation from OScar: the CAMs of VaN3Twin carry no altitude, so a station with unavailable altitude is
  considered at the same height as the VRU (otherwise its vertical distance would be 8000 m and no vehicle
  could ever trigger).

Station selection (fixed here and in OScar, `/mnt/xtra/OScar/src/VRUBasicService.cpp`): OScar replaced the stored
pedestrian/vehicle only with one closer on all three axes at once; on flat ground the vertical distance is always
0, so the first station found in the LDM was kept and a closer one was ignored. Now a station inside the safe
distances is always preferred to one outside them, and among equals the nearest is kept, so the condition is
detected whenever at least one station in the LDM satisfies it.

"New station" rule (added here and in OScar, `src/VRUBasicService.cpp` and `include/VRUBasicService.h`): TS 103 300-3
(clause 6.4.1, condition 7) asks for a VAM when one or more *new* stations come inside the safe distances "after
the lastly transmitted VAM". Before, a proximity VAM was sent at every 100 ms check for as long as a station was
inside. Now the stations inside the safe distances when a VAM is transmitted (for any cause) are remembered, and
`get_min_distance()` does not consider them until a VAM is transmitted while they are not inside. Consequences:

- a station which leaves the safe distances and comes back triggers again, if a VAM was sent in between;
- the same happens to a station which disappears from the LDM and reappears inside the safe distances. LDM
  entries expire after about 1 s, less than the usual 3 s between two VAMs of a walking pedestrian, so this
  accounted for 5 or 6 of the 19 or 20 proximity VAMs of each 55 s run (the other 14 were first encounters). In
  the 100 s runs, 8 to 11 of the 37 to 41 proximity VAMs repeat a sender/station pair already seen (not
  checked one by one);
- in `VAM_triggering/pedXXXX.log` the `[SAFE DISTANCES]` minimum distances no longer include the stations
  already reported.

Triggered proximity VAMs, baseline without TIP:

| Code | Run | Proximity VAMs |
|---|---|---|
| original | 55 s | 245 |
| OScar distance computation | 55 s | 114 |
| + station selection fix | 55 s | 447 |
| + "new station" rule | 55 s | 19 |
| + "new station" rule (current results) | 100 s | 37 |

## Other files

`SUMO_scenario.png` shows the SUMO network and the trajectories of the run (drawn by `scripts/plot_scenario.py`
from the network file and the FCD trace).

## How to reproduce

`scripts/run_all.sh` re-runs the four 100 s runs and the plots (`scripts/run_scenario.sh desync ...` runs one,
with the simulation time as optional last argument;
`scripts/plot_results.py` only regenerates the figures). The example must be built first:
`./ns3 build v2p-TIP-80211p`.

#!/usr/bin/env python3
"""Post-processing of the v2p-TIP-80211p scenarios stored in TIP-logs/.

For every scenario it generates, in the style of Figures 5.6 and 5.8 of the thesis:
  - the VAM dissemination per triggering cause (donut chart);
  - the VAM occurrences as a function of the VAM periodicity (stacked histogram);
both for all the pedestrians and for the pedestrians of interest (ped1003, ped1014).
It also generates the Channel Busy Ratio (CBR) plots and the cross-scenario comparisons.
"""
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.ticker
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
# Sets of runs: folder -> description used in the titles
SETS = {".": ""}
SIM_TIME = 100     # duration of the runs [s] (see run_all.sh)
BASE = ROOT        # folder of the set being processed (set by run_set())
SET_NAME = ""      # name of the set being processed

SCENARIOS = [  # (folder, label, colour used when scenarios are compared)
    ("0_no_TIP", "No TIP", "#555555"),
    ("1_TIP_ped1003", "TIP on ped1003", "#1f77b4"),
    ("2_TIP_ped1014", "TIP on ped1014", "#2ca02c"),
    ("3_TIP_all_VRUs", "TIP on all VRUs", "#ff7f0e"),
]
FOCUS_PEDS = [1003, 1014]
REF_PED = 1003  # reference VRU: the cross-scenario VAM comparisons are also generated for this pedestrian only

# triggcond_t (VRUBasicService.h) -> (label, colour); same palette as the thesis figures
CAUSES = [
    (3, "Position", "#17becf"),
    (2, "Heading", "#bcbd22"),
    (1, "Time", "#9467bd"),
    (4, "Speed", "#d62728"),
    (5, "Proximity", "#1f77b4"),
    (6, "TIP", "#ff7f0e"),
]
# TIP parameters configured in v2p-TIP-80211p.cc (used to draw the thresholds and the TIP curve)
TTC_MAX, TTC_MIN, TIP_K, STC_MIN, DELTA_TIP = 4.0, 1.0, 0.27, 5.0, 0.1
REMOTE_COLOURS = ["#7b3fa0", "#2ca02c", "#8c564b", "#e377c2", "#17becf", "#bcbd22"]
INK = "#222222"
MUTED = "#666666"

plt.rcParams.update({
    "font.size": 10, "axes.edgecolor": "#999999", "axes.labelcolor": INK, "text.color": INK,
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
    "figure.facecolor": "white", "savefig.facecolor": "white", "savefig.dpi": 160,
})


def load_vam(folder):
    """VAMs sent by the VRUs. The first VAM of each VRU (dissemination start, triggering condition 0) is
    discarded: it is sent automatically when the VRU starts and it is not triggered by any condition."""
    v = pd.read_csv(os.path.join(BASE, folder, "VAM_metrics.csv"))
    return v[v.triggering_condition != 0]


def cause_counts(vam):
    return [int((vam.triggering_condition == code).sum()) for code, _, _ in CAUSES]


def draw_donut(ax, vam, title):
    counts = cause_counts(vam)
    total = sum(counts)
    shown = [(c, col) for c, (_, _, col) in zip(counts, CAUSES) if c > 0]
    ax.pie([c for c, _ in shown], colors=[col for _, col in shown], startangle=90, counterclock=True,
           wedgeprops=dict(width=0.33, edgecolor="white", linewidth=1.2))
    ax.text(0, 0, f"{total}\nVAMs", ha="center", va="center", fontsize=11, color=INK)
    ax.set_title(title, fontsize=10, fontweight="bold")
    handles = [plt.Rectangle((0, 0), 1, 1, color=col) for _, _, col in CAUSES]
    labels = [f"{lab} -- {100 * c / total:.2f}% ({c})" if total else lab for c, (_, lab, _) in zip(counts, CAUSES)]
    return handles, labels


def draw_occurrences(ax, vam, title, ymax=None):
    per = vam
    xs = np.arange(0, 5001, 100)
    bottom = np.zeros(len(xs))
    for code, lab, col in CAUSES:
        sel = per[per.triggering_condition == code].time_elapsed_since_last_gen.clip(upper=5000)
        h = np.array([(sel == x).sum() for x in xs], dtype=float)
        ax.bar(xs, h, width=80, bottom=bottom, color=col, label=lab, linewidth=0)
        bottom += h
    ax.set_xlim(-150, 5150)
    ax.set_xticks(np.arange(0, 5001, 500))
    if ymax:
        ax.set_ylim(0, ymax)
    ax.grid(axis="y", color="#dddddd", linewidth=0.6)
    ax.set_axisbelow(True)
    ax.set_xlabel("VAM periodicity [ms]")
    ax.set_ylabel("Occurrences")
    ax.set_title(title, fontsize=10, fontweight="bold")
    return bottom.max()


def presence(folder):
    """First/last time [s] in which each station is in the SUMO simulation (from the FCD output)."""
    t, rows = None, []
    with open(os.path.join(BASE, folder, "sumo", "fcd.xml")) as f:
        for line in f:
            m = re.search(r'<timestep time="([\d.]+)"', line)
            if m:
                t = float(m.group(1))
                continue
            m = re.search(r'<(?:vehicle|person) id="(\w+)"', line)
            if m:
                rows.append((m.group(1), t))
    d = pd.DataFrame(rows, columns=["station_ID", "t"])
    return d.groupby("station_ID").t.agg(t_in="min", t_out="max")


def load_cbr(folder):
    """CBR of the stations, only for the windows in which they are in the simulation."""
    c = pd.read_csv(os.path.join(BASE, folder, "CBR.csv")).join(presence(folder), on="station_ID", how="inner")
    return c[(c.window_start_ms / 1000 >= c.t_in) & (c.window_end_ms / 1000 <= c.t_out + 0.1)]


def load_fcd(folder):
    """Position, heading and speed of every station every 100 ms (SUMO FCD output)."""
    t, rows = None, []
    pat = re.compile(r'<(?:vehicle|person) id="(\w+)" x="([\d.\-]+)" y="([\d.\-]+)" angle="([\d.\-]+)".*? speed="([\d.\-]+)"')
    with open(os.path.join(BASE, folder, "sumo", "fcd.xml")) as f:
        for line in f:
            m = re.search(r'<timestep time="([\d.]+)"', line)
            if m:
                t = round(float(m.group(1)) * 10)
                continue
            m = pat.search(line)
            if m:
                rows.append((t, m.group(1)) + tuple(float(x) for x in m.groups()[1:]))
    return pd.DataFrame(rows, columns=["step", "id", "x", "y", "ang", "v"])


def ttc_from_positions(fcd, ego, remote):
    """TTC/STC between two stations from the SUMO positions, with the same formulas and thresholds of
    VRUBasicService::compute_ttc_stc (i.e., what the VRU would compute if it received a message every 100 ms)."""
    j = fcd[fcd.id == ego].set_index("step").join(fcd[fcd.id == remote].set_index("step"), rsuffix="_r", how="inner")
    dx, dy = j.x - j.x_r, j.y - j.y_r
    dist = np.hypot(dx, dy)
    dvx = j.v * np.sin(np.radians(j.ang)) - j.v_r * np.sin(np.radians(j.ang_r))
    dvy = j.v * np.cos(np.radians(j.ang)) - j.v_r * np.cos(np.radians(j.ang_r))
    closing = (dx * dvx + dy * dvy) / dist
    ttc = np.where(closing < 0, dist / -closing, np.nan)
    stc = np.hypot(dx + dvx * ttc, dy + dvy * ttc)
    ttc = np.where((ttc < TTC_MAX) & (stc <= STC_MIN), ttc, np.nan)
    return pd.DataFrame({"t": j.index / 10, "ttc": ttc, "tip": np.minimum(1.0, np.exp(-TIP_K * (ttc - TTC_MIN)))})


def plot_ttc_tip(folder, label, ped):
    """Evolution of the TTC (T2C) and of the TIP of a TIP-enabled VRU, and the VAMs it sends."""
    tip = pd.read_csv(os.path.join(BASE, folder, "TIP_metrics.csv"))
    tip = tip[tip.station_ID == ped].copy()
    tip["remote"] = np.where(tip.remote_is_vehicle == 1, "veh", "ped") + tip.remote_station_ID.astype(str)
    tip["t"] = tip.timestamp_ms / 1000
    valid = tip[tip.tip >= 0]
    vam = load_vam(folder)
    vam = vam[vam.station_ID == ped]
    fcd = load_fcd(folder)
    remotes = list(valid.groupby("remote").t.min().sort_values().index)
    t0 = max(0, np.floor(valid.t.min() - 3)) if len(valid) else 0

    vam = vam[vam.timestamp / 1000 >= t0]
    fig, axes = plt.subplots(3, 1, figsize=(11, 10.5), sharex=True, gridspec_kw=dict(height_ratios=[3, 3, 1.6]))
    ax_ttc, ax_tip, ax_vam = axes
    tip_vams = vam[vam.triggering_condition == 6].timestamp / 1000
    for ax in axes:
        for t in tip_vams:
            ax.axvline(t, color="#ff7f0e", linewidth=1.0, alpha=0.55, zorder=0)

    for remote, col in zip(remotes, REMOTE_COLOURS):
        truth = ttc_from_positions(fcd, f"ped{ped}", remote)
        s = valid[valid.remote == remote]
        ax_ttc.plot(truth.t.to_numpy(), truth.ttc.to_numpy(), color=col, linewidth=1.4, alpha=0.55)
        ax_ttc.plot(s.t.to_numpy(), s.ttc_s.to_numpy(), "o", color=col, markersize=6, markeredgecolor="white", markeredgewidth=0.8,
                    label=f"{remote} ({len(s)} valid of {int((tip.remote == remote).sum())} received)")
        ax_tip.plot(truth.t.to_numpy(), truth.tip.to_numpy(), color=col, linewidth=1.4, alpha=0.55)
        ax_tip.plot(s.t.to_numpy(), s.tip.to_numpy(), "o", color=col, markersize=6, markeredgecolor="white", markeredgewidth=0.8, label=remote)
    # Messages received from any station for which no collision is predicted (TTC >= TTCmax or STC > STCmin)
    inval = tip[tip.tip < 0]
    ax_ttc.plot(inval.t.to_numpy(), np.full(len(inval), TTC_MAX + 0.45), "|", color=MUTED, markersize=7, alpha=0.7,
                label=f"message received, no collision predicted ({len(inval)})")
    for y, txt in ((TTC_MAX, f"TTCmax = {TTC_MAX:g} s"), (TTC_MIN, f"TTCmin = {TTC_MIN:g} s (TIP = 1 below)")):
        ax_ttc.axhline(y, color=MUTED, linewidth=0.8, linestyle="--")
        ax_ttc.text(SIM_TIME, y + 0.05, txt, ha="right", va="bottom", fontsize=8, color=MUTED)
    ax_ttc.set_ylim(0, TTC_MAX + 0.8)
    ax_ttc.set_ylabel("TTC (T2C) [s]")
    ax_ttc.set_title(f"TTC towards the stations on a collision course — ped{ped}, {label}\n"
                     "dots: computed by the VRU from received CAMs/VAMs; lines: same formula on the SUMO positions every 100 ms",
                     fontsize=9.5, fontweight="bold", loc="left", pad=6)
    ax_ttc.legend(fontsize=8, frameon=False, loc="lower center", bbox_to_anchor=(0.5, 1.13), ncol=3)

    ax_tip.axhline(np.exp(-TIP_K * (TTC_MAX - TTC_MIN)), color=MUTED, linewidth=0.8, linestyle="--")
    ax_tip.text(SIM_TIME, np.exp(-TIP_K * (TTC_MAX - TTC_MIN)) - 0.02, f"TIP at TTCmax = {np.exp(-TIP_K * (TTC_MAX - TTC_MIN)):.2f}",
                ha="right", va="top", fontsize=8, color=MUTED)
    ax_tip.set_ylim(0, 1.08)
    ax_tip.set_ylabel("TIP")
    ax_tip.set_title(f"TIP = min(1, exp(-{TIP_K}·(TTC - {TTC_MIN:g}))); a TIP VAM needs an increase ≥ {DELTA_TIP} since the last VAM sent\n"
                     f"vertical orange lines: the {len(tip_vams)} TIP-triggered VAMs", fontsize=9.5, fontweight="bold", loc="left")

    order = [c for c in CAUSES if (vam.triggering_condition == c[0]).any()]
    for i, (code, lab, col) in enumerate(order):
        ts = vam[vam.triggering_condition == code].timestamp / 1000
        ax_vam.plot(ts.to_numpy(), np.full(len(ts), i), "|", color=col, markersize=13, markeredgewidth=1.6)
    ax_vam.set_yticks(range(len(order)))
    ax_vam.set_yticklabels([f"{lab} ({int((vam.triggering_condition == code).sum())})" for code, lab, _ in order])
    ax_vam.set_ylim(-0.6, len(order) - 0.4)
    ax_vam.set_title(f"VAMs sent by ped{ped} from t = {t0:g} s, per triggering cause", fontsize=9.5, fontweight="bold", loc="left")
    ax_vam.set_xlabel("Simulation time [s]")
    ax_vam.set_xlim(t0, SIM_TIME)
    for ax in axes:
        ax.grid(axis="x", color="#eeeeee", linewidth=0.6)
    fig.tight_layout()
    fig.savefig(os.path.join(BASE, folder, "plots", f"TTC_TIP_evolution_ped{ped}.png"))
    plt.close(fig)


def run_set(set_name):
    global BASE, SET_NAME
    BASE, SET_NAME = os.path.join(ROOT, set_name), set_name
    vams = {f: load_vam(f) for f, _, _ in SCENARIOS}
    cbrs = {f: load_cbr(f) for f, _, _ in SCENARIOS}
    cmp_dir = os.path.join(BASE, "comparison")
    os.makedirs(cmp_dir, exist_ok=True)

    # Common y-axis of the occurrence histograms, so that the scenarios can be compared at a glance
    def occ_max(v):
        return v.time_elapsed_since_last_gen.clip(upper=5000).value_counts().max() if len(v) else 1
    ymax_all = 1.08 * max(occ_max(v) for v in vams.values())

    # ---------- Per-scenario figures (Figure 5.6 and Figure 5.8 style) ----------
    for folder, label, _ in SCENARIOS:
        out = os.path.join(BASE, folder, "plots")
        os.makedirs(out, exist_ok=True)
        subsets = [("AllPed", vams[folder], ymax_all)]
        for ped in FOCUS_PEDS:
            sub = vams[folder][vams[folder].station_ID == ped]
            ymax_ped = 1.08 * max(occ_max(vams[f][vams[f].station_ID == ped]) for f, _, _ in SCENARIOS)
            subsets.append((f"ped{ped}", sub, ymax_ped))
        for tag, vam, ymax in subsets:
            fig, ax = plt.subplots(figsize=(8.6, 4.2))
            handles, labels = draw_donut(ax, vam, f"VAM dissemination — {label} - {tag}")
            ax.legend(handles, labels, loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=True, fontsize=8.5)
            fig.tight_layout()
            fig.savefig(os.path.join(out, f"VAM_dissemination_{tag}.png"))
            plt.close(fig)

            fig, ax = plt.subplots(figsize=(8.6, 4.6))
            draw_occurrences(ax, vam, f"VAM occurrences — {label} - {tag}", ymax)
            ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.78), frameon=True, fontsize=8.5)
            fig.tight_layout()
            fig.savefig(os.path.join(out, f"VAM_occurrences_{tag}.png"))
            plt.close(fig)

        # CBR of the scenario: mean over the stations in the simulation and spread among them (1 s bins)
        c = cbrs[folder].copy()
        c["sec"] = (c.window_start_ms // 1000).astype(int)
        per_station = c.groupby(["sec", "station_ID"]).CBR.mean().reset_index()
        g = per_station.groupby("sec").CBR
        fig, ax = plt.subplots(figsize=(8.6, 4.2))
        secs = g.mean().index.to_numpy() + 0.5
        ax.fill_between(secs, 100 * g.min().to_numpy(), 100 * g.max().to_numpy(), color="#17becf", alpha=0.25, linewidth=0,
                        label="Min–max among stations")
        ax.plot(secs, 100 * g.mean().to_numpy(), color="#0b7f8c", linewidth=2, label="Mean over stations")
        ax.set_xlabel("Simulation time [s]")
        ax.set_ylabel("CBR [%] (1 s average)")
        ax.set_xlim(0, SIM_TIME)
        ax.set_ylim(0, None)
        ax.grid(axis="y", color="#dddddd", linewidth=0.6)
        ax.set_title(f"Channel Busy Ratio — {label} - all stations", fontsize=10, fontweight="bold")
        ax.legend(loc="upper left", frameon=False, fontsize=8.5)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "CBR_over_time.png"))
        plt.close(fig)

    # ---------- Cross-scenario comparison ----------
    # Generated for all the pedestrians and for the reference VRU only (files with the _pedXXXX suffix)
    def comparison_figures(sel, who, suffix):
        """sel: dict folder -> VAMs to be considered; who: text for the titles; suffix: appended to the file names"""
        # Donuts
        fig, axes = plt.subplots(1, 4, figsize=(15, 6.2))
        for ax, (folder, label, _) in zip(axes, SCENARIOS):
            handles, _ = draw_donut(ax, sel[folder], label)
            counts = cause_counts(sel[folder])
            ax.text(0, -1.2, "\n".join(f"{lab}: {100 * c / max(sum(counts), 1):.1f}% ({c})" for c, (_, lab, _) in zip(counts, CAUSES)),
                    ha="center", va="top", fontsize=8.5, color=MUTED, linespacing=1.35)
            ax.set_ylim(-2.45, 1.15)
        fig.legend(handles, [lab for _, lab, _ in CAUSES], loc="lower center", ncol=len(CAUSES), frameon=False, fontsize=9)
        fig.suptitle(f"Dissemination of the triggered VAMs per cause — {who}", fontsize=12, fontweight="bold")
        fig.tight_layout(rect=(0, 0.06, 1, 0.95))
        fig.savefig(os.path.join(cmp_dir, f"VAM_dissemination_all_scenarios{suffix}.png"))
        plt.close(fig)

        # Occurrences: the y-axis is clipped just above the second highest bar, and the bars off scale are annotated
        tops = sorted((int(n) for v in sel.values()
                       for n in v.time_elapsed_since_last_gen.clip(upper=5000).value_counts().to_numpy()), reverse=True)
        peak = max(v.time_elapsed_since_last_gen.clip(upper=5000).value_counts().max() if len(v) else 0 for v in sel.values())
        per_scen_second = [sorted(v.time_elapsed_since_last_gen.clip(upper=5000).value_counts().to_numpy(), reverse=True)[1:2]
                           for v in sel.values()]
        second = max([int(x[0]) for x in per_scen_second if len(x)] + [1])
        clip = max(5, int(np.ceil(1.25 * second)))
        clipped = peak > 2 * clip   # clip only when the highest bar would otherwise flatten all the others
        if not clipped:
            clip = 1.08 * peak
        fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True, sharey=True)
        for ax, (folder, label, _) in zip(axes.flat, SCENARIOS):
            v = sel[folder]
            draw_occurrences(ax, v, label, clip)
            vc = v.time_elapsed_since_last_gen.clip(upper=5000).value_counts()
            for x_bar, n_bar in vc[vc > clip].items():
                ax.annotate(f"{int(n_bar)} (off scale)", xy=(x_bar, clip), xytext=(x_bar + 230, 0.945 * clip), fontsize=8, color=INK,
                            arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8), va="center")
        for ax in axes.flat:
            ax.tick_params(labelbottom=True, labelleft=True)
            ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
        h, l = axes[0, 0].get_legend_handles_labels()
        fig.legend(h, l, loc="lower center", ncol=6, frameon=False, fontsize=9)
        note = f" (y-axis clipped at {clip:g})" if clipped else ""
        fig.suptitle(f"VAM occurrences vs. periodicity — {who}{note}", fontsize=12, fontweight="bold")
        fig.tight_layout(rect=(0, 0.05, 1, 0.96))
        fig.savefig(os.path.join(cmp_dir, f"VAM_occurrences_all_scenarios{suffix}.png"))
        plt.close(fig)

        # VAMs per cause, grouped by scenario
        table = pd.DataFrame({label: cause_counts(sel[folder]) for folder, label, _ in SCENARIOS},
                             index=[lab for _, lab, _ in CAUSES])
        table.loc["Total"] = table.sum()
        table.to_csv(os.path.join(cmp_dir, f"VAM_counts_per_cause{suffix}.csv"))
        fig, ax = plt.subplots(figsize=(10, 4.6))
        x = np.arange(len(CAUSES))
        w = 0.2
        for i, (folder, label, col) in enumerate(SCENARIOS):
            vals = table[label].iloc[:-1].values
            bars = ax.bar(x + (i - 1.5) * w, vals, width=w * 0.9, color=col, label=f"{label} (total {table[label]['Total']})")
            ax.bar_label(bars, fontsize=7, color=INK, padding=1)
        ax.set_xticks(x)
        ax.set_xticklabels([lab for _, lab, _ in CAUSES])
        ax.set_ylabel("Number of VAMs")
        ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
        ax.grid(axis="y", color="#dddddd", linewidth=0.6)
        ax.set_axisbelow(True)
        ax.legend(frameon=False, fontsize=8.5)
        ax.set_title(f"Triggered VAMs per cause — {who}, {SIM_TIME} s", fontsize=11, fontweight="bold")
        fig.tight_layout()
        fig.savefig(os.path.join(cmp_dir, f"VAM_counts_per_cause{suffix}.png"))
        plt.close(fig)
        return table

    table = comparison_figures(vams, "all pedestrians", "")
    table_ref = comparison_figures({f: v[v.station_ID == REF_PED] for f, v in vams.items()},
                                   f"ped{REF_PED} only (reference VRU)", f"_ped{REF_PED}")

    # CBR over time: mean over all the stations, and CBR measured by the two pedestrians of interest
    panels = [("Mean over all stations", None)] + [(f"Measured by ped{p}", f"ped{p}") for p in FOCUS_PEDS]
    t_first_tip = min(v[v.triggering_condition == 6].timestamp.min() for v in vams.values()
                      if (v.triggering_condition == 6).any()) / 1000
    fig, axes = plt.subplots(len(panels), 1, figsize=(10, 9), sharex=True)
    for ax, (ptitle, station) in zip(axes, panels):
        for folder, label, col in SCENARIOS:
            c = cbrs[folder]
            if station:
                c = c[c.station_ID == station]
            s = c.groupby((c.window_start_ms // 1000).astype(int)).CBR.mean()
            ax.plot(s.index.to_numpy() + 0.5, 100 * s.to_numpy(), color=col, linewidth=2.2 if folder == "0_no_TIP" else 1.5,
                    alpha=0.9, label=label)
        ax.set_ylabel("CBR [%] (1 s average)")
        ax.set_ylim(0, None)
        ax.grid(axis="y", color="#dddddd", linewidth=0.6)
        ax.set_title(ptitle, fontsize=10, fontweight="bold", loc="left")
        # The runs are identical until the first TIP-triggered VAM: before it the four lines overlap exactly
        ax.axvline(t_first_tip, color=MUTED, linewidth=0.8, linestyle=":")
        note = f"first TIP-triggered VAM (t = {t_first_tip:.1f} s):\nbefore it the four runs are identical and the lines overlap"
        if station:
            t_in = cbrs[SCENARIOS[0][0]][cbrs[SCENARIOS[0][0]].station_ID == station].t_in.iloc[0]
            ax.axvline(t_in, color=MUTED, linewidth=0.8, linestyle="--")
            left = t_in > 15
            ax.annotate(f"{station} enters the\nsimulation (t = {t_in:.1f} s)", xy=(t_in, ax.get_ylim()[1]),
                        xytext=(-4 if left else 4, -4), textcoords="offset points", ha="right" if left else "left", va="top",
                        fontsize=8, color=MUTED)
        else:
            ax.annotate(note, xy=(t_first_tip, 0), xytext=(8, 8), textcoords="offset points", ha="left", va="bottom",
                        fontsize=8, color=MUTED, bbox=dict(facecolor="white", alpha=0.85, edgecolor="none", pad=2))
    axes[0].legend(ncol=4, frameon=False, fontsize=9, loc="lower right", bbox_to_anchor=(1, 1.02))
    axes[-1].set_xlabel("Simulation time [s]")
    axes[-1].set_xlim(0, SIM_TIME)
    fig.suptitle(f"Channel Busy Ratio over time", fontsize=12, fontweight="bold", x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(os.path.join(cmp_dir, "CBR_over_time.png"))
    plt.close(fig)

    # CBR summary per scenario
    rows = []
    for folder, label, _ in SCENARIOS:
        c = cbrs[folder]
        row = {"scenario": label, "total_VAMs": len(vams[folder]),
               "TIP_VAMs": int((vams[folder].triggering_condition == 6).sum()),
               "mean_CBR_all_stations_percent": 100 * c.CBR.mean(),
               "mean_CBR_pedestrians_percent": 100 * c[c.station_ID.str.startswith("ped")].CBR.mean(),
               "mean_CBR_vehicles_percent": 100 * c[c.station_ID.str.startswith("veh")].CBR.mean(),
               "peak_CBR_100ms_window_percent": 100 * c.CBR.max()}
        for p in FOCUS_PEDS:
            row[f"mean_CBR_ped{p}_percent"] = 100 * c[c.station_ID == f"ped{p}"].CBR.mean()
        ms = pd.read_csv(os.path.join(BASE, folder, "PRR_latency_CBR_summary.csv")).iloc[-1]
        row["avg_PRR_150m_baseline"] = ms.avg_PRR_overall
        row["received_CAMs_by_pedestrians"] = int(ms.num_received_CAMs_ped)
        row["received_VAMs_by_vehicles"] = int(ms.num_received_VAMs_veh)
        rows.append(row)
    summary = pd.DataFrame(rows)
    summary.to_csv(os.path.join(cmp_dir, "CBR_summary.csv"), index=False, float_format="%.5f")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    cols = [col for _, _, col in SCENARIOS]
    for ax, (key, ptitle) in zip(axes, [("mean_CBR_all_stations_percent", "Mean CBR (all stations, whole run)"),
                                         ("peak_CBR_100ms_window_percent", "Peak CBR (worst station, 100 ms window)")]):
        bars = ax.bar(summary.scenario, summary[key], color=cols, width=0.6)
        ax.bar_label(bars, labels=[f"{v:.4f}%" for v in summary[key]], fontsize=8.5, color=INK, padding=2)
        ax.set_ylabel("CBR [%]")
        ax.set_ylim(0, summary[key].max() * 1.18)
        ax.grid(axis="y", color="#dddddd", linewidth=0.6)
        ax.set_axisbelow(True)
        ax.tick_params(axis="x", labelsize=8.5)
        ax.set_title(ptitle, fontsize=10, fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(cmp_dir, "CBR_summary.png"))
    plt.close(fig)

    # ---------- TTC / TIP evolution of the VRUs for which TIP is enabled ----------
    plot_ttc_tip("1_TIP_ped1003", "TIP on ped1003", 1003)
    plot_ttc_tip("2_TIP_ped1014", "TIP on ped1014", 1014)
    plot_ttc_tip("3_TIP_all_VRUs", "TIP on all VRUs", 1003)
    plot_ttc_tip("3_TIP_all_VRUs", "TIP on all VRUs", 1014)

    print(f"===== {set_name} =====")
    print(table.to_string())
    print(f"--- ped{REF_PED} only ---")
    print(table_ref.to_string())
    print(summary.T.to_string())
    return summary


if __name__ == "__main__":
    for name in SETS:
        run_set(name)

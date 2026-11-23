#!/usr/bin/env python3
"""Draw, on the SUMO map, one dot for each VAM sent by the scenario-selected pedestrian, coloured by the condition
which triggered it (same colours as the other plots).

Usage: plot_vam_map.py

No TIP and all-VRU TIP scenarios generate maps for both ped1003 and ped1014.
Individual TIP scenarios generate the map for their configured pedestrian.
The comparison pairs each pedestrian's No TIP map with its individual TIP map.

The position of each VAM is the position of the pedestrian in the SUMO FCD trace at the time of the VAM.
VAMs sent too close to each other to be drawn side by side along the path are stacked sideways from it.
The Proximity dots are labelled with the road user which came inside the safe distances (VAM_metrics.csv), the TIP
dots with the station(s) the TIP which triggered the VAM refers to (VAM_triggering/pedXXXX.log). The dots about the
same road user, sent within LABEL_GAP seconds of each other, share one label.
"""
import os
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
import numpy as np
import pandas as pd

from plot_results import CAUSES, INK, MUTED, ROOT, SCENARIOS, SETS
from plot_scenario import draw_net, load_fcd, load_net

SET = "."
SCENARIO_PEDESTRIANS = {
    "0_no_TIP": (1003, 1014),
    "1_TIP_ped1003": (1003,),
    "2_TIP_ped1014": (1014,),
    "3_TIP_all_VRUs": (1003, 1014),
}
# Ordered row by row: upper left, upper right, lower left, lower right.
COMPARISON_PANELS = (
    ("0_no_TIP", 1003, "No TIP: map of ped1003"),
    ("1_TIP_ped1003", 1003, "TIP on ped1003: map of ped1003"),
    ("0_no_TIP", 1014, "No TIP: map of ped1014"),
    ("2_TIP_ped1014", 1014, "TIP on ped1014: map of ped1014"),
)
RADIUS = 0.38                      # radius of a dot [m]
LABEL_PT = 7.5                     # font size of the labels of the dots
LABEL_GAP = 10                     # the dots about the same road user share a label if they are at most this far apart [s]
START = (0, "Dissemination start (first VAM)", "white")   # not triggered by a condition: no colour of the palette
COLOURS = {code: colour for code, _, colour in CAUSES + [START]}
PROXIMITY, TIP = 5, 6
NOTE = ("One dot per VAM, at the position of the pedestrian when it was sent; the VAMs too close to fit along the path are stacked sideways from it, on the side of their label.\n"
        "Each label names another road user and is linked to the dots about it: the road user which came inside the safe distances "
        "(Proximity dots) or the station the TIP refers to (TIP dots).")


def name(station_id):
    return f"veh{station_id}" if station_id < 1000 else f"ped{station_id}"


def load_vams(folder, ped):
    """All the VAMs sent by the pedestrian in a scenario, including the first one (dissemination start), with the
    label of the Proximity and TIP ones (empty string for the others)."""
    v = pd.read_csv(os.path.join(ROOT, SET, folder, "VAM_metrics.csv"))
    v = v[v.station_ID == ped].sort_values("timestamp").reset_index(drop=True)
    # Stations of the TIPs which required a VAM, at each check of the VAM conditions
    tips, t = {}, None
    with open(os.path.join(ROOT, SET, folder, "VAM_triggering", f"ped{ped}.log")) as f:
        for line in f:
            if line.startswith("[LOG] Timestamp="):
                t = int(re.match(r"\[LOG\] Timestamp=(\d+)", line).group(1))
            elif line.startswith("[TIP] TIPs found"):
                tips[t] = [int(i) for i in re.findall(r"StationID=(\d+)", line)]
    labels = []
    for r in v.itertuples():
        if r.triggering_condition == PROXIMITY:
            labels.append(name(int(r.stationID_safe_dist)))
        elif r.triggering_condition == TIP:
            labels.append(", ".join(name(i) for i in tips.get(int(r.timestamp), [])) or "?")
        else:
            labels.append("")
    return v.assign(label=labels)


def position(path, t):
    return np.array([np.interp(t, path.t.to_numpy(), path.x.to_numpy()), np.interp(t, path.t.to_numpy(), path.y.to_numpy())])


def normal(path, t):
    """Unit vector perpendicular to the walking direction at time t."""
    d = position(path, min(t + 1, path.t.max())) - position(path, max(t - 1, path.t.min()))
    if np.hypot(*d) < 1e-6:
        d = position(path, path.t.max()) - position(path, path.t.min())
    return np.array([-d[1], d[0]]) / np.hypot(*d)


def user_sides(vams):
    """Side of the path (+1 or -1) used for the dots and the label of each road user: alternated, in order of
    first appearance, so that the dots about different road users sent in the same place end up on opposite sides."""
    users = []
    for label in vams.label.to_numpy():
        users += [u for u in filter(None, label.split(", ")) if u not in users]
    return {u: 1 if i % 2 == 0 else -1 for i, u in enumerate(users)}


def place_dots(path, times, sides):
    """Centre of the dot of each VAM: on the path if there is room, otherwise shifted sideways (perpendicular to
    the walking direction) until it no longer covers the dots already placed. sides: preferred side of each dot
    (+1 or -1), or 0 to use the nearest free place on either side."""
    centres = []
    for t, side in zip(times, sides):
        p, n = position(path, t), normal(path, t)
        order = [s * i for i in range(1, 30) for s in (1, -1)] if side == 0 else [s * i for s in (side, -side) for i in range(1, 30)]
        for k in [0] + order:
            c = p + k * 2.04 * RADIUS * n
            if all(np.hypot(*(c - o)) >= 1.7 * RADIUS for o in centres):
                break
        centres.append(c)
    return centres


def data_box(ax, artist):
    """Bounding box (x0, y0, x1, y1) of an artist, in data coordinates."""
    (x0, y0), (x1, y1) = ax.transData.inverted().transform(artist.get_window_extent())
    return x0, y0, x1, y1


def place_labels(ax, path, times, centres, vams, sides, boxes, xlim, ylim):
    """One label for each road user, linked by thin lines to the Proximity and TIP dots about it (a new label is used
    if more than LABEL_GAP seconds have passed since the previous dot about the same road user).
    The label is put at the first free place found moving away from the path, on either side of it."""
    fig = ax.figure
    px_per_m = (ax.transData.transform((1, 0)) - ax.transData.transform((0, 0)))[0]
    pt = fig.dpi / 72 / px_per_m                                    # one typographic point, in metres
    circles = [(c, RADIUS) for c in centres] + [(np.array(p), 0.25) for p in zip(path.x.to_numpy()[::3], path.y.to_numpy()[::3])]

    def free(box):
        x0, y0, x1, y1 = box
        if x0 < xlim[0] or x1 > xlim[1] or y0 < ylim[0] or y1 > ylim[1]:
            return False
        if any(x0 < b[2] and b[0] < x1 and y0 < b[3] and b[1] < y1 for b in boxes):
            return False
        return all(np.hypot(c[0] - min(max(c[0], x0), x1), c[1] - min(max(c[1], y0), y1)) >= r for c, r in circles)

    groups = {}                                                     # road user -> groups of dots (indexes)
    for i, label in enumerate(vams.label.to_numpy()):
        for user in filter(None, label.split(", ")):
            g = groups.setdefault(user, [])
            if g and times[i] - times[g[-1][-1]] <= LABEL_GAP:
                g[-1].append(i)
            else:
                g.append([i])
    runs = sorted(((user, run) for user, g in groups.items() for run in g), key=lambda r: r[1][0])

    for text, run in runs:
        w, h = (0.6 * len(text) * LABEL_PT + 7) * pt, (1.25 * LABEL_PT + 6) * pt
        mid = np.mean([centres[i] for i in run], axis=0)
        n = normal(path, np.mean(times[run]))
        tangent = np.array([n[1], -n[0]])
        # Nearest places first, with a strong preference for the side of the path where the dots of this road user are
        candidates = sorted(((d + 0.3 * abs(shift) + (0 if side == sides[text] else 5), tuple(mid + side * d * n + shift * tangent))
                             for d in (1.6, 2.4, 3.2, 4.0, 5.0, 6.0, 7.5, 9.0, 11.0, 13.0)
                             for shift in (0, 1.5, -1.5, 3.0, -3.0) for side in (1, -1)))
        candidates = [np.array(c) for _, c in candidates]
        c = next((c for c in candidates if free((c[0] - w / 2, c[1] - h / 2, c[0] + w / 2, c[1] + h / 2))), candidates[0])
        box = (c[0] - w / 2, c[1] - h / 2, c[0] + w / 2, c[1] + h / 2)
        boxes.append(box)
        for i in run:
            end = (min(max(centres[i][0], box[0]), box[2]), min(max(centres[i][1], box[1]), box[3]))
            ax.plot([centres[i][0], end[0]], [centres[i][1], end[1]], color=INK, linewidth=0.5, zorder=6)
        ax.text(c[0], c[1], text, ha="center", va="center", fontsize=LABEL_PT, color=INK, zorder=8,
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor="#bbbbbb", linewidth=0.5))


def draw_panel(ax, net, path, vams, title, xlim, ylim, collision, ped):
    draw_net(ax, net, xlim, ylim, zoom=True)
    ax.plot(path.x.to_numpy(), path.y.to_numpy(), color=INK, linewidth=1.2, zorder=5)
    times = (vams.timestamp / 1000).to_numpy()
    sides = user_sides(vams)
    centres = place_dots(path, times, [sides.get(label.split(", ")[0], 0) for label in vams.label.to_numpy()])
    for c, code in zip(centres, vams.triggering_condition.to_numpy()):
        ax.add_patch(Circle(c, RADIUS, facecolor=COLOURS[code], edgecolor=INK if code == 0 else "white",
                            linewidth=0.9, zorder=7))
    box = dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor="#bbbbbb", linewidth=0.6)
    line = dict(arrowstyle="-", color=INK, linewidth=0.8)
    notes = [ax.annotate(f"ped{ped} enters\n(t = {path.t.min():.1f} s)", xy=position(path, path.t.min()), xytext=(-8, -52),
                         textcoords="offset points", fontsize=8, color=INK, ha="center", va="top", bbox=box,
                         arrowprops=line, zorder=6),
             ax.annotate(f"end of the run\n(t = {path.t.max():.1f} s)", xy=position(path, path.t.max()), xytext=(8, 46),
                         textcoords="offset points", fontsize=8, color=INK, ha="center", va="bottom", bbox=box,
                         arrowprops=line, zorder=6)]
    if collision is not None:
        notes.append(ax.annotate(f"hit by {collision.get('collider')}\n(t = {float(collision.get('time')):.2f} s)",
                                 xy=[float(v) for v in collision.get("victimFront").split(",")], xytext=(-30, -58),
                                 textcoords="offset points", fontsize=8, color=INK, ha="center", va="top", bbox=box,
                                 arrowprops=line, zorder=6))
    ax.figure.canvas.draw()
    place_labels(ax, path, times, centres, vams, sides, [data_box(ax, a.get_bbox_patch()) for a in notes], xlim, ylim)
    ax.set_title(f"{title} — {len(vams)} VAMs", fontsize=10, fontweight="bold", loc="left")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.tick_params(labelsize=8)


def legend_handles(ped=None):
    handles = [Line2D([], [], marker="o", linestyle="none", markersize=9, markerfacecolor=colour,
                      markeredgecolor=INK if code == 0 else "white", label=label) for code, label, colour in [START] + CAUSES]
    handles.append(Line2D([], [], color=INK, linewidth=1.2, label=f"Path of ped{ped}" if ped is not None else "Pedestrian path"))
    return handles


def main():
    net, fcd = load_net(), load_fcd()
    root = Path(ROOT)
    data = {}
    saved = []

    def save_map(fig, relative, pedestrians, panels=None):
        destination = root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        fig.vam_map_pedestrians = pedestrians
        fig.vam_map_panels = panels
        fig.savefig(destination)
        titles = [fig._suptitle.get_text()] if fig._suptitle else []
        for ax in fig.axes:
            titles.extend(ax.get_title(loc=loc) for loc in ("left", "center", "right"))
        titles = list(dict.fromkeys(title for title in titles if title))
        entry = dict(file=relative, titles=titles, pedestrians=pedestrians)
        if panels is not None:
            entry["panels"] = panels
        saved.append(entry)

    # Render both pedestrians for the baseline and all-VRU scenarios, and the matching
    # pedestrian for each individual-TIP scenario. Cache each scenario/pedestrian pair.
    for folder, label, _ in SCENARIOS:
        collisions = ET.parse(root / SET / folder / "sumo/collisions.xml").getroot()
        for ped in SCENARIO_PEDESTRIANS[folder]:
            path = fcd[fcd.id == f"ped{ped}"]
            if path.empty:
                sys.exit(f"ped{ped} is not in the SUMO FCD trace")
            xlim = (path.x.min() - 8, path.x.max() + 8)
            ylim = (path.y.min() - 9, path.y.max() + 9)
            ratio = (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
            collision = next((c for c in collisions.iter("collision") if c.get("victim") == f"ped{ped}"), None)
            vams = load_vams(folder, ped)
            data[folder, ped] = dict(ped=ped, path=path, vams=vams, collision=collision)

            width = 15
            fig = plt.figure(figsize=(width, 0.9 * width * ratio + 2.0))
            ax = fig.add_axes([0.05, 1.5 / fig.get_figheight(), 0.93, 1 - 2.0 / fig.get_figheight()])
            draw_panel(ax, net, path, vams, f"VAMs sent by ped{ped} — {label}", xlim, ylim, collision, ped)
            fig.legend(handles=legend_handles(ped), loc="lower center", ncol=8, frameon=False, fontsize=9,
                       bbox_to_anchor=(0.5, 0.55 / fig.get_figheight()))
            fig.text(0.5, 0.1 / fig.get_figheight(), NOTE, ha="center", va="bottom", fontsize=8, color=MUTED)
            save_map(fig, f"{folder}/plots/VAM_map_ped{ped}.png", {folder: ped})
            plt.close(fig)

    selected = [data[folder, ped] for folder, ped, _ in COMPARISON_PANELS]
    xlim = (min(d["path"].x.min() for d in selected) - 8,
            max(d["path"].x.max() for d in selected) + 8)
    ylim = (min(d["path"].y.min() for d in selected) - 9,
            max(d["path"].y.max() for d in selected) + 9)
    ratio = (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    width = 26
    fig, axes = plt.subplots(2, 2, figsize=(width, 0.93 * width * ratio + 2.6))
    fig.subplots_adjust(left=0.03, right=0.995, top=1 - 0.9 / fig.get_figheight(), bottom=1.7 / fig.get_figheight(),
                        wspace=0.06, hspace=0.12)
    panels = []
    positions = ("upper_left", "upper_right", "lower_left", "lower_right")
    for ax, position_name, (folder, ped, title) in zip(axes.flat, positions, COMPARISON_PANELS):
        d = data[folder, ped]
        draw_panel(ax, net, d["path"], d["vams"], title, xlim, ylim, d["collision"], ped)
        panels.append(dict(position=position_name, scenario=folder, pedestrian=ped, vam_count=len(d["vams"]), title=title))
    fig.legend(handles=legend_handles(), loc="lower center", ncol=8, frameon=False, fontsize=11,
               bbox_to_anchor=(0.5, 0.6 / fig.get_figheight()))
    fig.text(0.5, 0.1 / fig.get_figheight(), NOTE, ha="center", va="bottom", fontsize=9.5, color=MUTED)
    fig.suptitle("VAM maps — No TIP and individual TIP triggering", fontsize=14, fontweight="bold")
    save_map(fig, "comparison/VAM_map_all_scenarios.png",
             {"0_no_TIP": [1003, 1014], "1_TIP_ped1003": [1003], "2_TIP_ped1014": [1014]}, panels)
    plt.close(fig)

    active = {entry["file"] for entry in saved}
    previous = list(root.glob("*/plots/VAM_map*.png")) + list((root / "comparison").glob("VAM_map*.png"))
    for previous_map in previous:
        relative = previous_map.relative_to(root).as_posix()
        if relative not in active:
            archive = root / ".plot-history/pedestrian-selection" / relative
            archive.parent.mkdir(parents=True, exist_ok=True)
            if archive.exists():
                raise FileExistsError(f"Preserve existing archived map: {archive}")
            previous_map.rename(archive)

    manifest_path = root / "plot_titles.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else []
    manifest = [entry for entry in manifest if (root / entry["file"]).is_file() and not Path(entry["file"]).name.startswith("VAM_map")]
    manifest_path.write_text(json.dumps(manifest + saved, indent=2))
    print("Saved six scenario maps and the comparison:", panels)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Draw the SUMO scenario used by v2p-TIP-80211p: road network (sumo_files_v2p_tip) and the trajectories
followed by the vehicles and pedestrians during the simulation (from the SUMO FCD output)."""
import os
import re
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
import numpy as np
import pandas as pd

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
from pathlib import Path
NS3_ROOT = next(p for p in Path(ROOT).parents if (p / "ns3").is_file())
NET = str(NS3_ROOT / "src/automotive/examples/sumo_files_v2p_tip/v2p_map_t2c.net.xml")
RUN = os.path.join(ROOT, "0_no_TIP")   # the mobility is identical in all the runs

ROAD, SIDEWALK, CROSSING = "#5b5b5b", "#cfcfcf", "#f4f4f4"
VEH, PED = "#1f77b4", "#2ca02c"
FOCUS = {"ped1003": "#d62728", "ped1014": "#ff7f0e", "veh12": "#9467bd"}
INK, MUTED = "#222222", "#666666"


def shape(txt):
    return np.array([[float(v) for v in p.split(",")] for p in txt.split()])


def load_net():
    roads, sidewalks, crossings, areas = [], [], [], []
    for edge in ET.parse(NET).getroot().iter("edge"):
        function = edge.get("function", "normal")
        for lane in edge.iter("lane"):
            pts, width = shape(lane.get("shape")), float(lane.get("width", 3.2))
            if function == "walkingarea":
                areas.append(pts)
            elif function == "crossing":
                crossings.append((pts, width))
            elif lane.get("allow") == "pedestrian":
                sidewalks.append((pts, width))
            else:
                roads.append((pts, width))
    return roads, sidewalks, crossings, areas


def load_fcd():
    t, rows = None, []
    pat = re.compile(r'<(vehicle|person) id="(\w+)" x="([\d.\-]+)" y="([\d.\-]+)"')
    with open(os.path.join(RUN, "sumo", "fcd.xml")) as f:
        for line in f:
            m = re.search(r'<timestep time="([\d.]+)"', line)
            if m:
                t = float(m.group(1))
                continue
            m = pat.search(line)
            if m:
                rows.append((t, m.group(1), m.group(2), float(m.group(3)), float(m.group(4))))
    return pd.DataFrame(rows, columns=["t", "kind", "id", "x", "y"])


def draw_net(ax, net, xlim, ylim, zoom):
    """Road network only (lanes drawn with their real width)."""
    roads, sidewalks, crossings, areas = net
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.figure.canvas.draw()
    # Points per metre, to draw the lanes with their real width
    ppm = ax.get_window_extent().width / (xlim[1] - xlim[0]) * 72 / ax.figure.dpi

    for pts in areas:
        ax.add_patch(Polygon(pts, closed=True, facecolor=SIDEWALK, edgecolor="none", zorder=1))
    for pts, w in sidewalks:
        ax.plot(pts[:, 0], pts[:, 1], color=SIDEWALK, linewidth=max(w * ppm, 0.4), solid_capstyle="butt", zorder=1)
    for pts, w in roads:
        ax.plot(pts[:, 0], pts[:, 1], color=ROAD, linewidth=max(w * ppm, 0.5), solid_capstyle="butt", zorder=2)
    for pts, w in crossings:
        ax.plot(pts[:, 0], pts[:, 1], color=CROSSING, linewidth=max(w * ppm, 0.6), solid_capstyle="butt", zorder=3,
                linestyle=(0, (0.35, 0.35)) if zoom else "-")


def draw(ax, net, fcd, xlim, ylim, zoom):
    draw_net(ax, net, xlim, ylim, zoom)
    for sid, g in fcd.groupby("id"):
        if sid in FOCUS:
            continue
        ax.plot(g.x.to_numpy(), g.y.to_numpy(), color=VEH if g.kind.iloc[0] == "vehicle" else PED,
                linewidth=1.3 if zoom else 0.9, alpha=0.75, zorder=4)
    for sid, col in FOCUS.items():
        g = fcd[fcd.id == sid]
        ax.plot(g.x.to_numpy(), g.y.to_numpy(), color=col, linewidth=2.6 if zoom else 1.8, zorder=6)
        ax.plot(g.x.iloc[0], g.y.iloc[0], "o", color=col, markersize=7 if zoom else 4, markeredgecolor="white",
                markeredgewidth=0.8, zorder=7)
    ax.set_xlabel("x [m]")
    ax.tick_params(labelsize=8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def main():
    net, fcd = load_net(), load_fcd()
    coll = ET.parse(os.path.join(RUN, "sumo", "collisions.xml")).getroot().find("collision")
    if coll is not None:
        cx, cy = (float(v) for v in coll.get("victimFront").split(","))
    ped = fcd[fcd.kind == "person"]
    all_pts = np.vstack([p for p, _ in net[0]])

    fig = plt.figure(figsize=(14, 9.5), facecolor="white")
    ax_all = fig.add_axes([0.05, 0.14, 0.36, 0.76])
    ax_zoom = fig.add_axes([0.46, 0.14, 0.52, 0.76])

    xlim = (all_pts[:, 0].min() - 10, all_pts[:, 0].max() + 10)
    ylim = (all_pts[:, 1].min() - 10, all_pts[:, 1].max() + 10)
    draw(ax_all, net, fcd, xlim, ylim, zoom=False)
    ax_all.set_ylabel("y [m]")
    ax_all.set_title("Whole network", fontsize=10, fontweight="bold", loc="left")

    # Zoom on the crossing used by the two pedestrians of interest (ped1003 and ped1014)
    foc = fcd[fcd.id.isin(["ped1003", "ped1014"])]
    zx = (foc.x.min() - 35, foc.x.max() + 35)
    zy = (foc.y.min() - 45, foc.y.max() + 45)
    ax_all.add_patch(plt.Rectangle((zx[0], zy[0]), zx[1] - zx[0], zy[1] - zy[0], fill=False, edgecolor=INK,
                                   linewidth=1, linestyle="--", zorder=8))
    draw(ax_zoom, net, fcd, zx, zy, zoom=True)
    ax_zoom.set_title("Crossing used by ped1003 and ped1014 (dashed box on the left)", fontsize=10, fontweight="bold", loc="left")
    if coll is not None:
        ax_zoom.plot(cx, cy, "X", color="black", markersize=13, markeredgecolor="white", markeredgewidth=1.2, zorder=9)
        ax_zoom.annotate(f"collision {coll.get('collider')} – {coll.get('victim')}\nt = {float(coll.get('time')):.2f} s", xy=(cx, cy), xytext=(-60, -95),
                         textcoords="offset points", fontsize=9, color=INK, zorder=10,
                         bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="#bbbbbb", linewidth=0.6), ha="right", va="center",
                         arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8))
    else:
        ax_zoom.text(0.02, 0.02, "No collisions recorded by SUMO", transform=ax_zoom.transAxes, fontsize=9, color=MUTED)
    offsets = {"ped1003": (30, 45, "left"), "ped1014": (-30, 45, "right"), "veh12": (30, 0, "left")}
    for sid, col in FOCUS.items():
        g = fcd[fcd.id == sid]
        if zx[0] <= g.x.iloc[0] <= zx[1] and zy[0] <= g.y.iloc[0] <= zy[1]:
            dx, dy, ha = offsets[sid]
            ax_zoom.annotate(f"{sid} enters\n(t = {g.t.iloc[0]:.1f} s)", xy=(g.x.iloc[0], g.y.iloc[0]), xytext=(dx, dy),
                             textcoords="offset points", fontsize=9, color=INK, zorder=10,
                     bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="#bbbbbb", linewidth=0.6), ha=ha,
                             arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8))

    n_veh, n_ped = fcd[fcd.kind == "vehicle"].id.nunique(), ped.id.nunique()
    duration = np.ceil(fcd.t.max())
    handles = [Line2D([], [], color=ROAD, linewidth=6, label="Vehicle lanes"),
               Line2D([], [], color=SIDEWALK, linewidth=6, label="Sidewalks and walking areas"),
               Line2D([], [], color=CROSSING, linewidth=6, label="Pedestrian crossings", path_effects=None,
                      markeredgecolor=MUTED),
               Line2D([], [], color=VEH, linewidth=1.5, label=f"Vehicle trajectories ({n_veh} vehicles enter in {duration:.0f} s)"),
               Line2D([], [], color=PED, linewidth=1.5, label=f"Pedestrian trajectories ({n_ped} pedestrians)")]
    handles += [Line2D([], [], color=c, linewidth=2.6, marker="o", markeredgecolor="white", label=f"{s} (dot: entry point)")
                for s, c in FOCUS.items()]
    if coll is not None:
        handles.append(Line2D([], [], color="black", marker="X", linestyle="none", markersize=10, markeredgecolor="white",
                              label="Collision recorded by SUMO"))
    leg = fig.legend(handles=handles, loc="lower center", ncol=3, frameon=True, fontsize=9, facecolor="#e6e6e6",
                     edgecolor="none")
    fig.suptitle(f"SUMO scenario of v2p-TIP-80211p (sumo_files_v2p_tip) — network and trajectories over the {duration:.0f} s run",
                 fontsize=12, fontweight="bold")
    fig.savefig(os.path.join(ROOT, "SUMO_scenario.png"), dpi=160)


if __name__ == "__main__":
    main()

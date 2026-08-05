#!/usr/bin/env python3
"""Apply VaN3Twin compatibility updates to an OpenCDA checkout."""

from pathlib import Path
import argparse
import re
import sys


def patch_numpy_aliases(opencda_root):
    changed = 0
    for path in (opencda_root / "opencda").rglob("*.py"):
        text = path.read_text()
        updated = re.sub(r"\bnp\.float(?![A-Za-z0-9_])", "float", text)
        updated = re.sub(r"\bnp\.int(?![A-Za-z0-9_])", "int", updated)
        if updated != text:
            path.write_text(updated)
            changed += 1
    return changed


def patch_map_manager(opencda_root):
    path = opencda_root / "opencda/core/map/map_manager.py"
    if not path.exists():
        raise FileNotFoundError(path)

    changed = 0
    text = path.read_text()
    updated = text.replace(
        "Path(trigger_poly.boundary)",
        "Path(np.asarray(trigger_poly.boundary.coords))",
    )
    if updated != text:
        text = updated
        changed += 1

    if "next_waypoints = waypoint.next(self.lane_sample_resolution)" not in text:
        start_marker = "            waypoints = [waypoint]\n"
        end_marker = "\n\n            # waypoint is the centerline"
        start = text.find(start_marker)
        end = text.find(end_marker, start)
        if start == -1 or end == -1:
            raise RuntimeError("Could not locate OpenCDA lane waypoint loop")
        old = text[start:end]
        if "nxt = waypoint.next(self.lane_sample_resolution)[0]" not in old:
            raise RuntimeError("OpenCDA lane waypoint loop has an unexpected shape")
        new = (
            "            waypoints = [waypoint]\n"
            "            next_waypoints = waypoint.next(self.lane_sample_resolution)\n"
            "            if not next_waypoints:\n"
            "                continue\n"
            "            nxt = next_waypoints[0]\n"
            "            # looping until next lane\n"
            "            while nxt.road_id == waypoint.road_id \\\n"
            "                    and nxt.lane_id == waypoint.lane_id:\n"
            "                waypoints.append(nxt)\n"
            "                next_waypoints = nxt.next(self.lane_sample_resolution)\n"
            "                if not next_waypoints:\n"
            "                    break\n"
            "                nxt = next_waypoints[0]"
        )
        text = text[:start] + new + text[end:]
        changed += 1

    if changed:
        path.write_text(text)
    return changed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("opencda_root", help="Path to the OpenCDA checkout")
    args = parser.parse_args()

    opencda_root = Path(args.opencda_root).resolve()
    if not (opencda_root / "opencda").is_dir():
        print(f"error: OpenCDA package not found under {opencda_root}", file=sys.stderr)
        return 2

    numpy_files = patch_numpy_aliases(opencda_root)
    map_manager_changes = patch_map_manager(opencda_root)
    print(
        "OpenCDA compatibility patch applied "
        f"(numpy files: {numpy_files}, map manager changes: {map_manager_changes})."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

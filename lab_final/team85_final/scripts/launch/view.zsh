#!/bin/zsh
# Watch a run on the robot (scripts/onboard.sh) from the laptop: RViz with the navigation
# layout, the detector's debug image pulled compressed and decoded here (raw images over
# Tailscale load the robot's tailscaled heavily).
# --light: no costmaps / voxels / images either (each subscription costs the robot CPU in
#          tailscaled; with Nav2 + detector onboard its Pi has none to spare).
LIGHT=0; rest=()
for a in "$@"; do [[ $a == --light ]] && LIGHT=1 || rest+=("$a"); done
source ${0:A:h}/../common.zsh "${rest[@]}"
FINAL=$WS/install/team85_final/share/team85_final
cfg=${TMPDIR:-/tmp}/team85_view.rviz
python - $FINAL/rviz/navigation.rviz $cfg $LIGHT <<'PY'
import sys, yaml
src, dst, light = sys.argv[1], sys.argv[2], sys.argv[3] == "1"
cfg = yaml.safe_load(open(src))
HEAVY = ("costmap", "voxel", "image", "points")
def walk(displays):
    for d in displays:
        walk(d.get("Displays", []))
        topic = d.get("Topic", {})
        t = topic.get("Value", "") if isinstance(topic, dict) else ""
        if t == "/sign_detections/image":
            topic["Value"] = "/sign_detections/image_view"
        if light and any(h in t for h in HEAVY):
            d["Enabled"] = False
walk(cfg["Visualization Manager"]["Displays"])
yaml.safe_dump(cfg, open(dst, "w"), sort_keys=False)
PY
if (( ! LIGHT )); then
  ros2 run image_transport republish compressed raw --ros-args \
      -r in/compressed:=/sign_detections/image/compressed -r out:=/sign_detections/image_view &
  rep=$!
  trap "kill $rep 2>/dev/null" EXIT
fi
rviz2 -d $cfg

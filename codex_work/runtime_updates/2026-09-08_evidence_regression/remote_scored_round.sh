#!/usr/bin/env bash
set -o pipefail
round_name="$1"
shift
summary="/home/ros/dev_ws/logs/${round_name}_summary.log"
: > "$summary"
source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ros/dev_ws/config/fastdds_udp_only.xml
round_start=$(date +%s.%N)
item=0
total="$#"
successful=0
for spec in "$@"; do
  item=$((item + 1))
  colour="${spec%%:*}"
  destination="${spec##*:}"
  # Re-evaluate the live robot-to-cube distance after every placement/egress.
  # This intentionally follows the competition policy requested for this
  # workspace: nearest currently available cube of the required colour.
  selector="nearest-${colour}"
  if [[ "$colour" == *_cube_* ]]; then
    selector="$colour"
  fi
  log="/home/ros/dev_ws/logs/${round_name}_item_${item}.log"
  extra=()
  if [ "$item" -lt "$total" ]; then
    extra+=(--egress-after-place)
  fi
  item_start=$(date +%s.%N)
  ros2 run moon_warehouse_coordinator navigation_pick_place_test \
    --object "$selector" --destination "$destination" "${extra[@]}" \
    > "$log" 2>&1
  rc=$?
  item_end=$(date +%s.%N)
  elapsed=$(awk -v s="$item_start" -v e="$item_end" 'BEGIN {printf "%.3f", e-s}')
  selected=$(grep -m1 -oE '(Fastest-mission selection|Nearest-object selection): [a-z_0-9]+' "$log" | awk '{print $NF}')
  max_recovery=$(grep -oE 'recoveries=[0-9]+' "$log" | cut -d= -f2 | sort -nr | head -1)
  max_recovery=${max_recovery:-0}
  printf 'ITEM=%d OBJECT=%s DEST=%s SECONDS=%s RECOVERIES=%s EXIT=%d LOG=%s\n' \
    "$item" "${selected:-unknown}" "$destination" "$elapsed" "$max_recovery" "$rc" "$log" | tee -a "$summary"
  if [ "$rc" -ne 0 ]; then
    break
  fi
  successful=$((successful + 1))
done
round_end=$(date +%s.%N)
round_elapsed=$(awk -v s="$round_start" -v e="$round_end" 'BEGIN {printf "%.3f", e-s}')
printf 'ROUND=%s COMPLETED_ITEMS=%d TOTAL_ITEMS=%d TOTAL_SECONDS=%s TARGET_350=%s\n' \
  "$round_name" "$successful" "$total" "$round_elapsed" "$([ "$successful" -eq "$total" ] && awk -v t="$round_elapsed" 'BEGIN {print (t <= 350.0 ? "PASS" : "MISS")}' || echo INCOMPLETE)" | tee -a "$summary"

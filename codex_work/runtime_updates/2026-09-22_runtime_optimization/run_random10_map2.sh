#!/usr/bin/env bash
set -o pipefail

workspace=/home/ros/dev_ws
tag=random10_final6_20261005
summary="$workspace/logs/${tag}.csv"
progress="$workspace/logs/${tag}.progress"
cleanup="$workspace/src/moon_warehouse_bringup/scripts/ros_runtime_cleanup.sh"

labels=(
  blueA_redB blueC_redB blueC_redA blueB_redC blueA_redC
  blueB_redA blueA_redB blueA_redC blueC_redB blueC_redB
)
batches=(
  'fastest-blue:A,fastest-blue:A,fastest-blue:A,fastest-red:B,fastest-red:B'
  'fastest-blue:C,fastest-blue:C,fastest-blue:C,fastest-red:B,fastest-red:B'
  'fastest-blue:C,fastest-blue:C,fastest-blue:C,fastest-red:A,fastest-red:A'
  'fastest-blue:B,fastest-blue:B,fastest-blue:B,fastest-red:C,fastest-red:C'
  'fastest-blue:A,fastest-blue:A,fastest-blue:A,fastest-red:C,fastest-red:C'
  'fastest-blue:B,fastest-blue:B,fastest-blue:B,fastest-red:A,fastest-red:A'
  'fastest-blue:A,fastest-blue:A,fastest-blue:A,fastest-red:B,fastest-red:B'
  'fastest-blue:A,fastest-blue:A,fastest-blue:A,fastest-red:C,fastest-red:C'
  'fastest-blue:C,fastest-blue:C,fastest-blue:C,fastest-red:B,fastest-red:B'
  'fastest-blue:C,fastest-blue:C,fastest-blue:C,fastest-red:B,fastest-red:B'
)

printf 'round,label,result,total_s,completed_items,item_times_s,mission_log,launch_log\n' > "$summary"
: > "$progress"

for index in "${!labels[@]}"; do
  round=$((index + 1))
  label=${labels[$index]}
  batch=${batches[$index]}
  launch_log="$workspace/logs/${tag}_r${round}_${label}_launch.log"
  mission_log="$workspace/logs/${tag}_r${round}_${label}_mission.log"

  printf 'ROUND_START,%s,%s,%s\n' "$round" "$label" "$(date +%s)" | tee -a "$progress"
  ready=0
  for launch_attempt in 1 2 3; do
    bash "$cleanup" >> "$progress" 2>&1
    source /opt/ros/humble/setup.bash
    source "$workspace/install/setup.bash"
    cd "$workspace"
    launch_log="$workspace/logs/${tag}_r${round}_${label}_launch_a${launch_attempt}.log"
    nohup ros2 launch moon_warehouse_bringup mission_system.launch.py \
      start_rviz:=false gazebo_gui:=false start_perception:=true \
      start_manipulation:=true start_foxglove:=false start_rosbridge:=true \
      > "$launch_log" 2>&1 < /dev/null &
    for _ in $(seq 1 55); do
      spawned=$(grep -c 'Successfully spawned entity' "$launch_log" 2>/dev/null || true)
      active=$(grep -c 'Managed nodes are active' "$launch_log" 2>/dev/null || true)
      errors=$(grep -Eci '\[ERROR\]|Traceback|Exception' "$launch_log" 2>/dev/null || true)
      controller_count=$(grep -c 'Configured and activated' "$launch_log" 2>/dev/null || true)
      if [[ "$spawned" -ge 1 && "$active" -ge 2 && "$errors" -eq 0 \
            && "$controller_count" -ge 3 ]]; then
        # The controller service sometimes sends its DDS response after the
        # CLI client deadline even though all three spawners have completed.
        # The launch events are authoritative; allow Gazebo another 15 s for
        # entity queries and action servers to settle before starting timing.
        sleep 15
        ready=1
        break
      fi
      sleep 1
    done
    if [[ "$ready" -eq 1 ]]; then
      printf 'LAUNCH_READY,%s,%s,%s\n' "$round" "$launch_attempt" "$(date +%s)" | tee -a "$progress"
      break
    fi
    printf 'LAUNCH_RETRY,%s,%s,%s\n' "$round" "$launch_attempt" "$(date +%s)" | tee -a "$progress"
  done

  if [[ "$ready" -ne 1 ]]; then
    printf '%s,%s,LAUNCH_FAILED,,0,,%s,%s\n' \
      "$round" "$label" "$mission_log" "$launch_log" >> "$summary"
    printf 'ROUND_END,%s,%s,LAUNCH_FAILED,%s\n' "$round" "$label" "$(date +%s)" | tee -a "$progress"
    continue
  fi

  timeout 420 ros2 run moon_warehouse_coordinator navigation_pick_place_test \
    --batch "$batch" > "$mission_log" 2>&1
  runner_rc=$?
  complete_line=$(grep 'BATCH COMPLETE:' "$mission_log" | tail -n 1 || true)
  total=$(printf '%s' "$complete_line" | sed -n 's/.*elapsed=\([0-9.]*\)s.*/\1/p')
  completed=$(grep -c 'BATCH ITEM [1-5]/5 complete' "$mission_log" 2>/dev/null || true)
  item_times=$(grep 'BATCH ITEM [1-5]/5 complete' "$mission_log" \
    | sed -n 's/.*elapsed=\([0-9.]*\)s.*/\1/p' | paste -sd ';' -)
  if [[ "$runner_rc" -eq 0 && "$completed" -eq 5 && -n "$total" ]]; then
    result=PASS
  elif [[ "$runner_rc" -eq 124 ]]; then
    result=TIMEOUT
  else
    result=FAIL
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$round" "$label" "$result" "$total" "$completed" "$item_times" \
    "$mission_log" "$launch_log" >> "$summary"
  printf 'ROUND_END,%s,%s,%s,%s,%s\n' \
    "$round" "$label" "$result" "${total:-NA}" "$(date +%s)" | tee -a "$progress"
done

bash "$cleanup" >> "$progress" 2>&1
printf 'ALL_DONE,%s\n' "$(date +%s)" | tee -a "$progress"

#!/usr/bin/env bash
set -o pipefail

workspace=/home/ros/dev_ws
tag=${1:-all24_260_20261007}
only_round=${2:-}
batch_override=${3:-}
summary="$workspace/logs/${tag}.csv"
progress="$workspace/logs/${tag}.progress"
cleanup="$workspace/src/moon_warehouse_bringup/scripts/ros_runtime_cleanup.sh"

if [[ -n $only_round && ! $only_round =~ ^([1-9]|1[0-9]|2[0-4])$ ]]; then
  printf 'ERROR: optional round must be an integer from 1 to 24\n' >&2
  exit 2
fi

# Four non-empty colour splits times six directed pairs of distinct zones.
# Format: blue_count blue_destination red_destination
specs=()
for blue_count in 1 2 3 4; do
  for pair in 'A B' 'A C' 'B A' 'B C' 'C A' 'C B'; do
    specs+=("$blue_count $pair")
  done
done

make_batch() {
  local blue_count=$1 blue_destination=$2 red_destination=$3
  local red_count=$((5 - blue_count))
  local items=() i
  for i in $(seq 1 "$blue_count"); do
    items+=("fastest-blue:${blue_destination}")
  done
  for i in $(seq 1 "$red_count"); do
    items+=("fastest-red:${red_destination}")
  done
  local IFS=,
  printf '%s' "${items[*]}"
}

printf 'round,label,blue_count,blue_destination,red_count,red_destination,result,total_s,le260,completed_items,item_times_s,mission_log,launch_log\n' > "$summary"
: > "$progress"
case_count=${#specs[@]}
if [[ -n $only_round ]]; then
  case_count=1
fi
printf 'CASE_COUNT,%s\n' "$case_count" | tee -a "$progress"

for index in "${!specs[@]}"; do
  round=$((index + 1))
  if [[ -n $only_round && $round -ne $only_round ]]; then
    continue
  fi
  read -r blue_count blue_destination red_destination <<< "${specs[$index]}"
  red_count=$((5 - blue_count))
  label="b${blue_count}${blue_destination}_r${red_count}${red_destination}"
  batch=$(make_batch "$blue_count" "$blue_destination" "$red_destination")
  if [[ -n $batch_override ]]; then
    batch=$batch_override
    label="${label}_ordered"
  fi
  printf 'ROUND_START,%s,%s,%s\n' "$round" "$label" "$(date +%s)" | tee -a "$progress"

  result=LAUNCH_FAILED
  total=
  completed=0
  item_times=
  mission_log=
  launch_log=
  for mission_attempt in 1 2; do
    ready=0
    for launch_attempt in 1 2 3; do
      bash "$cleanup" >> "$progress" 2>&1
      source /opt/ros/humble/setup.bash
      source "$workspace/install/setup.bash"
      cd "$workspace" || exit 1
      launch_log="$workspace/logs/${tag}_r${round}_${label}_launch_m${mission_attempt}_a${launch_attempt}.log"
      nohup ros2 launch moon_warehouse_bringup mission_system.launch.py \
        start_rviz:=false gazebo_gui:=false start_perception:=true \
        start_manipulation:=true start_foxglove:=false start_rosbridge:=true \
        > "$launch_log" 2>&1 < /dev/null &
      for _ in $(seq 1 55); do
        spawned=$(grep -c 'Successfully spawned entity' "$launch_log" 2>/dev/null || true)
        active=$(grep -c 'Managed nodes are active' "$launch_log" 2>/dev/null || true)
        errors=$(grep -Eci '\[ERROR\]|Traceback|Exception' "$launch_log" 2>/dev/null || true)
        controllers=$(grep -c 'Configured and activated' "$launch_log" 2>/dev/null || true)
        if [[ $spawned -ge 1 && $active -ge 2 && $errors -eq 0 && $controllers -ge 3 ]]; then
          sleep 15
          ready=1
          break
        fi
        sleep 1
      done
      if [[ $ready -eq 1 ]]; then
        printf 'LAUNCH_READY,%s,%s,%s,%s\n' "$round" "$mission_attempt" "$launch_attempt" "$(date +%s)" | tee -a "$progress"
        break
      fi
      printf 'LAUNCH_RETRY,%s,%s,%s,%s\n' "$round" "$mission_attempt" "$launch_attempt" "$(date +%s)" | tee -a "$progress"
    done
    [[ $ready -eq 1 ]] || break

    mission_log="$workspace/logs/${tag}_r${round}_${label}_mission_m${mission_attempt}.log"
    timeout 420 ros2 run moon_warehouse_coordinator navigation_pick_place_test \
      --batch "$batch" > "$mission_log" 2>&1
    runner_rc=$?
    complete_line=$(grep 'BATCH COMPLETE:' "$mission_log" | tail -n 1 || true)
    total=$(printf '%s' "$complete_line" | sed -n 's/.*elapsed=\([0-9.]*\)s.*/\1/p')
    completed=$(grep -c 'BATCH ITEM [1-5]/5 complete' "$mission_log" 2>/dev/null || true)
    item_times=$(grep 'BATCH ITEM [1-5]/5 complete' "$mission_log" \
      | sed -n 's/.*elapsed=\([0-9.]*\)s.*/\1/p' | paste -sd ';' -)
    if [[ $runner_rc -eq 0 && $completed -eq 5 && -n $total ]]; then
      result=PASS
    elif [[ $runner_rc -eq 124 ]]; then
      result=TIMEOUT
    else
      result=FAIL
    fi
    if [[ $result != PASS ]]; then
      # Lifecycle-manager fatal messages can arrive just after the mission
      # client exits.  Give the launch log a short flush window so a collapsed
      # controller server is replayed as infrastructure instead of being
      # mislabelled as an algorithm failure.
      sleep 3
    fi
    infrastructure_failure=$(grep -Eci \
      'gzserver.*process has died|planner_server.*process has died|controller_server.*process has died|map_server.*process has died|CRITICAL FAILURE: SERVER .* IS DOWN' \
      "$launch_log" 2>/dev/null || true)
    if [[ $result != PASS && $infrastructure_failure -gt 0 && $mission_attempt -lt 2 ]]; then
      printf 'INFRA_RETRY,%s,%s,%s,%s\n' "$round" "$label" "$mission_attempt" "$(date +%s)" | tee -a "$progress"
      continue
    fi
    break
  done

  le260=NO
  if [[ $result == PASS && -n $total ]] && awk "BEGIN {exit !($total <= 260.0)}"; then
    le260=YES
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$round" "$label" "$blue_count" "$blue_destination" \
    "$red_count" "$red_destination" "$result" "$total" "$le260" \
    "$completed" "$item_times" "$mission_log" "$launch_log" >> "$summary"
  printf 'ROUND_END,%s,%s,%s,%s,%s,%s\n' \
    "$round" "$label" "$result" "${total:-NA}" "$le260" "$(date +%s)" | tee -a "$progress"
done

bash "$cleanup" >> "$progress" 2>&1
printf 'ALL_DONE,%s\n' "$(date +%s)" | tee -a "$progress"

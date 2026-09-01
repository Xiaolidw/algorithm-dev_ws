#!/usr/bin/env bash
set -Eeuo pipefail

# The competition stack owns all ROS 2 processes for this VM user.  After a
# crash Fast DDS can leave port/segment lock files behind, causing live nodes
# to exist but remain undiscoverable.  systemd runs this only after the old
# service control group has stopped and before a new launch is created.
removed="$(find /dev/shm -maxdepth 1 -type f -user "$(id -un)" \
  -name 'fastrtps_*' -print -delete | wc -l)"
echo "[startup] removed ${removed} stale Fast DDS shared-memory file(s)"

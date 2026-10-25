#!/usr/bin/env bash
# Sync source from the laptop, preserving Pi build outputs and LiDAR settings.
set -euo pipefail

sync_host=${1:-thainguyen1233@192.168.100.205}
sync_remote_dir=${2:-/home/thainguyen1233/ros_ws/forklift}
sync_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! $sync_host =~ ^[a-zA-Z0-9_.@-]+$ || $sync_host == -* ||
      ! $sync_remote_dir =~ ^/[a-zA-Z0-9_./-]+$ || $sync_remote_dir == / ]]; then
    printf 'Invalid SSH host or remote workspace path.\n' >&2
    exit 2
fi
command -v rsync >/dev/null
sync_tmp=$(mktemp -d /tmp/forklift-pi-sync.XXXXXX)
sync_socket=$sync_tmp/ssh
sync_ssh=(ssh -o ControlMaster=auto -o ControlPersist=60 -S "$sync_socket")
cleanup() {
    ssh -S "$sync_socket" -O exit "$sync_host" >/dev/null 2>&1 || true
    rm -rf -- "$sync_tmp"
}
trap cleanup EXIT

# Run in a regular laptop terminal so SSH can request the password directly.
"${sync_ssh[@]}" "$sync_host" bash -s -- "$sync_remote_dir" <<'REMOTE'
set -e
cd -- "$1"
test -f src/forklift_hardware/package.xml
test -f src/forklift_bringup/package.xml
command -v rsync >/dev/null
if ! command -v fuser >/dev/null; then
    echo 'On Pi, install the auto-detection dependency first: sudo apt install psmisc' >&2
    exit 1
fi
printf 'Target: %s:%s (%s)\n' "$(hostname)" "$PWD" "$(uname -m)"
REMOTE

sync_backup="${sync_remote_dir}.sync-backup/$(date +%Y%m%d-%H%M%S)-$$"
printf 'Backup of replaced files on Pi: %s\n' "$sync_backup"
rsync -az --checksum --itemize-changes --backup --backup-dir="$sync_backup" \
    --filter=':- .gitignore' \
    --exclude='/.git/' --exclude='/.agents/' \
    --exclude='/results/' \
    --exclude='/src/forklift_hardware/config/ydlidar.yaml' \
    -e "ssh -S $sync_socket" "$sync_root/" "$sync_host:$sync_remote_dir/"

# Supply the LiDAR config only if the Pi has none; never replace its local port.
rsync -az --ignore-existing -e "ssh -S $sync_socket" \
    "$sync_root/src/forklift_hardware/config/ydlidar.yaml" \
    "$sync_host:$sync_remote_dir/src/forklift_hardware/config/"

"${sync_ssh[@]}" "$sync_host" bash -s -- "$sync_remote_dir" <<'REMOTE'
set -e
cd -- "$1"
source /opt/ros/humble/setup.bash
if test -f install/local_setup.bash; then
    source install/local_setup.bash
fi
colcon build --symlink-install --packages-select \
    forklift_interfaces forklift_control forklift_hardware forklift_bringup
source install/local_setup.bash
ros2 launch forklift_bringup real.launch.py --show-args
echo 'Sync and build complete. No ROS nodes were started and no firmware was flashed.'
REMOTE

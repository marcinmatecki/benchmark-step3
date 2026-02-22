#!/bin/bash

IMAGE_NAME='superodom_humble'
TMUX_SESSION='ros2_session'

DATASET_CONTAINER_PATH='/ros2_ws/dataset'
BAG_OUTPUT_CONTAINER='/ros2_ws/recordings'

RECORDED_BAG_NAME="recorded-superOdom"
HDMAPPING_OUT_NAME="output_hdmapping"

usage() {
  echo "Usage:"
  echo "  $0 <input_bag_dir> <output_dir>"
  echo
  echo "If no arguments are provided, a GUI file selector will be used."
  exit 1
}

echo "=== SuperOdom ROS2 rosbag pipeline ==="

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
fi

if [[ $# -eq 2 ]]; then
  DATASET_HOST_PATH="$1"
  BAG_OUTPUT_HOST="$2"
elif [[ $# -eq 0 ]]; then
  command -v zenity >/dev/null || {
    echo "Error: zenity is not available"
    exit 1
  }
  DATASET_HOST_PATH=$(zenity --file-selection --directory --title="Select BAG directory")
  BAG_OUTPUT_HOST=$(zenity --file-selection --directory --title="Select output directory")
else
  usage
fi

if [[ -z "$DATASET_HOST_PATH" || -z "$BAG_OUTPUT_HOST" ]]; then
  echo "Error: no file or directory selected"
  exit 1
fi

if [[ ! -d "$DATASET_HOST_PATH" ]]; then
  echo "Error: BAG directory does not exist: $DATASET_HOST_PATH"
  exit 1
fi

mkdir -p "$BAG_OUTPUT_HOST"

DATASET_HOST_PATH=$(realpath "$DATASET_HOST_PATH")
BAG_OUTPUT_HOST=$(realpath "$BAG_OUTPUT_HOST")

echo "Input bag dir : $DATASET_HOST_PATH"
echo "Output dir    : $BAG_OUTPUT_HOST"

xhost +local:docker >/dev/null

# Fix livox_ros_driver -> livox_ros_driver2 type name in bag (if needed)
python3 -c "
import sqlite3, pathlib, sys
bag = pathlib.Path(sys.argv[1])
for db in bag.glob('*.db3'):
    c = sqlite3.connect(str(db))
    c.execute(\"UPDATE topics SET type='livox_ros_driver2/msg/CustomMsg' WHERE type='livox_ros_driver/msg/CustomMsg'\")
    c.commit(); c.close()
m = bag / 'metadata.yaml'
if m.exists():
    t = m.read_text()
    if 'livox_ros_driver/msg/CustomMsg' in t:
        m.write_text(t.replace('livox_ros_driver/msg/CustomMsg', 'livox_ros_driver2/msg/CustomMsg'))
        print('Fixed bag type: livox_ros_driver -> livox_ros_driver2')
" "$DATASET_HOST_PATH" 2>/dev/null || true

docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c '

    tmux new-session -d -s '"$TMUX_SESSION"'

    # ---------- PANEL 1: ROS2 launch ----------
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 5
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch super_odometry livox_mid360.launch.py
'\'' C-m

    # ---------- PANEL 2: rosbag record ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

rm -rf '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"'
echo "[record] start"
ros2 bag record /registered_scan /state_estimation \
  -o '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"' --storage sqlite3

echo "[record] exit"
'\'' C-m

    # ---------- PANEL 3: rosbag play ----------
    tmux split-window -h -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 10
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "[play] start"
ros2 bag play '"$DATASET_CONTAINER_PATH"' --clock; tmux wait-for -S BAG_DONE;
echo "[play] done"
'\'' C-m

    # ---------- WINDOW 2: rviz2 ----------
    tmux new-window -t '"$TMUX_SESSION"' -n rviz '\''
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
sleep 8
rviz2 -d /ros2_ws/src/SuperOdom/super_odometry/ros2.rviz
'\''

    # ---------- WINDOW 3: controller ----------
    tmux new-window -t '"$TMUX_SESSION"' -n control '\''
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "[control] waiting for play to finish"
tmux wait-for BAG_DONE

echo "[control] stopping record"
tmux send-keys -t '"$TMUX_SESSION"'.1 C-c

sleep 5
pkill ros2

sleep 7
pkill -f ros2 || true
'\''

    tmux attach -t '"$TMUX_SESSION"'
  '

docker run -it --rm \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c "
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    ros2 run superOdom-to-hdmapping listener \
      \"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME\" \
      \"$BAG_OUTPUT_CONTAINER/$HDMAPPING_OUT_NAME-superOdom\"
  "

echo "=== DONE ==="

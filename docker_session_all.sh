#!/bin/bash

usage() {
  echo "Usage:"
  echo "  $0 <ros1_bag> <ros2_bag_dir> <output_dir>"
  echo
  echo "ros1_bag      : path to a ROS1 .bag file"
  echo "ros2_bag_dir  : path to a ROS2 bag directory"
  echo "output_dir    : directory to store outputs"
  exit 1
}

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    usage
fi

if [[ $# -ne 3 ]]; then
    usage
fi

ROS1_BAG=$(realpath "$1")
ROS2_BAG_DIR=$(realpath "$2")
OUTPUT_DIR=$(realpath "$3")

mkdir -p "$OUTPUT_DIR"

ROS1_ALGOS=(
  "super-lio"
  "dlio"
  "dlo"
  "fast-lio"
  "faster-lio"
  "ig-lio"
  "i2ekf-lo"
  "ct-icp"
  "loam"
  "slict"
  "lio-ekf"
  "lego-loam"
  "point-lio"
  "voxel-map"
)

for algo in "${ROS1_ALGOS[@]}"; do
    OUTPUT="$OUTPUT_DIR/$algo"
    mkdir -p "$OUTPUT"

    if [[ "$algo" == "dlio" || "$algo" == "dlo"  || "$algo" ==  "loam"  || "$algo" == "ct-icp" || "$algo" ==  "lego-loam" ]]; then
        INPUT="${ROS1_BAG}-pc"
    else
        INPUT="$ROS1_BAG"
    fi

    echo "=== Waiting 5 seconds before running $algo ==="
    sleep 5

    echo "=== Running $algo on $INPUT ==="
    ./docker_session_run-ros1-"$algo".sh "$INPUT" "$OUTPUT"
    echo "=== Finished $algo ==="
done

ROS2_ALGOS=(
  "superOdom"
  "kiss-icp"
  "genz-icp"
  "glim"
  "resple"
  "lidar_odometry_ros_wrapper"
)

for algo in "${ROS2_ALGOS[@]}"; do
    OUTPUT="$OUTPUT_DIR/$algo"
    mkdir -p "$OUTPUT"

    if [[ "$algo" == "resple" ]]; then
        INPUT="${ROS2_BAG_DIR}-lidar"
    else
        INPUT="$ROS2_BAG_DIR"
    fi

    echo "=== Waiting 5 seconds before running $algo ==="
    sleep 5

    echo "=== Running $algo on $INPUT ==="
    ./docker_session_run-ros2-"$algo".sh "$INPUT" "$OUTPUT"
    echo "=== Finished $algo ==="
done
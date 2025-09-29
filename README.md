# AATB Crowd Tracker ROS2 node

A dependency-free (no PCL) 2d lidar crowd tracker used in some of our installations for simple person tracking, including automatic background substraction (dynamic occupancy map creation and update with adjustable accumulation/decay rates). Lot of params to fiddle around for cluster size, threshold, etc.

## Features

- automatic background substraction, takes a couple minutes to warm up the occupancy map
- temporal tracking with `id` (sequential and incrementing), `entered` timestamp, `age` duration, `moving` flag, velocity and heading estimation

# Usage

clone this repo and [aatb_msgs](https://github.com/aatb-ch/aatb_msgs), build, launch. Tracks are transformed and published in `world` frame, check `crowd_tracker.yaml` in `/config` for params.

A PoseArray topic is published for debugging.

# Future desirable features

- make the tracker more robust to false positives/negatives
- handle multiple lidar sources to avoid obstacle shadows
- user-provided static_tf for primary lidar and automatic detection of secondary lidars' tf via ICP

# License

MIT, see LICENSE.

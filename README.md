# AATB Crowd Tracker ROS2 node

A dependency-free (no PCL) 2d lidar crowd tracker used in some of our installations for simple person tracking, including automatic background substraction (dynamic occupancy map creation and update with adjustable accumulation/decay rates). Lot of params to fiddle around for cluster size, threshold, etc.

# Usage

clone this repo and [aatb_msgs](https://github.com/aatb-ch/aatb_msgs), build, launch. Tracks are published in world frame, check crowd_tracker.yaml in /config for params.

# License

MIT, see LICENSE.

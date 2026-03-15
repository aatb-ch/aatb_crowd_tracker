# AATB Crowd Tracker ROS2 node

A dependency-free (no PCL) 2D lidar crowd tracker used in some of our installations for person detection and tracking.

## Features

**Scan-based tracker (classic pipeline)**
- Automatic background subtraction via dynamic polar occupancy map (EMA-based, adjustable accumulation/decay rates, takes a couple minutes to warm up)
- Cluster extraction with adaptive distance threshold (scales with range)
- Temporal tracking with GUID (`id`), `entered` timestamp, `age` duration, `moving` flag, velocity and heading estimation
- Configurable output frame — data transformed via TF2 (default `world`)
- Optional geofence filtering

**Heatmap detection layer (new)**
- Fixed-resolution XY grid accumulating raw scan hits per cell
- Per-cell slow/fast EMA: slow EMA acts as the implicit background, fast EMA reacts to changes
- Schmitt trigger activation with derivative gating: activates only on significant rising edges, deactivates only when fully settled — prevents chatter and streak artefacts
- Publishes two `OccupancyGrid` topics:
  - `/heatmap` — continuous intensity (`fast - slow` normalized to 0–100)
  - `/heatmap_active` — binary activation map (0 or 100)
- BFS blob detection on the active grid → GUID-tracked centroids on `/heatmap_tracks` / `/heatmap_tracks_posearray`
- Grid auto-sized from geofence bounds (if enabled) or `max_range` parameter at startup
- 10 Hz fallback decay when no scan is received (e.g. after lidar failure), preventing cells from staying active indefinitely

## Topics

| Topic | Type | Description |
|---|---|---|
| `/tracks` | `aatb_msgs/Tracks` | Scan-based tracked persons |
| `/tracks_posearray` | `geometry_msgs/PoseArray` | Same, for RViz2 |
| `/heatmap` | `nav_msgs/OccupancyGrid` | fast−slow intensity (0–100) |
| `/heatmap_active` | `nav_msgs/OccupancyGrid` | Binary Schmitt activation |
| `/heatmap_tracks` | `aatb_msgs/Tracks` | Blob-tracked persons (no velocity) |
| `/heatmap_tracks_posearray` | `geometry_msgs/PoseArray` | Same, for RViz2 |

## Usage

Clone this repo and [aatb_msgs](https://github.com/aatb-ch/aatb_msgs), build, launch:

```bash
ros2 launch aatb_crowd_tracker crowd_tracker.launch.py
```

All parameters are in `config/crowd_tracker.yaml`. Key ones:

| Parameter | Default | Description |
|---|---|---|
| `output_frame_id` | `world` | All output data is transformed to this frame |
| `background_model.enable` | `true` | Toggle background subtraction |
| `heatmap.enable` | `true` | Toggle heatmap layer |
| `heatmap.resolution` | `0.2` | Grid cell size in meters |
| `heatmap.max_range` | `15.0` | Grid half-size when geofence is off (meters) |
| `heatmap.fast_decay_rate` | `0.10` | How quickly activation fades (~1.5 s at 10 Hz) |
| `heatmap.schmitt_high` | `0.10` | Activation threshold |
| `geofence.enable` | `false` | Restrict output to a spatial bounding box |

## Performance (Raspberry Pi 4)

At default settings (0.2 m resolution, 15 m range → 150×150 = 22 500 cells, 10 Hz): heatmap update + grid publish adds < 1 ms per scan. A warning is logged at startup if cell count exceeds 500K.

# Future desirable features

- Make the tracker more robust to false positives/negatives
- Handle multiple lidar sources to avoid obstacle shadows
- User-provided static TF for primary lidar and automatic detection of secondary lidars' TF via ICP

# License

MIT, see LICENSE.

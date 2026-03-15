# pointcloud_to_laserscan

Small ROS 2 point cloud to `LaserScan` converter written to work cleanly on macOS.

This repository exists because the usual `pointcloud_to_laserscan` options were unreliable on a MacBook setup. The implementation here is intentionally simple: one Python node, no custom message types, and only standard ROS 2 Python APIs plus `numpy`.

## What It Does

The node subscribes to a `sensor_msgs/msg/PointCloud2` topic, projects the 3D cloud into a 2D planar scan, and publishes a `sensor_msgs/msg/LaserScan`.

Default topics:

- Input: `/cloud_in`
- Output: `/scan`

Default behavior:

- Keeps only points whose `z` value is between `min_height` and `max_height`
- Computes each point's planar distance with `sqrt(x^2 + y^2)`
- Computes each point's angle with `atan2(y, x)`
- Assigns the point to an angular beam
- Stores the closest point seen for each beam
- Publishes the final beam ranges as a `LaserScan`

This is effectively a 3D-to-2D projection for lidar-like consumers that expect scan data instead of a point cloud.

## How It Works

The conversion pipeline in this file is:

1. Read `x`, `y`, `z` points from the incoming `PointCloud2`.
2. Drop NaNs.
3. Filter points outside the configured height window.
4. Filter points outside the configured range window.
5. Convert each remaining point to an angle.
6. Map that angle into a beam index using `angle_min`, `angle_max`, and `angle_increment`.
7. Keep the minimum range seen for each beam.
8. Publish a `LaserScan` message using the input header.

Important implementation details:

- Empty beams are filled with `range_max`.
- The output scan keeps the original point cloud header.
- The node logs periodic status messages when it is actively converting points.
- If a cloud contains points but none survive filtering, the node publishes a scan filled with `range_max` values and logs a warning.

## Parameters

The node declares these ROS 2 parameters:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `min_height` | `-0.5` | Minimum accepted `z` value |
| `max_height` | `0.5` | Maximum accepted `z` value |
| `angle_min` | `-pi` | Start angle of the scan |
| `angle_max` | `pi` | End angle of the scan |
| `angle_increment` | `pi / 180.0` | Angular resolution, 1 degree by default |
| `scan_time` | `0.1` | Reported scan duration |
| `range_min` | `0.1` | Minimum accepted planar range |
| `range_max` | `30.0` | Maximum accepted planar range |

With the defaults, the node produces a 360 degree scan with 1 degree beam spacing.

## Requirements

You need:

- ROS 2 with Python support (`rclpy`)
- `sensor_msgs`
- `sensor_msgs_py`
- `numpy`

This script is especially useful if your ROS 2 setup is on macOS/another little used platform for robotics and the usual pointcloud-to-laserscan packages are not behaving correctly.

## Running It

If your ROS 2 environment is already sourced, you can run the node directly:

```bash
python3 pointcloud_to_laserscan.py
```

If you want to override parameters at launch time:

```bash
python3 pointcloud_to_laserscan.py --ros-args \
  -p min_height:=-0.2 \
  -p max_height:=0.3 \
  -p range_max:=20.0 \
  -p angle_increment:=0.0174533
```

## Example Data Flow

Typical usage looks like this:

1. A depth camera, lidar, or simulator publishes `PointCloud2` on `/cloud_in`.
2. This node filters and projects the cloud into a 2D scan.
3. Navigation, mapping, or visualization tools subscribe to `/scan`.

## Limitations

This implementation is intentionally minimal.

- Topic names are fixed in code to `/cloud_in` and `/scan`
- There is no TF-based transform step before projection
- The script converts the full point cloud in Python, so very large clouds may be slower than a native C++ implementation
- Empty beams are set to `range_max` rather than `inf`

## Why This Exists

Some standard Linux-first ROS packages are difficult to build or unreliable on macOS, a plain Python node can be easier to inspect, patch, and run. This repository keeps the converter small enough that the user can understand the full behavior in one file and adjust it for their own sensor setup.

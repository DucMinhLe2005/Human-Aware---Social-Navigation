# social_nav_perception

Multi-person 3D human detection from an RGB-D camera: YOLO pose keypoints are
back-projected to 3D with the depth image and published as
`thesis_msgs/MultiplePoseResult3D` in the `odom` frame. Optimised for CPU-only
computers (ONNX Runtime, throttled inference, reduced input size).

## Python dependencies

Not available through rosdep; install with pip on the system Python of ROS 2 Jazzy:

```bash
pip3 install --break-system-packages -r requirements-sim.txt   # from the workspace root
```

numpy is pinned below 2: `scipy.optimize.linear_sum_assignment` (used by the tracker) fails
to import with numpy 2.x on Ubuntu 24.04.

## Model

`weights/yolo26n-pose.onnx`, exported at 256x320 with `tools/export_onnx.py`.

## Topics

- Subscribes: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/color/camera_info`
- Publishes: `/perception/yolov8_multiple_pose_results_3d_ori` (`thesis_msgs/MultiplePoseResult3D`),
  debug markers and an optional debug image.

## Main parameters (`yolo_pose_node1`)

| Parameter | Default | Meaning |
|---|---|---|
| `model_path` | `""` | model file; empty = the ONNX model installed in `share/weights` |
| `detection_period_sec` | 0.15 | minimum time between two inferences |
| `inference_resize` | [320, 240] | resize the RGB image before inference; keypoints are scaled back before back-projection |
| `onnx_num_threads` | 2 | ONNX Runtime threads per inference |
| `depth_patch_radius` | 2 | median depth over a (2r+1)^2 patch around each keypoint |
| `depth_consistency_m` | 0.5 | drop keypoints farther than this from the person's median depth |
| `camera_transform_max_stale_seconds` | 0.15 | reuse of the last good camera transform |
| `publish_debug_image` | false | publish the image with drawn skeletons |

The depth image is converted to metres explicitly: Gazebo publishes `32FC1` in
metres, the RealSense D435 publishes `16UC1` in millimetres.

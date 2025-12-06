# How to Build and Run the ROS2 Nodes

These are the exact commands required to build and run the ROS2 perception system nodes
(camera publisher, YOLO detection, and AprilTag distance node).

---

## 🛠️ 1. Build the ROS2 Workspace

Open a terminal:

```bash
cd ~/GUnitFinalProject/ros2_ws
colcon build


source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Terminal Setup (Run this in every new terminal)
```bash
source /opt/ros/jazzy/setup.bash
cd ~/GUnitFinalProject/ros2_ws
source install/setup.bash
```

If doing enemies vs vehicles

1. Terminal 1
```bash
ros2 run rpi_camera camera_publisher
```

4. Terminal 2
```bash
ros2 run rpi_camera yolov8_node \
  --ros-args -p model_path:={CHANGE PATH}
```

5. Terminal 3
```bash
ros2 topic echo /yolo/detections
ros2 topic echo /camera/image_yolo
```

If doing AprilTags

1. Terminal 1
```bash
ros2 run rpi_camera camera_publisher
```

2. Terminal 2
```bash
ros2 run rpi_camera apriltag_node \
  --ros-args -p tag_size:=0.05
```

3. Terminal 3
```bash
ros2 topic echo /yolo/detections
ros2 topic echo /apriltag/distance
ros2 topic echo /apriltag/info
```

To see a livestream of the video
```bash
ros2 run web_video_server web_video_server
```
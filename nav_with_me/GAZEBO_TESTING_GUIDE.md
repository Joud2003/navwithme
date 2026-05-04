# Gazebo Testing Guide - Harmful Object Detection

This guide explains how to test the harmful object detection system in Gazebo simulation.

## What Changed

The robot now:
1. **Filters images** - Only keeps images containing harmful objects
2. **Separates outputs**:
   - `robot_data/images/` - All captured images
   - `robot_data/harmful_images/` - Only images with harmful objects
   - `robot_data/annotations/dataset_annotations.json` - Metadata with `has_harmful_objects` flag

## Harmful Objects Detected

The system currently recognizes these dangerous/harmful objects:
```
person, dog, cat, knife, gun, scissors, axe, bottle, cup, fire, smoke
```

Edit `HARMFUL_OBJECTS` in [robot_navigation.py](nav_with_me/robot_navigation.py#L27) to customize.

---

## Quick Start: Test in Gazebo

### 1. Install Dependencies

```bash
# Install YOLO and CV Bridge
pip install ultralytics opencv-python cv-bridge

# Install Gazebo (if not already installed)
sudo apt-get install gazebo
```

### 2. Launch Gazebo with Turtlebot3

```bash
# Terminal 1: Start Gazebo with a simple world
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Or create a custom world with harmful objects
gazebo --verbose my_world.world
```

### 3. Add Harmful Objects to Gazebo

Create a world file `my_world.world`:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Add a knife (red box) -->
    <model name="knife">
      <pose>3 0 0.5 0 0 0</pose>
      <link name="blade">
        <visual>
          <geometry>
            <box>
              <size>0.1 0.05 0.3</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Orange</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### 4. Run Your Robot Navigation

```bash
# Terminal 2: Source and run
cd ~/ros2_ws
source install/setup.bash
ros2 run nav_with_me robot_navigation
```

### 5. Navigate Robot Near Objects

- Use controller commands to navigate the robot
- Robot will capture images when objects are detected via LIDAR
- YOLO processes images in real-time (background thread)
- Harmful objects logged to terminal: `⚠️ HARMFUL OBJECTS DETECTED: [...]`

### 6. Press Ctrl+C to Stop

Results will be saved:
```
robot_data/
├── images/                    # All images
│   ├── image_0000.png
│   ├── image_0001.png
│   └── ...
├── harmful_images/            # Only harmful detections
│   ├── harmful_0000.png
│   ├── harmful_0001.png
│   └── ...
├── annotations/
│   └── dataset_annotations.json
├── trajectories.csv
└── occupancy_grid_map.npz
```

---

## Analyze Results

### View Dataset Annotations

```python
import json

with open("robot_data/annotations/dataset_annotations.json") as f:
    annotations = json.load(f)

# Show statistics
harmful_count = sum(1 for ann in annotations if ann["has_harmful_objects"])
print(f"Total images: {len(annotations)}")
print(f"Harmful images: {harmful_count}")

# Show details of harmful detections
for ann in annotations:
    if ann["has_harmful_objects"]:
        print(f"\n{ann['filename']}:")
        for det in ann["harmful_detections"]:
            print(f"  - {det['class_name']}: {det['confidence']:.2f}")
        print(f"  Position: x={ann['ground_truth_pose']['x']:.2f}, y={ann['ground_truth_pose']['y']:.2f}")
```

### Visualize Harmful Detections

```python
import cv2
import glob
from PIL import Image

# View all harmful images
harmful_images = glob.glob("robot_data/harmful_images/*.png")
for img_path in harmful_images:
    img = Image.open(img_path)
    img.show()
```

---

## Advanced Testing Scenarios

### Test 1: Accuracy Check
1. Place known harmful objects in Gazebo
2. Run robot multiple times
3. Check detection rate in `dataset_annotations.json`
4. Compare `ground_truth_pose` vs detected bounding boxes

### Test 2: False Positive Filtering
1. Add benign objects (chairs, tables)
2. Verify they're NOT in `harmful_images/`
3. Confirm they appear in `images/` with `has_harmful_objects: false`

### Test 3: Confidence Threshold
Edit `CONFIDENCE_THRESHOLD` in code (default: 0.5):
```python
CONFIDENCE_THRESHOLD = 0.7  # Stricter filtering
```

### Test 4: Custom Harmful Objects
Edit the list at the top of `robot_navigation.py`:
```python
HARMFUL_OBJECTS = {
    "person", "dog", "knife", "gun",
    "your_custom_object",  # Add here
}
```

---

## Troubleshooting

### No Images Captured
- Check if LIDAR is detecting objects: `ros2 topic echo /scan`
- Check if camera is working: `ros2 topic echo /camera/image_raw`
- Verify `object_is_detected` is True in logs

### YOLO Not Running
```bash
pip install ultralytics --upgrade
# First run downloads model (~100MB)
```

### Low Detection Accuracy
- Move robot closer to objects
- Increase `CONFIDENCE_THRESHOLD` (default: 0.5)
- Use faster-but-less-accurate YOLO: `yolov8n.pt` (current)
- Or more accurate: `yolov8m.pt` or `yolov8l.pt` (slower)

### Harmful Images Folder Empty
- Check `dataset_annotations.json` for `"has_harmful_objects": true`
- Verify object names match `HARMFUL_OBJECTS` set
- Check YOLO detection confidence > 50%

---

## Performance Tips

| Component | Default | For Speed | For Accuracy |
|-----------|---------|-----------|--------------|
| YOLO Model | yolov8n.pt | yolov8n.pt | yolov8l.pt |
| Confidence | 0.5 | 0.7 | 0.3 |
| Image Queue | 5 images | 3 images | 10 images |
| YOLO Threads | 1 | 1 | 4 |

To use multiple threads for YOLO (modify `_yolo_worker`):
```python
# Run inference with multiple threads
results = self.yolo_model(image, verbose=False, workers=4)
```

---

## Next Steps

1. **Train custom YOLO model** on your specific harmful objects
2. **Integrate with motion controller** to stop robot when harmful object detected
3. **Add confidence scoring** to alert system
4. **Real robot testing** using actual TurtleBot3 camera

For more info: [Ultralytics YOLOv8 Docs](https://docs.ultralytics.com/)

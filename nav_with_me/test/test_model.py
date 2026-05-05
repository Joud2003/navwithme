import os

from ultralytics import YOLO

model = YOLO("yolov9c.pt")
image_folder = "/home/joud-ros2/ros2_ws/src/navwithme/nav_with_me/test/test_images/"
image_list = [
    f"{image_folder}{f}" for f in os.listdir(image_folder) if f.endswith(".png")
]
for im in image_list:
    print(im)
    results = model(im, verbose=False)
    print(len(results[0].boxes))
    for box in results[0].boxes:
        print(model.names[int(box.cls)], float(box.conf), box.xyxy)

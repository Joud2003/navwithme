from .img_object_detection import ImageProcessor
from .object_detection import ObjectDetection


def test_no_matches():
    """Test when no objects match - all should be added"""
    robot = type("Robot", (), {})()
    robot.estimated_pose = (1.0, 2.0, 0.5)

    object_detector = ObjectDetection(robot)

    # Old detections far away
    old_objects = [{"class": "knife", "pose": [0.0, 0.0, 0.0], "conf": 0.8, "count": 1}]
    # New detections in different location
    new_objects = [{"class": "knife", "pose": [2.0, 3.0, 0.5], "conf": 0.9}]

    filtered = object_detector.filter_objects(old_objects, new_objects)
    assert len(filtered) == 2  # Old + new (no match)
    assert filtered[0]["pose"] == [0.0, 0.0, 0.0]
    assert filtered[1]["pose"] == [2.0, 3.0, 0.5]
    print("✓ Test no matches passed")


def test_all_matches():
    """Test when all objects match - should update existing"""
    robot = type("Robot", (), {})()
    robot.estimated_pose = (1.0, 2.0, 0.5)

    object_detector = ObjectDetection(robot)

    # Same object detected twice close together
    old_objects = [{"class": "knife", "pose": [1.0, 2.0, 0.5], "conf": 0.8, "count": 1}]
    new_objects = [
        {"class": "knife", "pose": [1.1, 2.1, 0.6], "conf": 0.9},
    ]

    filtered = object_detector.filter_objects(old_objects, new_objects)
    assert len(filtered) == 1
    # Pose should be averaged
    expected_pose = [(1.0 + 1.1) / 2, (2.0 + 2.1) / 2, (0.5 + 0.6) / 2]

    assert filtered[0]["pose"] == expected_pose
    assert filtered[0]["conf"] == 0.9  # Max confidence
    assert filtered[0]["count"] == 2
    print("✓ Test all matches passed")


def test_partial_matches():
    """Test mix of matches and new objects"""
    robot = type("Robot", (), {})()
    robot.estimated_pose = (1.0, 2.0, 0.5)

    object_detector = ObjectDetection(robot)

    old_objects = [{"class": "knife", "pose": [1.0, 2.0, 0.0], "conf": 0.7, "count": 1}]
    new_objects = [
        {"class": "knife", "pose": [1.05, 2.05, 0.1], "conf": 0.8},  # Match
        {"class": "fork", "pose": [2.0, 3.0, 0.0], "conf": 0.9},  # New
    ]

    filtered = object_detector.filter_objects(old_objects, new_objects)
    assert len(filtered) == 2
    print("✓ Test partial matches passed")


def test_different_classes():
    """Test objects of different classes don't match even if close"""
    robot = type("Robot", (), {})()
    robot.estimated_pose = (1.0, 2.0, 0.5)

    object_detector = ObjectDetection(robot)

    old_objects = [{"class": "knife", "pose": [1.0, 2.0, 0.0], "conf": 0.8, "count": 1}]
    new_objects = [
        {
            "class": "fork",
            "pose": [1.01, 2.01, 0.0],
            "conf": 0.9,
        }  # Same pose, different class
    ]

    filtered = object_detector.filter_objects(old_objects, new_objects)
    assert len(filtered) == 2  # Old + new (different classes, no match)
    print("✓ Test different classes passed")


# Run new tests
test_no_matches()
test_all_matches()
test_partial_matches()
test_different_classes()

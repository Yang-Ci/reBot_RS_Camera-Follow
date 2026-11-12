"""Reject fixed gripper detections in infrared frames."""


def in_ir_gripper_region(u, v, width, height):
    """Conservative exclusion for this camera's fixed visible gripper."""
    if u is None or v is None or width <= 0 or height <= 0:
        return False
    return .37 * width <= u <= .90 * width and .50 * height <= v < height


def ir_gripper_detection(u, v, landmarks, width, height):
    if in_ir_gripper_region(u, v, width, height):
        return True
    # A detection centred outside the region may still be anchored on the
    # gripper. Reject it if the wrist or palm bases fall inside the region.
    if landmarks is not None and len(landmarks) >= 18:
        return any(in_ir_gripper_region(landmarks[i].x * width,
                                        landmarks[i].y * height, width, height)
                   for i in (0, 5, 9, 13, 17))
    return False

# src/yolo_realsense/utils.py
import pyrealsense2 as rs
import numpy as np
import cv2
import time

class RealSenseDirect:
    def __init__(self, width=640, height=480, fps=30):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.profile = self.pipeline.start(self.config)
        time.sleep(0.5)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        if not frames:
            return None, None
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            return None, None
        color_image = np.asanyarray(color.get_data())
        depth_image = np.asanyarray(depth.get_data())
        return color_image, depth_image

    def stop(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass

def cv2_to_msg(frame, bridge):
    return bridge.cv2_to_imgmsg(frame, encoding='bgr8')
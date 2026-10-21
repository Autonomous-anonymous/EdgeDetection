#!/usr/bin/env python3
import threading
import time

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node


CAMERA_TOPIC = "/aeb360/camera/image"
TIMEOUT_SECONDS = 8.0


class CameraCheck:
    def __init__(self):
        self.node = Node()
        self.event = threading.Event()
        self.frames = 0
        self.width = None
        self.height = None
        self.step = None
        self.pixel_format = None

        if not self.node.subscribe(
            Image,
            CAMERA_TOPIC,
            self.callback,
        ):
            raise RuntimeError(
                f"Failed to subscribe to {CAMERA_TOPIC}"
            )

    def callback(self, msg):
        self.frames += 1
        self.width = msg.width
        self.height = msg.height
        self.step = msg.step
        self.pixel_format = msg.pixel_format_type
        self.event.set()

    def run(self):
        print(f"Waiting for camera frames on {CAMERA_TOPIC} ...")

        if not self.event.wait(TIMEOUT_SECONDS):
            print("CAMERA RESULT: FAIL")
            print("No camera frame received before timeout.")
            return False

        start = time.monotonic()
        first_count = self.frames
        time.sleep(1.0)
        frames_in_one_second = self.frames - first_count

        print(f"Frame size     : {self.width} x {self.height}")
        print(f"Row step       : {self.step}")
        print(f"Pixel format   : {self.pixel_format}")
        print(f"Frames / ~1 s  : {frames_in_one_second}")
        print("CAMERA RESULT: PASS")
        return True


def main():
    check = CameraCheck()
    raise SystemExit(0 if check.run() else 1)


if __name__ == "__main__":
    main()

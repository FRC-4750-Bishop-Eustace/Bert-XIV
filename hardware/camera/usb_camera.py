# MIT License
#
# Copyright (c) 2025 Beʳᵗ FRC Team 4750
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from ..camera_base import Camera
from typing import Any

class USBCamera(Camera):
    backendName: str = "WPI_USBCamera"

    def __init__(self, deviceId: int = 0, resolution: tuple = (640, 480), fps: int = 30) -> None:
        super().__init__()
        self.deviceId = deviceId
        self.resolution = resolution
        self.fps = fps
        self.running = False

        try:
            import cv2
            self.cv2 = cv2
            self.cap = None
        except Exception:
            self.cv2 = None
            self.cap = None

    def Start(self) -> None:
        if self.cv2 is not None:
            self.cap = self.cv2.VideoCapture(self.deviceId)
        self.running = True

    def Stop(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.running = False

    def CaptureFrame(self) -> Any:
        if not self.running:
            return None
        if self.cap is not None:
            ret, frame = self.cap.read()
            return frame
        return None

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

from time import time
from pathlib import Path
from requests import RequestException, get
from subsystems import *
from commands2 import InstantCommand

class CaptureSnapshot(InstantCommand):
    def __init__(self, vision: Vision) -> None:
        super().__init__(
            lambda: [self.getScreenshots(vision)],
            vision
        )

    def getScreenshots(self, vision: Vision) -> None:
        for camera in vision.cameras:
            url = f"http://{camera.name}.local:5800/snapshot.jpg"
            try:
                response = get(url, timeout=0.2)
                if response.status_code == 200 and response.content is not None:
                    with open(f"./deploy/snapshots/snapshot{camera.name}-{int(time())}.jpg", "wb") as f:
                        f.write(response.content)
            except RequestException:
                pass

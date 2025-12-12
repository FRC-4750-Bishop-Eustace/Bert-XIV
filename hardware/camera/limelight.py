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

class Limelight(Camera):
    backendName: str = "Hailo_Limelight"

    def __init__(self, tableName: str = "limelight") -> None:
        super().__init__()
        self.tableName = tableName

        try:
            import ntcore
            self.nt = ntcore.NetworkTables.getTable(self.tableName)
        except Exception:
            self.nt = None

    def Start(self) -> None:
        pass

    def Stop(self) -> None:
        pass

    def CaptureFrame(self) -> Any:
        if self.nt is not None:
            return dict(tx=self.nt.getNumber("tx", 0.0), ty=self.nt.getNumber("ty", 0.0), ta=self.nt.getNumber("ta", 0.0))
        return None

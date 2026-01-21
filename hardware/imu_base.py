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

from abc import abstractmethod
from typing import Any, Dict
from .device import Device
from wpimath.geometry import Translation3d, Rotation3d, Pose3d

class IMU(Device):
    deviceType: str = "imu"

    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def GetPosition(self) -> Translation3d:
        raise NotImplementedError

    @abstractmethod
    def GetRotation(self) -> Rotation3d:
        raise NotImplementedError

    @abstractmethod
    def GetAcceleration(self) -> Translation3d:
        raise NotImplementedError

    @abstractmethod
    def GetRate(self) -> float:
        raise NotImplementedError

    @abstractmethod
    def Reset(self, pose: Pose3d = Pose3d()) -> None:
        raise NotImplementedError

    @classmethod
    def Create(cls, vendor: str, model: str, **kwargs) -> "IMU":
        backendName = f"{vendor}_{model}".replace(" ", "")
        return Device.Create("imu", backendName, **kwargs)

    @classmethod
    def CreateFromConfig(cls, config: Dict[str, Any]) -> "IMU":
        cfg = dict(config)
        cfg["device_type"] = cfg.get("device_type", "imu")
        return Device.CreateFromConfig(cfg)

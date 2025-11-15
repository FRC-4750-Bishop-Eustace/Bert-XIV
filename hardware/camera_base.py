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

class Camera(Device):
    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def Start(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def Stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def CaptureFrame(self) -> Any:
        raise NotImplementedError

    @classmethod
    def Create(cls, vendor: str, typ: str, **kwargs) -> "Camera":
        backendName = f"{vendor}_{typ}".replace(" ", "").lower()
        return cls.Create("camera", backendName, **kwargs)

    @classmethod
    def CreateFromConfig(cls, config: Dict[str, Any]) -> "Camera":
        cfg = dict(config)
        cfg["device_type"] = cfg.get("device_type", "camera")
        return cls.Create(cfg)

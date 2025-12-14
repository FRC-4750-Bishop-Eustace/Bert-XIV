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

from typing import Type
from .enums import *
from .device import Device
from .motor_base import Motor
from .encoder_base import Encoder
from .imu_base import IMU
from .camera_base import Camera
from .solenoid_base import Solenoid
from .motor import *
from .encoder import *
from .imu import *
from .camera import *
from .solenoid import *

class Loader:
    def __init__(self, load: bool = True) -> None:
        self.LoadAll() if load else self.UnloadAll()

    def Load(self, deviceType: str, backendName: str, backendClass: Type[Device]) -> None:
        Device.RegisterBackend(deviceType, backendName, backendClass)

    def LoadAll(self) -> None:
        import inspect
        import sys

        backendModules = [
            sys.modules[__name__.rsplit(".", 1)[0] + ".motor"],
            sys.modules[__name__.rsplit(".", 1)[0] + ".encoder"],
            sys.modules[__name__.rsplit(".", 1)[0] + ".imu"],
            sys.modules[__name__.rsplit(".", 1)[0] + ".camera"],
            sys.modules[__name__.rsplit(".", 1)[0] + ".solenoid"],
        ]

        for module in backendModules:
            for name, cls in inspect.getmembers(module, inspect.isclass):
                if not issubclass(cls, Device):
                    continue
                if cls is Device:
                    continue

                if hasattr(cls, "deviceType") and hasattr(cls, "backendName"):
                    self.Load(cls.deviceType, cls.backendName, cls)

    def Unload(self, backendName: str) -> None:
        toDelete = [
            key for key in Device.registry
            if key[1] == backendName
        ]

    def UnloadAll(self) -> None:
        Device.registry.clear()

    def Reload(self) -> None:
        self.UnloadAll()
        self.LoadAll()

    def Create(self, deviceType: DeviceType, backendName: Enum, **kwargs) -> Device:
        return Device.Create(deviceType.value, backendName.value, **kwargs)

    def CreateMotor(self, name: MotorType, **kwargs) -> Motor:
        return self.Create(DeviceType.kMotor, name, **kwargs)

    def CreateEncoder(self, name: EncoderType, **kwargs) -> Encoder:
        return self.Create(DeviceType.kEncoder, name, **kwargs)

    def CreateIMU(self, name: IMUType, **kwargs) -> IMU:
        return self.Create(DeviceType.kIMU, name, **kwargs)

    def CreateCamera(self, name: CameraType, **kwargs) -> Camera:
        return self.Create(DeviceType.kCamera, name, **kwargs)

    def CreateSolenoid(self, name: SolenoidType, **kwargs) -> Solenoid:
        return self.Create(DeviceType.kSolenoid, name, **kwargs)

    def GetBackends(self) -> str:
        if len(Device.registry) == 0:
            return "\033[1;31mNo backends registered.\033[0m"

        str = "\033[1;32mBackends:\n"
        for (typ, backend), cls in Device.registry.items():
            str += f"  {typ} -> {backend}: <{cls.__name__}>\n"
        str += "\033[0m"
        return str

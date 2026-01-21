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

from ..imu_base import IMU
from wpimath.geometry import Translation3d, Rotation3d, Pose3d

class Pigeon2(IMU):
    backendName: str = "CTRE_Pigeon2"

    def __init__(self, deviceId: int, canbus: str) -> None:
        super().__init__()
        self.deviceId = deviceId
        self.canbus = canbus

        try:
            from phoenix6 import hardware

            self.hw = hardware.Pigeon2(self.deviceId, self.canbus)

        except Exception:
            class Dummy:
                class Value:
                    def __init__(self, value: float = 0) -> None:
                        self.value: float = value

                def __init__(self, id: int, bus: str) -> None:
                    self.id = id
                    self.bus = bus
                    self.pose = Pose3d()
                    self.accel = Translation3d()
                    self.rate = 0.0

                def get_accum_gyro_x(self) -> "Value":
                    return self.Value(self.pose.translation.x)

                def get_accum_gyro_y(self) -> "Value":
                    return self.Value(self.pose.translation.y)

                def get_accum_gyro_z(self) -> "Value":
                    return self.Value(self.pose.translation.z)

                def getRotation3d(self) -> Rotation3d:
                    return self.pose.rotation()

                def get_acceleration_x(self) -> "Value":
                    return self.Value(self.accel.x)

                def get_acceleration_y(self) -> "Value":
                    return self.Value(self.accel.y)

                def get_acceleration_z(self) -> "Value":
                    return self.Value(self.accel.z)

                def get_angular_velocity_z_device(self) -> "Value":
                    return self.Value(self.rate)

                def reset(self) -> None:
                    pass

            self.hw = Dummy(self.deviceId, self.canbus)

    def GetPosition(self) -> Translation3d:
        try:
            return Translation3d(self.hw.get_accum_gyro_x().value, self.hw.get_accum_gyro_y().value, self.hw.get_accum_gyro_z().value)
        except Exception:
            return Translation3d(self.hw.get_accum_gyro_x().value, self.hw.get_accum_gyro_y().value, self.hw.get_accum_gyro_z().value)

    def GetRotation(self) -> Rotation3d:
        try:
            return self.hw.getRotation3d()
        except Exception:
            return self.hw.getRotation3d()

    def GetAcceleration(self) -> Translation3d:
        try:
            return Translation3d(self.hw.get_acceleration_x().value, self.hw.get_acceleration_y().value, self.hw.get_acceleration_z().value)
        except Exception:
            return Translation3d(self.hw.get_acceleration_x().value, self.hw.get_acceleration_y().value, self.hw.get_acceleration_z().value)

    def GetRate(self) -> float:
        try:
            return float(self.hw.get_angular_velocity_z_device().value)
        except Exception:
            return self.hw.get_angular_velocity_z_device().value

    def Reset(self, pose: Pose3d = Pose3d()) -> None:
        try:
            self.hw.reset()
        except Exception:
            self.hw.reset()

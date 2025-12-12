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

class NavX(IMU):
    backendName: str = "KauaiLabs_NavX"

    def __init__(self) -> None:
        super().__init__()

        try:
            import navx

            self.hw = navx.AHRS.create_spi()
            self.hw.reset()

        except Exception:
            class Dummy:
                def __init__(self) -> None:
                    self.pose = Pose3d()

                def GetPosition(self) -> Translation3d:
                    return self.pose.translation()

                def GetRotation(self) -> Rotation3d:
                    return self.pose.rotation()

                def GetAcceleration(self) -> Translation3d:
                    return self.Translation3d()

                def Reset(self, pose: Pose3d = Pose3d()) -> None:
                    self.pose = pose

            self.hw = Dummy()

    def GetPosition(self) -> Translation3d:
        try:
            return Translation3d(self.hw.getDisplacementX(), self.hw.getDisplacementY(), self.hw.getDisplacementZ())
        except Exception:
            return self.hw.GetPosition()

    def GetRotation(self) -> Rotation3d:
        try:
            return Rotation3d(self.hw.getRoll(), self.hw.getYaw(), self.hw.getPitch())
        except Exception:
            return self.hw.GetRotation()

    def GetAcceleration(self) -> Translation3d:
        try:
            return Translation3d(self.hw.getRawAccelX(), self.hw.getRawAccelY(), self.hw.getRawAccelZ())
        except Exception:
            return self.hw.GetAcceleration()

    def Reset(self, pose: Pose3d = Pose3d()) -> None:
        try:
            self.hw.reset()
        except Exception:
            self.hw.Reset(pose)

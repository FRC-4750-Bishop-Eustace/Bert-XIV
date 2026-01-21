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

class ADIS16470(IMU):
    backendName: str = "WPI_ADIS16470"

    def __init__(self) -> None:
        super().__init__()

        try:
            import wpilib
            self.IMUAxis = wpilib.ADIS16470_IMU.IMUAxis

            self.hw = wpilib.ADIS16470_IMU()
            self.hw.calibrate()

        except Exception:
            from enum import Enum

            class Dummy:
                class IMUAxis(Enum):
                    kX = 0
                    kY = 1
                    kZ = 2
                    kYaw = 3
                    kPitch = 4
                    kRoll = 5

                def __init__(self) -> None:
                    self.pose = Pose3d()
                    self.accel = Translation3d()
                    self.rate = Pose3d()

                def getAngle(self, axis: IMUAxis) -> float:
                    match axis:
                        case self.IMUAxis.kX:
                            return self.pose.translation.x
                        case self.IMUAxis.kY:
                            return self.pose.translation.y
                        case self.IMUAxis.kZ:
                            return self.pose.translation.z
                        case self.IMUAxis.kYaw:
                            return self.pose.rotation.yaw
                        case self.IMUAxis.kPitch:
                            return self.pose.rotation.pitch
                        case self.IMUAxis.kRoll:
                            return self.pose.rotation.pitch

                def getAccelX(self) -> float:
                    return self.accel.x

                def getAccelY(self) -> float:
                    return self.accel.y

                def getAccelZ(self) -> float:
                    return self.accel.z

                def getRate(self, axis: IMUAxis) -> float:
                    match axis:
                        case self.IMUAxis.kX:
                            return self.rate.translation.x
                        case self.IMUAxis.kY:
                            return self.rate.translation.y
                        case self.IMUAxis.kZ:
                            return self.rate.translation.z
                        case self.IMUAxis.kYaw:
                            return self.rate.rotation.yaw
                        case self.IMUAxis.kPitch:
                            return self.rate.rotation.pitch
                        case self.IMUAxis.kRoll:
                            return self.rate.rotation.pitch

                def reset(self) -> None:
                    self.pose = Pose3d()

                def setGyroAngle(self, axis: IMUAxis, angle: float) -> None:
                    match axis:
                        case self.IMUAxis.kX:
                            self.pose.translation.x = angle
                        case self.IMUAxis.kY:
                            self.pose.translation.y = angle
                        case self.IMUAxis.kZ:
                            self.pose.translation.z = angle
                        case self.IMUAxis.kYaw:
                            self.pose.rotation.yaw = angle
                        case self.IMUAxis.kPitch:
                            self.pose.rotation.pitch = angle
                        case self.IMUAxis.kRoll:
                            self.pose.rotation.pitch = angle

            self.hw = Dummy()

    def GetPosition(self) -> Translation3d:
        try:
            return Translation3d(self.hw.getAngle(self.IMUAxis.kX), self.hw.getAngle(self.IMUAxis.kY), self.hw.getAngle(self.IMUAxis.kZ))
        except Exception:
            return Translation3d(self.hw.getAngle(self.IMUAxis.kX), self.hw.getAngle(self.IMUAxis.kY), self.hw.getAngle(self.IMUAxis.kZ))

    def GetRotation(self) -> Rotation3d:
        try:
            return Rotation3d(self.hw.getAngle(self.IMUAxis.kPitch), self.hw.getAngle(self.IMUAxis.kYaw), self.hw.getAngle(self.IMUAxis.kRoll))
        except Exception:
            return Rotation3d(self.hw.getAngle(self.IMUAxis.kPitch), self.hw.getAngle(self.IMUAxis.kYaw), self.hw.getAngle(self.IMUAxis.kRoll))

    def GetAcceleration(self) -> Translation3d:
        try:
            return Translation3d(self.hw.getAccelX(), self.hw.getAccelY(), self.hw.getAccelZ())
        except Exception:
            return Translation3d(self.hw.getAccelX(), self.hw.getAccelY(), self.hw.getAccelZ())

    def GetRate(self) -> float:
        try:
            return self.hw.getRate(self.IMUAxis.kYaw)
        except Exception:
            return self.hw.getRate(self.IMUAxis.kYaw)

    def Reset(self, pose: Pose3d = Pose3d()) -> None:
        try:
            self.hw.reset()
            self.hw.setGyroAngle(self.IMUAxis.kPitch, pose.rotation().x)
            self.hw.setGyroAngle(self.IMUAxis.kYaw, pose.rotation().y)
            self.hw.setGyroAngle(self.IMUAxis.kRoll, pose.rotation().z)
        except Exception:
            self.hw.reset()

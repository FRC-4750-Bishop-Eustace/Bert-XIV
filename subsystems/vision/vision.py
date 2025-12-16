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
import math
from ..subsystem_base import SubsystemBase
import wpimath.geometry
from hardware import *
import wpimath.estimator
import wpimath.geometry
import ntcore
import photonlibpy
from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy

class VisionData:
    def __init__(self, tagCount: int, pose: wpimath.geometry.Pose2d, timestamp: float):
        self.tagCount: int = tagCount
        self.pose: wpimath.geometry.Pose2d = pose
        self.timestamp: float = timestamp

class Vision(SubsystemBase):
    def __init__(self, typ: CameraType, name: str, imu: IMU, robotToCam: wpimath.geometry.Translation3d = wpimath.geometry.Translation3d(0, 0, 0)) -> None:
        super().__init__("Vision")
        self.typ = typ
        self.name = name
        self.gyro = imu

        self.estimate = None
        match self.typ:
            case CameraType.kLimelight:
                self.estimate = ntcore.NetworkTableInstance.getDefault().getTable(self.name).getEntry("botpose_orb_wpiblue")
            case CameraType.kPhotonvision:
                self.camera = PhotonCamera(self.name)
                self.layout = photonlibpy.AprilTagFieldLayout.loadField(photonlibpy.AprilTagFieldLayout.k2025Reefscape)
                self.robotToCamera = wpimath.geometry.Transform3d(robotToCam, wpimath.geometry.Rotation3d())
                self.estimator = PhotonPoseEstimator(self.camera, PoseStrategy.MULTI_TAG_PNP_COPROCESSOR, self.layout, self.robotToCamera)
            case CameraType.kUSBCamera:
                raise ValueError("Vision control not supported for this camera type")
            case _:
                raise ValueError("Invalid camera type")

    def SubsystemPeriodic(self) -> None:
        if self.typ is CameraType.kLimelight:
            ntcore.NetworkTableInstance.getDefault().getTable(self.name).getEntry("robot_orientation_set").setDoubleArray
            (
                [
                    self.gyro.GetRotation().Z(), 0.0,
                    self.gyro.GetRotation().Y(), 0.0,
                    self.gyro.GetRotation().X(), 0.0
                ],
                0
            )
            self.estimate = self.estimate = ntcore.NetworkTableInstance.getDefault().getTable(self.name).getEntry("botpose_orb_wpiblue")
        elif self.typ is CameraType.kPhotonvision:
            res = self.estimator.getLatestResult()
            if res.hasTargets():
                self.estimate = self.estimator.update(res)
        else:
            pass

    def GetLatestResult(self) -> VisionData:
        if self.typ is CameraType.kLimelight:
            if self.estimate is None:
                return None
            arr = self.estimate.getDoubleArray(None)
            if arr is None:
                return None
            return VisionData(
                int(arr[7]),
                wpimath.geometry.Pose2d(
                    wpimath.geometry.Translation2d(arr[0], arr[1]),
                    wpimath.geometry.Rotation2d(arr[2] * (math.pi / 180), arr[3] * (math.pi / 180))
                ),
                (self.estimate.getLastChange() / 1000000.0) - (arr[6] / 1000.0)
            )
        elif self.typ is CameraType.kPhotonvision:
            if self.estimate is None:
                return None
            return VisionData(
                len(self.camera.getLatestResult().getTargets()),
                self.estimate.estimatedPose.toPose2d(),
                self.estimate.timestampSeconds
            )
        else:
            return None

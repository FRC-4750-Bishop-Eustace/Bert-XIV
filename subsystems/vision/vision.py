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

from .limelight_camera import LimelightCamera
from ..swerve.drivetrain import Drivetrain
from .limelight_helpers import setCameraPoseRobotSpace, setRobotOrientation, PoseEstimate
from wpimath.geometry import Pose2d
from commands2 import Subsystem

class Vision(Subsystem):
    def __init__(self, swerve: Drivetrain, cameras: list[LimelightCamera], block: bool = False) -> None:
        self.swerve = swerve
        self.cameras = cameras
        self.stddevs = [0.0] * 3
        self.pose = Pose2d()
        self.timestamp = 0
        self.block = block

    def blockVision(self, block: bool = True) -> None:
        self.block = block

    def useMT1(self, camera: LimelightCamera) -> bool:
        mt1 = PoseEstimate.getRobotPoseEstimateBlueMT1(camera.name)
        mt1_valid = False
        self.stddevs = [1.0, 1.0, 9999999]

        if mt1.tagCount > 0:
            if mt1.tagCount >= 2:
                mt1_valid = True
                self.stddevs = [0.4, 0.4, 9999999]
                self.pose = mt1.pose
                self.timestamp = mt1.timestamp
            elif len(mt1.fiducials) == 1:
                fid = mt1.fiducials[0]
                if fid.ambiguity < 0.8 and fid.cameraDistance < 4.0:
                    mt1_valid = True
                    scale = min(fid.cameraDistance / 4.0, 1.0)
                    self.stddevs = [
                        0.5 + 0.7 * scale,
                        0.5 + 0.7 * scale,
                        9999999
                    ]
                    self.pose = mt1.pose
                    self.timestamp = mt1.timestamp

        return mt1_valid

    def useMT2(self, camera: LimelightCamera) -> bool:
        mt2 = PoseEstimate.getRobotPoseEstimateBlueMT2(camera.name)

        gyro_rate = abs(self.swerve.gyro.GetRate())
        self.stddevs = [
            0.6 + 0.002 * gyro_rate,
            0.6 + 0.002 * gyro_rate,
            9999999
        ]

        self.pose = mt2.pose
        self.timestamp = mt2.timestamp

        return mt2.tagCount >= 2 and gyro_rate < 360

def periodic(self) -> None:
    if not self.block:
        for camera in self.cameras:
            if camera.pose is not None:
                setCameraPoseRobotSpace(
                    camera.name,
                    camera.pose.translation(),
                    camera.pose.rotation()
                )
            setRobotOrientation(
                self.camera,
                self.swerve.gyro.GetRotation().Z(), 0, # Yaw (Z)
                0, 0,                                  # Pitch (Y)
                0, 0                                   # Roll (X)
            )

            if self.useMT1(camera) or self.useMT2(camera):
                self.swerve.estimator.setVisionMeasurementStdDevs(self.stddevs)
                self.swerve.estimator.addVisionMeasurement(self.pose, self.timestamp)

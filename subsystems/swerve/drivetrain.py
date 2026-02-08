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

import math
from pathlib import Path
from .swerve_module import SwerveModule
from .limelight_helpers import setRobotOrientation, PoseEstimate
from hardware import *
import constants
from wpilib import SmartDashboard
import wpimath
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.controller import PIDController
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from commands2 import Subsystem, Command
import commands2.cmd as cmd
import choreo
from choreo import SwerveTrajectory

class Drivetrain(Subsystem):
    def __init__(self, loader: Loader, camera: str|None = None) -> None:
        super().__init__()
        self.frontLeftLocation = Translation2d(-constants.chassisHalfLength, constants.chassisHalfLength)
        self.frontRightLocation = Translation2d(-constants.chassisHalfLength, -constants.chassisHalfLength)
        self.backLeftLocation = Translation2d(constants.chassisHalfLength, constants.chassisHalfLength)
        self.backRightLocation = Translation2d(constants.chassisHalfLength, -constants.chassisHalfLength)

        self.frontLeft = SwerveModule(loader, constants.frontLeftDriveMotorId, constants.frontLeftTurnMotorId, constants.frontLeftTurnEncoderId)
        self.frontRight = SwerveModule(loader, constants.frontRightDriveMotorId, constants.frontRightTurnMotorId, constants.frontRightTurnEncoderId)
        self.backLeft = SwerveModule(loader, constants.backLeftDriveMotorId, constants.backLeftTurnMotorId, constants.backLeftTurnEncoderId)
        self.backRight = SwerveModule(loader, constants.backRightDriveMotorId, constants.backRightTurnMotorId, constants.backRightTurnEncoderId)

        self.gyro = loader.CreateIMU(IMUType.kNavX)

        self.kinematics = SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backRightLocation,
            self.backLeftLocation
        )
        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.GetRotation().toRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backRight.getPosition(),
                self.backLeft.getPosition(),
            ),
            Pose2d(),
            [0.05, 0.05, math.pi / 36],
            [0.5, 0.5, math.pi / 6]
        )

        self.drivePID = PIDController(
            constants.swerveAutoDrivePID.kP,
            constants.swerveAutoDrivePID.kI,
            constants.swerveAutoDrivePID.kD
        )
        self.turnPID = PIDController(
            constants.swerveAutoTurnPID.kP,
            constants.swerveAutoTurnPID.kI,
            constants.swerveAutoTurnPID.kD
        )

        self.trajectories: dict[str, SwerveTrajectory|None] = {}
        for path in Path("./deploy/choreo").glob("*.traj"):
            filename = path.name[:-5] # Remove '.traj' from path
            try:
                self.trajectories[filename] = choreo.load_swerve_trajectory(filename)
                print(f"\033[34;1mLoaded trajectory \'{filename}\'\033[0m")
            except Exception as e:
                print(f"\033[31;1mFailed to load trajectory \'{filename}\':\033[0m\n{e}")
                self.trajectories[filename] = None

        self.camera = camera

        self.gyro.Reset()

        SmartDashboard.putData("Lateral Teleop PID", constants.swerveDrivePID.toPIDController())
        SmartDashboard.putData("Angular Teleop PID", constants.swerveTurnPID.toProfiledPIDController())
        SmartDashboard.putData("Lateral Autonomous PID", constants.swerveAutoDrivePID.toPIDController())
        SmartDashboard.putData("Angular Autonomous PID", constants.swerveAutoTurnPID.toPIDController())

    def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool, period: float) -> None:
        states = self.kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,
                    ySpeed,
                    rot,
                    Rotation2d(wpimath.units.degreesToRadians(self.gyro.GetRotation().Z()))
                ) if fieldRelative else ChassisSpeeds(
                    xSpeed,
                    ySpeed,
                    rot
                ),
                period
            )
        )
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, constants.maxSpeed)

        self.frontLeft.setDesiredState(states[0])
        self.frontRight.setDesiredState(states[1])
        self.backLeft.setDesiredState(states[2])
        self.backRight.setDesiredState(states[3])

    def periodic(self) -> None:
        self.estimator.update(
            self.gyro.GetRotation().toRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backRight.getPosition(),
                self.backLeft.getPosition(),
            )
        )

        if self.camera:
            mt1 = PoseEstimate.getRobotPoseEstimateBlueMT1(self.camera)
            setRobotOrientation(
                self.camera,
                self.gyro.GetRotation().Z(), 0, # Yaw (Z)
                0, 0,                           # Pitch (Y)
                0, 0                            # Roll (X)
            )
            mt2 = PoseEstimate.getRobotPoseEstimateBlueMT2(self.camera)

            gyro_rate = abs(self.gyro.GetRate())
            mt1_valid = False
            mt1_std = [1.0, 1.0, 9999999]

            if mt1.tagCount > 0:
                if mt1.tagCount >= 2:
                    mt1_valid = True
                    mt1_std = [0.4, 0.4, 9999999]
                elif len(mt1.fiducials) == 1:
                    fid = mt1.fiducials[0]
                    if fid.ambiguity < 0.8 and fid.cameraDistance < 4.0:
                        mt1_valid = True
                        scale = min(fid.distToCamera / 4.0, 1.0)
                        mt1_std = [
                            0.5 + 0.7 * scale,
                            0.5 + 0.7 * scale,
                            9999999
                        ]

            mt2_valid = (mt2.tagCount >= 2 and gyro_rate < 360)
            mt2_std = [
                0.6 + 0.002 * gyro_rate,
                0.6 + 0.002 * gyro_rate,
                9999999
            ]

            if mt2_valid:
                self.estimator.setVisionMeasurementStdDevs(mt2_std)
                self.estimator.addVisionMeasurement(mt2.pose, mt2.timestamp)
            elif mt1_valid:
                self.estimator.setVisionMeasurementStdDevs(mt1_std)
                self.estimator.addVisionMeasurement(mt1.pose, mt1.timestamp)

    def stop(self) -> Command:
        return cmd.run(
            lambda: [self.drive(0, 0, 0, False, 0.02)],
            self
        )

    def getPose(self) -> Pose2d:
        return self.estimator.getEstimatedPosition()

    def getPosition(self) -> Translation2d:
        return self.getPose().translation()

    def getRotation(self) -> Rotation2d:
        return self.getPose().rotation()

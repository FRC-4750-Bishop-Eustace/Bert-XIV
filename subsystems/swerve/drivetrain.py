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
from .swerve_module import SwerveModule
from hardware import *
import constants
import wpimath
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from wpimath.estimator import SwerveDrive4PoseEstimator
from commands2 import Subsystem

class Drivetrain(Subsystem):
    def __init__(self, loader: Loader) -> None:
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

        self.gyro.Reset()

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

    def updateOdometry(self) -> None:
        self.estimator.update(
            self.gyro.GetRotation().toRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backRight.getPosition(),
                self.backLeft.getPosition(),
            )
        )

    def stop(self) -> None:
        self.drive(0, 0, 0, False, 0)

    def getPose(self) -> Pose2d:
        return self.estimator.getEstimatedPosition()

    def getPosition(self) -> Translation2d:
        return self.getPose().translation()

    def getRotation(self) -> Rotation2d:
        return self.getPose().rotation()

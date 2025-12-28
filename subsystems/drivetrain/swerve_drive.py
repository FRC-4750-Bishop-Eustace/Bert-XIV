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

from .drivetrain_base import *
from ..vision import *
from hardware import *
import wpimath.controller
import wpimath.kinematics
import wpimath.geometry
import wpimath.estimator
from wpimath.trajectory import TrapezoidProfile

class SwerveDrive(Drivetrain):
    def __init__(self, config: DrivetrainConfig, modules: list[Module], imu: IMU, vision: Vision):
        super().__init__(config, modules, imu, vision)

        self.config = config
        self.driveModules = modules[0::2]
        self.steerModules = modules[1::2]
        self.gyro = imu
        self.vision = vision

        self.gyro.Reset()
        for module in self.driveModules:
            module.encoder.Reset()
        for module in self.steerModules:
            module.encoder.Reset()

        self.drivePID = []
        self.steerPID = []
        for index in range(len(self.driveModules)):
            self.drivePID.append(
                wpimath.controller.PIDController(
                    config.pid[0].kP,
                    config.pid[0].kI,
                    config.pid[0].kD,
                )
            )
        for index in range(len(self.steerModules)):
            self.steerPID.append(
                wpimath.controller.ProfiledPIDController(
                    config.pid[1].kP,
                    config.pid[1].kI,
                    config.pid[1].kD,
                    TrapezoidProfile.Constraints(
                        config.maxAngularVelocity,
                        config.maxAngularAcceleration
                    )
                )
            )
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            config.locations[0],
            config.locations[1],
            config.locations[2],
            config.locations[3]
        )
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            config.feedforward[0].kS,
            config.feedforward[0].kV,
            config.feedforward[0].kA
        )
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardRadians(
            config.feedforward[1].kS,
            config.feedforward[1].kV,
            config.feedforward[1].kA
        )
        self.estimator = wpimath.estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.GetRotation().toRotation2d(),
            (
                wpimath.kinematics.SwerveModulePosition(
                    self.driveModules[0].encoder.GetPosition(),
                    wpimath.geometry.Rotation2d(self.steerModules[0].encoder.GetPosition())
                ),
                wpimath.kinematics.SwerveModulePosition(
                    self.driveModules[1].encoder.GetPosition(),
                    wpimath.geometry.Rotation2d(self.steerModules[1].encoder.GetPosition())
                ),
                wpimath.kinematics.SwerveModulePosition(
                    self.driveModules[2].encoder.GetPosition(),
                    wpimath.geometry.Rotation2d(self.steerModules[2].encoder.GetPosition())
                ),
                wpimath.kinematics.SwerveModulePosition(
                    self.driveModules[3].encoder.GetPosition(),
                    wpimath.geometry.Rotation2d(self.steerModules[3].encoder.GetPosition())
                )
            ),
            wpimath.geometry.Pose2d(
                self.gyro.GetPosition().toTranslation2d(),
                self.gyro.GetRotation().toRotation2d()
            )
        )

        for pid in self.steerPID:
            pid.enableContinuousInput(-math.pi, math.pi)

    def Drive(self, x: float, y: float, theta, fieldRelative: bool, period: float) -> None:
        realSpeeds = wpimath.kinematics.ChassisSpeeds.discretize(
            (
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    x,
                    y,
                    theta,
                    self.gyro.GetRotation()
                )
                if fieldRelative
                else wpimath.kinematics.ChassisSpeeds(x, y, theta)
            ),
            period
        )
        speeds = self.kinematics.toSwerveModuleStates(realSpeeds)

        for index in range(len(self.driveModules)):
            rot = wpimath.geometry.Rotation2d(self.steerModules[index].encoder.GetPosition())
            speeds[index].optimize(rot)
            speeds[index].cosineScale(rot)

            driveOutput = self.driveFeedforward.calculate(speeds[index].speed) + self.drivePID[index].calculate(
                self.driveModules[index].encoder.GetVelocity(),
                speeds[index].speed
            )

            steerOutput = self.steerPID[index].calculate(
                self.steerModules[index].encoder.GetPosition(),
                speeds[index].angle.radians()
            ) + self.turnFeedforward.calculate(
                self.steerPID[index].getSetpoint().velocity
            )

            self.driveModules[index].motor.SetVoltage(driveOutput)
            self.steerModules[index].motor.SetVoltage(steerOutput)

    def SubsystemPeriodic(self) -> None:
        self.estimator.update(
            self.gyro.GetRotation().toRotation2d(),
            (
                wpimath.kinematics.SwerveModulePosition(
                    self.driveModules[0].encoder.GetPosition(),
                    wpimath.geometry.Rotation2d(self.steerModules[0].encoder.GetPosition())
                ),
                wpimath.kinematics.SwerveModulePosition(
                    self.driveModules[1].encoder.GetPosition(),
                    wpimath.geometry.Rotation2d(self.steerModules[1].encoder.GetPosition())
                ),
                wpimath.kinematics.SwerveModulePosition(
                    self.driveModules[2].encoder.GetPosition(),
                    wpimath.geometry.Rotation2d(self.steerModules[2].encoder.GetPosition())
                ),
                wpimath.kinematics.SwerveModulePosition(
                    self.driveModules[3].encoder.GetPosition(),
                    wpimath.geometry.Rotation2d(self.steerModules[3].encoder.GetPosition())
                )
            )
        )

        data = self.vision.GetLatestResult()
        if data is None:
            return

        if data.tagCount > 0:
            self.estimator.setVisionMeasurementStdDevs([0.7, 0.7, 9999999.0])
            self.estimator.addVisionMeasurement(data.pose, data.timestamp)

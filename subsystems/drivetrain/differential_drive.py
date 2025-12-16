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

class DifferentialDrive(Drivetrain):
    def __init__(self, config: DrivetrainConfig, modules: list[Module], imu: IMU, vision: Vision):
        super().__init__(config, modules, imu, vision)

        self.config = config
        self.leftModules = modules[:len(modules) // 2]
        self.rightModules = modules[len(modules) // 2:]
        self.gyro = imu
        self.vision = vision

        self.gyro.Reset()
        for module in self.leftModules:                                module.encoder.Reset()
        for module in self.rightModules:
            module.encoder.Reset()

        self.leftPID = wpimath.controller.PIDController(
            config.pid[0].kP,
            config.pid[0].kI,
            config.pid[0].kD
        )
        self.rightPID = wpimath.controller.PIDController(
            config.pid[1].kP,
            config.pid[1].kI,
            config.pid[1].kD
        )
        self.kinematics = wpimath.kinematics.DifferentialDriveKinematics(
            config.trackWidth
        )
        self.feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            config.feedforward[0].kS,
            config.feedforward[0].kV,
            config.feedforward[0].kA
        )
        self.estimator = wpimath.estimator.DifferentialDrivePoseEstimator(
            self.kinematics,
            self.gyro.GetRotation().toRotation2d(),
            sum([module.encoder.GetPosition() for module in self.leftModules]) / len(self.leftModules),
            sum([module.encoder.GetPosition() for module in self.rightModules]) / len(self.rightModules),
            wpimath.geometry.Pose2d(
                self.gyro.GetPosition().toTranslation2d(),
                self.gyro.GetRotation().toRotation2d()
            )
        )

    def Drive(self, x: float, y: float, theta, fieldRelative: bool, period: float) -> None:
        wheelSpeeds = self.kinematics.toWheelSpeeds(x, y, theta)

        leftOutput = self.feedforward.calculate(wheelSpeeds.left) + self.leftPID.calculate(
            sum([module.encoder.GetVelocity() for module in self.leftModules]) / len(self.leftModules),
            wheelSpeeds.left
        )
        rightOutput = self.feedforward.calculate(wheelSpeeds.right) + self.rightPID.calculate(
            sum([module.encoder.GetVelocity() for module in self.rightModules]) / len(self.rightModules),
            wheelSpeeds.right
        )

        for module in self.leftModules:
            module.motor.SetVoltage(leftOutput)
        for module in self.rightModules:
            module.motor.SetVoltage(rightOutput)

    def SubsystemPeriodic(self) -> None:
        self.estimator.update(
            self.gyro.GetRotation().toRotation2d(),
            sum([module.encoder.GetPosition() for module in self.leftModules]) / len(self.leftModules),
            sum([module.encoder.GetPosition() for module in self.rightModules]) / len(self.rightModules)
        )

        data = self.vision.GetLatestResult()
        if data is None:
            return

        if data.tagCount > 0:
            self.estimator.setVisionMeasurementStdDevs([0.7, 0.7, 9999999.0])
            self.estimator.addVisionMeasurement(data.pose, data.timestamp)

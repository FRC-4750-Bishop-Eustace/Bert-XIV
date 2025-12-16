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

from enum import Enum
from .drivetrain_base import *
from .differential_drive import DifferentialDrive
from .mecanum_drive import MecanumDrive
from .swerve_drive import SwerveDrive
from ..vision import *
from hardware import *
import wpilib
import wpimath.filter
import wpimath.geometry

class DriveType(Enum):
    kDifferential = 0
    kMecanum = 1
    kSwerve = 2

class DrivetrainController:
    def __init__(self, typ: DriveType, controller: wpilib.interfaces.GenericHID, hasGyro: bool) -> None:
        self.typ = typ
        self.controller = controller
        self.hasGyro = hasGyro
        self.fieldRelative = False
        self.drive = None

        self.xSlewRate = None
        self.ySlewRate = None
        self.rSlewRate = None

        self.xDeadband = None
        self.yDeadband = None
        self.rDeadband = None

        self.fieldSwitchKey = 3

        self.gyro = None
        self.vision = None

    def SetFieldSwitchKey(self, key: int) -> None:
        self.fieldSwitchKey = key

    def SetTrackingConfig(self, imu: IMU, typ: CameraType, name: str) -> Vision:
        self.gyro = imu
        self.vision = Vision(
            typ,
            name,
            self.gyro if self.hasGyro else None,
            self.gyro.GetPosition()
        )

    def SetDriveConfig(self, modules: [Module], config: DrivetrainConfig = DrivetrainConfig()) -> Drivetrain:
        match self.typ:
            case DriveType.kDifferential:
                self.drive = DifferentialDrive(DrivetrainConfig(), modules, self.gyro if self.hasGyro else None, self.vision)
            case DriveType.kMecanum:
                self.drive = MecanumDrive(DrivetrainConfig(), modules, self.gyro if self.hasGyro else None, self.vision)
            case DriveType.kSwerve:
                self.drive = SwerveDrive(DrivetrainConfig(), modules, self.gyro if self.hasGyro else None, self.vision)
            case _:
                self.drive = None
                raise ValueError("Invalid drive type")

        self.xSlewRate = wpimath.filter.SlewRateLimiter(config.slewRates[0])
        self.ySlewRate = wpimath.filter.SlewRateLimiter(config.slewRates[1])
        self.rSlewRate = wpimath.filter.SlewRateLimiter(config.slewRates[2])

        self.xDeadband = config.deadbands[0]
        self.yDeadband = config.deadbands[1]
        self.rDeadband = config.deadbands[2]

        return self.drive

    def DriveWithJoystick(self, period: float) -> None:
        xSpeed = -self.xSlewRate.calculate(
            wpimath.applyDeadband(
                self.controllergetRawAxis(1),
                self.xDeadband
            )
        )
        ySpeed = -self.ySlewRate.calculate(
            wpimath.applyDeadband(
                self.controller.getRawAxis(0),
                self.yDeadband
            )
        )
        rSpeed = self.rSlewRate.calculate(
            wpimath.applyDeadband(
                (self.controller.getRawAxis(3) + 1) / 2,
                self.rDeadband
            )
        ) + self.rSlewRate.calculate(
            wpimath.applyDeadband(
                (self.controller.getRawAxis(4) + 1) / 2,
                self.rDeadband
            )
        )

        if self.controller.getRawButton(self.fieldSwitchKey):
            self.fieldRelative = not self.fieldRelative

        self.drive.Drive(xSpeed, ySpeed, rSpeed, self.fieldRelative, period)

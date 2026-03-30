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

import constants
from subsystems import *
from wpilib import DriverStation, SmartDashboard
from commands2 import Command
from wpilib import PS4Controller
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter

class DriveWithJoystick(Command):
    def __init__(self, swerve: Drivetrain, controller: PS4Controller) -> None:
        super().__init__()
        self.swerve = swerve
        self.controller = controller
        self.xSlewRate = SlewRateLimiter(constants.xSlewRate)
        self.ySlewRate = SlewRateLimiter(constants.ySlewRate)
        self.rotSlewRate = SlewRateLimiter(constants.rotSlewRate)
        self.fieldRelative = True

        self.addRequirements(self.swerve)

        SmartDashboard.putBoolean("Field Relative", self.fieldRelative)

    def toggleFieldRelative(self) -> None:
        self.fieldRelative = not self.fieldRelative
        SmartDashboard.putBoolean("Field Relative", self.fieldRelative)

    def resetGyro(self):
        self.swerve.gyro.Reset()

    def execute(self) -> None:
        if not DriverStation.isTeleopEnabled():
            return

        xSpeed = -self.xSlewRate.calculate(
            applyDeadband(
                self.controller.getRawAxis(constants.ps4LeftY),
                constants.xDeadband
            ) * constants.maxSpeed
        )
        ySpeed = -self.ySlewRate.calculate(
            applyDeadband(
                self.controller.getRawAxis(constants.ps4LeftX),
                constants.yDeadband
            ) * constants.maxSpeed
        )
        rotSpeed = -self.rotSlewRate.calculate(
            applyDeadband(
                self.controller.getRawAxis(constants.ps4RightX),
                constants.rotDeadband
            ) * constants.rMaxSpeed
        )

        if self.controller.getPOV() == constants.ps4Up:
            xSpeed = constants.microTSpeed
        if self.controller.getPOV() == constants.ps4Right:
            ySpeed = -constants.microTSpeed
        if self.controller.getPOV() == constants.ps4Down:
            xSpeed = -constants.microTSpeed
        if self.controller.getPOV() == constants.ps4Left:
            ySpeed = constants.microTSpeed

        if self.controller.getRawButton(constants.ps4L1) == 1:
            rotSpeed = constants.microRSpeed
        if self.controller.getRawButton(constants.ps4R1) == 1:
            rotSpeed = -constants.microRSpeed

        self.swerve.drive(xSpeed, ySpeed, rotSpeed, self.fieldRelative, 0.02)

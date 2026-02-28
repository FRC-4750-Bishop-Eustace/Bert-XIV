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
from subsystems import *
import constants
from wpilib import DriverStation
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Translation2d, Rotation2d
from commands2 import Command

class LockOntoPoint(Command):
    def __init__(self, swerve: Drivetrain, point: Translation2d):
        super().__init__()
        self.swerve = swerve
        self.point = point
        self.mirrorPoint()

        current_pos = self.swerve.getPosition()
        self.target_theta = Rotation2d(point.X() - current_pos.X(), point.Y() - current_pos.Y())

        self.addRequirements(self.swerve)

    def mirrorPoint(self) -> None:
        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kBlue and self.point.X() > (constants.fieldLength / 2) or \
            alliance == DriverStation.Alliance.kRed and self.point.X() < (constants.fieldLength / 2):
            self.point = Translation2d(
                constants.fieldLength - self.point.X(),
                constants.fieldWidth - self.point.Y()
            )

    def initialize(self) -> None:
        self.swerve.drivePID.reset()
        self.swerve.turnPID.reset()

    def execute(self) -> None:
        speeds = ChassisSpeeds(
            0.0,
            0.0,
            self.swerve.turnPID.calculate(
                self.swerve.getRotation().radians(),
                self.target_theta.radians()
            )
        )
        self.swerve.drive(0.0, 0.0, speeds[2], True, 0.02)

    def isFinished(self):
        return abs(self.swerve.getRotation().minus(self.target_theta).radians()) < 0.02

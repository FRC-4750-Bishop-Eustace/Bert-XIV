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

from subsystems import *
import constants
from wpilib import DriverStation
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Translation2d, Rotation2d
from commands2 import Command

class LockOntoGoal(Command):
    def __init__(self, swerve: Drivetrain):
        super().__init__()
        self.swerve = swerve
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.point = constants.blueAllianceHub
        else:
            self.point = constants.redAllianceHub

        current_pos = self.swerve.getPosition()
        self.target_theta = Rotation2d(self.point.X() - current_pos.X(), self.point.Y() - current_pos.Y())

        self.addRequirements(self.swerve)

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
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
from wpimath.kinematics import ChassisSpeeds
from commands2 import Command
import commands2.cmd as cmd
from choreo.trajectory import SwerveTrajectory

class FollowPath(Command):
    def __init__(self, swerve: Drivetrain, path: str):
        super().__init__()
        self.swerve = swerve
        self.path = path

        self.addRequirements(self.swerve)

    def execute(self) -> None:
        sample: SwerveTrajectory = self.swerve.trajectories[self.path]
        if sample:
            pose = self.getPose()
            speeds = ChassisSpeeds(
                sample.vx + self.drivePID.calculate(
                    pose.X(),
                    pose.Y()
                ),
                sample.vy + self.drivePID.calculate(
                    pose.X(),
                    pose.Y()
                ),
                sample.omega + self.turnPID.calculate(
                    pose.rotation().radians(),
                    sample.heading
                )
            )

            self.drive(speeds[0], speeds[1], speeds[2], True, 0.02)
        else:
            print(f"\033[31;1mPath {self.path} not found\033[0m")



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

from hardware import *
from subsystems import *
from commands import *
from wpilib import PS4Controller, Field2d, SmartDashboard
from commands2 import Command, InstantCommand
from commands2.button import JoystickButton

class RobotContainer:
    def __init__(self) -> None:
        self.loader = Loader()
        print(self.loader.GetBackends())

        self.controller = PS4Controller(0)

        self.swerve = Drivetrain(self.loader, "limelight")
        self.swerveCmd = DriveWithJoystick(self.swerve, self.controller)
        self.swerve.setDefaultCommand(self.swerveCmd)

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def bindKey(self, button: int, function) -> None:
        if button <= self.controller.getButtonCount():
            JoystickButton(self.controller, button).onTrue(
                InstantCommand(
                    function,
                    self
                )
            )

    def configureBindings(self) -> None:
        self.bindKey(
            self.controller.Button.kCross,
            lambda: [self.swerveCmd.setFieldRelative(not self.swerveCmd.getFieldRelative())]
        )

    def updateField(self) -> None:
        self.field.setRobotPose(self.swerve.getPose())

    def getAutonomousCommand(self) -> Command:
        return DefaultAuto()

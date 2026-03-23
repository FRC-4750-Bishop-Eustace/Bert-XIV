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
from hardware import *
from subsystems import *
from commands import *
from wpilib import PS4Controller, Joystick, Field2d, SmartDashboard
from commands2 import *
from commands2.button import JoystickButton

class RobotContainer:
    def __init__(self) -> None:
        self.loader = Loader()
        print(self.loader.GetBackends())

        self.controller = PS4Controller(constants.ps4ControllerPort)
        self.dashboard = Joystick(constants.dashboardPort)

        self.swerve = Drivetrain(self.loader)
        self.swerveCmd = DriveWithJoystick(self.swerve, self.controller)
        self.swerve.setDefaultCommand(self.swerveCmd)

        self.vision = Vision(self.swerve, [LimelightCamera("limelight")])
        self.vision.setDefaultCommand(EnableVisionFusion(self.vision))

        self.feeder = Feeder(self.loader)
        self.feeder.setDefaultCommand(FeedShooter(self.feeder, self.dashboard))
        self.shooter = Shooter(self.loader)
        self.shooter.setDefaultCommand(RunShooter(self.shooter, self.dashboard))

        self.intake_actuator = IntakeActuator(self.loader)
        self.intake = Intake(self.loader)

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def configureBindings(self) -> None:
        JoystickButton(self.controller, constants.ps4Cross).onTrue(
            InstantCommand(
                lambda: self.swerveCmd.toggleFieldRelative(),
                None
            )
        )

        JoystickButton(self.dashboard, 8).whileTrue(
            DeployIntake(self.intake_actuator)
        ).whileFalse(
            InstantCommand(
                lambda: self.intake_actuator.stop(),
                self.intake_actuator
            )
        )
        JoystickButton(self.dashboard, 10).whileTrue(
            StowIntake(self.intake_actuator)
        ).whileFalse(
            InstantCommand(
                lambda: self.intake_actuator.stop(),
                self.intake_actuator
            )
        )

        JoystickButton(self.controller, constants.ps4Square).onTrue(
            InstantCommand(
                lambda: self.intake.start(1) if self.intake.isStopped() else self.intake.stop(),
                self.intake
            )
        )

        JoystickButton(self.dashboard, 12).onTrue(
            InstantCommand(
                lambda: self.shooter.setSpeed(self.shooter.getSpeed() + 0.5),
                self.shooter
            )
        )
        JoystickButton(self.dashboard, 11).onTrue(
            InstantCommand(
                lambda: self.shooter.setSpeed(self.shooter.getSpeed() - 0.5),
                self.shooter
            )
        )

    def updateField(self) -> None:
        self.field.setRobotPose(self.swerve.getPose())

    def getAutonomousCommand(self) -> Command:
        return DefaultAuto()

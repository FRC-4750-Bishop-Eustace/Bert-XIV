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

from typing import override
import wpilib
from commands2 import TimedCommandRobot, CommandScheduler
from hardware import *
from subsystems import *

class MyRobot(TimedCommandRobot):
    def __init__(self) -> None:
        super().__init__()

    @override
    def robotInit(self) -> None:
        self.loader = Loader()
        print(self.loader.GetBackends())

        self.controller = wpilib.PS4Controller(0)

        self.drivetrain = DrivetrainController(
            DriveType.kSwerve,
            self.controller,
            True
        )
        self.drivetrain.SetTrackingConfig(self.loader.CreateIMU(IMUType.kNavX), CameraType.kLimelight, "limelight")
        self.drivetrain.SetDriveConfig(
            [
                Module(
                    self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=False),
                    self.loader.CreateEncoder(EncoderType.kAbsoluteEncoder, motor=self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=False))
                ), # front left drive
                Module(
                    self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=True),
                    self.loader.CreateEncoder(EncoderType.kCANcoder, deviceId=0, canbus="rio")
                ), # front left steer
                Module(
                    self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=False),
                    self.loader.CreateEncoder(EncoderType.kAbsoluteEncoder, motor=self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=False))
                ), # front right drive
                Module(
                    self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=True),
                    self.loader.CreateEncoder(EncoderType.kCANcoder, deviceId=0, canbus="rio")
                ), # front right steer
                Module(
                    self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=False),
                    self.loader.CreateEncoder(EncoderType.kAbsoluteEncoder, motor=self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=False))
                ), # back left drive
                Module(
                    self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=True),
                    self.loader.CreateEncoder(EncoderType.kCANcoder, deviceId=0, canbus="rio")
                ), # back left steer
                Module(
                    self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=False),
                    self.loader.CreateEncoder(EncoderType.kAbsoluteEncoder, motor=self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=False))
                ), # back right drive
                Module(
                    self.loader.CreateMotor(MotorType.kSparkMAX, deviceId=0, typ=MotorMode.kBrushless, inverted=True),
                    self.loader.CreateEncoder(EncoderType.kCANcoder, deviceId=0, canbus="rio")
                ), # back right steer
            ]
        )

    @override
    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    @override
    def autonomousInit(self) -> None:
        pass

    @override
    def autonomousPeriodic(self) -> None:
        pass

    @override
    def autonomousExit(self) -> None:
        pass

    @override
    def teleopInit(self) -> None:
        pass

    @override
    def teleopPeriodic(self) -> None:
        self.drivetrain.DriveWithJoystick(self.getPeriod())

    @override
    def teleopExit(self) -> None:
        pass

    @override
    def disabledInit(self) -> None:
        pass

    @override
    def disabledPeriodic(self) -> None:
        pass

    @override
    def disabledExit(self) -> None:
        pass

    @override
    def testInit(self) -> None:
        pass

    @override
    def testPeriodic(self) -> None:
        pass

    @override
    def testExit(self) -> None:
        pass

    @override
    def _simulationInit(self) -> None:
        pass

    @override
    def _simulationPeriodic(self) -> None:
        pass

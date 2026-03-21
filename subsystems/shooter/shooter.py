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
from wpilib import SmartDashboard, DriverStation
from wpimath import Translation2d
from commands2 import Subsystem

class Shooter(Subsystem):
    def __init__(self, loader: Loader) -> None:
        super().__init__()
        self.motor1 = loader.CreateMotor(MotorType.kSparkFlex, deviceId=constants.shooterMotor1Id, typ=MotorMode.kBrushless, inverted=True)
        self.motor2 = loader.CreateMotor(MotorType.kSparkFlex, deviceId=constants.shooterMotor2Id, typ=MotorMode.kBrushless, inverted=False)

        self.motor1.SetParameters(True, IdleMode.kCoast)
        self.motor2.SetParameters(False, IdleMode.kCoast)
        self.setSpeed(constants.shooterDefaultSpeed)

    def getSpeed(self) -> float:
        return self.speed

    def setSpeed(self, speed: float) -> None:
        if speed > 12:
            speed = 1
        elif speed < 1:
            speed = 12

        self.speed = speed
        SmartDashboard.putNumber("Shooter Voltage", self.speed)

    def setSpeedByPosition(self, position: Translation2d) -> None:
        # voltage = distance / constants.shooterMetersPerVolt
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.setSpeed(position.distance(constants.blueAllianceHub) / constants.shooterMetersPerVolt)
        else:
            self.setSpeed(position.distance(constants.redAllianceHub) / constants.shooterMetersPerVolt)

    def start(self, direction: int) -> None:
        self.motor1.SetVoltage(self.speed * direction)
        self.motor2.SetVoltage(self.speed * direction)

    def stop(self) -> None:
        self.motor1.SetVoltage(0)
        self.motor2.SetVoltage(0)

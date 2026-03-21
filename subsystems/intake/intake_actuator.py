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

import wpilib
import constants
from hardware import *
from commands2 import Subsystem

class IntakeActuator(Subsystem):
    def __init__(self, loader: Loader) -> None:
        super().__init__()
        self.actuatorMotor = loader.CreateMotor(MotorType.kSparkMAX, deviceId=constants.intakeActuatorMotorId, typ=MotorMode.kBrushed, inverted=True)
        self.actuatorMotor2 = loader.CreateMotor(MotorType.kSparkMAX, deviceId=constants.intakeActuatorMotor2Id, typ=MotorMode.kBrushed, inverted=True)
        self.limitSwitch = wpilib.DigitalInput(8)
        

    def deploy(self) -> None:
        if self.limitSwitch.get() == True:
            self.actuatorMotor.SetVoltage(constants.actuatorSpeed)
            self.actuatorMotor2.SetVoltage(constants.actuatorSpeed)
        else:
            self.actuatorMotor.Stop()
            self.actuatorMotor2.Stop()

    def stow(self) -> None:
        self.actuatorMotor.SetVoltage(-constants.actuatorSpeed)
        self.actuatorMotor2.SetVoltage(-constants.actuatorSpeed)

    def stop(self) -> None:
        self.actuatorMotor.Stop()
        self.actuatorMotor2.Stop()
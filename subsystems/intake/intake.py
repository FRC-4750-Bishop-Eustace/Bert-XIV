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
import constants
from commands2 import Subsystem, Command
import commands2.cmd as cmd

class Intake(Subsystem):
    def __init__(self, loader: Loader) -> None:
        super().__init__()
        self.motors: list[Motor] = [
            loader.CreateMotor(MotorType.kSparkFlex, deviceId=constants.intakeLeftMotorId, typ=MotorMode.kBrushless, inverted=False),
            loader.CreateMotor(MotorType.kSparkFlex, deviceId=constants.intakeRightMotorId, typ=MotorMode.kBrushless, inverted=False),
        ]
        self.pistons: list[Solenoid] = [
            loader.CreateSolenoid(SolenoidType.kSingle, module=constants.intakeSolenoid1Module, channel=constants.intakeSolenoid1Channel),
            loader.CreateSolenoid(SolenoidType.kSingle, module=constants.intakeSolenoid2Module, channel=constants.intakeSolenoid2Channel),
            loader.CreateSolenoid(SolenoidType.kSingle, module=constants.intakeSolenoid3Module, channel=constants.intakeSolenoid3Channel),
        ]
        self.speed: float = (2 / 3)

    def spin(self, direction: bool) -> Command:
        speed = self.speed if direction else -self.speed
        return cmd.run(
            lambda: [motor.Set(speed) for motor in self.motors],
            self
        )

    def stop(self) -> Command:
        return cmd.run(
            lambda: [motor.Set(0) for motor in self.motors],
            self
        )

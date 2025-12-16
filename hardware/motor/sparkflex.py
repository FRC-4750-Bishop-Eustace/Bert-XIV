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

from typing import Any
from ..motor_base import Motor
from ..enums import MotorMode

class SparkFlex(Motor):
    backendName = "REV_SparkFlex"

    def __init__(self, deviceId: int, typ: MotorMode, inverted: bool) -> None:
        super().__init__()
        self.deviceId = deviceId
        self.typ = typ
        self.inverted = inverted

        try:
            import rev

            self.hw = rev.SparkFlex(self.deviceId, rev.SparkMax.MotorType.kBrushless if self.typ == MotorMode.kBrushless else rev.SparkMax.MotorType.kBrushed)
            config = rev.SparkFlexConfig()
            config.inverted(self.inverted)
            self.hw.configure(config, rev.SparkBase.ResetMode.kNoResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

        except Exception:
            class Dummy:
                def __init__(self, id: int, typ: Any, inv: bool) -> None:
                    self.id = id
                    self.typ = typ
                    self.inv = inv

                def Set(self, speed: float) -> None:
                    self.speed = speed

                def SetVoltage(self, voltage: float) -> None:
                    self.voltage = voltage

                def GetPosition(self) -> float:
                    return self.position

                def GetVelocity(self) -> float:
                    return self.speed

                def Stop(self) -> None:
                    self.speed = 0
                    self.voltage = 0

            self.hw = Dummy(self.deviceId, self.typ, self.inverted)

    def Set(self, speed: float) -> None:
        try:
            self.hw.set(speed)
        except Exception:
            self.hw.Set(speed)

    def SetVoltage(self, voltage: float) -> None:
        try:
            self.hw.setVoltage(voltage)
        except Exception:
            self.hw.SetVoltage(voltage)

    def GetPosition(self) -> float:
        try:
            return self.hw.getAbsoluteEncoder().getPosition()
        except Exception:
            return self.hw.GetPosition()

    def GetVelocity(self) -> float:
        try:
            return self.hw.getAbsoluteEncoder().getVelocity()
        except Exception:
            return self.hw.GetVelocity()

    def Stop(self) -> None:
        try:
            self.hw.stopMotor()
        except Exception:
            self.hw.Stop()

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

from ..motor_base import Motor

class TalonFX(Motor):
    backendName: str = "CTRE_TalonFX"

    def __init__(self, deviceId: int, canbus: str, inverted: bool) -> None:
        super().__init__()
        self.deviceId = deviceId
        self.canbus = canbus
        self.inverted = inverted

        try:
            from phoenix6 import configs
            from phoenix6 import hardware

            config = configs.MotorOutputConfigs()
            config.inverted = self.inverted

            self.hw = hardware.TalonFX(device_id=self.deviceId, canbus=self.canbus)
            self.hw.configurator.apply(config)

        except Exception:
            class Dummy:
                class Value:
                    def __init__(self, value: float = 0) -> None:
                        self.value: float = value

                def __init__(self, id: int, bus: str, inv: bool) -> None:
                    self.id = id
                    self.bus = bus
                    self.inv = inv
                    self.speed = 0
                    self.voltage = 0
                    self.position = self.Value()
                    self.velocity = self.Value()

                def set(self, speed: float) -> None:
                    self.speed = speed

                def setVoltage(self, voltage: float) -> None:
                    self.voltage = voltage

                def get_position(self) -> "Value":
                    return self.position

                def get_velocity(self) -> "Value":
                    return self.velocity

                def stopMotor(self) -> None:
                    self.speed = 0
                    self.voltage = 0

            self.hw = Dummy(self.deviceId, self.canbus, self.inverted)

    def SetParameters(self, inverted: bool, mode, velocityFactor: float|None = None, positionFactor: float|None = None) -> None:
        try:
            pass
        except Exception:
            pass

    def Set(self, speed: float) -> None:
        try:
            self.hw.set(speed)
        except Exception:
            self.hw.set(speed)

    def SetVoltage(self, voltage: float) -> None:
        try:
            self.hw.setVoltage(voltage)
        except Exception:
            self.hw.setVoltage(voltage)

    def GetPosition(self) -> float:
        try:
            return float(self.hw.get_position().value)
        except Exception:
            return float(self.hw.get_position().value)

    def GetVelocity(self) -> float:
        try:
            return float(self.hw.get_velocity().value)
        except Exception:
            return float(self.hw.get_velocity().value)

    def Stop(self) -> None:
        try:
            self.hw.stopMotor()
        except Exception:
            self.hw.stopMotor()

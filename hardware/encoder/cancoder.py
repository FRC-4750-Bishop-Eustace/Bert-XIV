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

from ..encoder_base import Encoder
from ..device import Device

class CANcoder(Encoder):
    def __init__(self, deviceId: int, canbus: str):
        super().__init__()
        self.deviceId = deviceId
        self.canbus = canbus

        try:
            from phoenix6 import hardware

            self.hw = hardware.CANcoder(self.deviceId, self.canbus)

        except Exception:
            class Dummy:
                def __init__(self, id: int, bus: str):
                    self.id = id
                    self.bus = bus

                def GetPosition(self) -> float:
                    return 0.0

                def GetVelocity(self) -> float:
                    return 0.0

                def Reset(self) -> None:
                    pass

            self.hw = Dummy(self.deviceId, self.canbus)

    def GetPosition(self) -> float:
        try:
            return float(self.hw.get_position().value)
        except Exception:
            return self.hw.GetPosition()

    def GetVelocity(self) -> float:
        try:
            return float(self.hw.get_velocity().value)
        except Exception:
            return self.hw.GetVelocity()

    def Reset(self) -> None:
        try:
            self.hw.set_position(0.0)
        except Exception:
            self.hw.Reset()

    def Reset(self) -> None:
        try:
            pass
        except Exception:
            self.hw.Reset()

Device.RegisterBackend("encoder", "CTRE_CANcoder", CANcoder)

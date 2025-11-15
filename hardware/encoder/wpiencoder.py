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

class WPIEncoder(Encoder):
    def __init__(self, channelA: int, channelB: int, inverted: bool):
        super().__init__()
        self.channelA = channelA
        self.channelB = channelB
        self.inverted = inverted

        try:
            import wpilib

            self.hw = wpilib.Encoder(self.channelA, self.channelB, self.inverted)

        except Exception:
            class Dummy:
                def __init__(self, a: int, b: int, inv: bool) -> None:
                    self.a = a
                    self.b = b
                    self.inv = inv

                def GetPosition(self) -> float:
                    return 0.0

                def GetVelocity(self) -> float:
                    return 0.0

                def Reset(self) -> None:
                    pass

            self.hw = Dummy(self.channelA, self.channelB, self.inverted)

    def GetPosition(self) -> float:
        try:
            return self.hw.getDistance()
        except Exception:
            return self.hw.GetPosition()

    def GetVelocity(self) -> float:
        try:
            return self.hw.getRate()
        except Exception:
            return self.hw.GetVelocity()

    def Reset(self) -> None:
        try:
            self.hw.reset()
        except Exception:
            self.hw.Reset()

Device.RegisterBackend("encoder", "WPI_Encoder", WPIEncoder)

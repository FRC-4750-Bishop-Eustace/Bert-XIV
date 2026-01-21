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

from ..solenoid_base import Solenoid

class DoubleSolenoid(Solenoid):
    backendName: str = "WPI_DoubleSolenoid"

    def __init__(self, module: int, channelFwd: int, channelRev: int):
        super().__init__()
        self.module = module
        self.channelFwd = channelFwd
        self.channelRev = channelRev

        try:
            import wpilib

            self.Position = wpilib.DoubleSolenoid.Value
            self.hw = wpilib.DoubleSolenoid(self.module, self.channelFwd, self.channelRev)

        except Exception:
            class Dummy:
                def __init__(self, mod: int, fwd: int, rev: int) -> None:
                    self.mod = mod
                    self.fwd = fwd
                    self.rev = rev
                    self.state = 0

                def set(self, value: int) -> None:
                    self.state = value

                def get(self) -> int:
                    return self.state

                def toggle(self) -> None:
                    self.state = not self.state

            self.hw = Dummy(self.module, self.channelFwd, self.channelRev)

    def Set(self, value: int) -> None:
        try:
            self.hw.set(self.Position.kForward if value == 1 else self.Position.kReverse if value == -1 else self.Position.kOff)
        except Exception:
            self.hw.set(value)

    def Get(self) -> int:
        try:
            return 1 if self.hw.get() == self.Position.kForward else -1 if self.hw.get() == self.Position.kReverse else 0
        except Exception:
            return self.hw.get()

    def Toggle(self) -> None:
        try:
            self.hw.toggle()
        except Exception:
            self.hw.toggle()

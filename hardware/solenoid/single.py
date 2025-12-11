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
from ..device import Device

class SingleSolenoid(Solenoid):
    def __init__(self, module: int, channel: int):
        super().__init__()
        self.module = module
        self.channel = channel

        try:
            import wpilib

            self.hw = wpilib.Solenoid(self.module, self.channel)

        except Exception:
            class Dummy:
                def __init__(self, mod: int, ch: int) -> None:
                    self.mod = mod
                    self.ch = ch
                    self.state = 0

                def Set(self, value: int) -> None:
                    self.state = value

                def Get(self) -> int:
                    return self.state

                def Toggle(self) -> None:
                    self.state = not self.state

            self.hw = Dummy(module, channel)

    def Set(self, value: int) -> None:
        try:
            self.hw.set(bool(value))
        except Exception:
            self.hw.Set(value)

    def Get(self) -> int:
        try:
            return int(self.hw.get())
        except Exception:
            return self.hw.Get()

    def Toggle(self) -> None:
        try:
            self.hw.toggle()
        except Exception:
            self.hw.Toggle()

Solenoid.RegisterBackend("WPI", "Single Solenoid", SingleSolenoid)

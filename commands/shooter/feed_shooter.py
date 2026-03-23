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

from subsystems import *
from wpilib import Joystick
from commands2 import Command

class FeedShooter(Command):
    def __init__(self, feeder: Feeder, controller: Joystick) -> None:
        super().__init__()
        self.feeder = feeder
        self.controller = controller

        self.addRequirements(self.feeder)

    def execute(self) -> None:
        if self.controller.getRawButton(1):
            self.feeder.start(1)
        elif self.controller.getRawButton(9):
            self.feeder.start(-1)
        else:
            self.feeder.stop()

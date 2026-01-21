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

from robot_container import RobotContainer
from typing import override
from commands2 import TimedCommandRobot, CommandScheduler
from urcl import URCL

class MyRobot(TimedCommandRobot):
    def __init__(self) -> None:
        super().__init__()

        URCL.start()

    @override
    def robotInit(self) -> None:
        self.robot = RobotContainer()
        self.autoCmd = None

    @override
    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    @override
    def autonomousInit(self) -> None:
        self.autoCmd = self.robot.getAutonomousCommand()
        if self.autoCmd:
            self.autoCmd.schedule()

    @override
    def autonomousPeriodic(self) -> None:
        pass

    @override
    def autonomousExit(self) -> None:
        pass

    @override
    def teleopInit(self) -> None:
        if self.autoCmd:
            self.autoCmd.cancel()

    @override
    def teleopPeriodic(self) -> None:
        pass

    @override
    def teleopExit(self) -> None:
        pass

    @override
    def disabledInit(self) -> None:
        pass

    @override
    def disabledPeriodic(self) -> None:
        pass

    @override
    def disabledExit(self) -> None:
        pass

    @override
    def testInit(self) -> None:
        pass

    @override
    def testPeriodic(self) -> None:
        pass

    @override
    def testExit(self) -> None:
        pass

    @override
    def _simulationInit(self) -> None:
        pass

    @override
    def _simulationPeriodic(self) -> None:
        pass

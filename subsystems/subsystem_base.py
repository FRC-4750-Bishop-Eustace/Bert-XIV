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

from typing import override
from abc import abstractmethod
from commands2 import Subsystem, Command, InstantCommand, CommandScheduler
from wpiutil import Sendable, SendableBuilder, SendableRegistry

class SubsystemBase(Subsystem, Sendable):
    def __init__(self, name: str = None) -> None:
        SendableRegistry.addLW(self, name, name)

        # Reload the command scheduler
        CommandScheduler.getInstance().unregisterSubsystem(self)
        CommandScheduler.getInstance().registerSubsystem(self)

        self.currentCommand: Command = Command().withName("[none]")

    @override
    def getName(self) -> str:
        return SendableRegistry.getName(self)
    def SetName(self, name: str) -> None:
        SendableRegistry.setName(self, name)

    def GetSubsystem(self) -> str:
        return SendableRegistry.getSubsystem(self)
    def SetSubsystem(self, subsystem: str) -> None:
        SendableRegistry.setSubsystem(self, subsystem)

    def AddChild(self, name: str, child: Sendable) -> None:
        SendableRegistry.addLW(child, GetSubsystem(), name)

    @override
    def getCurrentCommand(self) -> Command:
        return self.currentCommand
    def SetCurrentCommand(self, command: Command) -> None:
        self.currentCommand = command

    @override
    def periodic(self) -> None:
        if self.getDefaultCommand() is not None:
            print(f"\033[1;34mDefault Command: {self.getDefaultCommand().getName()}\033[0m")
        self.SubsystemPeriodic()

    @abstractmethod
    def SubsystemPeriodic(self) -> None:
        raise NotImplementedError

    @override
    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("Subsystem")

        builder.addBooleanProperty(".hasDefault", lambda: getDefaultCommand() is not None, None)
        builder.addStringProperty(".default", lambda: getDefaultCommand().getName() if getDefaultCommand() is not None else "[none]", None)
        builder.addBooleanProperty(".hasCommand", lambda: getCurrentCommand() is not None, None)
        builder.addStringProperty(".command", lambda: getCurrentCommand().getName() if getCurrentCommand() is not None else "[none]", None)

    def AsSubsystemCommand(self, commad: Command, name: str) -> Command:
        command.SetName(name)
        command.addRequirements(self)
        return command.beforeStarting(InstantCommand(lambda: self.SetDefaultCommand(command)))

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

import constants
import time
from wpilib import DriverStation, PowerDistribution, RobotController, SmartDashboard

class RobotHealth:
    def __init__(self, pdhId: int = 1, pdhType: PowerDistribution.ModuleType = PowerDistribution.ModuleType.kCTRE) -> None:
        self.pdh = PowerDistribution(pdhId, pdhType)
        self.enabled = True
        self.fatal = False
        self.fatalMsg = ""
        self.lastMsgTime = 0.0
        self.channelOvercurrentTime = [0.0] * 24
        self.brownedOut = False

    def update(self) -> None:
        if self.fatal:
            return

        voltage = RobotController.getBatteryVoltage()
        brownout = RobotController.isBrownedOut()
        temp = RobotController.getCPUTemp()
        can = RobotController.getCANStatus()

        if voltage < constants.voltageCritical:
            self.triggerFatalFault(f"Critical battery voltage: {voltage:.2f}V")
        elif voltage < constants.voltageWarn:
            self.triggerWarnFault(f"Low battery voltage: {voltage:.2f}V")

        if brownout and not self.brownedOut:
            self.triggerWarnFault("RoboRIO brownout detected")
        self.brownedOut = brownout

        if temp > constants.cpuTempCritical:
            self.triggerFatalFault(f"Critical CPU temp: {temp:.2f}C")
        elif temp > constants.cpuTempWarn:
            self.triggerWarnFault(f"High CPU temp: {temp:.2f}C")

        if can.percentBusUtilization > constants.canUsageWarn:
            self.triggerWarnFault(f"High CAN bus utilization: {can.percentBusUtilization * 100:.0f}%")
        if can.txFullCount > 0:
            self.triggerWarnFault("CAN TX buffer full")

        SmartDashboard.putBoolean("Health/FatalFault", self.fatal)
        SmartDashboard.putString("Health/FatalMessage", self.fatalMsg)
        SmartDashboard.putNumber("Health/BatteryVoltage", voltage)
        SmartDashboard.putNumber("Health/CPUTemp", temp)
        SmartDashboard.putNumber("Health/CANUsage", can.percentBusUtilization)

    def isSafe(self) -> bool:
        return not self.fatal

    def enforceShutdown(self) -> None:
        self.triggerFatalFault("Manual Shutdown Required")

    def triggerFatalFault(self, msg: str) -> None:
        if self.fatal:
            return

        self.fatal = True
        self.fatalMsg = msg

        DriverStation.reportError(f"Robot fault: {msg}", False)
        print(f"\033[31;1mRobot fault: {msg}\033[0m")

    def triggerWarnFault(self, msg: str) -> None:
        if self.fatal:
            return

        now = time.monotonic()
        if now - self.lastMsgTime >= constants.messageInterval:
            DriverStation.reportWarning(f"Robot warning: {msg}", False)
            print(f"\033[33;1mRobot warning: {msg}\033[0m")
            self.lastMsgTime = now

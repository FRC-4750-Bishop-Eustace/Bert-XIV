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

from abc import abstractmethod
import math
from ..subsystem_base import SubsystemBase
from ..vision import *
from hardware import *
import wpimath.geometry

class DrivetrainConfig:
    def __init__(self):
        self.maxSpeed: float = 3.0
        self.maxAngularSpeed: float = math.pi
        self.trackWidth: float = 0.762
        self.wheelRadius: float = 0.0508
        self.encoderResolution: float = 4096
        self.maxAngularVelocity: float = math.pi
        self.maxAngularAcceleration: float = math.tau

        class PID:
            def __init__(self, kP: float, kI: float, kD: float):
                self.kP = kP
                self.kI = kI
                self.kD = kD

        class Feedforward:
            def __init__(self, kS: float, kV: float, kA: float = 0.0):
                self.kS = kS
                self.kV = kV
                self.kA = kA

        self.pid: list[PID] = [
            PID(0.02, 0.0, 0.0),
            PID(7.8, 0.0, 0.055)
        ]
        self.feedforward: list[Feedforward] = [
            Feedforward(0.06, 2.5),
            Feedforward(0.0, 0.0)
        ]
        self.locations: list[wpimath.geometry.Translation2d] = [
            wpimath.geometry.Translation2d(+0.381, +0.381),
            wpimath.geometry.Translation2d(+0.381, -0.381),
            wpimath.geometry.Translation2d(-0.381, +0.381),
            wpimath.geometry.Translation2d(-0.381, -0.381)
        ]

        self.slewRates = [3.0, 3.0, 1.0]
        self.deadbands = [0.1, 0.1, 0.2]

class Module:
    def __init__(self, motor: Motor, encoder: Encoder):
        self.motor = motor
        self.encoder = encoder

class Drivetrain(SubsystemBase):
    def __init__(self, config: DrivetrainConfig, modules: list[Module], imu: IMU, vision: Vision):
        super().__init__("Drivetrain")

    @abstractmethod
    def Drive(self, x: float, y: float, theta: float, fieldRelative: bool, period: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def UpdateOdometry(self, data: VisionData) -> None:
        raise NotImplementedError

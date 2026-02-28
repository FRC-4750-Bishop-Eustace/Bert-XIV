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

import math
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController, ProfiledPIDController, SimpleMotorFeedforwardMeters, SimpleMotorFeedforwardRadians, ElevatorFeedforward
from wpimath.trajectory import TrapezoidProfile

# Robot health
voltageWarn = 10.5
voltageCritical = 9.3
cpuTempWarn = 75.0
cpuTempCritical = 85.0
canUsageWarn = 0.80
messageInterval = 1.0

# Input ports
ps4ControllerPort = 2
dashboardPort = 0

# Speed config
maxSpeed = 4.6
rMaxSpeed = 8.5
tMaxSpeed = 4.6
maxAngularSpeed = math.pi
frontLeftZero = 0
frontRightZero = 0
backLeftZero = 0
backRightZero = 0
zeroThreshold = Rotation2d(0.3)

# Chassis config
chassisHalfLength = 0.314
wheelRadius = 0.0492
driveReduction = 425 / 63
encoderResolution = 4096
vEncoderResolution = 42
maxAngularVelocity = 1000
maxAngularAcceleration = 1000

frontLeftDriveMotorId = 1
frontLeftTurnMotorId = 8
frontLeftTurnEncoderId = 12
frontRightDriveMotorId = 4
frontRightTurnMotorId = 5
frontRightTurnEncoderId = 9
backRightDriveMotorId = 7
backRightTurnMotorId = 6
backRightTurnEncoderId = 10
backLeftDriveMotorId = 2
backLeftTurnMotorId = 3
backLeftTurnEncoderId = 11

# Shooter config
shooterMotor1Id = 14
shooterMotor2Id = 17
shooterFeederMotorId = 18
shooterSpeed = 6

# Intake config
intakeActuatorMotorId = 13
intakeLeftMotorId = 16
intakeRightMotorId = 15 
intakeSpeed = 6
actuatorSpeed = 12
actuatorMaxRotations = 5

# PID and feed-forward
class PID:
    def __init__(self, kP, kI, kD) -> None:
        self.kP = kP
        self.kI = kI
        self.kD = kD

    def toPIDController(self) -> PIDController:
        return PIDController(
            self.kP,
            self.kI,
            self.kD
        )

    def toProfiledPIDController(self) -> ProfiledPIDController:
        return ProfiledPIDController(
            self.kP,
            self.kI,
            self.kD,
            TrapezoidProfile.Constraints(
                maxAngularVelocity,
                maxAngularAcceleration
            )
        )

class FeedForward:
    def __init__(self, kS, kV, kA=0, kG=0) -> None:
        self.kS = kS
        self.kV = kV
        self.kA = kA
        self.kG = kG

    def toMotorFeedforwardMeters(self) -> SimpleMotorFeedforwardMeters:
        return SimpleMotorFeedforwardMeters(self.kS, self.kV, self.kA)

    def toMotorFeedforwardRadians(self) -> SimpleMotorFeedforwardRadians:
        return SimpleMotorFeedforwardRadians(self.kS, self.kV, self.kA)

    def toElevatorFeedforward(self) -> ElevatorFeedforward:
        return ElevatorFeedforward(self.kS, self.kG, self.kV, self.kA)

swerveDrivePID = PID(0.02, 0, 0)
swerveDriveFF = FeedForward(0, 2.2)
swerveTurnPID = PID(7.8, 0, 0.055)
swerveTurnFF = FeedForward(0, 0)
swerveAutoDrivePID = PID(0.4, 0, 0.001)
swerveAutoTurnPID = PID(1.3, 0, 0.1)

# Controllers
xSlewRate = 3
ySlewRate = 3
rotSlewRate = 1
xDeadband = 0.1
yDeadband = 0.1
rotDeadband = 0.2

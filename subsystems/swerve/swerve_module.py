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

from hardware import *
import constants
import math
import wpimath
from wpimath.controller import PIDController, ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrapezoidProfile
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.geometry import Rotation2d

class SwerveModule:
    def __init__(self, loader: Loader, driveMotorId: int, turnMotorId: int, turnEncoderId: int) -> None:
        self.driveMotor = loader.CreateMotor(MotorType.kSparkMAX, deviceId=driveMotorId, typ=MotorMode.kBrushless, inverted=False)
        self.driveEncoder = loader.CreateEncoder(EncoderType.kAbsoluteEncoder, motor=self.driveMotor)
        self.turnMotor = loader.CreateMotor(MotorType.kSparkMAX, deviceId=turnMotorId, typ=MotorMode.kBrushless, inverted=False)
        self.turnEncoder = loader.CreateEncoder(EncoderType.kCANcoder, deviceId=turnEncoderId, canbus="rio")

        self.driveMotor.SetParameters(
            False,
            IdleMode.kCoast,
            (math.tau * constants.wheelRadius / 60) / constants.driveReduction,
            (constants.wheelRadius * math.tau) / constants.driveReduction
        )
        self.turnMotor.SetParameters(
            True,
            IdleMode.kCoast
        )

        self.drivePID = constants.swerveDrivePID.toPIDController()
        self.turnPID = constants.swerveTurnPID.toProfiledPIDController()
        self.driveFF = constants.swerveDriveFF.toMotorFeedforwardMeters()
        self.turnFF = constants.swerveTurnFF.toMotorFeedforwardMeters()

        self.turnPID.enableContinuousInput(-math.pi, math.pi)

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.driveEncoder.GetVelocity(),
            Rotation2d.fromRotations(self.turnEncoder.GetPosition())
        )

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
                self.driveEncoder.GetPosition(),
                Rotation2d.fromRotations(self.turnEncoder.GetPosition())
        )

    def setDesiredState(self, state: SwerveModuleState) -> None:
        rots = Rotation2d.fromRotations(self.turnEncoder.GetPosition())

        state.optimize(rots)
        state.cosineScale(rots)

        driveOutput = self.drivePID.calculate(
            self.driveEncoder.GetVelocity(),
            state.speed
        )
        driveOutput += self.driveFF.calculate(
            state.speed
        )

        turnOutput = self.turnPID.calculate(
            wpimath.units.rotationsToRadians(self.turnEncoder.GetPosition()),
            state.angle.radians()
        )
        turnOutput += self.turnFF.calculate(
            self.turnPID.getSetpoint().velocity
        )

        self.driveMotor.SetVoltage(driveOutput)
        self.turnMotor.SetVoltage(turnOutput)

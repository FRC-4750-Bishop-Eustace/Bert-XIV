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

from enum import Enum

class DeviceType(Enum):
    kMotor = "motor"
    kEncoder = "encoder"
    kIMU = "imu"
    kCamera = "camera"
    kSolenoid = "solenoid"

class MotorType(Enum):
    kTalonFX = "CTRE_TalonFX"
    kTalonFXS = "CTRE_TalonFXS"
    kSparkMAX = "REV_SparkMAX"
    kSparkFlex = "REV_SparkFlex"

class MotorMode(Enum):
    kBrushless = "Brushless"
    kBrushed = "Brushed"

class IdleMode(Enum):
    kCoast = 0
    kBrake = 1

class EncoderType(Enum):
    kEncoder = "WPI_Encoder"
    kCANcoder = "CTRE_CANcoder"
    kAbsoluteEncoder = "REV_AbsoluteEncoder"

class IMUType(Enum):
    kADIS16470 = "WPI_ADIS16470"
    kPigeon2 = "CTRE_Pigeon2"
    kNavX = "KauaiLabs_NavX"

class CameraType(Enum):
    kUSB = "WPI_USBCamera"
    kLimelight = "Hailo_Limelight"
    kPhotonVision = "Chameleon_PhotonVision"

class SolenoidType(Enum):
    kSingle = "WPI_SingleSolenoid"
    kDouble = "WPI_DoubleSolenoid"

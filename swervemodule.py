#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
from phoenix6 import hardware
#from phoenix6.hardware import CANCoder
import phoenix6.hardware
from rev import SparkMax, SparkMaxConfig, SparkBase
import wpimath.units
import drivetrain
import variables
from wpilib import DriverStation
from wpilib import SmartDashboard



#1.9375
kWheelRadius = 0.0492 # In meters
kDriveReduction = 425 / 63
kEncoderResolution = 4096
kVEncoderResolution = 42 #Need to confirm
kModuleMaxAngularVelocity = 1000#math.pi * 10  #need to understand how this applies to the motor
kModuleMaxAngularAcceleration = 1000#math.tau * 10 #need to understand how this applies to the motor


class SwerveModule:
    def __init__(
        self,
        driveMotorID: int,
        turningMotorID: int,
        driveEncoderID: int,
        turningEncoderID: int,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        BERT NOTES:

        - Using different encoder types, current module assumes MXP quadrature encoder that takes 2 channels.  
        - Will need to modify to enable encoders based on CAN ID.
        
        Front Left (+,+) 
            - Drive Motor: 4
            - Rotation Motor: 3
            - Drive Encoder: 13
            - Rotation Encoder: 3 
        
        Front Right (+,-) 
            - Drive Motor: 7
            - Rotation Motor: 8
            - Drive Encoder: 10
            - Rotation Encoder: 8

        Rear Left (-,+) 
            - Drive Motor: 2
            - Rotation Motor: 1
            - Drive Encoder: 11
            - Rotation Encoder: 1

        Rear Right (-,-) 
            - Drive Motor: 5
            - Rotation Motor: 6
            - Drive Encoder: 12
            - Rotation Encoder: 6

        """

        # NOTE: need to confirm output from SparkMaxAbsoluteEncoder - may shift to Relative Encoder
        ## No longer required 
        self.driveMotor = SparkMax(driveMotorID, SparkMax.MotorType.kBrushless)
        self.turningMotor = SparkMax(turningMotorID, SparkMax.MotorType.kBrushless)
        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = hardware.CANcoder(turningEncoderID, "rio")
        self.setParameters(SparkMaxConfig.IdleMode.kBrake)
        
        #print(turningEncoderID, self.turningEncoder.getAbsolutePosition())
        #self.turningEncoder.configFeedbackCoefficient((2 * math.pi / 4096), "rad", hardware.SensorTimeBase(1))
        #self.turningEncoder.configAbsoluteSensorRange(phoenix5.sensors.AbsoluteSensorRange(1))
        #self.turningEncoder.configSensorDirection(1)

        #print("setting:", turningEncoderID, self.turningEncoder.getAbsolutePosition())
        # NOTE: can we use the wpilib.encoder library for these encoders - may need to review

        # NOTE: This is the values we need to tweak 99, 102, 114, & 115
        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(variables.drivePID_P, variables.drivePID_I, variables.drivePID_D)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            variables.turnPID_P,
            variables.turnPID_I,
            variables.turnPID_D,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration, #How is the contraint applied
            ),
        )


        #self.turningPIDController.reset(wpimath.kinematics.SwerveModuleState(self.driveEncoder.getVelocity(),wpimath.geometry.Rotation2d.fromRotations(self.turningEncoder.get_position().value)))

        # Gains are for example purposes only - must be determined for your own robot!
        # NOTE: To review
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(variables.driveFF_1, variables.driveFF_2)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(variables.turnFF_1, variables.turnFF_2)

        # Set the distance per pulse for the drive encoder. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # resolution.

        # NOTE: Need to determine if this is the right distance per pulse value (from user guide it shows 42 counts per revolution)
        #self.driveEncoder.setVelocityConversionFactor(
        #    math.tau * kWheelRadius / kVEncoderResolution
        #    #(kWheelRadius * (4 * math.pi)) / kDriveReduction
        #)

        # self.driveEncoder.setPositionConversionFactor(
        #     (kWheelRadius * (4 * math.pi)) / kDriveReduction
        # )

        # Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        # This is the the angle through an entire rotation (2 * pi) divided by the
        # encoder resolution.

        # NOTE: Need to determine if this is the right distance per pulse value
        #self.turningEncoder.setStatusFramePeriod(math.tau / kEncoderResolution)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)
        #print("turn encoder", self.turningEncoder.get_position().value)

    def setParameters(self, mode) -> None:
        driveConfig = SparkMaxConfig()
        driveConfig.setIdleMode(mode)
        driveConfig.encoder.velocityConversionFactor((math.tau * kWheelRadius / 60)/kDriveReduction)
        driveConfig.encoder.positionConversionFactor((kWheelRadius * (2 * math.pi)) / kDriveReduction)
        self.driveMotor.disable()
        self.driveMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        turningConfig = SparkMaxConfig()
        turningConfig.inverted(True)
        turningConfig.setIdleMode(mode)
        self.turningMotor.disable()
        self.turningMotor.configure(turningConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    
    
    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        #print("Get State:", self.turningEncoder.getDeviceNumber, self.turningEncoder.get_position())
        #wpimath.units.rotationsToRadians(self.turningEncoder.get_position().value)
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d.fromRotations(self.turningEncoder.get_position().value),
        )


    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        #print("Get Position:", self.turningEncoder.getDeviceNumber, self.turningEncoder.get_position())
        return wpimath.kinematics.SwerveModulePosition(
            #self.driveEncoder.getVelocity(),
            self.driveEncoder.getPosition(),
            wpimath.geometry.Rotation2d.fromRotations(self.turningEncoder.get_position().value),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """
        #NOTE: Need to determine what the right units are for the rotation to be set - most likely Radians
        encoderRotation = wpimath.geometry.Rotation2d.fromRotations(self.turningEncoder.get_position().value)

        desiredState.optimize(encoderRotation)
        desiredState.cosineScale(encoderRotation)

        # Optimize the reference state to avoid spinning further than 90 degrees
        #state = wpimath.kinematics.SwerveModuleState.optimize(
        #    desiredState, encoderRotation
        #)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        #state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getVelocity(), desiredState.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(desiredState.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            wpimath.units.rotationsToRadians(self.turningEncoder.get_position().value), desiredState.angle.radians() 
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )
        
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)
        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
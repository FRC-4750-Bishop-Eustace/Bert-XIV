#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import navx
import wpilib
import wpimath.geometry
from wpimath.geometry import Pose2d
import wpimath.kinematics
import swervemodule
import variables
from wpilib import SmartDashboard
import ntcore
from wpimath.kinematics import SwerveModuleState
import time
import wpimath.estimator
from wpimath.estimator import SwerveDrive4PoseEstimator
import LimelightHelpers

'''
kMaxSpeed = 1.5  # meters per second
kRMaxSpeed = 0.1
kTMaxSpeed = 1.0
kMaxAngularSpeed = math.pi  # 1/2 rotation per second
frontLeftZero = 0
frontRightZero = 0
backLeftZero = 0
backRightZero = 0
zeroThreshold = wpimath.geometry.Rotation2d(0.3)
'''

class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        # NOTE: May need to tweak the center measurements 44.5mm rough measurement
        # NOTE: EVERYTHING IS MEASURE IN METERS! 
        # NOTE: Update center measure distance from each module
        # Wheel to Wheel = 63.4 cm
        # Chassis = 76cm
        '''
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.32, 0.32)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.32, -0.32)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.32, -0.32)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.32, 0.32)
        '''

        '''
        self.frontLeftLocation = wpimath.geometry.Translation2d(-variables.chassisHalfLength, variables.chassisHalfLength)
        self.frontRightLocation = wpimath.geometry.Translation2d(-variables.chassisHalfLength, -variables.chassisHalfLength)
        self.backRightLocation = wpimath.geometry.Translation2d(variables.chassisHalfLength, -variables.chassisHalfLength)
        self.backLeftLocation = wpimath.geometry.Translation2d(variables.chassisHalfLength, variables.chassisHalfLength)
        '''
        self.frontLeftLocation = wpimath.geometry.Translation2d(variables.chassisHalfLength, variables.chassisHalfLength)
        self.frontRightLocation = wpimath.geometry.Translation2d(variables.chassisHalfLength, -variables.chassisHalfLength)
        self.backRightLocation = wpimath.geometry.Translation2d(-variables.chassisHalfLength, -variables.chassisHalfLength)
        self.backLeftLocation = wpimath.geometry.Translation2d(-variables.chassisHalfLength, variables.chassisHalfLength)

        self.frontLeft = swervemodule.SwerveModule(variables.frontLeftDriveController, variables.frontLeftTurnController, variables.frontLeftDriveEncoder, variables.frontLeftTurnEncoder)
        self.frontRight = swervemodule.SwerveModule(variables.frontRightDriveController, variables.frontRightTurnController, variables.frontRightDriveEncoder, variables.frontRightTurnEncoder)
        self.backRight = swervemodule.SwerveModule(variables.backRightDriveController, variables.backRightTurnController, variables.backRightDriveEncoder, variables.backRightTurnEncoder)
        self.backLeft = swervemodule.SwerveModule(variables.backLeftDriveController, variables.backLeftTurnController, variables.backLeftDriveEncoder, variables.backLeftTurnEncoder)

        SmartDashboard.putData("TurnPID", self.frontLeft.turningPIDController)
        
        #SmartDashboard.putData("TurnFF", self.frontLeft.turnFeedforward)

        nt = ntcore.NetworkTableInstance.getDefault()
        topic = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        commandState = nt.getStructArrayTopic("/CommandStates", SwerveModuleState)
        self.pub = topic.publish()
        self.cmd = commandState.publish()
        
        '''
        BERT NOTES:
        
        driveMotorID: int,
        turningMotorID: int,
        driveEncoderID: int,
        turningEncoderID: int,

        Front Left (+,+) 
            - Drive Motor: 4
            - Rotation Motor: 3
            - Drive Encoder: 4
            - Rotation Encoder: 13 
        
        Front Right (+,-) 
            - Drive Motor: 7
            - Rotation Motor: 8
            - Drive Encoder: 7
            - Rotation Encoder: 10

        Rear Left (-,+) 
            - Drive Motor: 2
            - Rotation Motor: 1
            - Drive Encoder: 2
            - Rotation Encoder: 11

        Rear Right (-,-) 
            - Drive Motor: 5
            - Rotation Motor: 6
            - Drive Encoder: 5
            - Rotation Encoder: 12

        '''

        #self.gyro = wpilib.AnalogGyro(0)
        self.angler = navx.AHRS.create_spi()
        #print("gyroscope = ", self.angler)
        self.gyroinit = self.angler.getAngle()
        self.gyroradiansinit = wpimath.units.degreesToRadians(self.gyroinit)
        #self.resetPose = self.odometry.resetPose(self.odometry.getPose())
        #self.getRobotRelativeSpeeds = wpilib.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(self.xSpeed, self.ySpeed, self.#rot, wpimath.geometry.Rotation2d(self.gyroradians))
        #print(self.getRobotRelativeSpeeds)
        # print("gyro", self.gyro)
        #self.swervemodule = swervemodule.SwerveModule()

        self.x_controller = wpimath.controller.PIDController(variables.drivePID_P, variables.drivePID_I, variables.drivePID_D)
        self.y_controller = wpimath.controller.PIDController(variables.drivePID_P, variables.drivePID_I, variables.drivePID_D)
        self.heading_controller = wpimath.controller.ProfiledPIDController(
            1.5,
            0,
            0.043,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                1000,
                1000, #How is the contraint applied
            ),
        )

        self.x_controller2 = wpimath.controller.PIDController(0.4, 0, 0.001)
        self.y_controller2 = wpimath.controller.PIDController(0.4, 0, 0.001)
        self.heading_controller2 = wpimath.controller.PIDController(1.3, 0, 0.1)
        # 0.5, 0, 0.5

        SmartDashboard.putData("A-PIDx", self.x_controller2)
        SmartDashboard.putData("A-PIDy", self.y_controller2)
        SmartDashboard.putData("A-PIDtune", self.heading_controller2)
        #SmartDashboard.putData("A-PIDprof", self.heading_controller)

        #NOTE: Just defining the fixed kinematics of the bot
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backRightLocation,
            self.backLeftLocation,
        )

        #NOTE: getPosition - need to determine position value - velocity and angle -
        #NOTE: Need to understand expected units/values returned - is it meters & radians?
        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            wpimath.geometry.Rotation2d(self.gyroradiansinit),
            #self.angler.getAngle(),
            #self.angler.getRotation2d(),
            #self.gyro.Translation2d(),
            #self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backRight.getPosition(),
                self.backLeft.getPosition(),
            ),
        )

        self.estimator = wpimath.estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.angler.getRotation2d(),
            [
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backRight.getPosition(),
                self.backLeft.getPosition(),
            ],
            self.odometry.getPose(),
            ([0.1, 0.1, 0.1]),
            ([1.0, 1.0, 5.0])
        )

        self.angler.reset()

        self.timer = wpilib.Timer()
        self.timer.reset()
        self.timer.start()
        self.period = self.timer.get()
        #self.heading_controller.enableContinuousInput(-math.pi, math.pi)

    def setParameters(self, mode) -> None:
        self.frontLeft.setParameters(mode)
        self.frontRight.setParameters(mode)
        self.backLeft.setParameters(mode)
        self.backRight.setParameters(mode)

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """

        #NOTE
        self.xSpeed = xSpeed
        self.ySpeed = ySpeed
        self.rot = rot

        #if fieldRelative:
        #self.updateOdometry()

        self.gyro = self.angler.getAngle()
        self.gyroradians = wpimath.units.degreesToRadians(self.gyro)

        #print("gyro", self.gyroradians)

        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, wpimath.geometry.Rotation2d(-self.gyroradians)
                )
                if fieldRelative
                else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, variables.kMaxSpeed
        )
        
        #NOTE: Should we desaturate for Turning speed motors? 
        
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backRight.setDesiredState(swerveModuleStates[2])
        self.backLeft.setDesiredState(swerveModuleStates[3])

        self.pub.set([self.frontLeft.getState(),self.frontRight.getState(),self.backLeft.getState(),self.backRight.getState()])

        self.cmd.set([
            swerveModuleStates[0],
            swerveModuleStates[1],
            swerveModuleStates[3],
            swerveModuleStates[2]
        ])

        SmartDashboard.putNumber("frontLeftDriveVolt", (self.frontLeft.driveMotor.getAppliedOutput() * self.frontLeft.driveMotor.getAppliedOutput()))
        SmartDashboard.putNumber("navxRate", self.angler.getRate())

    
    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        SmartDashboard.putNumber("rOTATION", self.angler.getAngle())
        self.odometry.update(
            #wpimath.geometry.Rotation2d(self.gyroradiansinit),
            self.angler.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backRight.getPosition(),
                self.backLeft.getPosition(),
            ),
        )
                
    #NOTE
    def follow_trajectory(self, sample):
        # Get the current pose of the robot
        pose = self.getPose() #NOTE: tuning PID for auto choreo/try choreo without the wood blocks
        # Generate the next speeds for the robot
        speeds = wpimath.kinematics.ChassisSpeeds(
            sample.vx + self.x_controller2.calculate(pose.X(), sample.x),
            sample.vy + self.y_controller2.calculate(pose.Y(), sample.y),
            sample.omega + self.heading_controller2.calculate(pose.rotation().radians(), sample.heading)
        )

        # Apply the generated speeds
        self.drive(speeds[0], speeds[1], speeds[2], True, self.period)
        #print("pose =", self.pose)
        #print("0 =", speeds[0])
        #print("1 =", speeds[1])
        #print("2 =", speeds[2])

    def resetRobotPose(self, pose):
        self.estimator.resetPosition(
            self.angler.getRotation2d(),
            [
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backRight.getPosition(),
                self.backLeft.getPosition(),
            ],
            pose
        )
    
    def resetGyro(self):
        self.angler.reset()
    
    def getGyroAngle(self):
        self.angle = self.angler.getRotation2d()
        return self.angle
    
    def UpdateEstimator(self):
        self.estimator.update(
            self.angler.getRotation2d(),
            [
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backRight.getPosition(),
                self.backLeft.getPosition(),
            ]
        )
    
    def addVision(self):
        mt1 = LimelightHelpers.PoseEstimate.getRobotPoseEstimateBlueMT1("limelight")
        LimelightHelpers.setRobotOrientation(
            "limelight",
            self.getRotation().degrees(),
            0,
            0,
            0,
            0,
            0
        )
        mt2 = LimelightHelpers.PoseEstimate.getRobotPoseEstimateBlueMT2("limelight")

        gyro_rate = abs(self.angler.getRate())
        mt1_valid = False
        mt1_std = [1.0, 1.0, 9999999]

        if mt1.tagCount > 0:
            if mt1.tagCount >= 2:
                mt1_valid = True
                mt1_std = [0.4, 0.4, 9999999]
            elif len(mt1.fiducials) == 1:
                fid = mt1.fiducials[0]
                if fid.ambiguity < 0.8 and fid.cameraDistace < 4.0:
                    mt1_valid = True
                    scale = min(fid.cameraDistace / 4.0, 1.0)
                    mt1_std = [
                        0.5 + 0.7 * scale,
                        0.5 + 0.7 * scale,
                        9999999
                    ]

        mt2_valid = (mt2.tagCount >= 2 and gyro_rate < 360)
        mt2_std = [
            0.6 + 0.002 * gyro_rate,
            0.6 + 0.002 * gyro_rate,
            9999999
        ]

        if mt2_valid:
            self.estimator.setVisionMeasurementStdDevs(mt2_std)
            self.estimator.addVisionMeasurement(mt2.pose, mt2.timestamp)
        elif mt1_valid:
            self.estimator.setVisionMeasurementStdDevs(mt1_std)
            self.estimator.addVisionMeasurement(mt1.pose, mt1.timestamp)

    def getPose(self) -> wpimath.geometry.Pose2d:
        return self.estimator.getEstimatedPosition()
    
    def getRotation(self) -> wpimath.geometry.Rotation2d:
        return self.getPose().rotation()



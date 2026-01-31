#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import wpimath.estimator
import drivetrain
import variables
import elevator
import elevator2
import navxGyro
import ntcore
import commands2
#import limelight
#import limelightresults
import LimelightHelpers
import json
import time
#import limelight
#import vision
#import camera
#import auto
import ntcore
from wpilib import SmartDashboard, Field2d
from cscore import CameraServer
from wpilib import SmartDashboard
import choreo
# import urcl
from rev import SparkMaxConfig

#wpilib.TimedRobot
class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.Joystick(variables.joystickPort1)
        self.controller2 = wpilib.Joystick(variables.joystickPort2)
        self.pad = wpilib.Joystick(variables.joystickPort3)

        self.swerve = drivetrain.Drivetrain()
        #self.odometry = 

        #self.elevator = elevator.Elevator(16, 17, [100, 200, 300, 400])
        #self.elevator2 = elevator2.Elevator(18)
        #self.limelight = limelight.PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, avgTagDist, avgTagArea, fiducials)

        # navxGyro is a file to test the navx Gyro. This can be ignored/commented out.
        self.navxGyro = navxGyro.Gyro()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        # Speed limiters

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(variables.x_slewrate)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(variables.y_slewrate)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(variables.rot_slewrate)

        self.timer = wpilib.Timer()
        self.fieldDrive = 1
        #CameraServer.startAutomaticCapture()
         
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.lmtable = self.inst.getTable("limelight")

        #SmartDashboard.putData("Swerve", self.swerve)
        
        # Loads from deploy/choreo/myTrajectory.traj
        # ValueError is thrown if the file does not exist or is invalid
        
        try:
            self.trajectory = choreo.load_swerve_trajectory("NewPath8") 
        except ValueError as e:
        # If the trajectory is not found, ChoreoLib already prints to DriverStation
            print(f"\033[31;1m{e}\033[0m")
            self.trajectory = None

        self.field = Field2d()
        SmartDashboard.putData("field", self.field)
        # self.field.getObject("traj").setTrajectory(self.trajectory)
              
        wpilib.DataLogManager.start()

    def robotPeriodic(self):
        # self.swerve.updateOdometry()
        self.swerve.UpdateEstimator()
        self.swerve.addVision()

        self.field.setRobotPose(self.swerve.getPose())

    #FUTURE
    def autonomousInit(self):
        self.swerve.setParameters(SparkMaxConfig.IdleMode.kBrake)
        self.swerve.resetGyro()
        # self.swerve.resetRobotPose(Pose2d(0, 0, 0))

        #sample1 = self.trajectory.sample_at(self.timer.get(), self.is_red_alliance())
        #sample2 = self.trajectory2.sample_at(self.timer.get(), self.is_red_alliance())

        follow1 = commands2.cmd.run(lambda: self.FollowChoreoPath(self.trajectory)).withTimeout(3) #6.7
        # follow2 = commands2.cmd.run(lambda: self.FollowChoreoPath(self.trajectory2)).withTimeout(4.1)
        stop = commands2.cmd.run(lambda: self.StopPath())
        
        if self.trajectory:
            # Get the initial pose of the trajectory
            initial_pose = self.trajectory.get_initial_pose(self.is_red_alliance())
            self.swerve.resetRobotPose(initial_pose)

        self.path_command = commands2.SequentialCommandGroup([
            follow1,
            # follow2,
            stop
        ])
        self.path_command.schedule()

        # Reset and start the timer when the autonomous period begins
        self.timer.restart()

        #self.autoSelected = self.chooser.getSelected()
        #print("Auto selected:" + self.autoSelected)

    def autonomousPeriodic(self) -> None:

        self.matchTimer = self.timer.getMatchTime()

        # self.field.setRobotPose(self.swerve.getPose())

        commands2.CommandScheduler.getInstance().run()

        '''
        if self.trajectory:
            # Sample the trajectory at the current time into the autonomous period
            sample = self.trajectory.sample_at(self.timer.get(), self.is_red_alliance())

            if sample:
                self.swerve.follow_trajectory(sample)
                #print(self.trajectory.get_final_pose())
            
            #print(self.trajectory.get_total_time())
        
        if self.trajectory2:
            # Sample the trajectory at the current time into the autonomous period
            sample2 = self.trajectory2.sample_at(self.timer.get(), self.is_red_alliance())

            if sample2:
                self.swerve.follow_trajectory(sample2)
                #print(self.trajectory.get_final_pose())
            
            #print(self.trajectory.get_total_time())
        '''
    def is_red_alliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def teleopInit(self) -> None:
        self.swerve.setParameters(SparkMaxConfig.IdleMode.kCoast)

    def teleopPeriodic(self) -> None:

        #self.swerve.updateOdometry()
        #self.field.setRobotPose(self.swerve.odometry.getPose())

        #rint(self.getPeriod())
        #print(wpimath.geometry.Pose2d(self.botpose[0], self.botpose[1], self.botpose[5]), self.matchTimer)
        #print(self.visionPose)

        # CHANGE TO FIELD DRIVE VS BOT RELETIVE
        if self.controller.getRawButton(variables.crossButton) == 1:
            self.fieldDrive = 2
        if self.controller.getRawButton(variables.circleButton) == 1:
            self.fieldDrive = 1

        if self.controller.getRawButton(variables.triangleButton) == 1:
            self.swerve.resetGyro()
        
        #if self.controller.getRawButton(variables.squareButton) == 1:
            #self.elevator2.start_elevatorMotor()
        #else:
            #self.elevator2.stop_elevatorMotor()

        if self.fieldDrive == 2:
            self.driveWithJoystick(True)
        else:
            self.driveWithJoystick(False)

        #print(self.controller.getPOV())
    
        self.navxGyro.getGyro()
        
        '''
        if self.pad.getRawButton(6) == 1:
            self.elevator.start_elevatorMotor(5)
        elif self.pad.getRawButton(9) == 1:
            self.elevator.start_elevatorMotor(-5)
        else:
            self.elevator.stop_elevatorMotor()
        '''
        

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        # NOTE: Check if we need inversion here
        #if fieldRelative:
        #    self.swerve.updateOdometry()

        self.dPad = self.controller.getPOV()

        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(1), variables.x_deadband)
            )
            * variables.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        # NOTE: Check if we need inversion here
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(0), variables.y_deadband)
            )
            * variables.kTMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.

        '''
        rot = (
            (-self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(3) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed) +
            (self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(4) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed)
        )
        '''
        rot = (
            (self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(3) + 1) / 2, variables.rot_deadband)
            ) +
                -wpimath.applyDeadband((self.controller.getRawAxis(4) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed)
        

        if self.dPad == 0:
            xSpeed = 0.2
        if self.dPad == 90:
            ySpeed = -0.2
        
        if self.dPad == 180:
            xSpeed = -0.2
        if self.dPad == 270:
            ySpeed = 0.2
        
        if self.controller.getRawButton(variables.L1Button) == 1:
            rot = 0.5
        if self.controller.getRawButton(variables.R1Button) == 1:
            rot = -0.5
        
        if self.controller.getRawButton(variables.PSbutton) == 1:
            # self.rot_limelight = self.limelight_aim()
            # rot = self.rot_limelight

            # self.forward_limelight = self.limelight_range()
            # xSpeed = self.forward_limelight

            fieldRelative = False

    
        variables.setTurnState(rot)

        #self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
    
    def limelight_aim(self):
        self.kP = 0.035
        self.tx = self.lmtable.getNumber('tx', None)

        self.targetingAngularVelocity = self.tx * self.kP

        self.targetingAngularVelocity *= variables.kRMaxSpeed
        
        self.targetingAngularVelocity *= -1.0

        return self.targetingAngularVelocity

    def limelight_range(self):
        self.kP = 0.1
        self.ty = self.lmtable.getNumber('ty', None)

        self.targetingForwardSpeed = self.ty * self.kP
        self.targetingForwardSpeed *= variables.kMaxSpeed
        self.targetingForwardSpeed *= -1.0

        return self.targetingForwardSpeed
    
    def FollowChoreoPath(self, trajectory):
        sample = trajectory.sample_at(self.timer.get(), self.is_red_alliance())

        if sample:
            self.swerve.follow_trajectory(sample)
    
    def StopPath(self):
        self.swerve.drive(0, 0, 0, True, self.getPeriod())



"""

- [ ] Get position of AprilTag ID 18
- [ ] Move to that position
- [ ] Do something else...

"""
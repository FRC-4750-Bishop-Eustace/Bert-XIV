# Write your code here :-)
import math
import wpilib
from rev import SparkFlex



class Elevator:
    def __init__(
        self,
        elevatorMotorID: int,
    ) -> None:

        self.elevatorMotor = SparkFlex(elevatorMotorID, SparkFlex.MotorType.kBrushless)
        self.elevatorEncoder = self.elevatorMotor.getEncoder()

    def start_elevatorMotor(self):
        #print('motor moving')
        self.elevatorMotor.setVoltage(-5)

    def stop_elevatorMotor(self):
        #print('motor stopped')
        self.elevatorMotor.setVoltage(0)
    
    def get_elevatorEncoder(self):
        print('encoder revolutions = ', self.elevatorEncoder.getPosition())
        print('encoder velocity = ', self.elevatorEncoder.getVelocity())

    def set_elevatorEncoder(self, x):
        print('Setting Encoder to: ', x)
        self.elevatorEncoder.setPosition(x)
    


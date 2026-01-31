import navx
import wpimath
#import ctre

#self.gyro = AnalogGyro(0)

#self.armExtensionMotor = ctre.WPI_TalonSRX(ArmExtensionMotorPort)
#self.angleMotor = ctre.WPI_TalonSRX(AngleMotorPort)

class Gyro:
    def __init__(self) -> None:
        self.angler = navx.AHRS.create_spi()

    def getGyro(self) -> None:
        self.armangle = self.angler.getAngle()
        self.radiansgyro = wpimath.units.degreesToRadians(self.armangle)
        #self.sd.putNumber('armAngle', self.armangle)
        #print(self.radiansgyro)    
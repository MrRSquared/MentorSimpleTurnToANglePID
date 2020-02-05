import wpilib
from wpilib import drive
from wpilib.controller import PIDController

from navx import AHRS


class MyRobot(wpilib.TimedRobot):
    if wpilib.RobotBase.isSimulation():
            # These PID parameters are used in simulation
            kP = 0.06
            kI = 0.00
            kD = 0.00
            kF = 0.00
    else:
        # These PID parameters are used on a real robot
        kP = 0.03
        kI = 0.00
        kD = 0.00
        kF = 0.00

    kToleranceDegrees = 2.0

    def robotInit(self):
        self.left_motor = wpilib.Spark(0)
        self.right_motor = wpilib.Spark(1)
        self.drive = wpilib.drive.DifferentialDrive(self.left_motor, self.right_motor)
        self.stick = wpilib.Joystick(0)
        self.ahrs = AHRS.create_spi()
        # self.ahrs = AHRS.create_i2c()

        turnController = PIDController(
            self.kP, self.kI, self.kD, period = 1.0
        )

        self.turnController = turnController
        self.rotateToAngleRate = 0

    def autonomousPeriodic(self):
        tm = wpilib.Timer()
        tm.start()

        #self.MyRobot.setSafetyEnabled(True)

        if tm.hasPeriodPassed(1.0):
                print("NavX Gyro", self.ahrs.getYaw(), self.ahrs.getAngle())

if __name__ == "__main__":
    wpilib.run(MyRobot)

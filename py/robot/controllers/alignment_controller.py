from magicbot import tunable
from wpilib import PIDController
import hal
from networktables import NetworkTables

from components.drivetrain import Drivetrain

class AlignmentController:

    drivetrain : Drivetrain

    def setup(self):
        self.table = NetworkTables.getTable("limelight")
        self.kP = 0.2
        self.minCommand = 0.05
        self.tx = 1
        self.aligned = False

    def autoCheck(self):
        return self.table.getNumber("tv", 0) != 0

    def distCheck(self):
        return self.tx > 0.05

    def otherCheck(self):
        return self.tx < -0.05

    def isAligned(self):
        return abs(self.tx) < 0.75

    def autoAlign(self):
        if not self.isAligned():
            z = max([self.tx * self.kP + self.minCommand,0.25])
            self.drivetrain.set_manual_mode(True)
            if self.distCheck():
                self.drivetrain.manual_drive(z,-z)
            elif self.otherCheck():
                self.drivetrain.manual_drive(-z,z)

    def execute(self):
        if self.autoCheck():
            self.tx = self.table.getNumber("tx", 0)
        print("autocheck", self.autoCheck())
        print("align", self.isAligned())
        print("tx", self.tx)
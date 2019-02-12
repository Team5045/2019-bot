from wpilib import SerialPort
from magicbot import tunable
from math import degrees, atan2
from networktables import NetworkTables
import re

IR_SEPARATION = 128
table = NetworkTables.getTable('components/irsensor')

class IRSensor:

    serial = SerialPort
    threshold = tunable(300)

    def setup(self):
        self.activations = None
        self.angle = None
        self.displacement = None
        self.oriented = False
        self.saved_threshold = 300
        self.saved_activations = None

    def set_threshold(self):
        self.serial.writeString("t"+str(self.threshold))
        return self.serial.readString(3)==str(self.threshold)

    def get_array_one(self, stringify=False):
        self.serial.writeString('a')
        lst = []
        if stringify:
            return re.sub(r'\W+', '', self.serial.readString(16))
        for i in str(self.serial.readString(16)):
            if i=='1':
                lst.append(True)
            elif i=='0':
                lst.append(False)
            else:
                lst.append(None)
        return lst

    def get_array_two(self, stringify=False):
        self.serial.writeString('b')
        lst = []
        if stringify:
            return re.sub(r'\W+', '', self.serial.readString(16))
        for i in str(self.serial.readString(16)):
            if i=='1':
                lst.append(True)
            elif i=='0':
                lst.append(False)
            else:
                lst.append(None)
        return lst


    def compute_orientation(self):
        c1 = [i for i,j in enumerate(self.activations[:16]) if j]
        c1 = sum(c1)/len(c1)
        c2 = [i for i,j in enumerate(self.activations[16:]) if j]
        c2 = sum(c2)/len(c2)

        self.displacement = 62.5 - (8*c1+8*c2)/2
        self.angle = degrees(atan2(c1-62.5,IR_SEPARATION/2))

    def isOriented(self):
        return self.oriented

    def execute(self):
        if self.saved_threshold != self.threshold:
            success = self.set_threshold()
            if success:
                self.saved_threshold = self.threshold
            else:
                self.threshold = self.saved_threshold
        self.activations = self.get_array_one(True)+self.get_array_two(True)
        if self.saved_activations != self.activations:
            table.putValue('/activations', self.activations)
            self.saved_activations = self.activations

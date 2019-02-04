from wpilib import SerialPort
from magicbot import tunable
from math import degrees, atan2

IR_SEPARATION = 128

class IRSensor:

    serial = SerialPort
    timeout = tunable(2500)
    threshold = tunable(300)

    def setup(self):
        self.activations = None
        self.angle = None
        self.displacement = None

    def set_config(self):
        pass

    def get_config(self):
        pass
        
    def get_array_one(self, stringify=False):
        self.serial.writeString('a')
        lst = []
        if stringify:
            return str(self.serial.readString(16)) 
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
            return str(self.serial.readString(16)) 
        for i in str(self.serial.readString(16)):
            if i=='1':
                lst.append(True)
            elif i=='0':
                lst.append(False)
            else:
                lst.append(None)
        return lst

    def get_arrays(self, stringify=False):
        self.serial.writeString('c')
        lst = []
        if stringify:
            return str(self.serial.readString(16)) 
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

    def execute(self):
        self.activations = self.get_arrays()
        
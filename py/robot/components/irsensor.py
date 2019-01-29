from wpilib import AnalogInput, AnalogOutput
from magicbot import tunable


class IRSensor:

    dist_signal = AnalogInput
    angle_signal = AnalogInput
    configurator = AnalogOutput

    config_timeout = tunable(2500)
    config_readmode = tunable(True)

    def configure_arduino(self):
        '''
        Uses AnalogOutput to configure Arduino remotely.
        Timeout: 1000 - 3000 μs
        Readmode: QTR_EMITTERS_ON(True) / QTR_EMITTERS_OFF(False)

        Scales voltage linearly between 0V and 1.65V for QTR_EMITTERS_ON
        and between 1.65V and 3.3V for QTR_EMITTERS_OFF 
        '''
        volt = 1.65*(self.config_timeout-1000)/2000

        if not self.config_readmode:
            volt+=1.65

        self.configurator.setVoltage(volt)
    
    @property
    def distance(self):
        return self.dist_signal.getValue()

    @property
    def angle(self):
        return self.angle_signal.getValue()   

    def execute(self):
        #pipe to pid controller
        pass
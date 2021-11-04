import actuator
import time

class BB():
    def __init__(self, master :actuator.Master, ID, ADC):
        self.pio_channel = 3
        self.master = master
        self.Actuator = self.master.Actuators[ID]
        self.adc_path = "/sys/bus/iio/devices/iio\:device0/in_voltage%s_raw".format(ADC)
        
        dummy_act = actuator.Actuator(self.Actuator.Configuration.data.devID)
        dummy_act.PIOs.data.portMode[self.pio_channel] = self.Actuator.PIOs.data.Servo
        self.master.send(self.Actuator.Write(dummy_act, [actuator.Parameters.PIOMode]))
    
    def get_raw_position(self) -> int:
        with open(self.adc_path) as adc:
            return adc.read()

    def set_servo(self, servo_pos) -> None:
        dummy_act = actuator.Actuator(self.Actuator.Configuration.data.devID)
        dummy_act.PIOs.data.portData[self.pio_channel] = servo_pos
        self.master.send(self.Actuator.Write(dummy_act, [actuator.Parameters.PIOData]))



m = actuator.Master(4096, '/dev/tty0', 115200)
m.addActuator(0)

kp = 1
kd = 0

previous_error = 0
error_sum = 0

adc_filter_size = 10
adc_filter = [0] * adc_filter_size

iteration = 0
interval = 0.01

bb = BB(m, 0, 4)

while True:
    setpoint = 50

    adc_filter[iteration] = bb.get_raw_position()
    current_position = sum(adc_filter)/adc_filter_size

    error = setpoint - current_position
    p_term = error * kp

    d_error = error - previous_error
    d_term = d_error * kd

    output = p_term + d_term

    bb.set_servo(int(output))
    iteration = (iteration + 1) if iteration < adc_filter_size else 0
    time.sleep(interval)
import serial
import time
from threading import Thread

class Motion:
    def __init__(self):
        self.Read_RX = 0
        self.receiving_exit = 1
        self.threading_Time = 0.01
        self.lock = False
        self.distance = 0
        BPS = 4800

        self.serial_port = serial.Serial('/dev/ttyS0', BPS, timeout=0.01)
        self.serial_port.flush()
        self.serial_t = Thread(target=self.Receiving, args=(self.serial_port,))
        self.serial_t.daemon = True
        self.serial_t.start()
        time.sleep(0.1)

    def TX_data_py2(self, one_byte):
        self.lock = True
        self.serial_port.write(serial.to_bytes([one_byte]))
        time.sleep(0.1)
        
    def TX_data_py3(self, one_byte):
        self.lock = True
        self.serial_port.write(serial.to_bytes([one_byte]))
        time.sleep(0.1)
        
    def RX_data(self):
        if self.serial_port.inWaiting() > 0:
            result = self.serial_port.read(1)
            RX = ord(result)
            return RX
        else:
            return 0

    def getRx(self):
        return self.lock

    def wait_unlock(self):
        while self.lock:
            continue
        
    def Receiving(self, ser):
        self.receiving_exit = 1
        while True:
            time.sleep(self.threading_Time)
            while ser.inWaiting() > 0:
                result = ser.read(1)
                RX = ord(result)
                if RX == 38:
                    self.lock = False
                else:
                    self.distance = RX
                
            if self.receiving_exit == 0:
                break

    def initial(self):
        self.TX_data_py3(250)
        self.wait_unlock()
        return
    
    def init(self, neck=False):
        if neck:                 
            self.TX_data_py3(8)
        else:
            self.TX_data_py3(26)
        self.wait_unlock()
        return

    def walk(self, fast = False):
        if not fast:
            self.TX_data_py3(11)
        else:
            self.TX_data_py3(10)
        return

    def step(self, direction="FRONT", stride="small"):
        if direction == "FRONT" and stride == "small":
            self.TX_data_py3(36)
        elif direction == "FRONT" and stride == "big":
            self.TX_data_py3(5)
        else:
            self.TX_data_py3(39)
        self.wait_unlock()
        time.sleep(0.5)
        return

    def crab(self, direction):
        if direction == "LEFT":
            self.TX_data_py3(34)
        elif direction == "RIGHT":
            self.TX_data_py3(33)
        self.wait_unlock()
        return

    def view(self, target_angle=0):
        if target_angle == 0:
            self.TX_data_py3(21)
        elif target_angle == -45:
            self.TX_data_py3(28)
        elif target_angle == -90:
            self.TX_data_py3(17)
        elif target_angle == 45:
            self.TX_data_py3(30)
        elif target_angle == 90:
            self.TX_data_py3(27)
        self.wait_unlock()
        return

    def neckup(self, target_angle=100):
        serial_num = target_angle // 5 + 33
        self.TX_data_py3(serial_num)
        self.wait_unlock()
        time.sleep(0.3)
        return

    def turn(self, direction="LEFT", angle=10):
        if direction == "LEFT":
            if angle == 5:       
                self.TX_data_py3(1)
            elif angle == 10:
                self.TX_data_py3(4)
            elif angle == 20:
                self.TX_data_py3(7)
            elif angle == 45:
                self.TX_data_py3(22)
            elif angle == 60:
                self.TX_data_py3(25)
        else:
            if angle == 5:       
                self.TX_data_py3(3)
            elif angle == 10:
                self.TX_data_py3(6)
            elif angle == 20:
                self.TX_data_py3(9)
            elif angle == 45:
                self.TX_data_py3(24)
            elif angle == 60:
                self.TX_data_py3(19)
        self.wait_unlock()
        return

    def eagle(self):
        self.TX_data_py3(89)
        self.wait_unlock()
        return
    
    def circular_orbit(self, direction = "Left", leg_up = True):
        if leg_up and direction == "Left":
            self.TX_data_py3(29)
            print("left_turn 29")
        elif not leg_up and direction == "Left":
            self.TX_data_py3(30)
            print("left_turn 30")
        elif leg_up and direction == "Right":
            self.TX_data_py3(54)
            self.wait_unlock()
            self.crab("RIGHT")
            print("right_turn 54")
        else:
            self.TX_data_py3(55)
            print("right_turn 55")
        self.wait_unlock()
        return

    def circular_orbit_small(self, direction = "Left", angle = 5):
        if direction == "Left":
            self.turn("RIGHT", angle)
            for _ in range(angle // 5):
                self.crab("LEFT")
        else:
            self.turn("LEFT", angle)
            for _ in range(angle // 5):
                self.crab("RIGHT")
        self.wait_unlock()
        return
    
    def shot(self, direction = "LEFT", speed = 8):
        if direction == "LEFT":
            motion_num = speed + 55
            self.TX_data_py3(motion_num)
            print("left_shot", motion_num)
        else:
            if speed == 2:
                self.TX_data_py3(82)
                print("right_shot", 82)
            else:
                self.TX_data_py3(speed)
                print("right_shot", speed)
        self.wait_unlock()
        return

    def ceremony(self):
        self.TX_data_py3(83)
        self.wait_unlock()
        return
    
if __name__ == '__main__':
    Motion = Motion()
    Motion.initial()
    Motion.view(-90)
    time.sleep(7)
    pass

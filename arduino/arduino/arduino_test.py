import rclpy 
from rclpy.node import Node
import serial 
import time

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_test')
        self.arduino_port = 'COM6'
        self.baudrate = 115200
        self.arduino = serial.Serial(self.arduino_port,self.baudrate,timeout=1)
        self.declare_parameter('arduino_port',self.arduino_port)
        self.declare_parameter('baudrate',self.baudrate)

        self.timer = self.create_timer(1.0,self.send_command)
    
    def send_command(self):
        # promt user to enter command
        command = input("Enter F for forward and B for backward")

        # validate user input 
        if command.upper in ('F', 'B'):
            # send arduino command 
            self.arduino.write(command.encode())
        else: 
            self.get_logger().info("Invalid input. Please enter F or B")
    
    def close_connection(self):
        self.arduino.close()
    
def main (args=None):
    rclpy.init(args = args)
    arduino_control = ArduinoController()
    try:
        rclpy.spin(arduino_control)
    except:
        pass
    finally:
        arduino_control.close_connection()
        arduino_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

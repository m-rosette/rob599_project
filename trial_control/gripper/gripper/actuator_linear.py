import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gripper_msgs.srv import LinearActuator
import serial
class MoveMe(Node):
    def __init__(self):
        super().__init__('actuator_linear')
        self.get_logger().info("Starting linear actuator control")
        self.srv = self.create_service(LinearActuator, 'input_number', self.input_number_callback)
        self.pub = self.create_publisher(String, 'number_input', 10)
          # Define Arduino serial port and baud rate
        self.arduino_port = '/dev/ttyACM1'   # Change this to the correct port
        self.baudrate = 115200
        # Open the serial connection to Arduino
        self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)
    def input_number_callback(self, request, response):
        self.get_logger().info('Received input_number request')
        try:
            number = int(input("Please enter a number: "))
            msg = String()
            msg.data = str(number)
            self.pub.publish(msg)
            response.success = True
            response.message = 'Number received and published successfully.'
        except ValueError:
            response.success = False
            response.message = 'Invalid input. Please enter a valid integer.'
        return response
def main(args=None):
    rclpy.init(args=args)
    actuator_linear= MoveMe()
    rclpy.spin(actuator_linear)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
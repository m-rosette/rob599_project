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
        self.arduino_port = '/dev/ttyACM0'   # Change this to the correct port
        self.baudrate = 115200
        # Open the serial connection to Arduino
        self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)

    def input_number_callback(self, request, response):
        self.get_logger().info('Received input_number request')
        try:
            # number = int(input("Please enter a number: "))

            request = LinearActuator()
            # request.location_goals

            # Publish message
            msg = String()
            msg.data = str(request.location_goal)
            self.pub.publish(msg)

            #Update response
            response.success = True
            response.message = 'Number received and published successfully.'

            # Send command to arduino
            self.arduino.write(str(request.location_goal).encode())

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


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from gripper_msgs.srv import LinearActuator
# import serial
# import time
# class MoveStepper(Node):
#     def __init__(self):
#         super().__init__('actuator_linear')
#         self.get_logger().info("Starting stepper controller")
#         # Define the serial port and baud rate
#         self.serial_port = "/dev/ttyACM1"  # Change this to match your Arduino's serial port
#         self.baud_rate = 115200  # Should match the baud rate set in your Arduino script
#         # Open the serial port
#         self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
#         # Create service
#         self.srv = self.create_service(LinearActuator, 'move_stepper', self.move_stepper_callback)
#     def move_stepper_callback(self, request, response):
#         # Prompt the user to enter the target position
#         target_position = int(input("Enter the target position: "))
#         # Send the target position to Arduino
#         self.arduino.write(str(target_position).encode() + b'\n')
#         time.sleep(0.1)  # Wait for Arduino to process the command
#         self.get_logger().info(f"Stepper moved to position: {target_position}")
#         response.success = True
#         response.message = f"Stepper moved to position: {target_position}"
#         return response
# def main(args=None):
#     rclpy.init(args=args)
#     stepper_controller = MoveStepper()
#     rclpy.spin(stepper_controller)
#     stepper_controller.destroy_node()
#     rclpy.shutdown()
# if __name__ == '__main__':
#     main()
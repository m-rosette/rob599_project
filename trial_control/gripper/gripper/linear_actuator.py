import rclpy
from rclpy.node import Node
import serial
import time
from gripper_msg.srv import LinearActuator

class ArduinoControl(Node):
    def __init__(self):
        super().__init__('linear_actuator')

        # Define Arduino serial port and baud rate
        self.arduino_port = '/dev/ttyACM1'   # Change this to the correct port
        self.baudrate = 115200
        #  creating service for linear actuator 
        self.srv= self.create_service(LinearActuator,'location_target',self.location_target_callback)

        # Open the serial connection to Arduino
        self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)
    def location_target_callback(self,goal_handle):
        pass 

    # Function to send command to Arduino
    def send_command(self, command):
        self.arduino.write(command.encode())   # Send the command to Arduino
        self.get_logger().info(f"Sent command: {command}")   # Debug statement
        time.sleep(1)                      # Wait for Arduino to process the command

    # Main function
    def main(self):
        # Prompt the user to enter the command ('F' for forward, 'B' for backward)
        command = input("Enter 'F' for forward or 'B' for backward: ")
        
        # Validate user input
        if command.upper() in ('F', 'B'):
            # Send the command to Arduino
            self.send_command(command.upper())
        else:
            self.get_logger().error("Invalid input. Please enter 'F' for forward or 'B' for backward.")

        # Close the serial connection
        self.arduino.close()

def main(args=None):
    rclpy.init(args=args)
    arduino_control = ArduinoControl()
    arduino_control.main()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
import serial
import time
import sys

class LinearActuator(Node):
    def __init__(self):
        super().__init__('linear_actuator')
        self.get_logger().info("Starting linear actuator control")

        # Define Arduino serial port and baud rate
        self.arduino_port = '/dev/ttyACM1'   # Change this to the correct port
        self.baudrate = 115200

        try:
            # Open the serial connection to Arduino
            self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")

    # Function to send command to Arduino
    def send_command(self, command):
        try:
            # Send the command to Arduino
            self.arduino.write(str(command).encode())  
            self.get_logger().info(f"Sent position command: {command}") 

            # Wait for Arduino to process the command
            time.sleep(1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send command: {str(e)}")

    def close_connection(self):
        if self.arduino.is_open:
            self.arduino.close()
            self.get_logger().info("Serial connection closed.")

def main(args=None):
    rclpy.init(args=args)

    linear_actuator = LinearActuator()

    try:
        command = str(sys.argv[1])
    except IndexError:
        command = str(0)
        linear_actuator.get_logger().warn("Incorrect input. Not moving linear actuator")
    
    linear_actuator.send_command(command)
    linear_actuator.close_connection()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

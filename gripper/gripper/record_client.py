import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from gripper_msgs.action import RecordData
import sys
import time


class RecordClient(Node):

    def __init__(self):
        self.filename = 'new_data'

        # initialize the node
        super().__init__('record_client')
        self.action_client = ActionClient(self, RecordData, 'record_server')
        self.get_logger().info('RecordClient node has been started.')
        # Wait until the server is ready to accept an action request.
        self.action_client.wait_for_server()


def main(args=None):
    rclpy.init(args=args)

    # get goal position, if available
    try:
        goal_filename = str(sys.argv[1])
    except:
        goal_filename = "None"

    # initialize client
    client = RecordClient()

    goal_msg = RecordData.Goal()
    goal_msg.filename = goal_filename

    while rclpy.ok():
        # goal = RecordData(foa)
        client.action_client.send_goal(goal_msg)

        time.sleep(2.5)

        client.action_client._cancel_goal()
    
    rclpy.spin(client)


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
import numpy as np
from rb_interfaces.srv import ReqJnt


class JointCtrlCall(Node):

    def __init__(self):
        super().__init__('jnt_call')

        self.jntctrl = self.create_client(ReqJnt, 'req_jnt')
        while not self.jntctrl.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self):
        req = ReqJnt.Request()
        req.jntstate.position = np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).tolist()
        req.spd = -1.0
        req.acc = 5.0
        self.future = self.jntctrl.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    client = JointCtrlCall()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(f'Service call failed {e}')
            else:
                client.get_logger().info(f'Request is {response.success} with message {response.message}')
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

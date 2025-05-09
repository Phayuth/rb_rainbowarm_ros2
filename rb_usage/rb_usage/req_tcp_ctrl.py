import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion

from scipy.spatial.transform import Rotation as R
import numpy as np

from rb_interfaces.srv import ReqTcp


class TCPCtrlCall(Node):

    def __init__(self):
        super().__init__("tcp_call")

        self.tcpctrl = self.create_client(ReqTcp, "req_tcp")
        while not self.tcpctrl.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.tcpHome = [0.0, -265.70, 326.49, 174.00, -1.04, -180.0]  # xyzrpy
        self.tcpPreStamped = [0.0, -391.81, 214.97, 177.31, -1.04, 179.94]
        self.tcpStamped = [0.0, -391.81, 37.03, 177.31, -1.04, 179.94]

    def send_request(self):
        x = self.tcpPreStamped[0]
        y = self.tcpPreStamped[1]
        z = self.tcpPreStamped[2]
        rx = self.tcpPreStamped[3]
        ry = self.tcpPreStamped[4]
        rz = self.tcpPreStamped[5]
        r = np.array([rx, ry, rz])
        rpy = R.from_euler("zyx", r, degrees=True)
        q = rpy.as_quat()

        req = ReqTcp.Request()
        req.tcppose.pose.position = Point(x=x, y=y, z=z)
        req.tcppose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        req.movetype = 1
        req.spd = 50.0
        req.acc = 50.0
        self.future = self.tcpctrl.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    client = TCPCtrlCall()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(f"Service call failed {e}")
            else:
                client.get_logger().info(
                    f"Request is {response.success} with message {response.message}"
                )
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from bridge_api import RBRobotROS2Bridge
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rb_interfaces.srv import ReqTcp
from rb_interfaces.srv import ReqJnt
from rb_interfaces.action import ReqJntTraj
# from rb_interfaces.action import ReqTcpTraj


class RBController(Node):

    def __init__(self):
        super().__init__(node_name='rb_controller_node')
        # init robot
        RBRobotROS2Bridge()

        # callback group declare
        self.cbg = ReentrantCallbackGroup()

        # robot state publisher
        self.TcpPublisher = self.create_publisher(PoseStamped, '/current_rb_tcppose', 10, callback_group=self.cbg)
        self.JntPublisher = self.create_publisher(JointState, '/current_rb_joint', 10, callback_group=self.cbg)
        self.timerJnt = self.create_timer(0.1, self.get_joint_state)
        self.timerTcp = self.create_timer(0.1, self.get_tcp_state)

        # robot control service server
        self.reqTcpSrv = self.create_service(ReqTcp, 'req_tcp', self.req_tcp_srv)
        self.reqJntSrv = self.create_service(ReqJnt, 'req_jnt', self.req_jnt_srv)

        # robot control action server
        self.reqJntAct = ActionServer(self, ReqJntTraj, 'req_jnt_traj', self.jnt_traj_action)
        # self.reqTcpAct = ActionServer(self, ReqTcpTraj, 'req_tcp_traj', self.tcp_traj_action)

    def get_joint_state(self):
        jointMsg = RBRobotROS2Bridge.get_joint_state_msg()
        self.JntPublisher.publish(jointMsg)

    def get_tcp_state(self):
        poseMsg = RBRobotROS2Bridge.get_tcp_state_msg()
        self.TcpPublisher.publish(poseMsg)

    def req_tcp_srv(self, request, response):
        response = RBRobotROS2Bridge.req_tcp(request, response)
        return response

    def req_jnt_srv(self, request, response):
        response = RBRobotROS2Bridge.req_joint(request, response)
        return response

    def jnt_traj_action(self, goalHandle):
        result = RBRobotROS2Bridge.req_joint_trajectory(goalHandle)
        return result

    # def tcp_traj_action(self, goalHandle):
    #     result = RBRobotROS2Bridge.req_tcp_trajectory_itpl(goalHandle)
    #     return result


def main(args=None):
    rclpy.init(args=args)
    node = RBController()
    try:
        rclpy.spin(node)
    finally:
        RBRobotROS2Bridge.disconnect_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from .rb_rosbridge import RBRobotROS2Bridge
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from rb_interfaces.srv import ReqTcp
from rb_interfaces.srv import ReqJnt
from rb_interfaces.action import ReqJntTraj
from rb_interfaces.action import ReqTcpTraj


class RBController(Node):

    def __init__(self):
        super().__init__(node_name="rb_controller_node")
        # init robot
        self.robothandle = RBRobotROS2Bridge("192.168.0.200")

        # callback group declare
        self.cbg = ReentrantCallbackGroup()
        self.cba = ReentrantCallbackGroup()

        # robot state publisher
        self.TcpPublisher = self.create_publisher(PoseStamped, "/tcp_states", 2, callback_group=self.cbg)
        self.JntPublisher = self.create_publisher(JointState, "/joint_states", 2, callback_group=self.cbg)
        self.timerJnt = self.create_timer(0.008, self.get_joint_state)  # 125hz
        self.timerTcp = self.create_timer(0.008, self.get_tcp_state)  # 125hz

        # robot control single point service
        self.reqTcpSrv = self.create_service(ReqTcp, "req_tcp", self.req_tcp_srv, callback_group=self.cba)
        self.reqJntSrv = self.create_service(ReqJnt, "req_jnt", self.req_jnt_srv, callback_group=self.cba)

        # robot control multiple point action
        self.reqJntAct = ActionServer(
            self,
            ReqJntTraj,
            "req_jnt_traj",
            self.jnt_traj_action,
            goal_callback=self.req_jnt_traj_validate,
            cancel_callback=self.req_jnt_traj_cancel,
            callback_group=self.cba,
        )
        self.reqTcpAct = ActionServer(
            self,
            ReqTcpTraj,
            "req_tcp_traj",
            self.tcp_traj_action,
            goal_callback=self.req_tcp_traj_validate,
            cancel_callback=self.req_tcp_traj_cancel,
            callback_group=self.cba,
        )

    def get_joint_state(self):
        jointMsg = self.robothandle.get_joint_state_msg(self.get_clock().now().to_msg())
        self.JntPublisher.publish(jointMsg)

    def get_tcp_state(self):
        poseMsg = self.robothandle.get_tcp_state_msg(self.get_clock().now().to_msg())
        self.TcpPublisher.publish(poseMsg)

    def req_tcp_srv(self, request, response):
        response = self.robothandle.req_tcp(request, response)
        return response

    def req_jnt_srv(self, request, response):
        response = self.robothandle.req_joint(request, response)
        return response

    def req_jnt_traj_validate(self, goalReq: ReqJntTraj.Goal):
        return self.robothandle.req_joint_trajectory_goal_validate(goalReq)

    def req_tcp_traj_validate(self, goalReq: ReqTcpTraj.Goal):
        return self.robothandle.req_tcp_trajectory_goal_validate(goalReq)

    def req_jnt_traj_cancel(self, goalHandle: ServerGoalHandle):
        return self.robothandle.req_joint_trajectory_goal_cancel(goalHandle)

    def req_tcp_traj_cancel(self, goalHandle: ServerGoalHandle):
        return self.robothandle.req_tcp_trajectory_goal_cancel(goalHandle)

    def jnt_traj_action(self, goalHandle: ServerGoalHandle):
        return self.robothandle.req_joint_trajectory_goal_execute(goalHandle)

    def tcp_traj_action(self, goalHandle: ServerGoalHandle):
        return self.robothandle.req_tcp_trajectory_goal_execute(goalHandle)


def main(args=None):
    rclpy.init(args=args)
    node = RBController()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    finally:
        node.robothandle.disconnect_robot()
        node.destroy_node()
        exe.shutdown()


if __name__ == "__main__":
    main()

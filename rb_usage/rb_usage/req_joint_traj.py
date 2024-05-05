import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rb_interfaces.action import ReqJntTraj


class JointTrajectoryReq(Node):

    def __init__(self):
        super().__init__("trj_cliet")
        self.act = ActionClient(self, ReqJntTraj, "req_jnt_traj")
        self.waypoint = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.174533],
            [0.0, 0.0, 0.0, 0.0, 0.174533, 0.174533],
            [0.0, 0.0, 0.0, 0.0, 0.174533, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]

    def send_goal(self):
        goalMsg = ReqJntTraj.Goal()

        traj = JointTrajectory()
        traj.joint_names = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
        for p in self.waypoint:
            point = JointTrajectoryPoint()
            point.positions = p
            traj.points.append(point)

        goalMsg.jntseq = traj
        goalMsg.spd = 1.0
        goalMsg.acc = 1.0

        self.act.wait_for_server()
        self.sendGoalFuture = self.act.send_goal_async(goalMsg, feedback_callback=self.feedback_callback)
        self.sendGoalFuture.add_done_callback(self.goal_response_callback)

    def send_cancel(self):
        self.goalHandle.cancel_goal_async()
        self.get_logger().info("Cancel Goal requested")

    def goal_response_callback(self, future):
        self.goalHandle: ClientGoalHandle = future.result()
        if not self.goalHandle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self.getResultFuture = self.goalHandle.get_result_async()
        self.getResultFuture.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status: GoalStatus = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Canceled")
        successful = result.success
        message = result.message
        self.get_logger().info(f"successful: {successful}")
        self.get_logger().info(f"message: {message}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Progression : {feedback.progression}")


def main(args=None):
    rclpy.init(args=args)
    actcl = JointTrajectoryReq()
    actcl.send_goal()
    rclpy.spin(actcl)


if __name__ == "__main__":
    main()

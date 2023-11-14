import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rb_interfaces.action import ReqJntTraj


class JointTrajectoryReq(Node):

    def __init__(self):
        super().__init__('trj_cliet')
        self.act = ActionClient(self, ReqJntTraj, 'req_jnt_traj')
        self.waypoint = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                         [0.0, 0.0, 0.0, 0.0, 0.0, 10.0], 
                         [0.0, 0.0, 0.0, 0.0, 10.0, 10.0], 
                         [0.0, 0.0, 0.0, 0.0, 10.0, 0.0]]

    def send_goal(self):
        goalMsg = ReqJntTraj.Goal()

        traj = JointTrajectory()
        traj.joint_names = ['j0', 'j1', 'j2', 'j3', 'j4', 'j5']
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

    def goal_response_callback(self, future):
        goalHandle = future.result()
        if not goalHandle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.getResultFuture = goalHandle.get_result_async()
        self.getResultFuture.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        successful = result.success
        message = result.message
        self.get_logger().info(f'successful: {successful}')
        self.get_logger().info(f'message: {message}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progression : {feedback.progression}')


def main(args=None):
    rclpy.init(args=args)
    actcl = JointTrajectoryReq()
    actcl.send_goal()
    rclpy.spin(actcl)


if __name__ == '__main__':
    main()
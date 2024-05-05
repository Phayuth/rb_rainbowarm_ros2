import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import rb_controller.rb_driver as cobot

from rclpy.action import GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from rb_interfaces.action import ReqJntTraj
from rb_interfaces.action import ReqTcpTraj


class RBRobotROS2Bridge:

    def __init__(self, ip) -> None:
        # initialize robot
        self.init_robot(ip)

        # properties
        self.jointNames = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
        self.eulerOrder = "xyz"
        self.baseName = "Body_Base"

    def init_robot(self, ip):
        cobot.ToCB(ip)
        time.sleep(2)
        cobot.CobotInit()  # init robot
        time.sleep(2)

        while True:
            time.sleep(1)
            state = cobot.IsPause()
            if state == True:
                print("E-Stop is [Engaged]. Release to Continue.")
                continue
            else:
                break

        # speed slider
        spdslider = 1.0
        print("Robot Speed Slider is set to :")
        cobot.SetBaseSpeed(spdslider)

        # Mode Real or Simulation
        cobot.SetProgramMode(cobot.PG_MODE.REAL)  # set to real robot
        # cobot.SetProgramMode(cobot.PG_MODE.SIMULATION)
        if cobot.IsRobotReal():
            print("Robot Mode is [Real]")
        else:
            print("Robot Mode is [Simulation]")

        time.sleep(1)
        print("Robot is [Ready] for [Operation]")

    def disconnect_robot(self):
        cobot.DisConnectToCB()
        print("Robot is [Disconnected]")

    def get_joint_state_msg(self, timenow):
        jointMsg = JointState()
        jointMsg.header.stamp = timenow
        jointMsg.name = self.jointNames
        jointValue = cobot.GetCurrentJoint()
        jointMsg.position = np.deg2rad([jointValue.j0, jointValue.j1, jointValue.j2, jointValue.j3, jointValue.j4, jointValue.j5]).tolist()
        return jointMsg

    def get_tcp_state_msg(self, timenow):
        poseMsg = PoseStamped()
        poseMsg.header.stamp = timenow
        poseMsg.header.frame_id = self.baseName
        tcpValue = cobot.GetCurrentTCP()  # in mm, degree
        t = (0.001 * np.array([tcpValue.x, tcpValue.y, tcpValue.z])).tolist()
        r = np.array([tcpValue.rx, tcpValue.ry, tcpValue.rz])
        rpy = R.from_euler(self.eulerOrder, r, degrees=True)
        q = rpy.as_quat()
        poseMsg.pose.position = Point(x=t[0], y=t[1], z=t[2])
        poseMsg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return poseMsg

    def req_tcp(self, request, response):
        rqx = request.tcppose.pose.position.x * 1000.0
        rqy = request.tcppose.pose.position.y * 1000.0
        rqz = request.tcppose.pose.position.z * 1000.0
        rqqx = request.tcppose.pose.orientation.x
        rqqy = request.tcppose.pose.orientation.y
        rqqz = request.tcppose.pose.orientation.z
        rqqw = request.tcppose.pose.orientation.w
        movetype = request.movetype
        spd = request.spd
        acc = request.acc
        r = R.from_quat([rqqx, rqqy, rqqz, rqqw])
        rpy = r.as_euler(self.eulerOrder, degrees=True)

        # TODO : Check for limit and stuff
        inLimit = True
        if inLimit:
            if movetype == 0:
                cobot.MoveL(rqx, rqy, rqz, rpy[0], rpy[1], rpy[2], spd, acc)
            if movetype == 1:
                cobot.MoveJL(rqx, rqy, rqz, rpy[0], rpy[1], rpy[2], spd, acc)
            response.success = True
            response.message = "Request [Successful], Robot should be [MOVING]"
        else:
            response.success = False
            response.message = "Request [Failed], Request Value is [outside of limit]"
        return response

    def req_joint(self, request, response):
        jointReq = np.rad2deg(request.jntstate.position)
        spd = request.spd
        acc = request.acc

        if self.is_in_jointlimit_deg(jointReq):
            cobot.MoveJ(jointReq[0], jointReq[1], jointReq[2], jointReq[3], jointReq[4], jointReq[5], spd, acc)
            response.success = True
            response.message = "Request [Successful], Robot should be [MOVING]"
        else:
            response.success = False
            response.message = "Request [Failed], Request Value is [outside of joint limit]"
        return response

    def req_joint_trajectory_goal_validate(self, goalReq):
        req = goalReq.jntseq.points
        spd = goalReq.spd
        acc = goalReq.acc

        # check speed
        if not self.is_spd_valid(spd):
            return GoalResponse.REJECT

        # check acceleration
        if not self.is_acc_valid(acc):
            return GoalResponse.REJECT

        # check joint limit
        for jntpoint in req:
            position = np.rad2deg(jntpoint.positions)
            if not self.is_in_jointlimit_deg(position):
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def req_joint_trajectory_goal_cancel(self, goalHandle: ServerGoalHandle):
        return CancelResponse.REJECT # ACCEPT # TODO : check for different situation

    def req_joint_trajectory_goal_execute(self, goalHandle: ServerGoalHandle):
        req = goalHandle.request.jntseq.points
        spd = goalHandle.request.spd
        acc = goalHandle.request.acc

        # call to move hardware
        cobot.MoveJB_Clear()
        for jntpoint in req:
            position = np.rad2deg(jntpoint.positions)
            cobot.MoveJB_Add(position[0], position[1], position[2], position[3], position[4], position[5])
        cobot.MoveJB_Run(spd, acc)

        # check current progress and keep update
        feedbackMsg = ReqJntTraj.Feedback()
        feedbackMsg.progression = 0.0

        result = ReqJntTraj.Result()

        while not cobot.IsIdle():
            if goalHandle.is_cancel_requested:
                # set cancel if user want to cancel
                goalHandle.canceled()
                result.success = False
                result.message = "Joint Trajectory is [Canceled] by user"
                return result
            if cobot.IsPause():
                # set aborted, for example we lost connection to robot or robot hit a collision
                goalHandle.abort()
                result.success = False
                result.message = "Joint Trajectory is [Aborted] [probably collision or connection]"
                return result
            feedbackMsg.progression += 1.0
            goalHandle.publish_feedback(feedbackMsg)
            time.sleep(0.5)

        # set success if not collision or anything
        goalHandle.succeed()
        result.success = True
        result.message = "Joint Trajectory is [Successfully Executed]"
        return result

    def req_tcp_trajectory_goal_validate(self, goalReq):
        tcppose = goalReq.tcpseq
        spd = goalReq.spd
        acc = goalReq.acc
        movetype = goalReq.movetype

        if not self.is_spd_valid(spd):
            return GoalResponse.REJECT

        if not self.is_acc_valid(acc):
            return GoalResponse.REJECT

        if not movetype in range(9):
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def req_tcp_trajectory_goal_cancel(self, goalHandle: ServerGoalHandle):
        return CancelResponse.REJECT # ACCEPT # TODO : check for different situation

    def req_tcp_trajectory_goal_execute(self, goalHandle: ServerGoalHandle):
        tcppose = goalHandle.request.tcpseq
        spd = goalHandle.request.spd
        acc = goalHandle.request.acc
        movetype = self.get_itpl_movetype(goalHandle.request.movetype)

        # call to move hardware
        cobot.MoveITPL_Clear()
        for tcp in tcppose:
            x = tcp.pose.position.x * 1000.0
            y = tcp.pose.position.y * 1000.0
            z = tcp.pose.position.z * 1000.0
            rqqx = tcp.pose.orientation.x
            rqqy = tcp.pose.orientation.y
            rqqz = tcp.pose.orientation.z
            rqqw = tcp.pose.orientation.w
            r = R.from_quat([rqqx, rqqy, rqqz, rqqw])
            rpy = r.as_euler(self.eulerOrder, degrees=True)
            cobot.MoveITPL_Add(x, y, z, rpy[0], rpy[1], rpy[2], spd)
        cobot.MoveITPL_Run(acc, movetype)

        # check current progress and keep update
        feedbackMsg = ReqTcpTraj.Feedback()
        feedbackMsg.progression = 0.0

        result = ReqTcpTraj.Result()

        while not cobot.IsIdle():
            if goalHandle.is_cancel_requested:
                goalHandle.canceled()
                result.success = False
                result.message = "TCP Trajectory is [Canceled] by user"
                return result
            if cobot.IsPause():
                goalHandle.abort()
                result.success = False
                result.message = "TCP Trajectory is [Aborted] [probably collision or connection]"
                return result
            feedbackMsg.progression += 1.0
            goalHandle.publish_feedback(feedbackMsg)
            time.sleep(0.5)

        goalHandle.succeed()
        result.success = True
        result.message = "TCP Trajectory is [Successfully Executed]"
        return result

    def get_itpl_movetype(self, index):
        movetype = [
            cobot.ITPL_RTYPE.INTENDED,
            cobot.ITPL_RTYPE.CONSTANT,
            cobot.ITPL_RTYPE.RESERVED1,
            cobot.ITPL_RTYPE.SMOOTH,
            cobot.ITPL_RTYPE.RESERVED2,
            cobot.ITPL_RTYPE.CA_INTENDED,
            cobot.ITPL_RTYPE.CA_CONSTANT,
            cobot.ITPL_RTYPE.RESERVED3,
            cobot.ITPL_RTYPE.CA_SMOOTH,
        ]
        return movetype[index]

    def is_spd_valid(spd):
        if spd == -1.0:
            return True
        if 0.0 <= spd <= 1.0:
            return True
        return False

    def is_acc_valid(acc):
        if acc == -1.0:
            return True
        if 0.0 <= acc <= 1.0:
            return True
        return False

    def is_in_jointlimit_deg(self, jnt):
        isInLimitCheck = np.logical_and(jnt >= -360, jnt <= 360)
        return np.all(isInLimitCheck, axis=0)

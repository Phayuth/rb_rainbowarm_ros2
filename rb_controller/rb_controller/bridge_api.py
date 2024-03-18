from rainbow import cobot
from time import sleep
import numpy as np
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from rb_interfaces.action import ReqJntTraj


class RBRobotROS2Bridge:

    def __init__(self, ip) -> None:
        RBRobotROS2Bridge.init_robot(ip)

    def init_robot(ip):
        cobot.ToCB(ip)
        sleep(2)
        cobot.CobotInit()  # init robot
        sleep(2)

        while True:
            sleep(1)
            state = cobot.IsPause()
            if state == True:
                print('E-Stop is [Engaged]. Release to Continue.')
                continue
            else:
                break

        # speed slider
        spdslider = 1.0
        print('Robot Speed Slider is set to :')
        cobot.SetBaseSpeed(spdslider)

        # Mode Real or Simulation
        cobot.SetProgramMode(cobot.PG_MODE.REAL)  # set to real robot
        # cobot.SetProgramMode(cobot.PG_MODE.SIMULATION)
        if cobot.IsRobotReal:
            print('Robot Mode is [Real]')
        else:
            print('Robot Mode is [Simulation]')

        sleep(1)
        print('Robot is [Ready] for [Operation]')

    def disconnect_robot():
        cobot.DisConnectToCB()
        print('Robot is [Disconnected]')

    def get_joint_state_msg(timenow):
        jointValue = cobot.GetCurrentJoint()
        jointMsg = JointState()
        jointMsg.header.stamp = timenow
        jointMsg.name = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
        jointMsg.position = np.deg2rad([jointValue.j0, jointValue.j1, jointValue.j2, jointValue.j3, jointValue.j4, jointValue.j5]).tolist()
        return jointMsg

    def get_tcp_state_msg(timenow):
        poseMsg = PoseStamped()
        poseMsg.header.stamp = timenow
        poseMsg.header.frame_id = 'Body_Base'
        tcpValue = cobot.GetCurrentTCP() # in mm, degree
        t = (0.001*np.array([tcpValue.x, tcpValue.y, tcpValue.z])).tolist()
        r = np.array([tcpValue.rx, tcpValue.ry, tcpValue.rz])
        rpy = R.from_euler('xyz', r, degrees=True)
        q = rpy.as_quat()
        poseMsg.pose.position = Point(x=t[0], y=t[1], z=t[2])
        poseMsg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return poseMsg

    def req_tcp(request, response):
        rqx = request.tcppose.pose.position.x
        rqy = request.tcppose.pose.position.y
        rqz = request.tcppose.pose.position.z
        rqqx = request.tcppose.pose.orientation.x
        rqqy = request.tcppose.pose.orientation.y
        rqqz = request.tcppose.pose.orientation.z
        rqqw = request.tcppose.pose.orientation.w
        movetype = request.movetype
        spd = request.spd
        acc = request.acc
        r = R.from_quat([rqqx, rqqy, rqqz, rqqw])
        rpy = r.as_euler('xyz', degrees=True)

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

    def req_joint(request, response):
        jointReq = np.rad2deg(request.jntstate.position)
        spd = request.spd
        acc = request.acc

        # TODO : Check joint limit, if the request is in limit then move, return false otherwise
        inLimit = True
        if inLimit:
            cobot.MoveJ(jointReq[0], jointReq[1], jointReq[2], jointReq[3], jointReq[4], jointReq[5], spd, acc)
            response.success = True
            response.message = "Request [Successful], Robot should be [MOVING]"
        else:
            response.success = False
            response.message = "Request [Failed], Request Value is [outside of joint limit]"
        return response

    def req_joint_trajectory(goalHandle):
        req = goalHandle.request.jntseq.points
        spd = goalHandle.request.spd
        acc = goalHandle.request.acc

        cobot.MoveJB_Clear()
        for jntpoint in req:
            position = np.rad2deg(jntpoint.positions)
            cobot.MoveJB_Add(position[0], position[1], position[2], position[3], position[4], position[5])
        cobot.MoveJB_Run(spd, acc)

        # publish composed feedback
        feedbackMsg = ReqJntTraj.Feedback()
        feedbackMsg.progression = 0.0
        # TODO : Check current progress and keep update
        while not cobot.IsIdle():
            feedbackMsg.progression += 1.0
            goalHandle.publish_feedback(feedbackMsg)
            sleep(0.5)

        # set success
        goalHandle.succeed()

        # compose result and send out once finished
        result = ReqJntTraj.Result()
        result.success = True
        result.message = "Joint Trajectory is [Executed]"
        return result

    def req_tcp_trajectory_itpl(goalHandle):
        tcppose = goalHandle.request.tcpseq
        spd = goalHandle.request.spd
        acc = goalHandle.request.acc
        movetype = RBRobotROS2Bridge.get_itpl_movetype(goalHandle.request.movetype)

        cobot.MoveITPL_Clear()
        for tcp in tcppose:
            x = tcp.pose.position.x
            y = tcp.pose.position.y
            z = tcp.pose.position.z
            rqqx = tcp.pose.orientation.x
            rqqy = tcp.pose.orientation.y
            rqqz = tcp.pose.orientation.z
            rqqw = tcp.pose.orientation.w
            r = R.from_quat([rqqx, rqqy, rqqz, rqqw])
            rpy = r.as_euler('xyz', degrees=True)
            cobot.MoveITPL_Add(x, y, z, rpy[0], rpy[1], rpy[2], spd)
        cobot.MoveITPL_Run(acc, movetype)

        # set success
        goalHandle.succeed()

        # compose result and send out once finished
        result = ReqJntTraj.Result()
        result.success = True
        result.message = "TCP Trajectory is executed"
        return result

    def get_itpl_movetype(index):
        movetype = [cobot.ITPL_RTYPE.INTENDED,
                    cobot.ITPL_RTYPE.CONSTANT,
                    cobot.ITPL_RTYPE.RESERVED1,
                    cobot.ITPL_RTYPE.SMOOTH,
                    cobot.ITPL_RTYPE.RESERVED2,
                    cobot.ITPL_RTYPE.CA_INTENDED,
                    cobot.ITPL_RTYPE.CA_CONSTANT,
                    cobot.ITPL_RTYPE.RESERVED3,
                    cobot.ITPL_RTYPE.CA_SMOOTH]
        return movetype[index]
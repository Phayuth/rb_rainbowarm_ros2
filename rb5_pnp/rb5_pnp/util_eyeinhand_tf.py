import sys
import os

sys.path.append(str(wd=os.path.abspath(os.getcwd())))
from spatial_transformation import RigidBodyTransformation as rbt
import numpy as np

"""
Problem : Determine H `camera_link` to `Tool Center Point`
Solution : H `camera_link` to `tcp` = H `camera_color_optical_frame` to `tcp` @ H `camera_link` to `camera_color_optical_frame`
Note : If we have H `child` to `parent`, use <ros2 run tf2_ros tf2_echo `parent` `child`>

Step 1 : From eye-in-hand calibration, we get H `camera_color_optical_frame` to `tool0`.
Step 2 : From extrinsic information, we get H `camera_link` to `camera_color_optical_frame`.
         Terminal : <ros2 run tf2_ros tf2_echo camera_color_optical_frame camera_link>
Step 3 : Perform multiplication
RESULT : 0.02151967, -0.07698767,  0.08906624, 0.04974107,  0.12450784, -0.68294573,  0.71805902
"""


HccoTtool0 = [
    0.03641606494177693,
    -0.0759852953431285,
    0.08761958955199689,
    0.05612827864548544,
    -0.7861661407792222,
    0.615214712689562,
    0.017414727069324697,
]  # xyz qxqyqzqw
HccoTtool0 = rbt.conv_t_and_quat_to_h(HccoTtool0[0:3], HccoTtool0[3:7])

HclTcco = [0.015, 0.000, 0.000, 0.500, -0.498, 0.500, 0.503]
HclTcco = rbt.conv_t_and_quat_to_h(HclTcco[0:3], HclTcco[3:7])

HclTtool0 = HccoTtool0 @ HclTcco
print(f"> HclTtool0: {HclTtool0}")
HclTtool0 = rbt.conv_h_to_t_and_quat(HclTtool0)
print(f"> HclTtool0: {HclTtool0}")

import sys
for _p in ("/home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/lib/python3/dist-packages",
           "/home/uavg/ros_car/devel/lib/python3/dist-packages"):
    if _p not in sys.path: sys.path.insert(0, _p)
import hiwonder_kinematics.transform as tf
from hiwonder_kinematics.inverse_kinematics import get_ik, set_link, set_joint_range
set_link(tf.base_link, tf.link1, tf.link2, tf.link3, tf.tool_link)
set_joint_range(tf.joint1, tf.joint2, tf.joint3, tf.joint4, tf.joint5, 'deg')
def ik(x,y,z,p):
    s=get_ik([x,y,z],float(p),[-180,180],1)
    return [int(round(q)) for q in tf.angle2pulse([s[0][0][0]])[0]] if s else None
# 安全放置路径 (抓取点约 0.295,0.013,0.137)
path=[("W1 lift-up",0.295,0.013,0.237,80),
      ("W2 high-fwd-center",0.24,0.0,0.26,50),
      ("W3 high-fwd-RIGHT",0.24,-0.18,0.24,50),
      ("W4 place-RIGHT-low",0.26,-0.20,0.12,30)]
for name,x,y,z,p in path:
    r=ik(x,y,z,p)
    print("%-22s [%.3f,%.3f,%.3f] pitch=%d -> %s"%(name,x,y,z,p,("OK "+str(r)) if r else "无解"))

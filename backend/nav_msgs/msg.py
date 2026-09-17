from geometry_msgs.msg import Pose, Twist
class Header:
    stamp: any = None
    frame_id: str = ''
class PoseWithCovariance:
    pose: Pose = Pose()
class TwistWithCovariance:
    twist: Twist = Twist()
class Odometry:
    header: Header = Header()
    child_frame_id: str = ''
    pose: PoseWithCovariance = PoseWithCovariance()
    twist: TwistWithCovariance = TwistWithCovariance()

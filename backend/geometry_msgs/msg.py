class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
class Twist:
    linear: Vector3 = Vector3()
    angular: Vector3 = Vector3()
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0
class Pose:
    position: Point = Point()
    orientation: Quaternion = Quaternion()
class Header:
    stamp: any = None
    frame_id: str = ''
class PoseStamped:
    header: Header = Header()
    pose: Pose = Pose()

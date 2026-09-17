from geometry_msgs.msg import Point, Vector3, Header
class PositionTarget:
    FRAME_LOCAL_NED = 1
    FRAME_BODY_NED = 8
    IGNORE_PX = 1
    IGNORE_PY = 2
    IGNORE_PZ = 4
    IGNORE_AFX = 64
    IGNORE_AFY = 128
    IGNORE_AFZ = 256
    IGNORE_YAW = 1024
    header: Header = Header()
    coordinate_frame: int = 1
    type_mask: int = 0
    position: Point = Point()
    velocity: Vector3 = Vector3()
    yaw: float = 0.0
    yaw_rate: float = 0.0

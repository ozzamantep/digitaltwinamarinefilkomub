from geometry_msgs.msg import Quaternion, Vector3
class Header:
    stamp: any = None
    frame_id: str = ''
class Image:
    header: Header = Header()
    height: int = 0
    width: int = 0
    data: bytes = b''
class Imu:
    header: Header = Header()
    orientation: Quaternion = Quaternion()
    angular_velocity: Vector3 = Vector3()
    linear_acceleration: Vector3 = Vector3()
class BatteryState:
    header: Header = Header()
    voltage: float = 0.0
    current: float = 0.0
    percentage: float = 0.0
    temperature: float = 0.0
    present: bool = True
class Range:
    header: Header = Header()
    range: float = 0.0
    min_range: float = 0.0
    max_range: float = 0.0
class FluidPressure:
    header: Header = Header()
    fluid_pressure: float = 0.0

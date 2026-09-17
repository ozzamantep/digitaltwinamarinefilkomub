class Node:
    def __init__(self, node_name: str, **kwargs): pass
    def create_publisher(self, msg_type, topic: str, qos_profile): pass
    def create_subscription(self, msg_type, topic: str, callback, qos_profile): pass
    def create_client(self, srv_type, srv_name: str): pass
    def create_timer(self, period_sec: float, callback): pass
    def get_clock(self):
        class Clock:
            def now(self):
                class Time:
                    nanoseconds = 0
                    def to_msg(self): return None
                return Time()
        return Clock()
    def get_logger(self):
        class Logger:
            def info(self, msg): pass
            def warn(self, msg): pass
            def error(self, msg): pass
            def debug(self, msg): pass
        return Logger()
    def destroy_node(self): pass

class ReliabilityPolicy:
    RELIABLE = 1
    BEST_EFFORT = 2
class HistoryPolicy:
    KEEP_LAST = 1
    KEEP_ALL = 2
class QoSProfile:
    def __init__(self, reliability=None, history=None, depth=10): pass

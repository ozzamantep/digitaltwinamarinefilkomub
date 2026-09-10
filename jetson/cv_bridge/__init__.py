class CvBridge:
    def imgmsg_to_cv2(self, msg, desired_encoding='bgr8'):
        import numpy as np
        return np.zeros((480, 640, 3), dtype=np.uint8)
    def cv2_to_imgmsg(self, cvim, encoding='bgr8'):
        return None

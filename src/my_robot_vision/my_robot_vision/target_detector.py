import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


class TargetDetector(Node):

    def __init__(self):
        super().__init__('target_detector')
        self.bridge = CvBridge()
        self.lower1 = np.array([0, 120, 80])
        self.upper1 = np.array([10, 255, 255])
        self.lower2 = np.array([170, 120, 80])
        self.upper2 = np.array([180, 255, 255])
        self.min_area = 50.0
        self.frame_count = 0
        self.pub_detections = self.create_publisher(Detection2DArray, '/target/detections', 10)
        self.pub_debug = self.create_publisher(Image, '/target/image_debug', qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower1, self.upper1) + cv2.inRange(hsv, self.lower2, self.upper2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        out = Detection2DArray()
        out.header = msg.header

        if contours:
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(contour)
                bbox = BoundingBox2D()
                bbox.center.position.x = float(x + w / 2)
                bbox.center.position.y = float(y + h / 2)
                bbox.size_x = float(w)
                bbox.size_y = float(h)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = 'red_box'
                hypothesis.hypothesis.score = float(area)
                detection = Detection2D()
                detection.header = msg.header
                detection.results.append(hypothesis)
                detection.bbox = bbox
                out.detections.append(detection)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (x + w // 2, y + h // 2), 4, (0, 255, 0), -1)

        cv2.drawMarker(frame, (frame.shape[1] // 2, frame.shape[0] // 2), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
        self.pub_detections.publish(out)
        self.frame_count += 1
        if self.frame_count % 3 == 0 and self.pub_debug.get_subscription_count() > 0:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))


def main():
    rclpy.init()
    node = TargetDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

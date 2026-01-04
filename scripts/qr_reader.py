#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from std_msgs.msg import String
from geometry_msgs.msg import Point

class QRReader:
    def __init__(self):
        rospy.init_node('qr_reader')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/custom_camera/image_raw", Image, self.image_callback)
        self.qr_pub = rospy.Publisher("/detected_qr", String, queue_size=1)
        self.metric_pub = rospy.Publisher("/detected_qr_metric", Point, queue_size=1)
        self.detected_qr = set()
        print("QR Reader Started! Publishers: /detected_qr, /detected_qr_metric")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Decode QR codes
        decoded_objects = decode(cv_image)
        h, w, _ = cv_image.shape
        
        for obj in decoded_objects:
            qr_data = obj.data.decode("utf-8")
            
            # Draw
            points = obj.polygon
            if len(points) == 4:
                pts = np.array(points, dtype=np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(cv_image, [pts], True, (0, 255, 0), 3)
                
                # Calculate Centroid & Area
                M = cv2.moments(pts)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    area = cv2.contourArea(pts)
                    
                    # Publish Metric Data (x=cx, y=cy, z=area)
                    metric_msg = Point()
                    metric_msg.x = float(cx)
                    metric_msg.y = float(cy)
                    metric_msg.z = float(area)
                    self.metric_pub.publish(metric_msg)
            
            if qr_data not in self.detected_qr:
                self.detected_qr.add(qr_data)
                print(f"🔥 NEW QR CODE DETECTED: {qr_data}")
                self.qr_pub.publish(qr_data)

        # Show image (Optional, careful with X forwarding)
        cv2.imshow("Robot Camera", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        reader = QRReader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

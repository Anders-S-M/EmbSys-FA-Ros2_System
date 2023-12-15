import rclpy
import cv2
import time
import random
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32
from sensor_msgs.msg import Image


class CVAlign(Node):

    def __init__(self):
        super().__init__('cv_align_node')
        self.bridge = CvBridge()
        self.subsciber_ = self.create_subscription(Image, '/image_align', self.get_angle, 1)
        self.publisher_ = self.create_publisher(Float32, '/align_angle', 10)

    def find_contour_center(self, contour):
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
        else:
            return None

    def calculate_angle_from_center(self, center, point):
        dx = point[0] - center[0]
        dy = point[1] - center[1]
        angle = np.arctan2(dy, dx) * 180 / np.pi
        return (angle + 360) % 360
    
    def get_angle(self, img):
        # Indlæs billede
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e)
            print("Billed fucekd")
        
        start_time = time.time()
        # Skaler billedet ned
        #scale_percent = 50
        #width = int(cv_image.shape[1] * scale_percent / 100)
        #height = int(cv_image.shape[0] * scale_percent / 100)
        #dim = (width, height)
        #resized = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

        # Konverter til gråskala
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        filename = 'bw.jpg'
        cv2.imwrite(filename, gray) 

        print(gray.shape[1], gray.shape[0])

        # Anvend Gaussisk Blur for at reducere støj
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)


        # Anvend Canny-kantdetektor
        edges = cv2.Canny(blurred, threshold1=60, threshold2=150)


        # Morfologisk operation for at reducere små prikker
        kernel = np.ones((3,3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=1)


        # Find konturer
        contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Opret et tomt billede til at tegne de filtrerede konturer
        filtered_edges = np.zeros_like(edges)

        # Filtrér og tegn konturer
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:  # Juster tærskelværdien for areal efter behov
                cv2.drawContours(filtered_edges, [cnt], -1, (255, 255, 255), 2)

        # Find hjørner med Shi-Tomasi Corner Detector
        corners = cv2.goodFeaturesToTrack(filtered_edges, maxCorners=6, qualityLevel=0.02, minDistance=120)

        # Find møtrikkens centrum
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            center = self.find_contour_center(largest_contour)

            if center is not None:

                angles_to_corners = []
                for corner in corners:
                    x, y = corner.ravel()
                    # Konverter x, y til heltal
                    x, y = int(x), int(y)
                    angle = self.calculate_angle_from_center(center, (x, y))
                    angles_to_corners.append(angle)
                    cv2.line(filtered_edges, center, (x, y), (255, 0, 0), 2) # Tegn linje til hvert hjørne
                
                close_deg = []
                for angle in angles_to_corners:
                    close_deg.append(abs(angle-90))

                filename = 'finish.jpg'
                cv2.imwrite(filename, filtered_edges) 

                index_of_close = close_deg.index(min(close_deg))

                end_time = time.time()

                msg = Float32()
                msg.data = angles_to_corners[index_of_close]-90
                self.publisher_.publish(msg)
                self.get_logger().info("Publishing")


            elapsed_time_seconds = (end_time - start_time) 
            self.get_logger().info(f"Time to tun algorithm: {elapsed_time_seconds} seconds")

        else:
            print("Ingen konturer fundet")

def main(args=None):
    rclpy.init(args=args)

    cv_align = CVAlign()

    rclpy.spin(cv_align)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cv_align.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
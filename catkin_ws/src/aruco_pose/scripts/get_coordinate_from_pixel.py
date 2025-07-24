#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import csv
import rospkg
import tf2_ros
import geometry_msgs.msg
from aruco_pose.srv import Get3DPoint

class RealSenseCoordNode:
    def __init__(self):
        rospy.init_node('realsense_coord_node')

        # Subscribe to the color image for visualization
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.color_image = None

        # Setup TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Wait and setup service client
        rospy.loginfo("Waiting for /get_3d_coordinates service...")
        rospy.wait_for_service('/get_3d_coordinates')
        self.get_3d_point_srv = rospy.ServiceProxy('/get_3d_coordinates', Get3DPoint)
        rospy.loginfo("/get_3d_coordinates service connected.")

        # Get CSV path
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('aruco_pose')
        self.csv_file = pkg_path + '/detected_points.csv'

    def image_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))

    def get_3d_point(self, u, v):
        try:
            resp = self.get_3d_point_srv(u, v)
            if resp.valid:
                return (resp.x, resp.y, resp.z)
            else:
                rospy.logwarn("Invalid 3D point returned for pixel ({}, {})".format(u, v))
                return None
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: {}".format(e))
            return None

    def publish_tf(self, idx, x, y, z):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_depth_link"
        t.child_frame_id = "point_{}".format(idx)
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1
        self.tf_broadcaster.sendTransform(t)

    def process_csv_and_publish(self):
        points = []
        coords = []
        prompts = []

        try:
            with open(self.csv_file, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # skip header

                for row in reader:
                    if len(row) < 3:
                        continue
                    frame_no = row[0]
                    prompt = row[1]
                    try:
                        pixel_list = eval(row[2])  # expects format like "[(u,v), (u2,v2)]"
                    except Exception as e:
                        rospy.logwarn("Failed to eval pixel list: {}".format(e))
                        continue

                    rospy.loginfo("Frame {}, Prompt: {}".format(frame_no, prompt))

                    for idx, (u, v) in enumerate(pixel_list):
                        point_3d = self.get_3d_point(u, v)
                        if point_3d is not None:
                            x, y, z = point_3d
                            points.append((u, v))
                            coords.append((x, y, z))
                            prompts.append(prompt)
                            self.publish_tf(idx, x, y, z)
                        else:
                            rospy.logwarn("No valid depth at pixel ({}, {})".format(u, v))
        except IOError as e:
            rospy.logerr("Failed to open CSV file: {}".format(e))
            return

        # Visualization on the latest color image
        if self.color_image is not None:
            img = self.color_image.copy()
            for (u, v), (x, y, z), prompt in zip(points, coords, prompts):
                label = "{} X:{:.3f} Y:{:.3f} Z:{:.3f}m".format(prompt, x, y, z)
                cv2.circle(img, (u, v), 5, (0, 0, 255), -1)
                cv2.putText(img, label, (u + 10, v + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("3D Points with Prompts", img)
            cv2.waitKey(1)

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    node = RealSenseCoordNode()
    rate = rospy.Rate(1)  # 1 Hz
    try:
        while not rospy.is_shutdown():
            if node.color_image is not None:
                node.process_csv_and_publish()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()

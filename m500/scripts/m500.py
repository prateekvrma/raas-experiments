import rospy
import time
from sensor_msgs.msg import CameraInfo, Image


class ImageComb():
    def __init__(self):
        self.publisher = rospy.Publisher("/tracking/camera_info", CameraInfo, queue_size=10)
        self.subscriber = rospy.Subscriber("/tracking/image_raw", Image, self.image_cb)

        self.camera_info_msg = self.camera_info()

    def image_cb(self, msg):
        self.camera_info_msg.header = msg.header
        self.publisher.publish(self.camera_info_msg)

    def camera_info(self):
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_msg.header.frame_id = "robot_camera_link"
        camera_info_msg.width = 640
        camera_info_msg.height = 480
        camera_info_msg.K = [2.8460618358450944e+02, 0.0, 3.4433715463535600e+02, 0.0, 2.8466678305545383e+02, 2.1824166864367820e+02, 0.0, 0.0, 1.0]
        camera_info_msg.D = [-1.4098811199286247e-02, 3.1752368031364832e-03, 0., 0.]
        camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info_msg.P = [2.8460618358450944e+02, 0.0, 3.4433715463535600e+02, -0.0, 0.0, 2.8466678305545383e+02, 2.1824166864367820e+02, 0.0, 0.0, 0.0, 1.0, 0.0]
        camera_info_msg.distortion_model = "fisheye"

        return camera_info_msg

if __name__ == "__main__":
    # Initialize publisher node
    rospy.init_node("camera_info", anonymous=True)
    my_cameracomb = ImageComb()

    rate = rospy.Rate(30)

    # Run publisher 
    while not rospy.is_shutdown():
        # camera_info_msg.header.stamp = rospy.Time.now()
        # publisher.publish(camera_info_msg)
        rate.sleep()

"""
M: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 2.8460618358450944e+02, 0., 3.4433715463535600e+02, 0.,
       2.8466678305545383e+02, 2.1824166864367820e+02, 0., 0., 1. ]
D: !!opencv-matrix
   rows: 4
   cols: 1
   dt: d
   data: [ -1.4098811199286247e-02, 3.1752368031364832e-03, 0., 0. ]
reprojection_error: 3.8303682626126156e-01
width: 640
height: 480
distortion_model: fisheye
calibration_time: "2022-08-29 19:33:43"

"""


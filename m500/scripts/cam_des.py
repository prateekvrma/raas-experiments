
import rospy
import time
from sensor_msgs.msg import CameraInfo

def camera_info():

    camera_info_msg = CameraInfo()
    camera_info_msg.header.stamp = rospy.Time.now()
    camera_info_msg.header.frame_id = "robot_camera_link"
    camera_info_msg.width = 320
    camera_info_msg.height = 240
    camera_info_msg.K = [277.191356, 0.0, 320.5, 0.0, 277.191356, 240.5, 0.0, 0.0, 1.0]
    camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info_msg.P = [277.191356, 0.0, 320.5, -0.0, 0.0, 277.191356, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    camera_info_msg.distortion_model = "plumb_bob"

    return camera_info_msg

if __name__ == "__main__":
    # Initialize publisher node
    rospy.init_node("camera_info", anonymous=True)
    camera_info_msg = camera_info()
    publisher = rospy.Publisher("/iris/usb_cam/camera_info_anc", CameraInfo, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher 
    while not rospy.is_shutdown():
        camera_info_msg.header.stamp = rospy.Time.now()
        publisher.publish(camera_info_msg)
        rate.sleep()
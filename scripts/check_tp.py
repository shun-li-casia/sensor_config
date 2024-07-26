import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

# Initialize CvBridge
bridge = CvBridge()

def print_camera_info_timestamp1(camera_info_msg):
    # Print the timestamp from the camera_info message
    print(f"Camera Info 1 Timestamp: {camera_info_msg.header.stamp.secs}.{camera_info_msg.header.stamp.nsecs}")

def print_camera_info_timestamp2(camera_info_msg):
    # Print the timestamp from the camera_info message
    print(f"Camera Info 2 Timestamp: {camera_info_msg.header.stamp.secs}.{camera_info_msg.header.stamp.nsecs}")

def print_image_timestamp1(image_msg):
    # Convert the ROS Image message to OpenCV format if needed
    # cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    
    # Print the timestamp from the image message
    print(f"Image 1 Timestamp: {image_msg.header.stamp.secs}.{image_msg.header.stamp.nsecs}")

def print_image_timestamp2(image_msg):
    # Convert the ROS Image message to OpenCV format if needed
    # cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    
    # Print the timestamp from the image message
    print(f"Image 2 Timestamp: {image_msg.header.stamp.secs}.{image_msg.header.stamp.nsecs}")

def main():
    # Initialize the ROS node
    rospy.init_node('image_timestamps', anonymous=True)
    
    # Create subscribers for the two image topics
    rospy.Subscriber('/uav_0/rect/cam_0', Image, print_image_timestamp1)
    rospy.Subscriber('/uav_0/rect/cam_1', Image, print_image_timestamp2)
    rospy.Subscriber('/uav_0/rect/cam_0_info', CameraInfo, print_camera_info_timestamp1)
    rospy.Subscriber('/uav_0/rect/cam_1_info', CameraInfo, print_camera_info_timestamp2)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()

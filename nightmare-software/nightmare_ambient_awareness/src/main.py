import os
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# import local modules
sys.path.append(os.path.dirname(os.path.realpath(__file__)))  # noqa

from vision_modules.static_saliency import get_intensity, get_static_saliency
from vision_modules.motion_saliency import get_flicker
from vision_modules.pose_detector_coral import detect_poses


class imageProcessorNode():
    def __init__(self):
        # Instantiate CvBridge
        self.bridge = CvBridge()

        # utils
        self.first_image = True

        # initialize frame containers
        self.frame = 0
        self.prev_frame = 0
        self.prev_intensity_frame = 0
        self.image_static_saliency = 0
        self.image_intensiy = 0
        self.image_flicker = 0

        # publisher for cooked images
        self.image_publisher = rospy.Publisher('/image_processor/cooked_image', Image, queue_size=10)

    def process_frame(self):
        rospy.loginfo("frame processed!")
        self.image_static_saliency = get_static_saliency(self.frame)
        self.image_intensiy = get_intensity(self.frame)
        self.image_flicker = get_flicker(self.image_intensiy, self.prev_intensity_frame)
        img = detect_poses(self.frame)
        msg_frame = CvBridge().cv2_to_imgmsg(img)
        self.image_publisher.publish(msg_frame)

    def image_callback(self, msg):
        # rospy.loginfo("image received")
        try:
            # Convert ROS Image message to OpenCV2
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(str(e))
        else:
            # execute this on start to initialize image buffers
            if self.first_image:
                self.first_image = False
                self.prev_frame = self.frame
                self.prev_intensity_frame = get_intensity(self.frame)
                return

            self.process_frame()

            self.prev_frame = self.frame
            self.prev_intensity_frame = get_intensity(self.frame)


def main():
    rospy.init_node('image_listener')
    image_proc = imageProcessorNode()

    # subscriber for raw images
    rospy.Subscriber("/camera/color/image_raw", Image, image_proc.image_callback)

    rospy.spin()


if __name__ == '__main__':
    main()

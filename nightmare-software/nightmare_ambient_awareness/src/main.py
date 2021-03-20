import os
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# import local modules
sys.path.append(os.path.dirname(os.path.realpath(__file__)))  # noqa

from vision_modules.static_saliency import get_intensity, get_static_saliency
from vision_modules.motion_saliency import get_flicker
from vision_modules.pose_detector_coral import detect_poses, overlay_on_image
from vision_modules.marker_detector import detect_markers, overlay_marker_on_image


class imageProcessorNode():
    def __init__(self):
        # Instantiate CvBridge
        self.bridge = CvBridge()

        # utils
        self.first_image = True
        self.raw_poses = 0
        self.raw_markers = 0

        # initialize frame containers
        # utils
        self.frame = 0
        self.prev_frame = 0
        self.prev_intensity_frame = 0
        # filters
        self.image_static_saliency = 0
        self.image_intensiy = 0
        self.image_flicker = 0
        # output
        self.image_output = 0

        # publisher for cooked images
        self.image_publisher = rospy.Publisher('/image_processor/cooked_image', Image, queue_size=10)

    def process_frame(self):
        # execute filters
        self.image_static_saliency = get_static_saliency(self.frame)
        self.image_intensiy = get_intensity(self.frame)
        self.image_flicker = get_flicker(self.image_intensiy, self.prev_intensity_frame)
        # execute detectors
        self.raw_poses = detect_poses(self.frame)
        self.raw_markers = detect_markers(self.frame)
        print(self.raw_markers)
        # build final frame
        self.image_output = overlay_on_image(self.frame, self.raw_poses)
        self.image_output = overlay_marker_on_image(self.image_output, self.raw_markers)

        msg_frame = CvBridge().cv2_to_imgmsg(self.image_output)
        self.image_publisher.publish(msg_frame)
        rospy.loginfo("frame processed!")

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

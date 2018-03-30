# OpenCV package
import cv2
from cv_bridge import CvBridge

# rospy package
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan


class Server():
    def __init__(self):
        # initialize server node
        rospy.init_node("server", anonymous=True)

        self.OBSTACLE_MIN_DIST = 0.4
        self.RANGE_FILTER = "HSV"

        # setup cmd_vel publisher
        self.cmd_vel_publisher = rospy.Publisher(
            "cmd_vel", Twist, queue_size=1)

    # run the logic (search for balls)
    def run(self, show=False, dev=False):
        if dev:
            self.setup_range_filter_trackbar()

        self.twist = Twist()

        while not rospy.is_shutdown():
            # scan_msg = rospy.wait_for_message("scan", LaserScan)
            camera_msg = rospy.wait_for_message("/camera/rgb/image_raw", Image)

            cv_img = CvBridge().imgmsg_to_cv2(camera_msg, "bgr8")
            cv_img = cv2.resize(cv_img, (0, 0), fx=.5, fy=.5)

            frame_to_thresh = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

            if dev:
                v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = self.get_range_filter_trackbar_values()
            else:
                v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = self.get_range_filter_red() # can be change to green & blue

            mask = cv2.inRange(
                frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

            # get ball coordination
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            # only proceed if at least one contour was found
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                print center

            if show:
                cv2.imshow("Camera", cv_img)
                if dev:
                    res = cv2.bitwise_and(cv_img, cv_img, mask=mask)
                    cv2.imshow('res', res)
                cv2.waitKey(1)

            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.3

            # rospy.loginfo(self.twist)
            # self.cmd_vel_publisher.publish(self.twist)

    # setup trackbar for range filter
    def setup_range_filter_trackbar(self):
        cv2.namedWindow("Trackbars", 0)

        for j in self.RANGE_FILTER:
            for i in ["MIN", "MAX"]:
                vmax = 255
                if i == "MIN":
                    v = 0
                elif j == "H":
                    v = 179
                    vmax = 179
                else:
                    v = 255
                cv2.createTrackbar("%s_%s" %
                                   (j, i), "Trackbars", v, vmax, self.trackbar_callback)

    # trackback callback
    def trackbar_callback(self, value):
        pass

    # get range filter value from trackbar
    def get_range_filter_trackbar_values(self):
        values = []

        for i in ["MIN", "MAX"]:
            for j in self.RANGE_FILTER:
                v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
                values.append(v)

        return values

    # get range filter value for red
    def get_range_filter_red(self):
        return [0, 12, 0, 59, 255, 255]

    # get range filter value for green
    def get_range_filter_green(self):
        return [60, 0, 0, 119, 255, 255]

    # get range filter value for blue
    def get_range_filter_blue(self):
        return [120, 0, 0, 179, 255, 255]


if __name__ == '__main__':
    Server().run(True, True)

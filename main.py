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

        self.OBSTACLE_MIN_DIST = 0.4  # define the minimum distance to obstacle
        self.RANGE_FILTER = "HSV"  # define range filter type
        self.current_search_color = 0  # 0=red, 1=green, 2=blue

        # define a list objects that it need to find
        self.colorDetectionList = [
            self.get_range_filter_red(),
            self.get_range_filter_green(),
            self.get_range_filter_blue(),
            self.get_range_filter_yellow()
        ]

        # setup cmd_vel publisher
        self.cmd_vel_publisher = rospy.Publisher(
            "cmd_vel", Twist, queue_size=1)

    # run the logic (search for balls)
    def run(self, show=False, dev=False, testMode=False):
        if testMode:
            # setup range filter trackbar if in test mode
            self.setup_range_filter_trackbar()
        else:
            # setup predefine range filter colors
            v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = self.colorDetectionList[
                self.current_search_color]

        self.twist = Twist()

        while not rospy.is_shutdown():
            # enable range filter trackbar if in test mode
            if testMode:
                v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = self.get_range_filter_trackbar_values()

            # receive camera image from turtlebot
            camera_msg = rospy.wait_for_message("/camera/rgb/image_raw", Image)
            # convert image to openCV image
            cv_img = CvBridge().imgmsg_to_cv2(camera_msg, "bgr8")
            # resize image
            cv_img = cv2.resize(cv_img, (0, 0), fx=.5, fy=.5)
            # convert image color to HSV
            frame_to_thresh = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

            # get the mask based on the predefine HSV color
            mask = cv2.inRange(
                frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

            # get ball coordination
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            # this step will be skiped if it is in test mode
            if not testMode:
                # only proceed if at least one contour was found
                if len(cnts) > 0:
                    c = max(cnts, key=cv2.contourArea)
                    M = cv2.moments(c)

                    if M["m00"] == 0:
                        break

                    center = (int(M["m10"] / M["m00"]),
                              int(M["m01"] / M["m00"]))
                    cv2.circle(cv_img, center, 20, (0, 0, 0))

                    if center[0] <= cv_img.shape[1]/5*2:
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.1
                        # print "Adjust to left"
                    elif center[0] >= cv_img.shape[1]/5*3:
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = -0.1
                        # print "Adjust to right"
                    else:
                        scan_msg = rospy.wait_for_message("scan", LaserScan)

                        if scan_msg.ranges[1] > self.OBSTACLE_MIN_DIST+0.2:
                            self.twist.linear.x = 0.5
                            self.twist.angular.z = 0.0
                            # print "Forward to target ", self.current_search_color
                        elif scan_msg.ranges[1] > self.OBSTACLE_MIN_DIST:
                            self.twist.linear.x = 0.1
                            self.twist.angular.z = 0.0
                            # print "Forward to target ", self.current_search_color
                        else:
                            # found
                            self.twist.linear.x = 0.0
                            self.twist.angular.z = 0.0
                            print "Target Found ", self.current_search_color
                            self.current_search_color += 1
                            self.cmd_vel_publisher.publish(self.twist)
                            break
                else:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.3
                    # print "Searching..."

            # show camera view
            if show:
                if dev:
                    res = cv2.bitwise_and(cv_img, cv_img, mask=mask)
                    cv2.imshow('Target', res)

                cv2.line(cv_img, (cv_img.shape[1]/5*2, 0),
                         (cv_img.shape[1]/5*2, cv_img.shape[0]), (0, 0, 0), 3)
                cv2.line(cv_img, (cv_img.shape[1]/5*3, 0),
                         (cv_img.shape[1]/5*3, cv_img.shape[0]), (0, 0, 0), 3)
                cv2.imshow("Camera", cv_img)
                cv2.waitKey(1)

            # send the movement command
            self.cmd_vel_publisher.publish(self.twist)

        # check if there is still have objects to detect/find
        if self.current_search_color < len(self.colorDetectionList):
            self.run(show, dev)

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
        return [0, 12, 0, 29, 255, 255]

    # get range filter value for green
    def get_range_filter_green(self):
        return [60, 0, 0, 119, 255, 255]

    # get range filter value for blue
    def get_range_filter_blue(self):
        return [120, 0, 0, 179, 255, 255]

    # get range filter value for yellow
    def get_range_filter_yellow(self):
        return [1, 0, 0, 59, 255, 255]


if __name__ == '__main__':
    Server().run(True, True, False)

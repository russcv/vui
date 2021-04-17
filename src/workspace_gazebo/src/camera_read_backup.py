#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import MeanShift, estimate_bandwidth

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from voice_ui.srv import speech_to_text
# from voice_ui.srv import speech_to_text as stt_client
# from voice_ui.srv import speech_to_text, speech_to_textResponse
from voice_ui import stt_client as sttc
class camera:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/mybot/camera/image_raw", Image, self.callback)

    def callback(self,data):
        bridge = CvBridge()

        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        image = cv_image

        resized_image = cv2.resize(image, (320, 320))

        #cv2.imshow("Camera output normal", image)
        # cv2.imshow("Camera output resized", resized_image)

        # cv2.waitKey(3)

        # MASK

        # new
        # try:
        #     rospy.init_node("speech_to_text_client", anonymous = False)
        #     stt = rospy.ServiceProxy('speech_to_text',speech_to_text)
        #     # Call the service here
        #     service_response = stt(True)
        #     mask_color = get_workspace_mask()
        #     rospy.loginfo("color %s", mask_color)
        # except rospy.ServiceException as e:
        #     rospy.loginfo("Service call failed: %s"%e)
        # end new

        boundaries = [
        ([0, 0, 150], [100, 50, 255], ["red"]), # red
        ([0, 150, 0], [100, 255, 100], ["green"]), # Green
        ([150, 0, 0], [255, 100, 100], ["blue"]), # Blue
        # ([20, 100, 100], [30, 255, 255]) # yellow
        ]

        ctext = "my Blue world"
        c1 = pstt(ctext).get_color()
        rospy.loginfo(c1)
        print(c1)

        # Loop to update mask
        for (lower, upper, color) in boundaries:

            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            # find the colors within the specified boundaries and apply the mask
            mask = cv2.inRange(resized_image, lower, upper)
            output = cv2.bitwise_and(resized_image, resized_image, mask = mask)
            rospy.loginfo("Looking for %s objects.", color)
            # New code
            # returns true for each pixel if not blackros roslaunch python from another package
            mask_binary = np.all(output != 0, axis = -1)
            rows, cols = mask_binary.nonzero() # row, col of nonblack cell locations
            # write_data(cols,rows)

            # top_left, bot_right = get_object_coords([rows,cols])
            coords = rows, cols
            cluster_centers = get_clusters(coords)
            rospy.loginfo("Found %d clusters", len(cluster_centers))
            # Draw boundaries around objects of interest
            side_length = 50
            for centers in cluster_centers:
                top_left = int(centers[0] - side_length/2), int(centers[1] - side_length/2)
                rospy.loginfo(top_left)
                bot_right = int(centers[0] + side_length/2), int(centers[1] + side_length/2)
                image_binary = cv2.rectangle(resized_image, top_left, bot_right, color = (0, 0, 255), thickness = 1)

            # show the images
            cv2.imshow("images", np.hstack([resized_image, output]))
            # cv2.imshow("Camera output resized", resized_image)
            cv2.waitKey(0)
            # plot_image(resized_image)

def get_workspace_mask():

    parsed_color = service_response.color
    boundaries_dict = {
    "red":([0, 0, 150], [100, 50, 255]),
    "green":([0, 150, 0], [100, 255, 100]),
    "blue":([150, 0, 0], [255, 100, 100]),
    "yellow":([20, 100, 100], [30, 255, 255]),
    }
    mask_color = boundaries_dict[parsed_color]
    return mask_color


def get_clusters(coords):
    rows, cols = coords
    X = np.array(list(zip(cols,rows)))
    bandwidth = estimate_bandwidth(X, quantile=0.2)
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True).fit(X)
    cluster_centers = ms.cluster_centers_
    labels = ms.labels_
    labels_unique = np.unique(labels)
    n_clusters_=len(labels_unique)
    return cluster_centers

def write_data(x,y):
    # coords = [x,y]
    #
    # file = open("/home/russ/catkin_ws/src/workspace_gazebo/src/data.txt","a")
    # for element in coords:
    #     if element==coords[-1]:
    #         file.write('%s' % element)
    #     else:
    #         file.write('%s,' % element)
    # file.write('\n')
    # file.close()
    coords = list(zip(x,y))

    file = open("/home/russ/catkin_ws/src/workspace_gazebo/src/data.txt","a")
    for row in coords:
        file.write('%s,%s' % (row[0],row[1]))
        file.write('\n')

    file.close()


def main():
    camera()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()

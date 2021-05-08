#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import MeanShift, estimate_bandwidth

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from voice_ui.srv import speech_to_text, speech_to_textResponse
from Process_Image import get_image

import os

class camera():

    def __init__(self):
        self.image_sub = rospy.Subscriber("/mybot/camera/image_raw", Image, self.callback)

    def stt_client(self,stt_flag):
        parsed_phrase = ""
        parsed_color = ""
        parsed_ori = ""
        # Initialize the client ROS node.
        # rospy.init_node("speech_to_text_client", anonymous = False)
        # stt_flag = True #bool(sys.argv[1])
        if stt_flag:
            rospy.loginfo("Waiting for STT service...")
            rospy.wait_for_service('speech_to_text')
            # rospy.loginfo('Turning on mic') # output is already in parse.py
            # phrase = stt_client(stt_flag)
            # try:
                # create a service proxy for client
                # stt = rospy.ServiceProxy('speech_to_text',speech_to_text)
                # Call the client service here
                # service_response = stt(stt_flag)

                # parsed_phrase = service_response.text
                # parsed_color = service_response.color
                # parsed_ori = service_response.ori #orientation
                # # rospy.loginfo("stt output: %s"%parsed_phrase)
                # # rospy.loginfo("Robot is grabbing the %s object."%parsed_color)
                # rospy.loginfo('mic off')
                # #return the response to the calling function
                # return (parsed_phrase, parsed_color, parsed_ori)

            try:
                # create a service proxy
                stt = rospy.ServiceProxy('speech_to_text',speech_to_text)
                # Call the service here
                service_response = stt(stt_flag)
                # parsed_phrase = [service_response.text, service_response.color, service_response.ori]
                parsed_phrase = service_response.text
                parsed_color = service_response.color
                parsed_ori = service_response.ori
                rospy.loginfo("stt output: %s"%parsed_phrase)
                if service_response.color and service_response.ori:
                    rospy.loginfo("Robot is grabbing the %s %s object.", parsed_ori, parsed_color)
                elif service_response.color:
                    rospy.loginfo("Robot is grabbing the %s object."%parsed_color)
                elif service_response.ori:
                    rospy.loginfo("Robot is grabbing the %s object."%parsed_ori)
                rospy.loginfo('mic off')

                return parsed_phrase, parsed_color, parsed_ori
                #return the response to the calling function

            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s"%e)
        else:
            rospy.loginfo('Mic remains off')
            sys.exit(1)
        stt_flag = False

    def get_workspace_mask(self,color):
        # rospy.loginfo("get_workspace_mask color: %s",color)
        # rospy.loginfo("get_workspace_mask color Type: %s",type(color))
        # parsed_color = service_response.color
        boundaries_dict = { #BGR values
        "red":([0, 0, 150], [100, 50, 255]),
        "green":([0, 150, 0], [100, 255, 100]),
        "blue":([150, 0, 0], [255, 100, 100]),
        "yellow":([20, 100, 100], [30, 255, 255]),
        "wood":([0, 100, 180], [120, 200, 255]),
        }
                #boundaries_dict = {                "red":([0, 0, 150], [100, 50, 255]),                "green":([0, 150, 0], [100, 255, 100]),                "blue":([150, 0, 0], [255, 100, 100]),                "yellow":([20, 100, 100], [30, 255, 255]),                }
        lower, upper = boundaries_dict[color]

        return (lower, upper)

    def callback(self,data):
        bridge = CvBridge()
        image_pixels = 320,320 # width, height
        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # image = cv_image
        resized_image = cv2.resize(cv_image, (image_pixels))

        object_colors = ["red","green","blue"] # pre-defined object colors

        # create class instance
        p_i = get_image()
        # mask for only known object colors (removes background)
        output = p_i.remove_background(resized_image, object_colors)
        cv2.imshow("images", np.hstack([resized_image, output]))
        # cv2.imshow("Camera output resized", resized_image)
        cv2.waitKey(0)

        # save image
        # Image directory
        # directory = r'/home/russ/Documents/images'
        # filename = 'savedImage.jpg'
        # cv2.imwrite('/home/russ/Documents/images/savedImage.jpg', image)


        phrase, color, ori = self.stt_client(True)

        # these next lines are needed to populate row and cols if no color is present in STT
        mask_binary = np.all(output != 0, axis = -1)
        rows, cols = mask_binary.nonzero() # row, col of nonblack cell locations

        if color:
            output = p_i.apply_mask(resized_image,color)
            # lower, upper = p_i.get_workspace_mask(color)
            # create NumPy arrays from the boundaries
            # lower = np.array(lower, dtype = "uint8")
            # upper = np.array(upper, dtype = "uint8")
            # find the colors within the specified boundaries and apply the mask
            # mask = cv2.inRange(resized_image, lower, upper)
            # output = cv2.bitwise_and(resized_image, resized_image, mask = mask)
            rospy.loginfo("Looking for %s objects.", color)
            # returns true for each pixel if not blackros roslaunch python from another package
            mask_binary = np.all(output != 0, axis = -1)
            rows, cols = mask_binary.nonzero() # row, col of nonblack cell locations
            # write_data(cols,rows)

        # find clusters
        coords = rows, cols
        cluster_centers = get_clusters(coords)
        if ori:
            l = Locate(image_pixels,cluster_centers,ori)
            cluster_centers = l.get_location()

        # top_left, bot_right = get_object_coords([rows,cols])

        # else:
        #     lower, upper = self.get_workspace_mask("wood")
        #     # create NumPy arrays from the boundaries
        #     lower = np.array(lower, dtype = "uint8")
        #     upper = np.array(upper, dtype = "uint8")
        #     # find the colors within the specified boundaries and apply the mask
        #     mask = cv2.inRange(resized_image, lower, upper)
        #     output = cv2.bitwise_not(resized_image, resized_image, mask = mask)
        #     rospy.loginfo("Looking for objects.")
        #     # returns true for each pixel if not blackros roslaunch python from another package
        #     mask_binary = np.all(output != 0, axis = -1)
        #     rows, cols = mask_binary.nonzero() # row, col of nonblack cell locations

        ## This section is now in if, elif above May2

        # l = Locate(image_pixels,cluster_centers,ori)
        # cluster_centers = Locate.get_location(l)


        rospy.loginfo("Found %d clusters", len(cluster_centers))

        # Draw boundaries around objects of interest
        side_length = 50
        try:
            # check if more than one cluster is TRUE
            if cluster_centers.shape[1]:
                for centers in cluster_centers:
                    # rospy.loginfo(centers)
                    top_left = int(centers[0] - side_length/2), int(centers[1] - side_length/2)
                    rospy.loginfo(top_left)
                    bot_right = int(centers[0] + side_length/2), int(centers[1] + side_length/2)
                    image_binary = cv2.rectangle(resized_image, top_left, bot_right, color = (0, 0, 255), thickness = 1)
        # If only 1 cluster found
        except:
            top_left = int(cluster_centers[0] - side_length/2), int(cluster_centers[1] - side_length/2)
            rospy.loginfo(top_left)
            bot_right = int(cluster_centers[0] + side_length/2), int(cluster_centers[1] + side_length/2)
            image_binary = cv2.rectangle(resized_image, top_left, bot_right, color = (0, 0, 255), thickness = 1)
        # show the images
        cv2.imshow("images", np.hstack([resized_image, output]))
        # cv2.imshow("Camera output resized", resized_image)
        cv2.waitKey(0)

# class get_image:
#     def __init__(self, image, color ):
#         self.image = image
#         self.color = color
#     def get_mask(self,color):
#         # rospy.loginfo("get_workspace_mask color: %s",color)
#         # rospy.loginfo("get_workspace_mask color Type: %s",type(color))
#         # parsed_color = service_response.color
#         boundaries_dict = { #BGR values
#         "red":([0, 0, 150], [100, 50, 255]),
#         "green":([0, 150, 0], [100, 255, 100]),
#         "blue":([150, 0, 0], [255, 100, 100]),
#         "yellow":([20, 100, 100], [30, 255, 255]),
#         "wood":([0, 100, 180], [120, 200, 255]),
#         }
#                 #boundaries_dict = {                "red":([0, 0, 150], [100, 50, 255]),                "green":([0, 150, 0], [100, 255, 100]),                "blue":([150, 0, 0], [255, 100, 100]),                "yellow":([20, 100, 100], [30, 255, 255]),                }
#         lower, upper = boundaries_dict[color]
#         return (lower, upper)
#
#     pass

class Locate:
    # Locate needs pixels from camera and locations of clusters
    # top left of camera feed is (0,0) - bottom right = (image_pixels,image_pixels)
    def __init__(self, image_pixels, cluster_centers, ori):
        self.pixels = image_pixels
        self.coords = np.array(cluster_centers)
        self.orientation = ori

    def get_location(self):
        if self.orientation == "left":
            return self.get_left()
        if self.orientation == "right":
            return self.get_right()
        if self.orientation == "top":
            return self.get_top()
        if self.orientation == "bot":
            return self.get_bot()

    # example of cluster centers
    #   [x             y]
    # [[184.5        112.5       ]
    #  [141.4789916  207.54621849]]

    # example of cluster[0]
    # Cluster Centers [184.5 112.5]

    def get_left(self):
        # cluster = self.coords
        cluster = self.coords[np.argmin(self.coords[:,0])]
        # cluster = self.coords # initial value
        # for _cluster in cluster:
        #     if cluster[0][0] < _cluster[0]: # if one is further to the left
        #         cluster = self.coords
        return cluster

    def get_right(self):
        cluster = self.coords[np.argmax(self.coords[:,0])]

        # cluster = self.coords # initial value
        # for _cluster in self.coords[0]:
        #     if cluster[0] > _cluster[0]: # if one is further to the right
        #         cluster = _cluster
        # return cluster
        # cluster = self.coords
        # cluster = np.amin(cluster,0)
        # cluster = self.coords # initial value
        # for _cluster in cluster:
        #     if cluster[0][0] < _cluster[0]: # if one is further to the left
        #         cluster = self.coords
        return cluster

    def get_top(self):
        cluster = self.coords[np.argmin(self.coords[:,1])]
        # cluster = self.coords # initial value
        # for _cluster in self.coords[0]:
        #     if cluster[1] < _cluster[1]: # if one is further to the right
        #         cluster = _cluster
        return cluster

    def get_bot(self):
        cluster = self.coords[np.argmax(self.coords[:,1])]
        # cluster = self.coords # initial value
        # for _cluster in self.coords[0]:
        #     if cluster[1] > _cluster[1]: # if one is further to the right
        #         cluster = _cluster
        return cluster

def get_clusters(coords):
    rows, cols = coords
    X = np.array(list(zip(cols,rows)))
    bandwidth = estimate_bandwidth(X, quantile=0.2)
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True).fit(X)
    cluster_centers = ms.cluster_centers_
    print("Cluster Centers %s" %cluster_centers[0])
    # labels = ms.labels_
    # labels_unique = np.unique(labels)
    # n_clusters_=len(labels_unique)
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

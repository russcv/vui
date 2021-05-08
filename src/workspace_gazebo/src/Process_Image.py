#!/usr/bin/env python
import cv2
import numpy as np

class get_image():
    # def __init__(self):#, image, color, object_colors):
    #     self.a = 1

        # self.image = image
        # self.color = color
        # self.object_colors = object_colors
        # self.filler = 1
        # pass
        # self.image = image
        # self.color = color

    def get_mask(self,color):
        # rospy.loginfo("get_workspace_mask color: %s",color)
        # rospy.loginfo("get_workspace_mask color Type: %s",type(color))
        # parsed_color = service_response.color
        boundaries_dict = { #BGR values
        "red":([0, 0, 150], [100, 50, 255]),
        "green":([0, 150, 0], [100, 255, 100]),
        "blue":([150, 0, 0], [255, 100, 100]),
        "yellow":([20, 100, 100], [30, 255, 255])
        # "wood":([0, 100, 180], [120, 200, 255]),
        }
        #boundaries_dict = {                "red":([0, 0, 150], [100, 50, 255]),                "green":([0, 150, 0], [100, 255, 100]),                "blue":([150, 0, 0], [255, 100, 100]),                "yellow":([20, 100, 100], [30, 255, 255]),                }
        lower, upper = boundaries_dict[color]
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
        return (lower, upper)

    def apply_mask(self, image, color):
        lower,upper = self.get_mask(color)
        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)
        # output = cv2.bitwise_not(image, mask = mask)
        return output

    def remove_background(self,image,object_colors):
        output = []
        for obj_color in object_colors:
            lower,upper = self.get_mask(obj_color)
            mask = cv2.inRange(image, lower, upper)
            output.append(cv2.bitwise_and(image, image, mask = mask))
        # output = cv2.bitwise_not(image, mask = mask)
        print(output)
        output = np.array(output)
        out1 = cv2.bitwise_or(output[0],output[1])
        out2 = cv2.bitwise_or(out1,output[2])
        return out2
#
# def open_img():
#     img = cv2.imread('/home/russ/Documents/images/savedImage.jpg')
#     image_pixels = 320,320 # width, height
#     resized_image = cv2.resize(img, (image_pixels))
#     return resized_image
#
# #image to print
# print_img = open_img()
# colors = ["red","green","blue"]
# i = get_image(print_img, "red", colors)
#
# output = i.remove_background(print_img, colors)
# print(output.shape)
# print(output[0].shape)
# cv2.imshow("images", output)
# cv2.waitKey(0)

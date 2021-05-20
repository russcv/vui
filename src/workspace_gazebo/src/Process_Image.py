#!/usr/bin/env python3
'''
@author: Russell Valente
rcvalente2@wpi.edu
WPI - Robotics Engineering Dept
'''

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
        # print(output)
        output = np.array(output)
        out1 = cv2.bitwise_or(output[0],output[1])
        out2 = cv2.bitwise_or(out1,output[2])
        return out2

    # get shapes - values specific tailored for this application (image size)
    def get_contours(self,img):
        imgGry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret , thrash = cv2.threshold(imgGry, 20 , 240, cv2.THRESH_BINARY)
        contours , hierarchy = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        return contours

    def get_contour_centers(self,contours):
        centers = []
        for contour in contours:
            # obtain shape centers
            M = cv2.moments(contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append((cx,cy))
        return centers

    def get_shapes(self,img):
        contours = self.get_contours(img)
        # img = cv2.imread('/home/russ/Documents/images/savedImage.jpg')
        # imgGry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #
        # ret , thrash = cv2.threshold(imgGry, 20 , 240, cv2.THRESH_BINARY)
        # contours , hierarchy = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # print(len(contours))
        # print("contours 0",contours[0])
        # print("moment\n")
        # print(cv2.moments(contours[0]))
        label_color = (100, 255, 255)
        label_size = 0.5
        shapes = {"triangle": [], "square":[],"rectangle":[], "pentagon":[], "star":[], "circle":[]}
        # contours
        # shape, (x,y) {tuple of 2 values for x and y} ,
        for contour in contours:
            # obtain shape centers
            M = cv2.moments(contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            # cv2.circle draws circles to display centers
            # parameters (img,center coords, radius, color, thickness, optional-return img)
            cv2.circle(img, (cx, cy), 2, (255, 255, 255), -1)

            # get shapes
            approx = cv2.approxPolyDP(contour, 0.03* cv2.arcLength(contour, True), True)
            cv2.drawContours(img, [approx], 0, (255, 255, 255), 2)
            x = approx.ravel()[0]
            y = approx.ravel()[1] - 5
            if len(approx) == 3:
                # cv2.putText( img, "triangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, label_size, label_color )
                shapes["triangle"].append((cx,cy))
            elif len(approx) == 4 :
                x, y , w, h = cv2.boundingRect(approx)
                aspectRatio = float(w)/h
                # print(aspectRatio)
                if aspectRatio >= 0.75 and aspectRatio < 1.25:
                    # cv2.putText(img, "square", (x, y), cv2.FONT_HERSHEY_COMPLEX, label_size, label_color)
                    shapes["square"].append((cx,cy))
                else:
                    # cv2.putText(img, "rectangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, label_size, label_color)
                    shapes["rectangle"].append((cx,cy))

            elif len(approx) == 5 :
                # cv2.putText(img, "pentagon", (x, y), cv2.FONT_HERSHEY_COMPLEX, label_size, label_color)
                shapes["pentagon"].append((cx,cy))
            elif len(approx) == 10 :
                # cv2.putText(img, "star", (x, y), cv2.FONT_HERSHEY_COMPLEX, label_size, label_color)
                shapes["star"].append((cx,cy))
            else:
                # cv2.putText(img, "circle", (x, y), cv2.FONT_HERSHEY_COMPLEX, label_size, label_color)
                shapes["circle"].append((cx,cy))
        return shapes

    def apply_shape_mask(self, img, shape, shape_dict):
        label_color = (100, 255, 255)
        label_size = 0.5
        print(shape_dict)
        if shape == "box":
            box = ["square","rectangle"]
            for key in box:
                for coords in shape_dict[key]:
                    print(coords)
                    cv2.putText(img, key, (coords[0]+25,coords[1]+10), cv2.FONT_HERSHEY_COMPLEX,
                            label_size, label_color)
        else:
            for coords in shape_dict[shape]:
                cv2.putText(img, shape, (coords[0]+25,coords[1]+10), cv2.FONT_HERSHEY_COMPLEX,
                        label_size, label_color)
        # return img_labeled

        # lower,upper = self.get_mask(color)
        # mask = cv2.inRange(image, lower, upper)
        # output = cv2.bitwise_and(image, image, mask = mask)
        # output = cv2.bitwise_not(image, mask = mask)
        # return output
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

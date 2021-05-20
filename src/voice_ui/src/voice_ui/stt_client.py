#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  3 21:08:53 2021

@author: Russell Valente
rcvalente2@wpi.edu
WPI - Robotics Engineering Dept
"""

import sys
import rospy
from voice_ui.srv import speech_to_text, speech_to_textResponse

# Service callback function
def stt_client(x):
    rospy.loginfo("Waiting for STT service...")
    rospy.wait_for_service('speech_to_text')
    rospy.loginfo('Turning on mic')
    try:
        # create a service proxy
        stt = rospy.ServiceProxy('speech_to_text',speech_to_text)
        # Call the service here
        service_response = stt(x)
        parsed_phrase = [service_response.text, service_response.color,
            service_response.ori, service_response.shape]
        rospy.loginfo("stt output: %s"%parsed_phrase)
        if service_response.color:
            rospy.loginfo("Robot is grabbing the %s object."%parsed_phrase[1])
        if service_response.ori:
            rospy.loginfo("Robot is grabbing the %s object."%parsed_phrase[2])
        if service_response.shape:
            rospy.loginfo("Robot is grabbing the %s object."%parsed_phrase[3])
        rospy.loginfo('mic off')
        #return the response to the calling function
        print("STT CLient Parsed Phrase output: \n",parsed_phrase)
        return parsed_phrase

    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)

if __name__ == "__main__":

    # Initialize the client ROS node.
    rospy.init_node("speech_to_text_client", anonymous = False)
    stt_flag = True #bool(sys.argv[1])

    if stt_flag:
        # rospy.loginfo('Turning on mic')
        phrase = stt_client(stt_flag)
    else:
        rospy.loginfo('Mic remains off')
        sys.exit(1)
    stt_flag = False

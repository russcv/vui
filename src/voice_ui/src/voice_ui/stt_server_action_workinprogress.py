#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  3 21:08:12 2021

@author: russ
"""
from voice_ui.srv import speech_to_text, speech_to_textResponse
import rospy
import time
import speech_recognition as sr
from parse import parse_stt

# Speech Recognition function
class stt_server(object):
    # create messages that are used to publish feedback/result
    # calls .action file in /action folder
    def __init__(self,name):
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name, msg, callback, auto_start = False)
        self._as.start()

    def execute_cb(self,goal):
        # delay
        r = rospy.Rate(1)
        #Variable to decide the final state of the action server.
        success = True


def recognize_speech_from_mic(recognizer, microphone):
    """Transcribe speech from recorded from `microphone`.

    Returns a dictionary with three keys:
    "success": a boolean indicating whether or not the API request was
               successful
    "error":   `None` if no error occured, otherwise a string containing
               an error message if the API could not be reached or
               speech was unrecognizable
    "transcription": `None` if speech could not be transcribed,
               otherwise a string containing the transcribed text
    """
    # check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # adjust the recognizer sensitivity to ambient noise and record audio
    # from the microphone
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    # set up the response object
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    # try recognizing the speech in the recording
    # if a RequestError or UnknownValueError exception is caught,
    #     update the response object accordingly
    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        # API was unreachable or unresponsive
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        # speech was unintelligible
        response["error"] = "Unable to recognize speech"

    return response

# function to return transcript of audio
def mic():
    PROMPT_LIMIT = 5

    # create recognizer and mic instances
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    # wait .1 seconds
    time.sleep(.1)

    for j in range(PROMPT_LIMIT):
        print('\nPlease say a command')
        command = recognize_speech_from_mic(recognizer, microphone)
        if command["transcription"]:
            break
        if not command["success"]:
            break
        print("I didn't catch that. What did you say?\n")

    # if there was an error, stop
    if command["error"]:
        print("ERROR: {}".format(command["error"]))

    # show the user the transcription
    print("You said: {}".format(command["transcription"]))

    return command["transcription"]

# Service callback function
def stt_request(req):
    res = speech_to_textResponse()

    # Check flag
    if(req.stt_flag == True):
        rospy.loginfo('Turning on mic...')
        phrase = str(mic()).lower()
        # rospy.loginfo("phrase: %s" %phrase)
        phrase_parsed = parse_stt(phrase)
        color = parse_stt.get_color(phrase_parsed)
        # color = "blue"
        # rospy.loginfo("phrase: %s" %phrase)
        res.text = phrase
        res.color = color
        # rospy.loginfo("phrase: %s" %phrase)
        res.success = True

        # test_text = "grab Green box"
        # text1 = parse_stt(test_text)
        # print(text1.get_color())

    else:
        rospy.loginfo('Mic is off')
        res.text = "N/A"
        res.color = ""
        res.success = false
    # Return text from speech
    return res

def stt_server():
    # ROS node for the service server.
    rospy.init_node('speech_to_text_server', anonymous = False)

    # Create a ROS service type.
    # rospy.Service ("name", type, callback_function)
    service = rospy.Service('speech_to_text', speech_to_text, stt_request)
    stt_server(rospy.get_name())
    # Log message about service availability.
    rospy.loginfo('STT Service now available')
    rospy.spin()

if __name__ == "__main__":
    stt_server()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 7 14:52:12 2021

@author: russ
"""

class parse_stt():
    def __init__(self,text_from_speech):
        self.stt = text_from_speech.lower().split(" ")

    # def parse(self):
    #     print(self.stt)

    def get_color(self):
        phrase = self.stt
        color_list = [
        "white","red","orange","yellow","green","blue","purple","black",
        "gray","grey","magenta","cyan","teal","violet"
        ]

        # return color in phrase
        find_color = [color for color in phrase if(color in color_list)]
        return str(find_color[0])

    def get_location(self):
        phrase = self.stt
        bound = [0,300]
        near = ["near","nearer","close","closer","immediate","nearby"]
        far = ["far", "farther", "away","further"]
        if "left" in phrase:
            return 0
        elif "right" in phrase:
            return 300
        if any(element in near for element in phrase):
            return 300
        elif any(element in far for element in phrase):
            return 0



# used for debugging
# test_text = "grab the left Green box"
# text1 = parse_stt(test_text)
# # print(text1.get_color())
# print(parse_stt(test_text).get_color())
# print(parse_stt(test_text).get_location())

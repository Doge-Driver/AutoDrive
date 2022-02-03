#! /usr/bin/python3

import os
import pickle

import cv2

filename = input("File name to read:")
path = f"{os.path.dirname(os.path.abspath(__file__))}/{filename}"


with open(f"{path}", "rb") as f:
    frames = pickle.load(f)

    for frame, fps in frames:
        cv2.imshow("frame", frame)
        cv2.waitKey(fps)
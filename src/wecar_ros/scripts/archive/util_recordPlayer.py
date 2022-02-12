#! /usr/bin/python3

import os
import pickle

import cv2
import numpy as np

filename = input("File name to read: ")
path = f"{os.path.dirname(os.path.abspath(__file__))}/pickles/{filename}"


with open(f"{path}", "rb") as f:
    frames = pickle.load(f)

    for frame, fps in frames:
        np_arr = np.frombuffer(frame, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow("frame", image)
        cv2.waitKey(fps)

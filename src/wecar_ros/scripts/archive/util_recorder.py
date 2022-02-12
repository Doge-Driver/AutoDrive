#! /usr/bin/python3

import os
import pickle
from time import time_ns

import rospy

from subscribers import Camera

if __name__ == "__main__":
    rospy.init_node("camera_recorder", anonymous=True)

    filename = input("파일 이름: ")
    path = f"{os.path.dirname(os.path.abspath(__file__))}/pickles/{filename}"
    print(f"{path} 에 저장합니다")
    print("녹화를 중단하려면 Ctrl + C 입력")

    frames = []

    while not rospy.is_shutdown():
        startTime = time_ns()
        frame = Camera().retrieve().get().data
        endTime = time_ns()
        frames.append((frame, int((endTime - startTime) / 1000000)))

    with open(path, "wb") as f:
        pickle.dump(frames, f)
        print(f"{path} 에 저장합니다")

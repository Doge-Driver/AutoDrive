import cv2
import numpy as np
from matplotlib import pyplot as plt

mapImg = cv2.imread("colorLabeledMap.png", cv2.IMREAD_UNCHANGED)
height, width = mapImg.shape[:2]
newImg = np.zeros((height, width), np.uint8)

for i, arr1 in enumerate(mapImg):
    for j, arr2 in enumerate(arr1):
        b, g, r = arr2
        if b == 0 and g == 0 and r == 0:
            continue
        if b == 36 and g == 28 and r == 237:  # 중앙선 빨간색
            newImg[i][j] = 3
            continue
        if b == 204 and g == 72 and r == 63:  # 실선 파란색
            newImg[i][j] = 1
            continue
        if b == 255 and g == 255 and r == 255:  # 점선 하얀색
            newImg[i][j] = 2
            continue
        if b == 76 and g == 177 and r == 34:  # 횡단보도 초록색
            newImg[i][j] = 4
            continue

plt.imshow(newImg)
plt.show()
cv2.imwrite("labeledMap.png", newImg)

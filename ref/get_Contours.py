import cv2
import numpy as np

def getContours(img, imgContour):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # areaMin = cv2.getTrackbarPos("Area", "Parameters")
        areaMin = 0
        if area > areaMin:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)

def get_contours(frame):
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img_blur = cv2.GaussianBlur(img_gray, (7, 7), 1)
    threshold1, threshold2 = 0, 255
    img_canny = cv2.Canny(img_blur, threshold1, threshold2)
    contours, hierarchy = cv2.findContours(img_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contours



img = cv2.imread("../image/121.jpg")
img = cv2.resize(img, (1080, 1080))
contours = get_contours(img)
h, w, _ = img.shape
mask = img.copy()
# for cnt in contours:
#     areaMin = 100
#     area = cv2.contourArea(cnt)
#
#     if (area < (h / 10 * w / 10)):
#         c_min = []
#         c_min.append(cnt)
#         cv2.drawContours(img, c_min, -1, (0, 255, 0), thickness=-1)
    # if area > areaMin:
        # cv2.drawContours(mask, cnt, -1, (0, 255, 0), cv2.FILLED)
        # cv2.fillPoly(mask, [cnt], (0, 255, 0))
    # mask = cv2.drawContours(mask, cnt, -1, (0, 0, 255), 1)
cv2.drawContours(mask, contours, -1, (0, 255, 0), cv2.FILLED)


# imgContour = img.copy()
# imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# imgBlur = cv2.GaussianBlur(imgGray, (7, 7), 1)
# threshold1, threshold2 = 0, 255
# imgCanny = cv2.Canny(imgBlur, threshold1, threshold2)
# kernel = np.ones((5, 5))
# imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
# getContours(imgCanny, imgContour)

cv2.imshow("ff", mask)
cv2.waitKey(0)

# h, w, _ = img.shape
#
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#
# ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
#
# # Find Contour
# contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
#
# # 需要搞一个list给cv2.drawContours()才行！！！！！
# c_max = []
# for i in range(len(contours)):
#     cnt = contours[i]
#     area = cv2.contourArea(cnt)
#
#     # 处理掉小的轮廓区域，这个区域的大小自己定义。
#     if (area < (h / 10 * w / 10)):
#         c_min = []
#         c_min.append(cnt)
#         # thickness不为-1时，表示画轮廓线，thickness的值表示线的宽度。
#         cv2.drawContours(img, c_min, -1, (0, 0, 0), thickness=-1)
#         continue
#     #
#     c_max.append(cnt)
#
# cv2.drawContours(img, c_max, -1, (0, 0, 255), thickness=-1)
#
# cv2.imwrite("mask.png", img)
# cv2.imshow('mask', img)
# cv2.waitKey(0)
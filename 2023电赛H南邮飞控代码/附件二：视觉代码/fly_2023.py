import cv2
import numpy as np
import rospy
from std_msgs.msg import ColorRGBA

def nothing(x):
    pass
# img = cv2.imread()


cap = cv2.VideoCapture(0)

rospy.init_node('vision', anonymous=True)
vision_data=ColorRGBA()
vision_pub = rospy.Publisher('/vision/fire',ColorRGBA, queue_size=10)
cv2.namedWindow('frame')
cv2.createTrackbar('h_l', 'frame', 1, 255, nothing)
cv2.createTrackbar('s_l', 'frame', 1, 255, nothing)
cv2.createTrackbar('v_l', 'frame', 1, 255, nothing)
cv2.createTrackbar('h_h', 'frame', 1, 255, nothing)
cv2.createTrackbar('s_h', 'frame', 1, 255, nothing)
cv2.createTrackbar('v_h', 'frame', 1, 255, nothing)
cv2.setTrackbarPos('h_l', 'frame', 0)# 64
cv2.setTrackbarPos('s_l', 'frame', 0)
cv2.setTrackbarPos('v_l', 'frame', 0)# 38
cv2.setTrackbarPos('h_h', 'frame', 255)
cv2.setTrackbarPos('s_h', 'frame', 255)# 171
cv2.setTrackbarPos('v_h', 'frame', 57)# 255
a = 0
r = 0
g = 0
b = 0
led = [0, 0]
while cap.isOpened():
    a = 0
    r = 0
    g = 0
    b = 0
    h_l = cv2.getTrackbarPos('h_l', 'frame')
    s_l = cv2.getTrackbarPos('s_l', 'frame')
    v_l = cv2.getTrackbarPos('v_l', 'frame')
    h_h = cv2.getTrackbarPos('h_h', 'frame')
    s_h = cv2.getTrackbarPos('s_h', 'frame')
    v_h = cv2.getTrackbarPos('v_h', 'frame')
    lsv = np.array([h_l, s_l, v_l])
    hsv = np.array([h_h, s_h, v_h])
    _, frame = cap.read()
    img = frame.copy()
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    # cv2.rectangle(img)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_hsv, lsv, hsv)
    Canny = cv2.Canny(mask, 20, 50)
    contours, _ = cv2.findContours(Canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    red_led = []
    if len(contours) > 0:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area <= 500 and area >= 40:
                try:
                    # box = cv2.boxPoints(rect)
                    # box = np.int0(box)
                    M = cv2.moments(contour)
                    center_x = int(M['m10'] / M['m00'])
                    center_y = int(M['m01'] / M['m00'])
                    cv2.putText(img, f"{center_x}, {center_y}", (center_x, center_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 1)
                    cv2.drawContours(img, [contour], 0, (0, 255, 0), 2)
                    print(area)
                    red_led.append([center_x, center_y])
                except:
                    pass
    if len(red_led) >= 2:
        for fire in red_led:
            distance = ((fire[0] - led[0]) ** 2 + (fire[1] - led[1]) ** 2) ** (1/2)
            if distance >= 20:
                x_error = fire[0] - 320
                y_error = fire[1] - 240
                r = x_error
                g = y_error
                a = 1
    vision_data.r = r
    vision_data.g = g
    vision_data.b = b
    vision_data.a = a
    print("{} {} {} {}".format(r, g, b, a))
    vision_pub.publish(vision_data)
    cv2.imshow("img", img)
    cv2.imshow("mask", mask)
    if cv2.waitKey(1) == ord(' '):
        break

cap.release()
cv2.destroyAllWindows()
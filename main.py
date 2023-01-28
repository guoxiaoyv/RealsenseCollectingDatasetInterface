import cv2
import numpy as np

import realsenseCamera as rsc

camera = rsc.RealSenseCamera(215, 0)
camera.start()

point = (0, 0)


def show_distance(event, x, y, args, params):
    global point
    point = (x, y)


cv2.namedWindow("RealSenseColor")
cv2.setMouseCallback("RealSenseColor", show_distance)

while True:

    key = cv2.waitKey(1) & 0xFF

    depth_image, color_image, infrared_image = camera.get_frames()
    color_image = cv2.resize(color_image, (1280, 720))

    cv2.circle(depth_image, point, 4, (0, 0, 255))
    distance = camera.get_distance(point[1], point[0])
    print(f"Distance: {distance}")
    cv2.putText(color_image, "{}mm".format(distance),
                (point[0], point[1] - 20),
                cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03),
                                       cv2.COLORMAP_JET)

    cv2.imshow('RealSenseColor', color_image)
    # cv2.imshow('RealSenseColorDepth', depth_image)
    # cv2.imshow('RealSenseColorColorMap', depth_colormap)
    # cv2.imshow('RealSenseColorInfrared', infrared_image)

    if key == ord('q'):
        cv2.destroyAllWindows()
        camera.stop()
        break

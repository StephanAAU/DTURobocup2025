import numpy as np
import cv2 as cv

from orighelpers.scam import cam
from orighelpers.sgpio import gpio
from orighelpers.uservice import service

#from Libraries.ball_detection import ball_detector
from Libraries.golf_detection import golf_detector

if __name__ == "__main__":
    #image = cv.imread("balls_lined_up.jpg")
    #image = cv.imread("ball_img.jpg")
    #image = cv.imread("ball_zoom.png")
    #image = cv.imread("reflections.jpg")
    #image = cv.imread("ball_imgs/7.jpg")
    #image = cv.imread("ball_imgs/1_crop.jpg")
    #image = cv.imread("img_ball_grab/5.jpg")
    #image = cv.imread("img_ball_grab/5_crop.jpg")
    #image = cv.imread("golf/cropped_1.jpg")
    #image = cv.imread("golf/1.jpg")
    #image = cv.imread("golf/2.jpg")
    #image = cv.imread("golf/3.jpg")
    image = cv.imread("golf/4.jpg")

    hsv_extraction = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    #print(f"HSV Image: {hsv_extraction.shape}")
    #print(f"topleft: {hsv_extraction[475, 379]}")
    #print(f"topright: {hsv_extraction[495, 379]}")
    #print(f"bottomleft: {hsv_extraction[475, 391]}")
    #print(f"bottomright: {hsv_extraction[495, 391]}")
    print(image)

    print(f"avg h {np.mean(hsv_extraction[:, :, 0])}")
    print(f"avg s {np.mean(hsv_extraction[:, :, 1])}")
    print(f"avg v {np.mean(hsv_extraction[:, :, 2])}")

    print(f"max h {np.max(hsv_extraction[:, :, 0])}")
    print(f"max s {np.max(hsv_extraction[:, :, 1])}")
    print(f"max v {np.max(hsv_extraction[:, :, 2])}")

    print(f"min h {np.min(hsv_extraction[:, :, 0])}")
    print(f"min s {np.min(hsv_extraction[:, :, 1])}")
    print(f"min v {np.min(hsv_extraction[:, :, 2])}")

    image = golf_detector.process_frame(image)

    Alive = True
    while Alive:
        print("Ball Detection")
        for id, coords in zip(golf_detector.ball_color, golf_detector.ball_coordiantes):
            print(f"Ball: {id} at = ({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f})")
            print(f"Alt coords: {golf_detector.ball_alt_coordinates}")
        
        cv.imshow("Ball Detection", image)
        keypress = cv.waitKey(0)
        if keypress == ord('q') or keypress & 0xFF == 27:
            Alive = False
            
    cv.destroyAllWindows()

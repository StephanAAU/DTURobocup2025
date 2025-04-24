import numpy as np
import cv2 as cv

from Libraries.saruco import aruco

from orighelpers.scam import cam
from orighelpers.sgpio import gpio
from orighelpers.uservice import service



class BallDetector:
    def __init__(self, camera_matrix, dist_coeffs, ball_diameter_mm=43):
        # HSV color ranges for detecting blue and red balls
       
        # self.BLUE_LOWER = np.array([90, 50, 100])
        # self.BLUE_UPPER = np.array([130, 255, 255])
        # self.BLUE_LOWER = np.array([94, 125, 123])
        # self.BLUE_UPPER = np.array([108, 215, 255])

        # Defualt
        self.BLUE_LOWER = np.array([90, 125, 100])
        self.BLUE_UPPER = np.array([110, 215, 255])

        self.ORANGE_LOWER = np.array([10, 100, 100])  # H, S, V
        self.ORANGE_UPPER = np.array([25, 255, 255])

        self.RED_LOWER1 = np.array([0, 120, 100])
        self.RED_UPPER1 = np.array([10, 255, 255])
        self.RED_LOWER2 = np.array([0, 80, 60])
        self.RED_UPPER2 = np.array([10, 150, 150])
        self.RED_LOWER3 = np.array([170, 150, 150])
        self.RED_UPPER3 = np.array([180, 255, 255])

        # Camera calibration parameters
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.ball_diameter = ball_diameter_mm  # Ball diameter in millimeters

        # Variables to hold detected ball information
        self.ball_color         = []
        self.ball_coordiantes   = []

        self.ball_width     = 0.05
        self.ball_height    = 0.043

        self.ball_size = np.array([
            [ self.ball_height/2,                   0,  0],  # Ball diameter in mm
            [-self.ball_height/2,                   0,  0],
            [0,                     self.ball_width/2,  0],
            [0,                    -self.ball_width/2,  0]
        ])
        self.ball_alt_coordinates   = []

        self.max_area = 20000
        self.min_area = 200

    def config_normal_color(self):
        self.BLUE_LOWER = np.array([90, 125, 100])
        self.BLUE_UPPER = np.array([110, 215, 255])

    def config_grabber_color(self):
        self.BLUE_LOWER = np.array([80, 75, 220])
        self.BLUE_UPPER = np.array([125, 135, 255])

    def create_red_mask(self, hsv):
        """Create a mask for detecting red balls."""
        mask1 = cv.inRange(hsv, self.RED_LOWER1, self.RED_UPPER1)
        mask2 = cv.inRange(hsv, self.RED_LOWER2, self.RED_UPPER2)
        mask3 = cv.inRange(hsv, self.RED_LOWER3, self.RED_UPPER3)
        return cv.bitwise_or(mask1, cv.bitwise_or(mask2, mask3))

    def create_blue_mask(self, hsv):
        """Create a mask for detecting blue balls."""
        return cv.inRange(hsv, self.BLUE_LOWER, self.BLUE_UPPER)

    def find_bounding_boxes(self, image_bgr, mask, color_label):
        """Find and return bounding boxes for the detected balls."""
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        detected = []

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue

            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(image_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.putText(image_bgr, color_label, (x - 10, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            detected.append((x, y, w, h, color_label))

        return detected

    def cluster_balls(self, image_bgr, mask, color_label):
        """Cluster and detect balls from the mask."""
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        clusters = []

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue

            (cx, cy), radius = cv.minEnclosingCircle(cnt)

            if (cy < 300):
                continue
            
            cv.circle(image_bgr, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
            cv.putText(image_bgr, color_label, (int(cx - 10), int(cy - 10)), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            clusters.append((cx, cy, radius, color_label))

        return clusters

    def estimate_depth(self, focal_length, ball_diameter_pixels):
        """Estimate the depth of the ball based on its size in the image."""
        depth = (focal_length * self.ball_diameter) / ball_diameter_pixels
        return depth

    def process_frame(self, frame):
        """Process a single camera frame to detect balls and calculate robot coordinates."""
        # Undistort the frame
        h, w = frame.shape[:2]
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        frame = cv.undistort(frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)

        blur_frame = cv.GaussianBlur(frame, (11, 11), 0)

        hsv = cv.cvtColor(blur_frame, cv.COLOR_BGR2HSV)
        mask_red = self.create_red_mask(hsv)
        mask_blue = self.create_blue_mask(hsv)

        # Cluster the red and blue balls
        #red_clusters = self.cluster_balls(frame, mask_red, "red")
        red_clusters = []
        blue_clusters = self.cluster_balls(frame, mask_blue, "blue")

        # Reset ball info vars
        self.ball_color             = []
        self.ball_coordiantes       = []

        self.ball_alt_coordinates   = []

        # Process clusters and estimate their 3D coordinates
        for cluster in red_clusters + blue_clusters:
            cx, cy, radius, color_label = cluster
            ball_diameter_pixels = 2 * radius  # diameter is twice the radius
            focal_length = self.camera_matrix[0, 0]  # Assuming focal length is the fx value
            depth = self.estimate_depth(focal_length, ball_diameter_pixels)

            # Ensure depth (z_real) is always positive
            depth = max(depth, 0)

            # Convert coordinates from pixel to robot space
            x_real = (cx - w / 2) * depth / focal_length
            y_real = (cy - h / 2) * depth / focal_length
            z_real = depth

            # Convert to meters
            x_real_meters = x_real / 1000
            y_real_meters = y_real / 1000
            z_real_meters = z_real / 1000

            pxl_ball_edge_coords = np.array([
                    cx + radius, cy,
                    cx - radius, cy,
                    cx, cy + radius,
                    cx, cy - radius
                ]).reshape((4,2))
            
            #print(f"pxl_ball_edge_coords: {pxl_ball_edge_coords}")

            success, rvec, tvec = cv.solvePnP(self.ball_size, pxl_ball_edge_coords, aruco.intrinsic_camera, aruco.distortion)
            if success:
                #print(f"tvec: {tvec}")
                x__ = tvec[0][0]
                y__ = tvec[1][0]
                z__ = tvec[2][0]
                #print(f"x__: {x__}, y__: {y__}, z__: {z__}")
                alt_coords = aruco.rotation_translation_matrix @ aruco.frame_change_matrix @ np.array([x__, y__, z__, 1])
                #print(f"rx: {alt_coords[0]}, ry: {alt_coords[1]}, rz: {alt_coords[2]}")
                #print(f"x_meters: {x_real_meters}, y_meters: {y_real_meters}, z_meters: {z_real_meters}")
            else:
                alt_coords = np.array([0, 0, 0, 1])
            self.ball_alt_coordinates.append((alt_coords[0], alt_coords[1], alt_coords[2]))

            self.ball_color.append(color_label)
            self.ball_coordiantes.append((x_real_meters, y_real_meters, z_real_meters))

            #print(f"Ball: {color_label} at Robot Coordinates (x, y, z) = ({x_real_meters:.3f}, {y_real_meters:.3f}, {z_real_meters:.3f})")

        return frame

    def start_camera_stream(self):
        """Start the camera stream and process frames for ball detection."""
        cap = cv.VideoCapture(0)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            processed_frame = self.process_frame(frame)
            cv.imshow('Ball Detection', processed_frame)

            if cv.waitKey(1) & 0xFF == 27:  # Press 'ESC' to exit
                break

        cap.release()
        cv.destroyAllWindows()

    def capture_frame_analyze(self):
        if cam.useCam:
            ok, img, imgTime = cam.getImage()
            if not ok:  # size(img) == 0):
                if cam.imageFailCnt < 5:
                    print("% Failed to get image.")
            else:
                h, w, ch = img.shape
            if not service.args.silent:
                # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
                pass
            #edge.paint(img)
            if ok:
                img = self.process_frame(img)
            else:
                img = np.zeros((640, 800, 3), np.uint8)
            if not gpio.onPi:
                cv.imshow('frame for analysis', img)
            pass
            pass
        pass

    def clear(self):
        self.ball_color = []
        self.ball_coordiantes = []
        self.ball_alt_coordinates = []


camera_matrix = np.array([[624.3480342,    0,         402.53289042],
                          [  0,         622.25436976, 316.87176544],
                          [  0,            0,            1        ]])

distortion_matrix = np.array([1.80760080e-01, -4.83549201e-01, -5.26630811e-03,  2.78445106e-04, 3.96739795e-01])


ball_detector = BallDetector(camera_matrix, distortion_matrix)



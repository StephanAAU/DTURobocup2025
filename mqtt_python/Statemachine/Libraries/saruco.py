# AruCo marker and pose detection 
# Based on: https://github.com/Ritabrata-Chakraborty/ArUco/blob/main/Aruco_Detection.py
# Req: opencv 4.6.*

import cv2 as cv
import pickle
import numpy as np
import sys
import time
from orighelpers.scam import cam
from orighelpers.uservice import service
from orighelpers.sgpio import gpio

def imageAnalysis(save):
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
      aruco.robot_coords_id_list(img)
      aruco.debug_draw_cam(img)
      if not gpio.onPi:
        cv.imshow('frame for analysis', img)
      if save:
        fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
        cv.imwrite(fn, img)
        if not service.args.silent:
          print(f"% Saved image {fn}")
      pass
    pass
  pass

def search_id(id=5):
    
    time.sleep(0.05)
    imageAnalysis(False)
    found = False
    target_robot_x = [0]
    target_robot_y = [0]
    if (aruco.g_aruco is True):
        print(aruco.g_ids, aruco.g_coords, aruco.g_rvecs)
        for ids, coordiantes, rot in zip(aruco.g_ids, aruco.g_coords,aruco.g_rvecs):
            theta = float(rot[1])
            print(np.sin(theta))

            if ids == id:
                if rot[0] < 0.5:
                
                    target_robot_x = coordiantes[0] + np.cos(theta) * 0.03
                    target_robot_y = coordiantes[1] - np.sin(theta) * 0.03
                else:
                    target_robot_x = coordiantes[0]
                    target_robot_y = coordiantes[1]
                found = True

            print(target_robot_x, target_robot_y)

     
    return found, target_robot_x, target_robot_y



def search_id_list(ids=[5, 6, 7]):
    imageAnalysis(False)

    list_of_good_ids = np.array(ids)

    good_ids_found = []
    good_coords_found = []
    good_rotation_found = []

    # First check if any aruco code is detected
    if (aruco.g_aruco is True):
      # Then check if any of the found id's are in the provided list
      if any(item in aruco.g_ids for item in list_of_good_ids):
        # Now loop through the matching id's and sort the coordinates
        for ii, cc, rr in zip(aruco.g_ids, aruco.g_coords, aruco.g_rvecs):
          #print(f"ii: {ii} cc: {cc}")
          if ii in list_of_good_ids:
            good_ids_found.append(ii)
            good_coords_found.append(cc)
            good_rotation_found.append(rr)

        print(f"Found IDs: {good_ids_found}")
        return True, good_ids_found, good_coords_found, good_rotation_found


    #print(f"found none of the IDs: {ids}")
    return False, [0], [0], [[0,0,0,]]

class SAruCo:

    def __init__(self):
        self.aruco_type = "DICT_4X4_100"
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)
        self.aruco_params = cv.aruco.DetectorParameters_create()
        print("Initialized ArUco detection")

        self.c_zr = 0.192           # camera height [m]
        self.c_xr = 0.02               # camera offset [m]
        self.phi  = np.deg2rad(11)   # camera angle [rad]

        self.aruco_square_length = 0.035 # side length of the ArUco marker

        self.frame_change_matrix = np.array(
            [[ 0,  0, 1, 0],
             [-1,  0, 0, 0],
             [ 0, -1, 0, 0],
             [ 0,  0, 0, 1]]
        )

        self.rotation_translation_matrix = np.array(
            [[np.cos(self.phi),        0,  np.sin(self.phi),       -self.c_xr],
             [0,                       1,  0,                               0],
             [-np.sin(self.phi),       0,  np.cos(self.phi),       self.c_zr],
             [0,                       0,  0,                               1]]
        )

        self.intrinsic_camera = np.array([[624.3480342,    0,         402.53289042],
                                          [  0,         622.25436976, 316.87176544],
                                          [  0,            0,            1        ]])

        self.distortion = np.array([1.80760080e-01, -4.83549201e-01, -5.26630811e-03,  2.78445106e-04, 3.96739795e-01])

        self.aruco_qr_specs_small = np.array([
            [0,       0,        0], # Top-left-front
            [0.035,   0,        0], # Top-right-front
            [0.035, 0.035,      0], # Bottom-right-front
            [0,     0.035,      0]  # Bottom-left-front
        ])

        self.aruco_qr_specs_big = np.array([
            [0,       0,      0],   # Top-left-front
            [0.1,     0,      0],  # Top-right-front
            [0.1,   0.1,      0], # Bottom-right-front
            [0,     0.1,      0]  # Bottom-left-front
        ])

        self.aruco_qr_ids_small = [5, 23, 53, 20]

        self.g_aruco    = False
        self.g_ids      = []
        self.g_coords   = []

    def load_camera_calibration(self, filename="camera_calibration.pkl"):
        """
        Loads the camera calibration parameters
        """
        with open(filename, 'rb') as f:
            self.intrinsic_camera = np.array([[624.3480342,    0,         402.53289042],
                                          [  0,         622.25436976, 316.87176544],
                                          [  0,            0,            1        ]])

            self.distortion = np.array([1.80760080e-01, -4.83549201e-01, -5.26630811e-03,  2.78445106e-04, 3.96739795e-01])

            calibration_data = pickle.load(f)

            self.intrinsic_camera = calibration_data["camera_matrix"]
            print("Camera matrix")
            print(self.intrinsic_camera)
            self.distortion = calibration_data["dist_coeffs"]
            print("Distortion coefficients")
            print(self.distortion)

        


    def detect_markers_draw(self, image):
        """
        Detects AruCo markers, decodes ID and places a bounding box around them
        """
        aruco_type_list = []

        corners, ids, _ = cv.aruco.detectMarkers(image, self.aruco_dict, parameters=self.aruco_params)

        if len(corners) > 0:
            aruco_type_list.append(self.aruco_type)

            for markerCorner, markerId in zip(corners, ids.flatten()):
                corners_aruco = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners_aruco

                cv.polylines(image, [markerCorner.astype(int)], True, (0, 255, 0), 2)

                cX = int((topLeft[0] + bottomRight[0]) / 2)
                cY = int((topLeft[1] + bottomRight[1]) / 2)

                cv.circle(image, (cX, cY), 5, (255, 0, 0), -1)
                cv.putText(image, str(int(markerId)),
                            (int(topLeft[0] - 5), int(topLeft[1])), cv.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 255))

        return aruco_type_list


    def detect_markers_list(self, image):
        """
        Detects AruCo markers, decodes ID and lists their corners
        """

        corners, ids, _ = cv.aruco.detectMarkers(image, self.aruco_dict, parameters=self.aruco_params)

        aruco_ids = []
        aruco_corners = []
        if len(corners) > 0:
            for markerCorner, markerId in zip(corners, ids.flatten()):
                # Not viable solution since when the object is near the camera it will be too high.
                # if np.any(markerCorner[0,:,1] < 150):
                #     print(f"Marker {markerId} is too high ")
                #     continue

                aruco_ids.append(markerId)
                aruco_corners.append(markerCorner.reshape((4, 2)))

        return aruco_ids, aruco_corners


    def center_group_cubes(self, ids, corners):
        aruco_ids = []
        aruco_corners = []
        aruco_centers = []
        for i, c in zip(ids, corners):
            if not i in aruco_ids:
                aruco_ids.append(i)
                aruco_corners.append(c)

                Cx = (c[0][0] + c[1][0] + c[2][0] + c[3][0])/4
                Cy = (c[0][1] + c[1][1] + c[2][1] + c[3][1])/4
                aruco_centers.append([Cx, Cy])

        return aruco_ids, aruco_corners, aruco_centers


    def draw_centers(self, image, ids, centers):
        for i, c in zip(ids, centers):
            cv.circle(image, (int(c[0]), int(c[1])), 5, (255, 0, 0), -1)

        return image


    def convert_coordinates_to_3d(self, ids, corners, centers):
        """
        Converts the coordinates of the detected AruCo markers to 3d coordinates
        """
        aruco_ids = []
        aruco_rvecs = []
        aruco_tvecs = []
        aruco_distances = []

        for i, c, o in zip(ids, corners, centers):
            # Find lengths
            corner_coords = c.reshape((4, 2))
            _aruco_spec = self.aruco_qr_specs_big
            if (i in self.aruco_qr_ids_small):
                _aruco_spec = self.aruco_qr_specs_small

            #print(f"id: {i} using spec: {_aruco_spec}")
            success, rvec, tvec = cv.solvePnP(_aruco_spec, corner_coords, self.intrinsic_camera, self.distortion)

            if success:
                distance = np.linalg.norm(tvec)  # Compute Euclidean distance

                # print(f"id {i} --> distance: {distance} m")
                # print("rvec:")
                # print(rvec)

                # print("tvec:")
                # print(tvec)
                aruco_ids.append(i)
                aruco_rvecs.append(rvec)
                aruco_tvecs.append(tvec)
                aruco_distances.append(distance)


        return aruco_ids, aruco_rvecs, aruco_tvecs, aruco_distances
        #return (self.rotation_translation_matrix @ self.frame_change_matrix)


    def convert_coordinates_to_robot(self, ids, tvecs, distance_threshold=3):
        """
        Convert 3d coordinates to robot coordinates
        """
        aruco_ids = []
        aruco_robot_coords = []

        for i, t in zip(ids, tvecs):
            # Find lengths
            x = t[0][0]
            y = t[1][0]
            z = t[2][0]

            # Rotate and translate

            robot_coords = self.rotation_translation_matrix @ self.frame_change_matrix @ np.array([x, y, z, 1])

            distance = np.linalg.norm(robot_coords)
            if distance > distance_threshold:
                print(f"Distance {distance} for {i} is too high")
                continue
            #print(f"id {i} --> robot coords: {robot_coords}")
            aruco_ids.append(i)
            aruco_robot_coords.append(robot_coords)

        return aruco_ids, aruco_robot_coords


    def pose_estimation_draw(self, frame, matrix_coefficients, distortion_coefficients):
        """
        Estimates the pose of the detected AruCo markers
        """
        corners, ids, rejected = cv.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if len(corners) > 0:
            for i in range(0, len(ids)):
                rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners[i], 0.025, matrix_coefficients, distortion_coefficients)
                cv.aruco.drawDetectedMarkers(frame, corners)
                cv.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)


        return frame


    def pose_estimation_list(self, frame, matrix_coefficients, distortion_coefficients):
        """
        Estimates the pose of the detected AruCo markers
        """
        corners, ids, rejected = cv.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        rvec, tvec = None, None

        if len(corners) > 0:
            for i in range(0, len(ids)):
                rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners[i], 0.025, matrix_coefficients, distortion_coefficients)
                cv.aruco.drawDetectedMarkers(frame, corners)
                cv.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)


    def debug_draw_cam(self, frame):
        """
        Draws the camera frame
        """
        ids, corners                    = self.detect_markers_list(frame)
        ids, corners, origin            = self.center_group_cubes(ids, corners)
        ids, rvecs, tvecs, distance     = self.convert_coordinates_to_3d(ids, corners, origin)
        ids, robot_coords               = self.convert_coordinates_to_robot(ids, tvecs)


        h, w, ch = frame.shape

        if len(ids) > 0:

            for i, c, o, d, rc in zip(ids, corners, origin, distance, robot_coords):
                (topLeft, topRight, bottomRight, bottomLeft) = c

                cv.polylines(frame, [c.astype(int)], True, (0, 255, 0), 2)

                cv.circle(frame, (int(o[0]), int(o[1])), 5, (255, 0, 0), -1)

                cv.putText(frame, f"{[int(rc[0]*100), int(rc[1]*100), int(rc[2]*100)]}",
                            (int(topLeft[0]), int(topLeft[1] + 10)), cv.FONT_HERSHEY_COMPLEX, 0.3, (255, 0, 255))

                cv.putText(frame, f"i: {str(int(i))}",
                            (int(bottomLeft[0] - 5), int(bottomLeft[1])), cv.FONT_HERSHEY_COMPLEX, 0.3, (255, 0, 255))

                cv.putText(frame, f"d: {int(d*100)}",
                            (int(bottomRight[0] - 5), int(bottomRight[1])), cv.FONT_HERSHEY_COMPLEX, 0.3, (255, 0, 255))


    def robot_coords_id_list(self, frame):
        """
        Function to be called on the camera or image frame by the robot main loop to 
        return a list of aruco id's detected and their robot coordinates
        """
        i, c                            = self.detect_markers_list(frame)
        i, c, o                         = self.center_group_cubes(i, c)
        i, rvecs, tvecs, d              = self.convert_coordinates_to_3d(i, c, o)
        aruco_ids, aruco_robot_coords   = self.convert_coordinates_to_robot(i, tvecs)

        if len(aruco_ids) == 0:
            self.g_aruco    = False
            self.g_ids      = []
            self.g_coords   = []
            return False, None, None
        else:
            self.g_aruco    = True
            self.g_ids      = aruco_ids
            self.g_coords   = aruco_robot_coords
            self.g_rvecs    = rvecs
            print(f"{aruco_ids}")
            return True, aruco_ids, aruco_robot_coords


    def cam_mode_local(self):
        print("Local camera mode")

        cap = cv.VideoCapture(0)

        while cap.isOpened():
            has_frame, frame = cap.read()

            if (not has_frame):
                break

            i, c               = self.detect_markers_list(frame)
            i, c, o            = self.center_group_cubes(i, c)
            i, rvecs, tvecs, d = self.convert_coordinates_to_3d(i, c, o)
            self.convert_coordinates_to_robot(i, tvecs)

            for _ in self.detect_markers_draw(frame):
                frame = self.pose_estimation_draw(frame, self.intrinsic_camera, self.distortion)

            cv.imshow('Estimated Pose', frame)
                    
            if cv.waitKey(50) & 0xFF == 27:
                break

        cap.release()
        cv.destroyAllWindows()


    def image_mode_local(self, filename="test_image.jpg"):
        print("Local image mode")
        print(f"Reading image: {filename}")

        image = cv.imread(filename)

        # i, c               = self.detect_markers_list(image)
        # i, c, o            = self.center_group_cubes(i, c)
        # i, rvecs, tvecs, d = self.convert_coordinates_to_3d(i, c, o)
        # self.convert_coordinates_to_robot(i, tvecs)


        # self.detect_markers_draw(image)
        # #for _ in self.detect_markers_draw(image):
        #     #self.pose_estimation_list(image, self.intrinsic_camera, self.distortion)
        #     #image = self.pose_estimation_draw(image, self.intrinsic_camera, self.distortion)

        # self.draw_centers(image, i, o)

        self.debug_draw_cam(image)

        Alive = True
        while Alive:
            cv.imshow('Estimated Pose', image)
            keypress = cv.waitKey(1)
            if keypress == ord('q') or keypress & 0xFF == 27:
                Alive = False
        cv.destroyAllWindows()


aruco = SAruCo()


if __name__ == "__main__":
    """
    Main loop that demos the AruCo marker detection and pose estimation
    with local camera.

    Can be used on a PC with a webcam or a Raspberry Pi with a camera module.
    """

    args = sys.argv[1:]
    mode     = "cam"
    filename = None

    aruco.load_camera_calibration(filename="../camera_calibration.pkl")

    if len(args) > 0:
        if args[0] == "--cam":
            mode = "cam"
        elif args[0] == "--image":
            mode = "image"
            if len(args) > 1:
                filename = args[1]
        else:
            print("Invalid argument. Use --cam, --image or --robot")
            sys.exit()


    if mode == "cam":
        aruco.cam_mode_local()
    elif mode == "image":
        if filename is not None:
            aruco.image_mode_local(filename)
            # frame = cv.imread(filename)
            # success, ids, coords = aruco.robot_coords_id_list(frame)

            # print(f"ids:")
            # print(ids)
            # print("coords")
            # print(coords)
        else:
            aruco.image_mode_local()


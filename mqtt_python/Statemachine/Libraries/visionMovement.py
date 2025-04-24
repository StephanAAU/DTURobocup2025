import time as t
#import select
import numpy as np
import cv2 as cv
from datetime import *

# robot function
from orighelpers.spose import pose
from orighelpers.sir import ir
from orighelpers.srobot import robot
from orighelpers.scam import cam
from orighelpers.sedge import edge
from orighelpers.sgpio import gpio
from orighelpers.scam import cam
from orighelpers.uservice import service
from Libraries.saruco import aruco


CAMERA_OFFSET_X = 0.5  # Camera's x offset in robot coordinates (meters)
CAMERA_OFFSET_Y = 0.0  # Camera's y offset in robot coordinates (meters)
CAMERA_ORIENTATION = 0.0  # Camera's orientation relative to the robot (radians)

# Target position in camera coordinates
TARGET_CAMERA_X = 1.0  # Target x in camera coordinates (meters)
TARGET_CAMERA_Y = 1.0  # Target y in camera coordinates (meters)

# Robot movement parameters
MOVE_SPEED = 0.1  # Robot's forward speed (meters per step)
TURN_SPEED = 0.1  # Robot's turning speed (radians per step)
POSITION_TOLERANCE = 0.1  # Tolerance for reaching the target (meters)
ANGLE_TOLERANCE = 0.1  # Tolerance for facing the target (radians)


def camera_to_robot_coordinates(camera_x, camera_y):
  """
  Convert camera coordinates to robot coordinates.
  """
  robot_x = CAMERA_OFFSET_X + camera_x * np.cos(CAMERA_ORIENTATION) - camera_y * np.sin(CAMERA_ORIENTATION)
  robot_y = CAMERA_OFFSET_Y + camera_x * np.sin(CAMERA_ORIENTATION) + camera_y * np.cos(CAMERA_ORIENTATION)
  return robot_x, robot_y


def move_robot_toward_target(target_x, target_y):
  """
  Move the robot toward the target position.
  """
  # Get the robot's current position and heading
  robot_x = pose.pose[0]
  robot_y = pose.pose[1]
  robot_heading = pose.pose[2]

  print(f"robot_x: {robot_x:.2f}, robot_y: {robot_y:.2f}, robot_heading: {robot_heading:.2f}")

  # Calculate the distance to the target
  distance = np.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)
  
  print(f"distance to target: {distance:.2f}")
  # Check if the robot has reached the target
  if distance < POSITION_TOLERANCE:
    print("Target reached!")
    return True  # Target reached

  # Calculate the angle to the target
  target_angle = np.arctan2(target_y - robot_y, target_x - robot_x)

  print(f"target_angle: {target_angle:.2f}")
  # Calculate the difference between the robot's current orientation and the target angle
  angle_diff = target_angle - robot_heading
  print(f"angle_diff: {angle_diff:.2f}")

  # Normalize the angle difference to the range [-pi, pi]
  angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
  print(f"normalized angle_diff: {angle_diff:.2f}")

  # Turn the robot toward the target
  if abs(angle_diff) > ANGLE_TOLERANCE:
    service.send(service.topicCmd + "ti/rc", f"0 {np.sign(angle_diff) * TURN_SPEED}")  # Turn
    print(f"turning with speed: {np.sign(angle_diff) * TURN_SPEED}")
  else:
    # Move the robot forward
    service.send(service.topicCmd + "ti/rc", f"{MOVE_SPEED} 0")  # Move forward
    print(f"moving forward with speed: {MOVE_SPEED}")

  print("returning false")
  return False  # Target not yet reached





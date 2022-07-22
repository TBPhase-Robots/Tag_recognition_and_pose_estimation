"""
Code description: Code will identify ArUco tags, and then calculate the position of their centres. It will then use trigonemtry to calculate
their real world position. This position is then published to a ros topic as a geometry pose message. The code can identify new tags and
then make a new ros topics for each tag.
"""

import cv2
from cv2 import cvtColor
from cv2 import aruco

import numpy as np
import time

import os
import pickle

import rospy2 as rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String

class aruco_track():

    def __init__(self):
        if not os.path.exists('./CameraCalibration.pckl'):
            print("Calibration file not found.")
            exit()
        else:
            f = open('./CameraCalibration.pckl', 'rb')
            self.cameraMatrix, self.distCoeffs, _, _ = pickle.load(f)
            f.close()
            if self.cameraMatrix is None or self.distCoeffs is None:
                print("Invalid calibration file.")
                exit()

        # initiate ros parts
        rospy.init_node("camera_tracker")

        # make a topic for every robot (?) or potentially one topic for all robots
        # normally would use a geometry pose message, however these robots move in a 2D plane simplified turtlesim message...
        # ... the need for quarternion to euler transform.

        # as ArUco tags have IDs, the publisher objects are stored in a dictionary with their ID as the key
        self.pub_dict = {}

        rospy.on_shutdown(self.shutdown)

        # imput camera values
        # used later to convert the position in the image to the functions
        self.vert_res = 1920
        self.horiz_res = 1080

        self.cam_height = 1.55
        self.horiz_cam_aperture = 78 *2*np.pi/360
        self.vert_cam_aperture = (self.vert_res/self.horiz_res) * self.horiz_cam_aperture # vert cam aperture 

        self.horiz_dist_ground = self.cam_height * np.tan(self.horiz_cam_aperture)
        self.vert_dist_ground = self.cam_height * np.tan(self.vert_cam_aperture)

        # set up camera for cv2
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cam.set(cv2.CAP_PROP_EXPOSURE, 50)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cam.set(cv2.CAP_PROP_FPS, 60)

        # set up ArUco settings and parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters =  aruco.DetectorParameters_create()

        self.identified_markers = "" # will send a string of the markers that have been identified so that them
        self.ID_pub = rospy.Publisher("/robot_IDs",String,queue_size=1)

        while True: # would normally use a while rospy not shutdown, but caused opencv to crash
            self.get_image()
            self.detect_markers()
            self.calc_positions()
            self.publish_positions()
            self.publish_IDs() # changed so now it constantly publishes ID list
            if cv2.waitKey(1) and  0xFF == ord("q"):
                break

        self.cam.release()

        cv2.destroyAllWindows()


    def get_image(self): # retrieves the image from the webcam

        ret, self.frame = self.cam.read()

        # makes image greyscale
        self.grey = cvtColor(self.frame,cv2.COLOR_BGR2GRAY)

    def detect_markers(self):
        corner_list, id_list, rejectedImgPoints = aruco.detectMarkers(self.grey, self.aruco_dict, parameters=self.parameters)
        # self.frame_markers = aruco.drawDetectedMarkers(self.frame.copy(), corner_list, id_list)
        self.frame_markers = self.frame.copy()

        self.centres_list = []

        self.active_ids = id_list

        if len(corner_list) > 0:
            rotation_vectors, translation_vectors, _objPoints = aruco.estimatePoseSingleMarkers(corner_list, 24/1000, self.cameraMatrix, self.distCoeffs)

            for i in range(len(corner_list)):
                # corners = corner_list[i]
                id = id_list[i][0]

                # corners = corners[0] # there is double bracket, this is to get rid of one of those brackets

                # x_tot = 0
                # y_tot = 0

                # for corner in corners:
                #     x_tot += corner[0]
                #     y_tot += corner[1]

                # middle = np.array([x_tot/4,y_tot/4])

                x = translation_vectors[i][0][0]
                y = translation_vectors[i][0][1]
                orientation = rotation_vectors[i][0][1]

                # orientation = self.calculate_orientation(middle,corners[0],corners[1])

                # centre = [id,int(x_tot/4),int(y_tot/4),orientation]
                centre = [id, x, y, orientation]

                self.centres_list.append(centre)

                # cv2.circle(self.frame_markers,(centre[1],centre[2]),5,(255,0,0),2) # plots a circle in the middle of the marker 
                # cv2.circle(self.frame_markers,(centre[1],centre[2]),5,(255,0,0),2) # plots a circle in the middle of the marker 
                print(translation_vectors[i])
                cv2.drawFrameAxes(self.frame_markers, self.cameraMatrix, self.distCoeffs, rotation_vectors[i], translation_vectors[i], 0.01)


        cv2.imshow("markers",self.frame_markers)

            
    def calculate_orientation(self,centre,top_left,top_right):
        top_middle = (top_right + top_left) / 2
        vec_to_top = top_middle - centre
        # cv2.line(self.frame_markers,(int(centre[0]),int(centre[1])),(int(top_middle[0]),int(top_middle[1])),(0,0,255),8)
        orientation = np.arctan2(vec_to_top[0],vec_to_top[1])
        return orientation

    def calc_positions(self):

        """
        Aim of function: take a list of circle centres from the circle detection, and then calculate their position in real space using
        the info we know about the position of the camera and its resolution and aperture angle etc.

        The estimation may require a fair amount of tuning

        """

        self.vectors_to_robots = self.centres_list

        # for centre in self.centres_list: ## BUGFIX: WHEN ONLY ONE ROBOT, WILL CYCLE THROUGH SCALAR VALUES
        #     # method of working it out from the centre of the image means the vector is relative to directly below the camera

        #     vert_pixels_from_centre = centre[1] - (self.vert_res/2)
        #     horiz_pixels_from_centre = centre[2] - (self.horiz_res/2)

        #     #print(vert_pixels_from_centre, horiz_pixels_from_centre)

        #     vert_pos = vert_pixels_from_centre / (self.vert_res/2) * (self.vert_dist_ground/2)
        #     horiz_pos = horiz_pixels_from_centre / (self.horiz_res/2) * (self.horiz_dist_ground/2)

        #     vector_pos = [centre[0],horiz_pos,vert_pos,centre[3]]

        #     self.vectors_to_robots.append(vector_pos)
 
    def publish_positions(self):

        for pos in self.vectors_to_robots:

            positions = Pose()

            positions.position.x = -pos[2] # could potentially be an earlier maths error, but just this works
            positions.position.y = pos[1]

            positions.orientation.z = pos[3]

            try: # see if there is a publishable node for that ID number
                self.pub_dict[pos[0]].publish(positions)

            except KeyError : # if the key doesn't exist than a new publisher with that key is created.
                self.create_publisher(pos[0])
                self.pub_dict[pos[0]].publish(positions)
            
    def create_publisher(self,ID):
        """
        As the Aruco gives an ID number, a dictionary containing all the publishing objects is created, with each robots publisher as the key
        """

        self.identified_markers += str(ID) + " "

        self.publish_IDs()

        pub_name = "robot_" + str(ID) + "_position"
        self.pub_dict[ID] = rospy.Publisher(pub_name,Pose,queue_size=1) 

    def publish_IDs(self):

        ID_msg = String()

        ID_msg.data = self.identified_markers

        self.ID_pub.publish(ID_msg)

    def shutdown(self):

        print("shutdown")

        self.cam.release()

        cv2.destroyAllWindows()

def main():
    aruco_tacker = aruco_track()


if __name__ == "__main__":
    main()
import rospy
import numpy as np
import math

from nav_msgs.msg import MapMetaData, OccupancyGrid
from simple_odom.msg import PoseConverted, CustomPose
from geometry_msgs.msg import Pose

class CameraObservedAreaTracker:

    def __init__(self):
        rospy.init_node('camera_observed_area_tracker')
        rospy.loginfo('Camera observed area tracker node started')

        self.pose_converted = PoseConverted()
        self.pose = Pose()

        self.map =[[]]
        self.map_info = MapMetaData()

        self.map_camera_seen_initialised = False
        self.map_camera = [[]]
        self.map_camera_seen_seq = 0

        self.robot_x_old = 0
        self.robot_y_old = 0
        self.robot_yaw_old = 0

        # --- Publisher ---
        self.pub_seen_map = rospy.Publisher('/camera_seen_map', OccupancyGrid, queue_size=1)

        # --- Subscriber ---
        self.pose_subscriber = rospy.Subscriber('/simple_odom_pose', CustomPose, self._handle_pose_update)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self._handle_map_update)

        self._setup_camera_seen_masks()
        rospy.loginfo("--- ready ---")
        rospy.spin()    

    def _handle_pose_update(self, data):
        """
        Handles the data that was published to the simple_odom_pose topic.
        """
        self.pose_converted = data.pose_converted
        self.pose = data.pose

        try:
            # update only when the pose changed 
            if self.pose_converted.x != self.robot_x_old or self.pose_converted.y != self.robot_y_old or abs(self.pose_converted.yaw - self.robot_yaw_old) > 0.1:
                # update the mapt
                self._update_map_camera_seen(self.pose_converted.x, self.pose_converted.y, self.pose_converted.yaw)
                
                self.robot_x_old = self.pose_converted.x
                self.robot_y_old = self.pose_converted.y
                self.robot_yaw_old = self.pose_converted.yaw
        except:
            print('update map camera seen failed')

    def _handle_map_update(self, data):
        """
        Handles the data published to the map topic
        """
        self.map_info = data.info
        self.map = np.reshape(data.data, (data.info.height, data.info.width))

        # initialise the camera map if not done yet
        if not self.map_camera_seen_initialised:
            self.map_camera = np.full((data.info.height, data.info.width), -1)
            self._publish_map_camera()
            self.map_camera_seen_initialised = True

    def _update_map_camera_seen(self, x, y, yaw):
        """
        Updates the map that tracks the seen areas from the view of the camera
        """
        #print('Angle: ' + str(yaw))
        yaw = (yaw + math.pi/2.0 + 2*math.pi) % (2*math.pi)
        #print('Angle 2pi: ' + str(yaw))
        
        # North
        if (yaw >= (15.0/16.0 * 2.0 * math.pi) and yaw < (2 * math.pi)) or (yaw >= 0 and yaw < (1.0/16.0 * 2.0 * math.pi)):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_north, self.mask_camera_seen_north_offset_x, self.mask_camera_seen_north_offset_y)

        # North East
        if yaw >= (1.0/16.0 * 2.0 * math.pi) and yaw < (3.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_north_east, self.mask_camera_seen_north_east_offset_x, self.mask_camera_seen_north_east_offset_y)

         # East
        if yaw >= (3.0/16.0 * 2.0 * math.pi) and yaw < (5.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_east, self.mask_camera_seen_east_offset_x, self.mask_camera_seen_east_offset_y)

        # South East
        if yaw >= (5.0/16.0 * 2.0 * math.pi) and yaw < (7.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_south_east, self.mask_camera_seen_south_east_offset_x, self.mask_camera_seen_south_east_offset_y)

        # South
        if yaw >= (7.0/16.0 * 2.0 * math.pi) and yaw < (9.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_south, self.mask_camera_seen_south_offset_x, self.mask_camera_seen_south_offset_y)

        # South West
        if yaw >= (9.0/16.0 * 2.0 * math.pi) and yaw < (11.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_south_west, self.mask_camera_seen_south_west_offset_x, self.mask_camera_seen_south_west_offset_y)

        # West
        if yaw >= (11.0/16.0 * 2.0 * math.pi) and yaw < (13.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_west, self.mask_camera_seen_west_offset_x, self.mask_camera_seen_west_offset_y)

        # North West
        if yaw >= (13.0/16.0 * 2.0 * math.pi) and yaw < (15.0/16.0 * 2.0 * math.pi):
            self._map_camera_set_seen(x, y, self.mask_camera_seen_north_west, self.mask_camera_seen_north_west_offset_x, self.mask_camera_seen_north_west_offset_y)

        self.map_camera_seen_seq = self.map_camera_seen_seq + 1
        
        self._publish_map_camera()

    def _publish_map_camera(self):
        # create occupany grid to publish it
        oG = OccupancyGrid()
        # header
        oG.header.seq = self.map_camera_seen_seq
        oG.header.frame_id = "map"
        oG.header.stamp = rospy.Time.now()
        # set the info like it is in the original map
        oG.info = self.map_info
        # set the map reshaped as array
        oG.data = np.reshape(self.map_camera, self.map_info.height * self.map_info.width)

        self.pub_seen_map.publish(oG)

    def _map_camera_set_seen(self, x, y, mask, offset_x, offset_y):
        """
        This method sets the matrix of the map which represents the seen area by camera
        """
        y_mask = 0
        x_mask = 0

        y_start = y + offset_y
        y_end = (y + offset_y + len(mask))
        y_increment = 1

        x_start = x + offset_x
        x_end = (x + offset_x + len(mask[0]))
        x_increment = 1
       
        # rows
        for y1 in range(y_start, y_end, y_increment):
            x_mask = 0
            # cols
            for x1 in range(x_start, x_end, x_increment):
                # check that not outside of map
                if y1 >= 0 and x1 >= 0 and y1 < len(self.map_camera) and x1 < len(self.map_camera[0]):
                    if mask[y_mask][x_mask] == 1:
                        self.map_camera[y1][x1] = 10
                x_mask = x_mask + 1

            y_mask = y_mask + 1

    def _setup_camera_seen_masks(self):
        self.mask_camera_seen_north = np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                                [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
                                                [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0]])
        self.mask_camera_seen_north_offset_x = -6
        self.mask_camera_seen_north_offset_y = -9

        self.mask_camera_seen_north_east = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                                     [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.mask_camera_seen_north_east_offset_x = 0
        self.mask_camera_seen_north_east_offset_y = -14

        self.mask_camera_seen_east = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
                                               [0, 0, 0, 0, 0, 0, 1, 1, 1, 1],
                                               [0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
                                               [0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
                                               [0, 0, 0, 0, 0, 0, 1, 1, 1, 1],
                                               [0, 0, 0, 0, 0, 0, 0, 0, 1, 1]])
        self.mask_camera_seen_east_offset_x = 0
        self.mask_camera_seen_east_offset_y = -6

        self.mask_camera_seen_south_east = np.array([[1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                                     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                                     [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.mask_camera_seen_south_east_offset_x = 0
        self.mask_camera_seen_south_east_offset_y = 0

        self.mask_camera_seen_south = np.array([[0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
                                                [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                                [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]])
        self.mask_camera_seen_south_offset_x = -6
        self.mask_camera_seen_south_offset_y = 0

        self.mask_camera_seen_south_west = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                     [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.mask_camera_seen_south_west_offset_x = -14
        self.mask_camera_seen_south_west_offset_y = 0

        self.mask_camera_seen_west = np.array([[1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                               [1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                               [1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                               [1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                               [1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                               [1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                                               [1, 1, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.mask_camera_seen_west_offset_x = -10
        self.mask_camera_seen_west_offset_y = -6

        self.mask_camera_seen_north_west = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                                     [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                                     [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                     [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
                                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1]])
        self.mask_camera_seen_north_west_offset_x = -14
        self.mask_camera_seen_north_west_offset_y = -14

if __name__ == "__main__":
    try:
        camera_observed_area_tracker = CameraObservedAreaTracker() 
    except:
        print("Error while startup")
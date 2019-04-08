#!/usr/bin/env python2.7

from animator import Animator
from float_msg_smoother import FloatMsgSmoother
import logging
import math
from nav_msgs.msg import Odometry
import numpy as np
import rospy
import states
from std_msgs.msg import Float32, Float32MultiArray, ByteMultiArray, Int8MultiArray, Bool, String
import sys
from tf.transformations import euler_from_quaternion

class WaypointPlanner:
    """generates waypoints using data from object detection and lane detection"""

    def __init__(self):

        # Initialize node, animator and state
        rospy.init_node('waypoint_planner', anonymous=True)
        self.animator = Animator(self)
        self.state = states.SteadyState(self)

        # Publisher/Subscriber variables
        self.car_speed = 0
        self.lane_heading = FloatMsgSmoother(10, 0)
        self.lane_width = 3 # meters
        self.lane_center_offset = FloatMsgSmoother(3, 0)
        self.lane_curvature = []
        self.lane_change = "none"
        self.left_lane_type = 0
        self.right_lane_type = 0
        self.obstacles = [0,0,0,0,0]

        self.obstacle_flags = [0,0,0,0,0]
        self.obstacle_detect_time = [0,0,0,0,0]
        self.saved_obstacle_pos = [0,0,0,0,0]
        self.obst_removal_time = [0,0,0,0,0]
        self.time_set = [0,0,0,0,0]
        self.obstacle_dist = [0,0,0,0,0]

        self.objects_detected = [False, False, False]

        self.odom_msg_received = False
        self.car_position = [0, 0]
        self.car_position_temp = [0, 0]

        self.car_heading = 0

        self.critical_waypoints = []
        self.curvature_waypoints = []
        self.position_history = []
        self.position_history_counter = 0
        self.stop_sign_distance = 25000.0
        self.horiz_line_distance = -1

        # Waypoint generation class members
        self.DISTANCE_BETWEEN_POINTS = 3 # meters

        # Animator class constants
        self.POSITION_HIST_DIST = 3 # meters
        self.POSITION_HIST_LENGTH = 100

        # Logging
        self.logger = logging.getLogger("planner")
        self.logger.setLevel(logging.DEBUG)

        handler = logging.StreamHandler(sys.stdout)
        FORMAT = '%(message)s'
        handler.setFormatter(logging.Formatter(FORMAT))
        self.logger.addHandler(handler)

        self.logger.info("Waypoint planner initialized.\n")

        # Subscribers
        self.car_speed_sub = rospy.Subscriber('/autodrive_sim/output/speed', Float32, self.car_speed_callback)

        self.lane_heading_sub = rospy.Subscriber('/lane_detection/lane_heading', Float32, self.lane_heading_callback)
        self.lane_width_sub = rospy.Subscriber('/lane_detection/lane_width', Float32, self.lane_width_callback)
        self.lane_center_offset_sub = rospy.Subscriber('/lane_detection/lane_center_offset', Float32, self.lane_center_offset_callback)
        self.lane_curvature_sub = rospy.Subscriber('/lane_detection/lane_curvature', Float32MultiArray, self.lane_curvature_callback)
        self.lane_change_sub = rospy.Subscriber('/lane_detection/lane_change/TEST', String, self.lane_change_callback)
        self.left_lane_type_sub = rospy.Subscriber('/lane_detection/left_lane_type', Float32, self.left_lane_type_callback)
        self.right_lane_type_sub = rospy.Subscriber('/lane_detection/right_lane_type', Float32, self.right_lane_type_callback)
        self.obstacle_sub = rospy.Subscriber('/lane_occupation', Int8MultiArray, self.obstacle_callback)
        self.obstacle_dist_sub = rospy.Subscriber('/obstacle_distance', Float32MultiArray, self.obstacle_dist_callback)


        self.car_heading_sub = rospy.Subscriber('/autodrive_sim/output/heading', Float32, self.car_heading_callback)
        # self.self_objects_vector_sub = rospy.Subscriber('/lane_occupation', ByteMultiArray, self.detected_objects_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.position_sub = rospy.Subscriber('/autodrive_sim/output/position', Float32MultiArray, self.position_callback)
        self.stop_sign_dist_sub = rospy.Subscriber('/sign_detection/stop_sign_distance', Float32, self.stop_sign_dist_callback)
        self.horiz_line_dist_sub = rospy.Subscriber('/lane_detection/stop_line_distance', Float32, self.horiz_line_dist_callback)

        # Publisher
        self.waypoints_pub = rospy.Publisher('/autodrive_sim/output/waypoints', Float32MultiArray, queue_size = 1)
        self.stop_dist_pub = rospy.Publisher('/waypoint_planner/stop_distance', Float32, queue_size = 1)
        self.speed_factor = rospy.Publisher('/waypoint_planner/speed_factor', Float32MultiArray, queue_size = 1)
        self.horizontal_line_activate_pub = rospy.Publisher('/waypoint_planner/horizontal_line_detection', Bool, queue_size = 1)
        self.lane_angle_pub = rospy.Publisher('/waypoint_planner/lane_angle', Float32, queue_size = 1)

    def car_speed_callback(self, msg):
        """Processes speed messages from gps"""
        self.car_speed = msg.data

    def lane_heading_callback(self, msg):
        """Processes heading messages from lane detection"""
        limit = np.pi / 7
        heading = np.clip(msg.data, -limit, limit)
        self.lane_heading.add_value(heading)

        self.lane_angle_pub.publish(self.lane_angle())

    def lane_width_callback(self, msg):
        """Processes width messages from lane detection"""
        self.lane_width = msg.data

    def lane_center_offset_callback(self, msg):
        """Processes lane offset messages from lane detection"""
        limit = 1.5
        center = np.clip(msg.data, -limit, limit)
        self.lane_center_offset.add_value(center)

    def lane_curvature_callback(self, msg):
        """Processes lane curvature messages from lane detection"""
        self.lane_curvature = msg.data

    def lane_change_callback(self, msg):
        """Testing purposes, initiates lane change"""
        self.lane_change = msg.data

    def left_lane_type_callback(self, msg):
        """Processes left lane type messages from lane detection"""
        self.left_lane_type = msg.data

    def right_lane_type_callback(self, msg):
        """Processes right lane type messages from lane detection"""
        self.right_lane_type = msg.data

    def obstacle_callback(self, msg):
        """processes object vector from object detection"""
        self.obstacles = msg.data

    def obstacle_dist_callback(self, msg):
        """processes object vector from object detection"""
        self.obstacle_dist = msg.data

    def odom_callback(self, msg):
        """Processes messages from the odometry topic"""
        self.odom_msg_received = True
        pos = msg.pose.pose.position
        new_position = np.array([pos.x, pos.y])
        self.update_position_history(new_position)
        self.car_position = new_position
        self.car_position_temp = new_position

        ori = msg.pose.pose.orientation
        orientation_list = [ori.x, ori.y, ori.z, ori.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.car_heading = yaw

    def stop_sign_dist_callback(self, msg):
        """Processes distance messages from sign detection"""
        self.stop_sign_distance = msg.data

    def horiz_line_dist_callback(self, msg):
        """Processes distance messages from horizontal line detector"""
        self.horiz_line_distance = msg.data

    def car_heading_callback(self, msg):
        """processes lane offset messages from lane detection"""
        self.car_heading = msg.data

    def position_callback(self, msg):
        """processes lane offset messages from lane detection"""
        new_position = np.array([msg.data[0], msg.data[1]])
        self.update_position_history(new_position)

    def detected_objects_callback(self, msg):
        """processes object vector from object detection"""
        for i in range(0,3):
            if msg.data[i] == 1:
                self.objects_detected[i] = True
            else:
                self.objects_detected[i] = False

    def update_position_history(self, new_position):
        """Updates the car's position history with the given new position"""
        if len(self.position_history) == 0:
            self.position_history.append(new_position)
        else:
            if self.magnitude(self.position_history[-1], new_position) > self.POSITION_HIST_DIST:
                while len(self.position_history) > self.POSITION_HIST_LENGTH:
                    self.position_history.pop(0)
                self.position_history.append(self.car_position)
        #self.car_position = new_position

    def magnitude(self, point1, point2):
        mag_vec = point2 - point1
        return np.linalg.norm(mag_vec)

    def lane_angle(self):
        """returns the current lane heading in a global frame in radians"""
        lane_global_angle = self.car_heading + (self.lane_heading.value())
        return lane_global_angle

    def point_behind_car(self, point):
        """Checks whether a point has been passed by the car"""
        # Car position
        Cx = self.car_position[0]
        Cy = self.car_position[1]

        # Lane angle offset
        angle = self.lane_angle()
        Tx = Cx + np.sin(angle)
        Ty = Cy - np.cos(angle)

        # Point behind car
        Px = point[0]
        Py = point[1]

        determinant = (Tx - Cx) * (Py - Cy) - (Ty - Cy) * (Px - Cx)
        return determinant < 0

    def rotate_waypoint(self, point):
        px, py = point
        angle_vec = (math.cos(self.car_heading), math.sin(self.car_heading))
        new_x = angle_vec[0] * px - angle_vec[1] * py
        new_y = angle_vec[1] * px + angle_vec[0] * py
        return np.array([new_x, new_y])

    def generate_waypoints(self):
        """generates and publishes waypoints to the control teams"""
        self.critical_waypoints = []

        # Add points in front of the car
		
        angle_scale_factor = 3
        angle = math.atan(self.lane_heading.value()) / angle_scale_factor
        last_point = np.array([0, 0])
        for i in range(1, 6):
            new_angle = angle * i
            new_point = np.array([math.cos(new_angle), math.sin(new_angle)]) * self.DISTANCE_BETWEEN_POINTS
            new_point += last_point
            last_point = new_point

            new_point = self.rotate_waypoint(new_point)

            center_offset_value = self.lane_center_offset.value() / 3.5
            center_offset_point = np.array([0, center_offset_value])
            new_point += self.rotate_waypoint(center_offset_point)

            new_point += self.car_position
            self.critical_waypoints.append(new_point)

        self.publish_waypoints()

    def publish_waypoints(self):
        """Formats and publishes waypoints message to control teams"""
        msg = Float32MultiArray()

        # Append the car's position as the first waypoint
        #msg.data.append(self.car_position[0])
        #msg.data.append(self.car_position[1])

        # Append the critical waypoints
        for waypoint in self.critical_waypoints:
            #print "Adding Waypoint To Publish: " + str(len(msg.data))
            msg.data.append(waypoint[0])
            msg.data.append(waypoint[1])

        # Finally append the curvature waypoints (curently none)
        for waypoint in self.curvature_waypoints:
            msg.data.append(waypoint[0])
            msg.data.append(waypoint[1])

        self.waypoints_pub.publish(msg)

    def wait_for_sensor_msgs(self):
        """Waits for sensor messages to be published at least once"""
        self.logger.debug("Waiting for sensor msgs...")
        while True:
            if rospy.is_shutdown():
                sys.exit(0)
            elif self.odom_msg_received: #and len(self.lane_curvature) > 0:
                self.logger.debug("Sensor msgs received!")
                break

    def waypoint_lanechange(self, Vc): # Vc is current velocity in (m/s)

        if Vc < 4:
            Vc = 4

        Mlat = 0.4 # m/s^2
        r = (Vc**2)/Mlat # circle radius

        yf1 = 5.0 # single lane : meter (final y w.r.t initial point y = 0;)
        yf2 = 7.0 # double lane

        dx1 = np.sqrt((2*(yf1/2)*r)-(yf1/2)**2) # Longitudinal distance to the middle point of LC
        dx2 = np.sqrt((2*(yf2/2)*r)-(yf2/2)**2)

        y1_left = []

    # Single lane change
        dx = dx1
        x1 = np.linspace(0,int(dx*2),int(dx + 1 + 5)) # x position for single lane change
        y1_left = np.zeros(int(dx+1 + 5))
        y1_right = np.zeros(int(dx+1 + 5))

        
        for i in range(1,len(x1),1):
            if i < len(x1) - 5:
                if (x1[i]< dx) :
                    y1_left[i] = r-np.sqrt(r**2-x1[i]**2) # left lane change(single)
                    y1_right[i] = -y1_left[i] # right lane change(single)
                    k = i
                else:
                    y1_left[i] = yf1-r + np.sqrt(r**2-x1[i]**2+2*x1[i]*(2*dx)-(2*dx)**2)
                    y1_right[i] = -y1_left[i]
                    k = i
            else:
                y1_left[i] = y1_left[k]
                y1_right[i] = -y1_left[i] # right lane change(single)

    # Double lane change
        dx = dx2
        x2 = np.linspace(0,int(dx*2),int(dx + 1 + 5))  # x position for double lane change
        y2_left = np.zeros(int(dx+1 + 5))
        y2_right = np.zeros(int(dx+1 + 5))



        for i in range(1,len(x2),1):
            if i < len(x2) - 5:
                if (x2[i] < dx):
                    y2_left[i] = r-np.sqrt(r**2-x2[i]**2) # right lane change(double)
                    y2_right[i] = -y2_left[i] # right lane change(double)
                    k = i
                else:
                    y2_left[i] = yf2-r+np.sqrt(r**2-x2[i]**2+2*x2[i]*(2*dx)-(2*dx)**2)
                    y2_right[i] = - y2_left[i]
                    k = i
            else:
                y2_left[i] = y2_left[k]
                y2_right[i] = -y2_left[i] # right lane change(single)

        #r1 = [dx1, x1, y1_left ,y1_right]
        #r2 = [dx2, x2, y2_left, y2_right]
        #print r1 
        #print r2

        return [dx1, dx2, x1, x2, y1_left, y2_left]


    def scenario(self, current_lane, dis_object_0, dis_object_1, dis_object_2, v, minimum_d_1, minimum_d_2):
        #dis_object_i is the distance of object in lane i from car
        #dis_object_i is negative when there is no object in lane i
        
        #minimum_d_1, minimum_d_2, x1, x2, y1_left, y2_left = Waypoint_LaneChange(v)
        stop = False
        bias = 11
        possible_scenarios = [-1, -1, -1]

        print "min dist " + str(minimum_d_2)
        
        #Potential scenarios when the car is in lane 0 (left most lane):
        if current_lane == 0 and dis_object_0 < 0:
            possible_scenarios[0] = 3000
            print('A')
            return([stop, possible_scenarios])
        elif current_lane == 0 and dis_object_0 > 0: 
            if dis_object_0 < minimum_d_1/2 + bias - 2:
                stop = True
                print(minimum_d_1/2 + bias - 2)
                print(dis_object_0)
                print('B')
                return([stop, possible_scenarios])
            
            if dis_object_0 > minimum_d_1/2 + bias:
                #if dis_object_0 > minimum_d_1:
                print('C_1')
                possible_scenarios[0] = 5000
            
            else:
                #check for potential lane change from 0 to 1:
                if dis_object_1 < 0:
                    print('C_2')
                    possible_scenarios[1] = 2000
                elif dis_object_1 > 0 and dis_object_1 > 3/2*minimum_d_1 + 2*bias:
                    print(dis_object_1)
                    print('C_3')
                    possible_scenarios[1] = dis_object_1+10
                    
                #check for potential lane change from 0 to 2:
                if dis_object_1 > 3*minimum_d_2/4 + bias and (dis_object_2 < 0 or dis_object_2 > minimum_d_1/2+minimum_d_2+2*bias):
                    print('C_4')
                    possible_scenarios[2] = dis_object_1              
                
            print('C')   
            return([stop, possible_scenarios])

        
        #Potential scenarios when the car is in lane 1:
        elif current_lane == 1 and dis_object_1 < 0:
            possible_scenarios[1] = 3000
            print('D') 
            return([stop, possible_scenarios])
        elif current_lane == 1 and dis_object_1 > 0:
            if dis_object_1 < minimum_d_1/2 + bias - 2:
                stop = True
                print('E') 
                return([stop, possible_scenarios])
            
            if dis_object_1 > minimum_d_1/2 + bias:
                #if dis_object_1 > minimum_d_1:
                possible_scenarios[1] = 5000
            
            else:
                #check for potential lane change from 1 to 0:
                if dis_object_0 < 0:
                    possible_scenarios[0] = 2500
                elif dis_object_0 > 0 and dis_object_0 > 3/2*minimum_d_1 + 2*bias:
                    possible_scenarios[0] = dis_object_0 + 10
                    
                #check for potential lane change from 1 to 2:
                if dis_object_2 < 0:
                    possible_scenarios[2] = 2000
                elif dis_object_2 > 0 and dis_object_2 > 3/2*minimum_d_1 + 2*bias:
                    possible_scenarios[2] = dis_object_2 + 10               
            
            print('F') 
            return([stop, possible_scenarios])


        ##Potential scenarios when the car is in lane 2:
        if current_lane == 2 and dis_object_2 < 0:
            possible_scenarios[2] = 3000
            print('G') 
            return([stop, possible_scenarios])
        elif current_lane == 2 and dis_object_2 > 0: 
            if dis_object_2 < minimum_d_1/2 + bias - 2:
                stop = True
                print('H') 
                return([stop, possible_scenarios])
            
            if dis_object_2 > minimum_d_1/2 + bias:
                #if dis_object_2 > minimum_d_1:
                possible_scenarios[2] = 5000

            else:
                #check for potential lane change from 2 to 1:
                if dis_object_1 < 0:
                    possible_scenarios[1] = 2000
                elif dis_object_1 > 0 and dis_object_1 > 3/2*minimum_d_1 + 2*bias:
                    possible_scenarios[1] = dis_object_1+10

                #check for potential lane change from 2 to 0:
                if dis_object_1 > 3*minimum_d_2/4 + bias and (dis_object_0 < 0 or dis_object_0 > minimum_d_1/2+minimum_d_2+2*bias):
                    possible_scenarios[0] = dis_object_1 

            print('I') 
            return([stop, possible_scenarios])


    '''
    def scenario(self, current_lane, dis_object_0, dis_object_1, dis_object_2, v):
        #dis_object_i is the distance of object in lane i
        #dis_object_i is negative when there is no object in lane i
        
        minimum_d_1, minimum_d_2 = self.dis_velo(v)
        
        stop = False
        bias = 5
        possible_scenarios = [-1, -1, -1]
        
        #Potential scenarios when the car is in lane 0 (left most lane):
        if current_lane == 0 and dis_object_0 < 0:
            possible_scenarios[0] = 3000
            return([stop, possible_scenarios])
        elif current_lane == 0 and dis_object_0 > 0: 
            if dis_object_0 < minimum_d_1:
                stop = True
                return([stop, possible_scenarios])
            
            possible_scenarios[0] = 5000
            
            #check for potential lane change from 0 to 1:
            if dis_object_1 < 0:
                possible_scenarios[1] = 2000
            elif dis_object_1 > 0 and dis_object_1 > 3/2*minimum_d_1 + 2*bias:
                possible_scenarios[1] = dis_object_1
            
            #check for potential lane change from 0 to 2:
            if dis_object_1 < 0  and (dis_object_2 < 0 or dis_object_2 > minimum_d_1+minimum_d_2+bias):
                possible_scenarios[1] = dis_object_2
            
            elif dis_object_1 > minimum_d_2/2 + bias and (dis_object_2 < 0 or dis_object_2 > minimum_d_1+minimum_d_2+bias):
                possible_scenarios[1] = dis_object_1
            
            return([stop, possible_scenarios])

        
        #Potential scenarios when the car is in lane 1:
        elif current_lane == 1 and dis_object_1 < 1:
            possible_scenarios[1] = 3000
            return([stop, possible_scenarios])
        
        elif current_lane == 1 and dis_object_1 > 0:
            if dis_object_1 < minimum_d_1/2+bias:
                stop = True
                return([stop, possible_scenarios])
            
            possible_scenarios[1] = 5000
            
            #check for potential lane change from 1 to 0:
            if dis_object_0 < 0:
                possible_scenarios[0] = 2000
            elif dis_object_0 > 0 and dis_object_0 > 3/2*minimum_d_1 + 2*bias:
                possible_scenarios[0] = dis_object_1 + dis_object_0        
        
            #check for potential lane change from 1 to 2:
            if dis_object_2 < 0:
                possible_scenarios[2] = 2000
            elif dis_object_2 > 0 and dis_object_2 > 3/2*minimum_d_1 + 2*bias:
                possible_scenarios[2] = dis_object_1 + dis_object_2
            
            return([stop, possible_scenarios])

                
        ##Potential scenarios when the car is in lane 2:
        elif current_lane == 2 and dis_object_2 < 0:
            possible_scenarios[2] = 3000
            return([stop, possible_scenarios])

        elif current_lane == 2 and dis_object_2 > 0: 
            if dis_object_2 < minimum_d_1:
                stop = True
                return([stop, possible_scenarios])
            
            possible_scenarios[2] = 5000
            
            #check for potential lane change from 2 to 1:
            if dis_object_1 < 0:
                possible_scenarios[1] = 2000
            elif dis_object_1 > 0 and dis_object_1 > 3/2*minimum_d_1 + 2*bias:
                possible_scenarios[1] = dis_object_2 + dis_object_1

            #check for potential lane change from 2 to 0:
            if dis_object_1 < 0  and (dis_object_0 < 0 or dis_object_0 > minimum_d_1+minimum_d_2+bias):
                possible_scenarios[0] = dis_object_0
            
            elif dis_object_1 > minimum_d_2/2 + bias and (dis_object_0 < 0 or dis_object_0 > minimum_d_1+minimum_d_2+bias):
                possible_scenarios[0] = dis_object_1
            
            return([stop, possible_scenarios])
    '''

    def run(self):
        # Wait for sensor messages to prime
        self.wait_for_sensor_msgs()

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # Enter new state
            self.state.enter_state()
            new_state = None

            # Main state loop
            while new_state is None:
                if rospy.is_shutdown():
                    return
                self.state.run_state()
                self.animator.render()
                rate.sleep()
                new_state = self.state.check_transitions()

            # Exit current state
            self.state.exit_state()
            self.state = new_state


if __name__ == "__main__":
    planner = WaypointPlanner()
    planner.run()

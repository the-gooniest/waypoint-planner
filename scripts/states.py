import logging
import numpy as np
from std_msgs.msg import Float32
import time

class VehicleState(object):
    """State Machine parent class"""

    def __init__(self, planner):
        self.planner = planner
        self.transitions = []
        self.logger = logging.getLogger("planner")

    def set_transitions(self):
        """This method should be overridden to initialize the transitions list"""
        pass

    def check_transitions(self):
        """Initiates a transition if it's conditions are met"""
        for transition in self.transitions:
            potential_state = transition()
            if potential_state is not None:
                return potential_state

    def get_state_name(self):
        return self.__class__.__name__

    def enter_state(self):
        """Runs upon entering a new state"""
        self.logger.debug("Entering state " + self.get_state_name())

    def run_state(self):
        """The executing method for this state"""
        pass

    def exit_state(self):
        """Runs upon exiting the current state"""
        self.logger.debug("Exiting state " + self.get_state_name())


class SteadyState(VehicleState):

    def __init__(self, planner):
        super(SteadyState, self).__init__(planner)
        self.transitions.append(self.change_lane_transition)
        self.transitions.append(self.stop_sign_transition)
        self.start_time = time.time()
        self.initial_car_pos_roadblock = [0,0]
        self.dist_to_roadblock = 15
        self.is_roadblock = 0
        self.is_test = 0

    def change_lane_transition(self):
        #print "lane heading x: " + str(self.planner.lane_heading.value())
        #self.is_roadblock = 1
        #print "lane offset: " + str(self.planner.lane_center_offset.value())
        '''
        if(self.is_test == 0):
            self.is_test = 1
            return ChangeLaneState(self.planner, ChangeLaneState.LEFT_DIRECTION, ChangeLaneState.SINGLE)
        else:
            return None
        print(self.planner.lane_heading.value())
        '''
        if(self.is_roadblock == 0):
            if(np.abs(self.planner.lane_heading.value()) < 0.1):
                #obstacle in current lane
                if(self.planner.obstacles[2] == 1):
                    print('obstacle ahead')
                    #time.sleep(0.45)

                    # center object not in consideration
                    if(self.planner.obstacle_dist[2] >= 20.0):
                        return None

                    print "Valid lane change..."

                    #car in left lane
                    if((self.planner.left_lane_type == 0 and self.planner.right_lane_type == 1) or self.planner.left_lane_type == 0):
                        #obstacle in center lane and nothing in right lane
                        if(self.planner.obstacles[3] == 1 and self.planner.obstacles[4] == 0):
                            #turn double right
                            print('2 RIGHT')
                            return ChangeLaneState(self.planner, ChangeLaneState.RIGHT_DIRECTION, ChangeLaneState.DOUBLE)
                        #obstacle in center and right lane
                        elif(self.planner.obstacles[3] == 1 and self.planner.obstacles[4] == 1):
                            print('ALL LANES BLOCKED B, STOP!!!')
                            self.is_roadblock = 1
                            return None
                        else:
                            #turn right
                            print('1 RIGHT')
                            return ChangeLaneState(self.planner, ChangeLaneState.RIGHT_DIRECTION, ChangeLaneState.SINGLE)

                    #car in right lane
                    elif((self.planner.left_lane_type == 1 and self.planner.right_lane_type == 0) or self.planner.right_lane_type == 0):
                        #obstacle in center lane and nothing in left lane
                        if(self.planner.obstacles[1] == 1 and self.planner.obstacles[0] == 0):
                            #turn double left
                            print('2 LEFT')
                            return ChangeLaneState(self.planner, ChangeLaneState.LEFT_DIRECTION, ChangeLaneState.DOUBLE)
                        #obstacle in center and left lane
                        elif(self.planner.obstacles[1] == 1 and self.planner.obstacles[0] == 1):
                            print('ALL LANES BLOCKED C, STOP!!!')
                            self.is_roadblock = 1
                            return None
                        else:
                            #turn left
                            print('1 LEFT')
                            return ChangeLaneState(self.planner, ChangeLaneState.LEFT_DIRECTION, ChangeLaneState.SINGLE)

                    #car in center lane
                    elif(self.planner.left_lane_type == 1 and self.planner.right_lane_type == 1) or (self.planner.left_lane_type == -1 and self.planner.right_lane_type == -1):
                        #obstacle in left lane
                        if(self.planner.obstacles[1] == 1):
                            #obstacle in right lane
                            if(self.planner.obstacles[3] == 1):
                                #all lanes blocked. stop
                                print('ALL LANES BLOCKED D, STOP!!!')
                                self.is_roadblock = 1
                                return None
                            else:
                                #turn right
                                return ChangeLaneState(self.planner, ChangeLaneState.RIGHT_DIRECTION, ChangeLaneState.SINGLE)
                        else:
                            #turn left
                            print('1 LEFT center lane')
                            return ChangeLaneState(self.planner, ChangeLaneState.LEFT_DIRECTION, ChangeLaneState.SINGLE)

                    # safety
                    elif(self.planner.obstacles[1] == 1 and self.planner.obstacles[3] == 1):
                        #all lanes blocked. stop
                        print('all lanes blocked A. stop')
                        self.is_roadblock = 1
                        return None
                    #print('unknown lane type')
                    #self.is_roadblock = 1

                    # lol
                    else:
                        print "you areint ina lene gou"
            else:
                #print("ignore objects")
                return None
        else:
            car_pos = self.planner.car_position
            if(self.initial_car_pos_roadblock[0] == 0 and self.initial_car_pos_roadblock[1] == 0):
                self.initial_car_pos_roadblock = car_pos
            else:
                self.dist_to_roadblock = 0 #15 - np.sqrt((car_pos[0] - self.initial_car_pos_roadblock[0])**2 + (car_pos[1] - self.initial_car_pos_roadblock[1])**2)
                print "pub dist to roadblock: " + str(self.dist_to_roadblock)
                msg = Float32()
                msg.data = self.dist_to_roadblock
                self.planner.stop_dist_pub.publish(msg)

    def stop_sign_transition(self):
        if self.planner.stop_sign_distance == -1.0:
            return None

        if self.planner.stop_sign_distance < 55: # 55m away from stop sign
            return StopSignState(self.planner)
        return None

    def enter_state(self):
        super(SteadyState, self).enter_state()

    def run_state(self):
        if(self.is_roadblock == 0):
            self.planner.stop_dist_pub.publish(25000.00)
        self.planner.generate_waypoints()

    def exit_state(self):
        super(SteadyState, self).exit_state()


class ChangeLaneState(VehicleState):
    LEFT_DIRECTION = 1
    RIGHT_DIRECTION = -1
    SINGLE = 0
    DOUBLE = 1

    def __init__(self, planner, lane_change_direction, lane_change_type):
        super(ChangeLaneState, self).__init__(planner)
        self.transitions.append(self.steady_state_transition)
        self.lane_change_direction = lane_change_direction
        self.prev_car_pos_lane_change = [0,0]
        print("Lane Change: " + self.planner.lane_change)
        self.initial_car_pos_roadblock = [0,0]
        self.dist_to_roadblock = 15
        self.is_roadblock = 0
        self.lane_change_type = lane_change_type

    def steady_state_transition(self):
        #print()
        #if(np.sqrt((self.planner.car_position[0] - self.prev_car_pos_lane_change[0])**2 + (self.planner.car_position[1] - self.prev_car_pos_lane_change[1])**2) > 9):
        #    return SteadyState(self.planner)
        #return None
        if len(self.planner.critical_waypoints) < 4:
            return SteadyState(self.planner)
        return None

    def enter_state(self):
        """Publishes new lane change waypoints"""
        super(ChangeLaneState, self).enter_state()

        # Clear existing waypoints (lane change state now has override of waypoints)
        self.planner.critical_waypoints = []

        angle = self.planner.lane_angle()
        print "Lane Angle: " + str(angle)

        direction = (np.cos(angle), np.sin(angle))
        print "Direction: " + str(direction)
        print "Lane center offset: " + str(self.planner.lane_center_offset.value())
        #lane_change_distance = self.planner.lane_width * self.lane_change_direction

        #lane_center_offset = np.array([
        #    np.sin(angle) * (self.planner.lane_center_offset.value() + lane_change_distance),
        #    -np.cos(angle) * (self.planner.lane_center_offset.value() + lane_change_distance)])


        #lateral_offset = 0.75
        car_heading = self.planner.car_heading
        car_position = self.planner.car_position

        self.prev_car_pos_lane_change = car_position
        play_point = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
        '''
        play_point[0] = [4.1223,0.396]
        play_point[1] = [8.1377,0.9890]
        play_point[2] = [12.0436,1.7595]
        play_point[3] = [15.8395,2.6341]
        play_point[4] = [19.7093,3.5313]
        play_point[5] = [23.9013,4.3583]
        play_point[6] = [28.3973,4.7540]
        play_point[7] = [33.1095,4.8770]
        play_point[8] = [38.0,4.8780]
        play_point[9] = [43.0,4.8750]
        play_point[10] = [48.0,4.870]
        '''
        gain_value_x = 1.0
        gain_value_y = 1.0
        points_length = 14
        lane_offset = self.planner.lane_center_offset.value()

        if(self.lane_change_type == 0):
            #single lane change
            #lane_offset_gain = (3.5 + lane_offset)/1.0 #note: the denominator should be same as gain_value_y
            gain_value_x = 0.6
            #gain_value_y = 1.0
            gain_value_y = (3.5 + lane_offset*self.lane_change_direction)/(3.5) #note: the denominator should be same as gain_value_y
            print "y_distance_1: " + str(gain_value_y*3.5)

            points_length = 14
            play_point[0] = [3.2056,0.03]
            play_point[1] = [6.6765,0.2027]
            play_point[2] = [9.9406,0.5065]
            play_point[3] = [13.4507,0.9427]
            play_point[4] = [16.5097,1.3783]
            play_point[5] = [19.6240,1.8437]
            play_point[6] = [22.7244,2.3016]
            play_point[7] = [25.7428,2.7154]
            play_point[8] = [29.1773,3.1108]
            play_point[9] = [32.3510,3.3659]
            play_point[10] = [35.7288,3.4886]
            play_point[11] = [38.0,3.5]
            play_point[12] = [41.35,3.5]
            play_point[13] = [44.85,3.5]
            play_point[14] = [48.2,3.5]
        else:
            #double lane change
            gain_value_x = 0.55
            #gain_value_y = 2.0
            gain_value_y = (7 + lane_offset*self.lane_change_direction)/(3.5) #note: the denominator should be same as gain_value_y
            print "y_distance_2: " + str(gain_value_y*3.5)

            points_length = 14
            play_point[0] = [3.2056,0.03]
            play_point[1] = [6.6765,0.2027]
            play_point[2] = [9.9406,0.5065]
            play_point[3] = [13.4507,0.9427]
            play_point[4] = [16.5097,1.3783]
            play_point[5] = [19.6240,1.8437]
            play_point[6] = [22.7244,2.3016]
            play_point[7] = [25.7428,2.7154]
            play_point[8] = [29.1773,3.1108]
            play_point[9] = [32.3510,3.3659]
            play_point[10] = [35.7288,3.4886]
            play_point[11] = [38.0,3.5]
            play_point[12] = [41.35,3.5]
            play_point[13] = [44.85,3.5]
            play_point[14] = [48.2,3.5]
            '''
            gain_value_x = 1.3
            gain_value_y = 1.0

            points_length = 11
            play_point[0] = [3.095,0.0410]
            play_point[1] = [6.2916,0.3414]
            play_point[2] = [9.2670,1.1416]
            play_point[3] = [12.1401,2.6882]
            play_point[4] = [14.6978,4.4697]
            play_point[5] = [17.6312,5.9714]
            play_point[6] = [20.6808,6.7164]
            play_point[7] = [23.9353,6.9737]
            play_point[8] = [26.6000,7.00]
            play_point[9] = [29.2,7.00]
            play_point[10] = [31.8,7.00]
            play_point[11] = [34.4,7.00]
            '''

        for i in range(0, points_length):
            new_point = [0, 0]
            x_a = play_point[i][0] * gain_value_x
            y_a = play_point[i][1] * gain_value_y

            lane_heading = self.planner.lane_heading.value() * -1
            #lane_heading = -0.2 * -1

            x_b = x_a * np.cos(lane_heading) + y_a * np.sin(lane_heading)
            y_b = -1 * x_a * np.sin(lane_heading) + y_a * np.cos(lane_heading)

            #x_a = 18 -> if angle is positive, x_b less. if angle is negative, x_b is greater.
            #y_a = -6  -> if angle is positive, y_b is less. if angle is negative, y_b is greater.
            #angle = 0.2

            print(y_b - y_a)

            new_point[0] = car_position[0] + x_b * np.cos(car_heading) + y_b * np.sin(car_heading)
            new_point[1] = (car_position[1] + x_b * np.sin(car_heading) + y_b * np.cos(car_heading) * self.lane_change_direction)

            #new_point[0] = car_position[0] + play_point[i][0] * np.cos(car_heading) + play_point[i][1] * np.sin(car_heading)
            #new_point[1] = (car_position[1] + play_point[i][0] * np.sin(car_heading) + play_point[i][1] * np.cos(car_heading) * self.lane_change_direction)

            #print str(i) + "car x: " + str(play_point[i][0])
            #print str(i) + "car y: " + str(play_point[i][1])
            #print str(i) + "car x: " + str(car_position[0])
            #print str(i) + "car y: " + str(car_position[1])
            #print str(i) + "lane head: " + str(self.planner.lane_heading.value())

            #new_point[0] = car_position[0] + (self.planner.DISTANCE_BETWEEN_POINTS * 1.1 * i) * np.cos(car_heading) + (self.lane_change_direction * lateral_offset * i) * np.sin(car_heading)
            #new_point[1] = car_position[1] + (self.planner.DISTANCE_BETWEEN_POINTS * 1.1 * i) * np.sin(car_heading) + (self.lane_change_direction * lateral_offset * i) * np.cos(car_heading)
            self.planner.critical_waypoints.append(new_point)

        self.planner.publish_waypoints()

    def run_state(self):
        '''
        # Juans linear initial waypoint publish with lateral offset
        self.planner.critical_waypoints = []
        lateral_offset = 0.75
        car_heading = self.planner.car_heading
        car_position = self.planner.car_position
        print(np.sqrt((self.planner.car_position[0] - self.prev_car_pos_lane_change[0])**2 + (self.planner.car_position[1] - self.prev_car_pos_lane_change[1])**2))
        print "lane heading x: " + str(self.planner.lane_heading.value())
        for i in range(1, 8):
            new_point = [0, 0]
            new_point[0] = car_position[0] + (self.planner.DISTANCE_BETWEEN_POINTS * 1.1 * i) * np.cos(car_heading) + (self.lane_change_direction * lateral_offset * i) * np.sin(car_heading)
            new_point[1] = car_position[1] + (self.planner.DISTANCE_BETWEEN_POINTS * 1.1 * i) * np.sin(car_heading) + (self.lane_change_direction * lateral_offset * i) * np.cos(car_heading)
            self.planner.critical_waypoints.append(new_point)

        self.planner.publish_waypoints()
		'''

        #Roadblock during lane change
        '''
        if self.is_roadblock == 0:
            if(self.planner.obstacles[1] == 1 and self.planner.obstacles[2] == 1 and self.planner.obstacles[3] == 1):
                self.is_roadblock = 1
                print('All lanes blocked. Q')
                #elif(self.planner.left_lane_type == 0 and self.planner.right_lane_type == 1 and self.planner.obstacles[1] == 1 and self.planner.obstacles[2] == 1):
                #    #car in left lane, obstacles in left and center lane - double right
                #    print('double right')
                #    return ChangeLaneState(self.planner, ChangeLaneState.RIGHT_DIRECTION, ChangeLaneState.DOUBLE)
            else:
                # Delete points behind the car
                while len(self.planner.critical_waypoints) > 0 and self.planner.point_behind_car(self.planner.critical_waypoints[0]):
                    self.planner.critical_waypoints.pop(0)
                self.planner.publish_waypoints()
        else:
            car_pos = self.planner.car_position
            if(self.initial_car_pos_roadblock[0] == 0 and self.initial_car_pos_roadblock[1] == 0):
                self.initial_car_pos_roadblock = car_pos
            else:
                self.dist_to_roadblock = 0 #15 - np.sqrt((car_pos[0] - self.initial_car_pos_roadblock[0])**2 + (car_pos[1] - self.initial_car_pos_roadblock[1])**2)
                print "pub dist to roadblock: " + str(self.dist_to_roadblock)
                msg = Float32()
                msg.data = self.dist_to_roadblock
                self.planner.stop_dist_pub.publish(msg)
        '''

        #Assume no roadblocks during lane change
        # Delete points behind the car
        while len(self.planner.critical_waypoints) > 0 and self.planner.point_behind_car(self.planner.critical_waypoints[0]):
            self.planner.critical_waypoints.pop(0)
        self.planner.publish_waypoints()

    def exit_state(self):
        super(ChangeLaneState, self).exit_state()
        self.planner.critical_waypoints = []
        self.planner.lane_change = "none"

class StopSignState(VehicleState):

    def __init__(self, planner):
        super(StopSignState, self).__init__(planner)
        self.transitions.append(self.steady_state_transition)
        self.stop_state = "None"
        self.dist_rem = 6.0
        self.prev_car_pos = [0, 0]
        self.stopped = False
        self.wait_start_time = 0
        self.moving_past_stop_sign = False
        self.placedOdomPoints = False

    def steady_state_transition(self):
        if self.moving_past_stop_sign:
            return SteadyState(self.planner)
        return None

    def before_stopped(self):
        """Behaviour of the vehicle before stopping at the stop sign"""
        # Activation parameters
        actSignDetection  = 60.0
        actHorizDetection = 20.0
        actOdomDetection  = self.dist_rem

        # Possible ROS messages to send
        stopMsg = Float32(data = self.planner.stop_sign_distance)
        horizMsg = Float32(data = self.planner.horiz_line_distance)

        print "SD: " + str(stopMsg.data) + ", HD: " +  str(horizMsg.data) + ", OD: " + str(self.dist_rem)

        # Activate stop sign detection
        if(self.stop_state == "None"):
            self.stop_state = "Stop_Sign_Detection"

        # Activate horizontal line detection
        #if stopMsg.data <= actHorizDetection and stopMsg.data != -1.0 and self.stop_state == "Stop_Sign_Detection":
        if stopMsg.data <= actHorizDetection and stopMsg.data != -1.0 and self.stop_state == "Stop_Sign_Detection":
            self.planner.horizontal_line_activate_pub.publish(True)
            self.stop_state = "Horiz_Line_Detection"
        else:
            self.planner.horizontal_line_activate_pub.publish(False)

        # Activate odom detection
        if(horizMsg.data <= actOdomDetection and horizMsg.data != -1.0 and self.stop_state == "Horiz_Line_Detection"):
             self.stop_state = "Odom_Detection"

        # Send message to velocity node, based on state
        if(self.stop_state == "Stop_Sign_Detection"):
            if(stopMsg.data != -1.0):
                self.planner.stop_dist_pub.publish(stopMsg)
        elif(self.stop_state == "Horiz_Line_Detection"):
            if(horizMsg.data != -1.0):
                self.planner.stop_dist_pub.publish(horizMsg)
        elif(self.stop_state == "Odom_Detection"):
            car_position = self.planner.car_position
            # print self.planner.car_position_temp
            if(self.prev_car_pos[0] == 0 and self.prev_car_pos[1] == 0):
                self.prev_car_pos = car_position

            actualDistance = self.planner.magnitude(self.prev_car_pos, car_position)

            #print("Distance Travelled: " + str(actualDistance))
            if(actualDistance < self.dist_rem):
                self.dist_rem -= actualDistance

            self.prev_car_pos = car_position
            if(self.dist_rem <= 0.0):
                self.planner.stop_dist_pub.publish(Float32(data = 0.0))
                #print "Published 0.0"
            else:
                self.planner.stop_dist_pub.publish(Float32(data = self.dist_rem))
                #print "Published " + str(self.dist_rem)
        else:
            return # invalid stop_state

        # We are stopped at stop sign
        if self.planner.car_speed < 0.010 and self.dist_rem <= 0.01 and self.stop_state == "Odom_Detection":
            self.stopped = True
            self.wait_start_time = time.time()
            self.planner.horizontal_line_activate_pub.publish(False)
            self.stop_state = "Stopped"
        else:
            print self.stop_state

    def after_stopped(self):
        """Behaviour of the vehicle after coming to a stop at the stop sign"""

        # We want to wait 2 seconds before moving again
        if time.time() - self.wait_start_time < 5:
            return

        # Tell the longitude controller that we're done stopping
        self.planner.stop_dist_pub.publish(Float32(data = 25000.0))
        self.moving_past_stop_sign = True
        self.planner.stop_sign_distance = -1.0
        self.planner.horiz_line_distance = -1.0

    def enter_state(self):
        super(StopSignState, self).enter_state()

    def run_state(self):
        if(self.stop_state != "Odom_Detection"):
            self.planner.generate_waypoints()
        else:
            self.planner.critical_waypoints = []
            car_position = self.planner.car_position
            car_heading = self.planner.car_heading
            direction = [np.cos(car_heading), np.sin(car_heading)]

            for i in range(1, 8):
                new_point = [0, 0]
                new_point[0] = car_position[0] + (self.planner.DISTANCE_BETWEEN_POINTS * 1.1 * i) * np.cos(car_heading) + (0) * np.sin(car_heading)
                new_point[1] = car_position[1] + (self.planner.DISTANCE_BETWEEN_POINTS * 1.1 * i) * np.sin(car_heading) + (0) * np.cos(car_heading)
                #print "X: " + str(new_point[0]) + "Y: " + str(new_point[1])

                self.planner.critical_waypoints.append(new_point)

            self.planner.publish_waypoints()

        if not self.stopped:
            self.before_stopped()
        else:
            self.after_stopped()
            pass

    def exit_state(self):
        super(StopSignState, self).exit_state()

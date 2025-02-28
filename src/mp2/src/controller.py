import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = False
        self.accelData = []
        self.time = 0
        self.timeData = []

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0
        
        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y
        # yaw = quaternion_to_euler(currentPose.pose.Quaternion)[2]
        orientation = currentPose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        velocity_x = currentPose.twist.linear.x
        velocity_y = currentPose.twist.linear.y
        vel = math.hypot(velocity_x, velocity_y)

        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        target_velocity = 16
        if len(future_unreached_waypoints) >= 2:
            tar_x, tar_y = future_unreached_waypoints[1]
        else:
            tar_x, tar_y = future_unreached_waypoints[-1]
        distx = tar_x-curr_x
        disty = tar_y-curr_y
        tar_theta = math.atan2(disty, distx)
        theta_error = tar_theta - curr_yaw
        # print("CURRENT YAW OF: ", curr_yaw, " TARGETED ANGLE OF: ", tar_theta)
        # print("CURRENT ERROR OF:", abs(theta_error)*(180/np.pi))
        if abs(theta_error) > 10*(np.pi/180):
            target_velocity = 8
        # print("TARGET VELOCITY OF: ", target_velocity)

        ####################### TODO: Your TASK 2 code ends Here #######################
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        target_steering = 0
        # lookahead_dist = 5 #i am testing it with 10, can tune later (this is the second approach from the doc)
        # lookahead_point = None
        # i = future_unreached_waypoints[0]
        # dist_x = i[0] - curr_x
        # dist_y = i[1] - curr_y
        # tot_dist = math.hypot(dist_x, dist_y)
        # print("distance to next waypoint is: ", tot_dist)
        # if tot_dist < lookahead_dist:
        #     lookahead_point = i
        # if lookahead_point is None:
        #     lookahead_point = target_point
        tar_x = 0
        tar_y = 0

        if len(future_unreached_waypoints) >= 2:
            tar_x, tar_y = future_unreached_waypoints[1]
        else:
            tar_x, tar_y = future_unreached_waypoints[-1]

        
        dist_x = tar_x - curr_x
        dist_y = tar_y - curr_y

        ld = math.hypot(dist_x, dist_y)
        
        # print("distance we're using is: ", ld)

        #print(f"dist_x: {dist_x}, dist_y: {dist_y}, ld: {ld}")
        tar_theta = math.atan2(dist_y, dist_x)
        alpha = tar_theta - curr_yaw #normalize?
        #alpha = (tar_theta - curr_yaw + np.pi) % (2*np.pi) - np.pi #prolly not needed      

        target_steering = math.atan((2*self.L *math.sin(alpha))/ld)

        # print("L param: ", self.L, "ld param: ", ld)
        # print("TARGETED ANGLE OF: ", tar_theta, "CURRENT YAW OF: ", curr_yaw)
        # print("TARGET STEERING IS ", target_steering, " ERROR TO POINT IS ", alpha)
        #print(f"tar_theta: {tar_theta}, curr_yaw: {curr_yaw}, alpha: {alpha}, tar_steer: {target_steering}")

        ####################### TODO: Your TASK 3 code starts Here #######################
        return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
            self.accelData.append(acceleration)
            self.time = self.time + 0.01
            self.timeData.append(self.time)
            
            # print(acceleration/1000)

        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)

    def getAccelData(self):
        return self.accelData, self.timeData

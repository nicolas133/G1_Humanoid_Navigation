#!/usr/bin/python3
from math import sqrt
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
import math
import time
import sys
import numpy as np


#####make robot talk ######
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
# calculates the desired angle of rotation to get from current coordinates to goal coordinates
def angle_calc(curr_Coord, goal_Coord):
    diff_x = goal_Coord[0] - curr_Coord[0]
    diff_y = goal_Coord[1] - curr_Coord[1]
    return math.atan2(diff_y, diff_x)
# ensures that calculated angle is within the bounds of [-pi, pi]
def angle_verifier(angle):
    if (angle > math.pi):
        return angle - (2 * math.pi)
    elif (angle < -math.pi):
        return angle + (2 * math.pi)
    else:
        return angle
class PID_is_COOL:

    def __init__(self, iface):
        # initialize the DDS channel
        ChannelFactoryInitialize(0, iface)
        ### create and start loco client ###
        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        self.sport_client.Start()

        # create subscriber
        self.odom_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
        self.odom_sub.Init(self.odom_handler, 10)  # 10 is the queue depth


        ##### WAY_Point_Handiling #########
        # self.waypoint = [[0,0], [1,-1],[0,0]]
        # self.waypoint = [[0,0], [1,0],[1,-1],[0,-1],[0,0]]
        self.waypoint = [[0,0]]
        self.distance_threshold = 0.07

        # defines current goal coordinates
        self.wayindex = 0
        self.current_target = self.waypoint[self.wayindex]
        self.finished = False
        self.y_threshold = .2
        self.x_threshold = .2
        self.x = 0.0
        self.y = 0.0


        ### INtialize PID #############
        ##### Pos ####
        self.Kp_pos = 1.0
        #### Vx position #####
        self.Kp_v = 1.0
        self.Ki_v = 0.0
        self.Kd_v = 0.1
        #### Theta #####
        self.Kp_theta = 1.0
        self.Ki_theta = 0.0
        self.Kd_theta = 0.1
        self.previous_error_v = 0.0
        self.previous_error_theta = 0.0
        self.integral_v = 0.0
        self.integral_theta = 0.0
        # for timing
        self.last_time = time.time()

        ##### init audio client####
        self.audio_client = AudioClient()
        self.audio_client.SetTimeout(10.0)
        self.audio_client.Init()

    def odom_handler(self, msg):
        self.x = msg.position[0]
        self.y = msg.position[1]
        self.vx = msg.velocity[0]
        self.roll = msg.imu_state.rpy[0]
        self.pitch = msg.imu_state.rpy[1]
        self.yaw = msg.imu_state.rpy[2]

        print(f"X={self.x}, Y={self.y}")

    def init_PID(self, dt):
        # Calculate error this should maybe be absolute value
        error_x = self.current_target[0] - self.x
        error_y = self.current_target[1] - self.y
        ### Set up cascade control for Pos using foward velocity
        pos_error = sqrt(error_x ** 2 + error_y ** 2)
        vel_set = self.Kp_pos * pos_error
        vel_error = vel_set - self.vx
        desired_angle = angle_verifier(angle_calc([self.x, self.y], self.current_target))

        #(roll, pitch, theta) = euler_from_quaternion([self.rotx, self.roty, self.rotz, self.rotw])

        error_theta = desired_angle - self.yaw
        # 2) Proportional terms
        P_theta= self.Kp_theta * error_theta
        P_v = self.Kp_v*vel_error
        # 3) Integral terms
        self.integral_v +=  vel_error* dt
        self.integral_theta += error_theta * dt
        I_v = self.Ki_v * self.integral_v
        I_theta = self.Ki_theta * self.integral_theta
        # 4) Derivative terms
        derivative_v = (vel_error - self.previous_error_v) / dt if dt > 0 else 0.0
        derivative_theta = (error_theta - self.previous_error_theta) / dt if dt > 0 else 0.0
        D_v = self.Kd_v * derivative_v
        D_theta = self.Kd_theta * derivative_theta
        # 5) Total outputs
        output_v = P_v + I_v + D_v
        output_theta = P_theta + I_theta + D_theta
        # 6) Save for next iteration
        self.previous_error_v = vel_error
        self.previous_error_theta= error_theta
        print(f"pid_v= {output_v},pid_sa={output_theta}")
        return output_v, output_theta

    def run_PID(self):
        while not self.finished:
            print("enterd run")
            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            # print current waypoint for debugging purposes
            print("Current WAYPOINT:", self.current_target)
            # if robot is close enough to goal destination, sets goal to be next desired waypoint

            error_x = self.current_target[0] - self.x
            error_y = self.current_target[1] - self.y

            if (abs(error_x) < self.x_threshold and abs(error_y) < self.y_threshold):
                # sets goal waypoint to next designated waypoint
                if (self.wayindex + 1 < len(self.waypoint)):
                    self.wayindex += 1
                    self.current_target = self.waypoint[self.wayindex]
                    print("currently going towards", self.current_target)
                # sets finished boolean variable to true if final waypoint has been achieved
                else:
                    self.finished = True
                    print("Finished")


            # if we're done, stop and exit loop
            if self.finished:
                print("Nice BRO")
                self.sport_client.Move(0, 0, 0.0)
                self.sport_client.WaveHand()
                self.audio_client.TtsMaker("Hello", 1)

            # compute PID outputs
            vx, vtheta = self.init_PID(dt)
            # send velocity (z yaw = 0)

            #self.sport_client.Move(0, 0.0, vtheta)
            #time.sleep(1)
            self.sport_client.Move(vx, 0.0, vtheta)
            time.sleep(1)

            # self.sport_client.SetVelocity(vx, vy, 0.0, dt)
            # small sleep to avoid busy spin

def main():
    print("------------------------------------------------------------")
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)
    controller = PID_is_COOL(sys.argv[1])
    controller.run_PID()
if __name__ == "__main__":
    main()

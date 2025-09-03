#!/usr/bin/python3
import time
import sys
import math
import matplotlib.pyplot as plt

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

#### message type if sport mode even though topic is /odommodestate
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

#####publisher shit##########
#from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_, RequestHeader_
#from cyclonedds.util import TimeSpec_

#####make robot talk ######
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class PID_is_COOL:
    def __init__(self, iface):



        #### for plotting ####
        self.robot_positions = []
        self.velocity_commands = []
        # flag to start logging odom
        self.odom_initialized = False

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

        # create publisher #
        # need to understand DDS request message type


        ##### WAY_Point_Handiling #########
        #self.waypoint = [[0,0], [1,-1],[0,0]]
        #self.waypoint = [[0,0], [1,0],[1,-1],[0,-1],[0,0]]
        self.waypoint = [[2,0]]
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


        #### Vx position #####
        self.Kp_x = 1.0
        self.Ki_x = 0.0
        self.Kd_x = 0.1


        #### Vy position #####
        self.Kp_y = 1.0
        self.Ki_y = 0.0
        self.Kd_y = 0.1

        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.integral_x       = 0.0
        self.integral_y       = 0.0

         # for timing
        self.last_time = time.time()


        ##### init audio client####
        self.audio_client = AudioClient()
        self.audio_client.SetTimeout(10.0)
        self.audio_client.Init()







    def odom_handler(self, msg):
       if self.odom_initialized == False:
           self.intial_pose_x = msg.position[0]
           self.intial_pose_y= msg.position[1]



       self.x = msg.position[0] -self.intial_pose_x
       self.y = msg.position[1] -self.intial_pose_y



       self.current_V = msg.velocity[0]
       self.SA = msg.imu_state.rpy[2]
       self.odom_initialized=True
       print(f"X={self.x}, Y={self.y}")

    def init_PID(self,dt):
         # Calculate error
        error_x = self.current_target[0] - self.x
        error_y = self.current_target[1] - self.y

       # 2) Proportional terms
        P_x = self.Kp_x * error_x
        P_y = self.Kp_y * error_y

        # 3) Integral terms
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        I_x = self.Ki_x * self.integral_x
        I_y = self.Ki_y * self.integral_y

        # 4) Derivative terms
        derivative_x = (error_x - self.previous_error_x) / dt if dt > 0 else 0.0
        derivative_y = (error_y - self.previous_error_y) / dt if dt > 0 else 0.0
        D_x = self.Kd_x * derivative_x
        D_y = self.Kd_y * derivative_y

        # 5) Total outputs
        output_x = P_x + I_x + D_x
        output_y = P_y + I_y + D_y

        # 6) Save for next iteration
        self.previous_error_x = error_x
        self.previous_error_y = error_y


        print(f"pid_x= {output_x},pid_y={output_y}")

        return output_x, output_y

    def run_PID(self):
        while not self.finished:
            print("enterd run")
            now = time.time()
            dt  = now - self.last_time
            self.last_time = now

            # defines the current error of x and y coordinates
            x_error = abs(self.x - self.current_target[0])
            y_error = abs(self.y - self.current_target[1])

            # print current waypoint for debugging purposes
            print("Current WAYPOINT:", self.current_target)

            # if robot is close enough to goal destination, sets goal to be next desired waypoint
            if (x_error < self.x_threshold and y_error < self.y_threshold):

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
                #self.audio_client.TtsMaker("Hello. What is up my name is Ham, I am a robot programmed to destroy all formes of human life.",1)



            # compute PID outputs
            vx, vy = self.init_PID(dt)
            # send velocity (z yaw = 0)
            print(f" vel has been set to {vx},[{vy}")
            self.sport_client.Move(vx, vy, 0.0)

            if self.odom_initialized:
                    self.velocity_commands.append((vx, vy))
                    self.robot_positions.append((self.x, self.y))




            # small sleep to avoid busy spin
            time.sleep(0.01)

        robot_xs, robot_ys = zip(*self.robot_positions)
        vel_xs, vel_ys = zip(*self.velocity_commands)

        fig, ax = plt.subplots()
        ax.plot(robot_xs, robot_ys, label="Robot Path", linewidth=2)



        #downsample velocity to reduce over crowding

        downsample= 32

        down_vel_xs = vel_xs[::downsample]
        down_vel_ys = vel_ys[::downsample]
        down_pos_xs = robot_xs[::downsample]
        down_pos_ys = robot_ys[::downsample]






        ax.quiver(down_pos_xs, down_pos_ys, down_vel_xs, down_vel_ys,angles='xy', scale_units='xy', scale=3, color='red', alpha=0.4, width=0.0048,headwidth=3, headlength=4.5)

        #circle = plt.Circle((obs_x, obs_y), obs_r, color='red', fill=False, linestyle='--', label='Initial Obstacle')
        #ax.add_patch(circle)

        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel("X position")
        ax.set_ylabel("Y position")
        ax.set_title("Humanoid Postion and Velocity with Goal:[3,0]")

        ax.set_xlim(0,4.5)
        ax.set_ylim(1,-1)
        #ax.legend()
        plt.grid()
        print('save fig')
        plt.savefig("plot")

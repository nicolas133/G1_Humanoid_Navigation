#!/usr/bin/python3
import math
import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


#####make robot talk ######
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


from qpsolvers import solve_qp


import matplotlib.pyplot as plt




class clf:


    def __init__(self,iface):

        ###intialize dds
        ChannelFactoryInitialize(0, iface)


         ##### ODOM_Logging #########
        self.robot_positions = []
        self.velocity_commands = []
        # flag to start logging odom
        self.odom_initialized = False


         ### create and start loco client ###
        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        self.sport_client.Start()

        # create subscriber
        self.odom_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
        self.odom_sub.Init(self.odom_handler, 10)  # 10 is the queue depth


        ##### init audio client####
        self.audio_client = AudioClient()
        self.audio_client.SetTimeout(10.0)
        self.audio_client.Init()



        ##### WAY_Point_Handiling #########
        #self.waypoint = [[0,0], [1,-1],[0,0]]
        # self.waypoint = [[0,0], [1,0],[1,-1],[0,-1],[0,0]]
        self.waypoint = [[3,0]]

        self.distance_threshold = 0.07

        # defines current goal coordinates
        self.wayindex = 0
        self.current_target = self.waypoint[self.wayindex]
        self.finished = False
        self.y_threshold = .2
        self.x_threshold = .2
        self.x = 0.0
        self.y = 0.0

        # for timing
        self.last_time = time.time()





    def odom_handler(self, msg):

        if self.odom_initialized == False:
           self.intial_pose_x = msg.position[0]
           self.intial_pose_y= msg.position[1]



        self.x = msg.position[0] -self.intial_pose_x
        self.y = msg.position[1] -self.intial_pose_y
        print(f"xposition is {self.x} and yposition is {self.y} ")



        self.current_V = msg.velocity[0]
        self.SA = msg.imu_state.rpy[2]
        self.odom_initialized=True

    def run_clf(self):
        while not self.finished:
            print("entered run")
            now = time.time()


            ####################### Waypoint Handiling #########################
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

            alpha=1

            clf = alpha*((error_x)**2 + (error_y)**2)

            p = np.eye(2,2)
            q =- np.array([1.1*error_x,1.1*error_y])

            A = - np.array([2*(self.x-1.5),3*(self.y+0+1)**2])
            h = -5*np.array([1**2-((self.x-1.5)**2+((abs(self.y+1)**3)-1))])#5*((self.x-1.8)**2+(self.y+0)**2-(1.5)**2)])


            #x=solve_qp(p,q,solver="cvxopt")
            x=solve_qp(p,q,A,h,solver="cvxopt")


            print(f"Vx command is {x[0]} the Vy command is {x[1]}")

            vx = x[0]
            vy = x[1]

            vx = x[0]  * .05
            vy = x[1] *.4

            print(f"Vx command is {vx} the Vy command is {vy}")

        # Apply lower limit threshold to vx
            if abs(vx) < 0.1 and vx != 0:
                print("Lower vx limit hit")
                vx = 0.2 if vx > 0 else -0.2

            # Apply lower limit threshold to vy
            if abs(vy) < 0.1 and vy != 0:
                print("Lower vy limit hit")
                vy = 0.2 if vy > 0 else -0.2


            self.sport_client.Move(vx, vy, 0.0)

            if self.odom_initialized:
                    self.velocity_commands.append((vx, vy))
                    self.robot_positions.append((self.x, self.y))

            time.sleep(.01)


        robot_xs, robot_ys = zip(*self.robot_positions)
        vel_xs, vel_ys = zip(*self.velocity_commands)

        fig, ax = plt.subplots()
        ax.plot(robot_xs, robot_ys, label="Robot Path", linewidth=2)


        downsample= 16

        down_vel_xs = vel_xs[::downsample]
        down_vel_ys = vel_ys[::downsample]
        down_pos_xs = robot_xs[::downsample]
        down_pos_ys = robot_ys[::downsample]

        ax.quiver(down_pos_xs, down_pos_ys, down_vel_xs, down_vel_ys,angles='xy', scale_units='xy', scale=2.5, color='red', alpha=0.4, width=0.0048,headwidth=3, headlength=4.5)

        #circle = plt.Circle((obs_x, obs_y), obs_r, color='red', fill=False, linestyle='--', label='Initial Obstacle')
        #ax.add_patch(circle)

        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel("X position")
        ax.set_ylabel("Y position")
        ax.set_title("Robot Path and Velocity Commands")

        ax.set_xlim(0,3.3)
        ax.set_ylim(1,-3)
        #ax.legend()
        plt.grid()
        print('save fig')
        plt.savefig("plot")


def main():
    print("------------------------------------------------------------")
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)
    controller = clf(sys.argv[1])
    controller.run_clf()


if __name__ == "__main__":
    main()

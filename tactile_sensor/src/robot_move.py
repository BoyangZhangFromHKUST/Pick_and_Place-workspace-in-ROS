import time
import urx
import logging
import sys

import json

with open('./config/robot_move.json', 'r') as f:
            data = json.load(f)
            l = data['l']
            v = data['v']
            a = data['a']
            r = data['r']


robot_ip = "192.168.1.102"
rob = urx.Robot(robot_ip)
time.sleep(1)

class move(object):

    def get_joint(self):
        joint = rob.getj()
        print("joint is: ", joint)
        return joint
    
    def initial_pose(self):
        self.pose = rob.getl()
        print ("initial pose:")
        print (self.pose)
    
    def gripper_collision_avoid(self):
        if self.pose[2] < -0.01:
            print("gripper collision detected")
            print("the gripper height is: "+str(self.pose[2]))
            time.sleep(1)
            self.pose[2] = 0.2
            rob.movep(self.pose, acc=a, vel=v, wait=False)
            print("now is safe to move")
            time.sleep(10)
    #if the gripper is out of the work region, move it to a safe place
    def over_workregion_avoid(self):
        if self.pose[0] > 1.12 or self.pose[1] < 0:
            print("over work region")
            print("the gripper position is: "+str(self.pose[0])+" ; "+str(self.pose[1]))
            time.sleep(1)
            self.pose[0] = 0.8
            self.pose[1] = 0.25
            rob.movep(self.pose, acc=a, vel=v, wait=False)
            print("now is safe to move")
            time.sleep(10)
    
    def square_move_example(self):
            self.pose[2] += l
            print ("goal pose:")
            print (self.pose)
            rob.movep(self.pose, acc=a, vel=v, wait=False)
            while True:
                p = rob.getl(wait=True)
                if p[2] > self.pose[2] - 0.05:
                    break
            print("sleeping for 2 seconds")
            time.sleep(2)
                
            self.pose[1] += l 
            print("goal pose:")
            print(self.pose)
            rob.movep(self.pose, acc=a, vel=v, wait=False)
            while True:
                p = rob.getl(wait=True)
                if p[1] > self.pose[1] - 0.05:
                    break
                
            print("sleeping for 2 seconds")
            time.sleep(2)
                
            self.pose[2] -= l
            print ("goal pose:")
            print (self.pose)
            rob.movep(self.pose, acc=a, vel=v, wait=False)
            while True:
                p = rob.getl(wait=True)
                if p[2] < self.pose[2] + 0.05:
                    break
                
            print("sleeping for 2 seconds")        
            time.sleep(2)
                
            self.pose[1] -= l
            print("goal pose:")
            print(self.pose)
            rob.movep(self.pose, acc=a, vel=v, wait=False)
            while True:
                p = rob.getl(wait=True)
                if p[1] < self.pose[1] + 0.05:
                    break

            print("sleeping for 2 seconds")        
            time.sleep(2)

            rob.close()

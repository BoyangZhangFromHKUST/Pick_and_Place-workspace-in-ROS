#%%
import numpy as np
import cv2
import sys
# print(sys.path)

from scipy.interpolate import griddata
import getStitched2 as getStitched
from nHHD2 import nHHD
from tracker2 import Tracker
import matplotlib.pyplot as plt
# import getStitched as getStitched

import rospy
from std_msgs.msg import String


import json
import requests

# v4l2-ctl --list-devices


class TOUCH(Tracker):

    def __init__(self, camera_id, sensor_id):
        self.camera_id = camera_id
        self.sensor_id = sensor_id

    def init_camera_first(self):
        self.cap = cv2.VideoCapture(self.camera_id)
        self.tracker = Tracker(adaptive=True, cuda=False) 

        print("cam is opened?", self.cap.isOpened())
        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.rect=[[((300, 100), (480, 280), (390, 190)), ((626, 100), (806, 280), (716, 190)), ((953, 100), (1133, 280), (1043, 190)), ((1280, 100), (1460, 280), (1370, 190))], [((300, 440), (480, 620), (390, 530)), ((626, 440), (806, 620), (716, 530)), ((953, 440), (1133, 620), (1043, 530)), ((1280, 440), (1460, 620), (1370, 530))], [((300, 780), (480, 960), (390, 870)), ((626, 780), (806, 960), (716, 870)), ((953, 780), (1133, 960), (1043, 870)), ((1280, 780), (1460, 960), (1370, 870))]]

         ##test buffer
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def get_touch_info_before(self):
        for i in range(3):
            ret, self.frame = self.cap.read()
        if ret is True:
            self.tracker.reset()
            # get the stitched image, and convert it to grayscale
            self.pack = getStitched.get_packed_tiles(self.frame, self.rect)
            self.gray = cv2.cvtColor(self.pack, cv2.COLOR_BGR2GRAY)

            # track the optical flow
            self.flow = self.tracker.track(self.gray)

            # decompose the optical flow
            self.decomposition_obj = nHHD(grid=(self.flow.shape[0], self.flow.shape[1]), spacings=(1, 1))
            self.decomposition_obj.decompose(self.flow,verbose=1)
            self.flow_3d = self.decomposition_obj.get_force_distribute()
            
            return self.gray, self.flow, self.flow_3d
         
    def get_touch_info_after(self):
        for i in range(3):
            ret, self.frame = self.cap.read()
        if ret is True:
            self.pack = getStitched.get_packed_tiles(self.frame, self.rect)
            self.gray = cv2.cvtColor(self.pack, cv2.COLOR_BGR2GRAY)
            self.flow = self.tracker.track(self.gray)

            self.decomposition_obj.decompose(self.flow,verbose=1)
            self.flow_3d = self.decomposition_obj.get_force_distribute()
            
            return self.gray, self.flow, self.flow_3d
        
    def get_touch_info_realtime(self):
        ret, self.frame = self.cap.read()
        if ret is True:
            self.pack = getStitched.get_packed_tiles(self.frame, self.rect)
            self.gray = cv2.cvtColor(self.pack, cv2.COLOR_BGR2GRAY)
            self.flow = self.tracker.track(self.gray)

            self.decomposition_obj = nHHD(grid=(self.flow.shape[0], self.flow.shape[1]), spacings=(1, 1))
            self.decomposition_obj.decompose(self.flow,verbose=1)
            self.flow_3d = self.decomposition_obj.get_force_distribute()
            
            return self.gray, self.flow, self.flow_3d
        
    def release_touch_cam(self):
        self.cap.release()
        
    ##############################

    def scale_variation(self, frame):
        # Lifan: scale the image to 1/4
        self.frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        return self.frame
    
    def arrow(self, flow):
        # with open('./config/arrow_parameter.json', 'r') as f:
        #     data = json.load(f)
        self.xrange = 740
        self.yrange = 560
        self.step = 20
        self.arrow_length = 2

        # create grid of points to sample flow
        flow_start = np.stack(np.meshgrid(range(0, flow.shape[1], 12), range(0, flow.shape[0], 12)), axis=-1)
        flow_end =(flow[flow_start[:, :, 1], flow_start[:, :, 0]] + flow_start).astype(np.int32)

        # create grid of points to draw arrows
        grid_x, grid_y = np.mgrid[0:self.xrange:self.step, 0:self.yrange:self.step]
        x_axis = grid_x.reshape(-1) .astype(np.int64)
        y_axis = grid_y.reshape(-1) .astype(np.int64)

        # get flow vector at each grid point
        flow_distance = (flow_start-flow_end) * self.arrow_length

        distance = np.linalg.norm((flow_distance[:,:,0],flow_distance[:,:,1]), axis=2) # try to change the color of the arrows
        color_param = np.sqrt(pow(np.mean(distance[0]),2)+pow(np.mean(distance[1]),2))

        grid_vector = griddata(flow_start.reshape(-1, 2).astype(np.int64), flow_distance.reshape(-1, 2).astype(np.int64), (grid_x, grid_y), method='cubic')    

        # draw arrows
        grid_vector[np.isnan(grid_vector)] = 0
        grid_flat = grid_vector.reshape(-1, 2).astype(np.int64)

        for i in range(grid_flat.shape[0]):
            cv2.arrowedLine(self.pack, (x_axis[i], y_axis[i]), (np.int64(grid_flat[i, 0] / 3+ x_axis[i]), np.int64(grid_flat[i, 1] / 3+ y_axis[i])), color = (0,0,255-color_param), thickness = 1, tipLength=0.3)
        
        return self.pack
    
    def three_dimension_reconstruction(self, flow_3d):
    #     # flow3d -> 555, 740, 3
    #     # x (555, 740, 1) y (555, 740, 1) 
        x_axis_len = flow_3d.shape[0]
        y_axis_len = flow_3d.shape[1]
        # z = flow_3d[:,:,2]

        x, y, z = np.meshgrid(np.linspace(0, x_axis_len, x_axis_len), np.linspace(0, y_axis_len, y_axis_len), np.linspace(0, 0, 1))
        # x_num, y_num, z_num = 

        u = flow_3d[:,:,0]
        v = flow_3d[:,:,1]
        w = flow_3d[:,:,2]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.quiver(x, y, z, u, v, w, length=0.1, normalize=True,color = 'r')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Force Distribution')
        plt.show()

    
    def signal(self, flow_3d):

        z = np.average(flow_3d[:,:,2])
        upper_threshold = 5
        flag = False
        if z > upper_threshold: # 5 is a threshold I assumed. It can be changed.
            flag = True
            ros_talker()
        return flag
    
def ros_talker():
    pub = rospy.Publisher('touch_signal', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        message = "touch the object"
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()
        break

def ros_talker_detection(): # this is for signal detection
    pub = rospy.Publisher('touch_signal', String, queue_size=10)
    rospy.init_node('touch_signal', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        message = "touch the object"
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()
        break

if __name__ == '__main__':

    touch_1 = TOUCH(camera_id = 2, sensor_id='01')
    touch_1.init_camera_first()

    
    while True:
        touch_1.get_touch_info_realtime()
        flow1 = touch_1.flow
        flow_3d1=touch_1.flow_3d

        arrows1 = touch_1.arrow(flow1)
        flag = touch_1.signal(flow_3d1)

        if flag == True:
            break

        cv2.imshow('frame', arrows1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


# %%

#%%
import numpy as np
import cv2
import src.utils as utils
# from src.getDensity import getFilteredDensity
from src.tracker import Tracker
import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import src.getStitched as getStitched
from src.nHHD import nHHD

from scipy.interpolate import griddata
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
        self.rect=getStitched.get_vertice("./config/sensor_%s.json"%self.sensor_id)
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

            self.decomposition_obj = nHHD(grid=(self.flow.shape[0], self.flow.shape[1]), spacings=(1, 1))
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
        with open('./config/arrow_parameter.json', 'r') as f:
            data = json.load(f)
            self.xrange = data['xrange']
            self.yrange = data['yrange']
            self.step = data['step']
            self.arrow_length = data['arrow_length']

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
        #555 740 540 720
        interval = 79
        z_axis = 50
        x_threshold = flow_3d.shape[0] // interval + 1
        y_threshold = flow_3d.shape[1] // interval + 1
        print(x_threshold, y_threshold)

        x, y, z = np.meshgrid(np.linspace(0, flow_3d.shape[0], x_threshold), np.linspace(0, flow_3d.shape[1], y_threshold), np.linspace(0, z_axis, 1))

        u = flow_3d[::interval,::interval,0] 
        v = flow_3d[::interval,::interval,1] 
        w = flow_3d[::interval,::interval,2]

        u = u.reshape(y_threshold,x_threshold,1)
        v = v.reshape(y_threshold,x_threshold,1)
        w = w.reshape(y_threshold,x_threshold,1)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.quiver(x, y, z, u, v, w, length = 1, arrow_length_ratio = 0.3, lw = 1.5, color = 'y')
        ax.set_zlim(-50,50)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Force Distribution')

        ax.set_facecolor('black') # set the background color
        ax.set_axis_off() # set the frame off

        plt.show()

    
    def signal(self, flow_3d):

        z = np.average(flow_3d[:,:,2])
        print(flow_3d[:,:,2].shape)

        flag = False
        if z > 5: # 5 is a threshold I assumed. It can be changed.
            flag = True

        return flag
           

if __name__ == '__main__':

    touch_1 = TOUCH(camera_id = 0, sensor_id='01')
    touch_1.init_camera_first()
    
    while True:
        touch_1.get_touch_info_realtime()
        flow_3d1=touch_1.flow_3d
    
        touch_1.three_dimension_reconstruction(flow_3d1) 
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break



# %%

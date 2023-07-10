#%%
import numpy as np
import cv2
import src.utils as utils
# from src.getDensity import getFilteredDensity
from src.tracker import Tracker
import matplotlib.pyplot as plt
import src.getStitched as getStitched
from src.nHHD import nHHD

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

            self.decomposition_obj.decompose(self.flow,verbose=1)
            self.flow_3d = self.decomposition_obj.get_force_distribute()
            
            return self.gray, self.flow, self.flow_3d
        
    def get_touch_info_realtime(self):
        ret, self.frame = self.cap.read()
        if ret is True:
            self.pack = getStitched.get_packed_tiles(self.frame, self.rect)
            self.gray = cv2.cvtColor(self.pack, cv2.COLOR_BGR2GRAY)
            self.flow = self.tracker.track(self.gray)

            self.decomposition_obj.decompose(self.flow,verbose=1)
            self.flow_3d = self.decomposition_obj.get_force_distribute()
            
            return self.gray, self.flow, self.flow_3d
        
    def release_touch_cam(self):
        self.cap.release()
           

if __name__ == '__main__':
    # Lifan: No contact in a bright environment to find the reference for the initialization
    touch_1 = TOUCH(camera_id = 0, sensor_id='00')
    touch_1.init_camera_first()
    touch_1.get_touch_info_before()

    #%%
    # Lifan: Read the flow after contact
    touch_1.get_touch_info_after()
    gray1 = touch_1.gray
    flow1 = touch_1.flow
    flow_3d1=flow1.reshape(-1,2)
    arrows1 = touch_1.arrows

    force = touch_1.force
    print(force.shape)

    plt.imshow(arrows1)


# %%

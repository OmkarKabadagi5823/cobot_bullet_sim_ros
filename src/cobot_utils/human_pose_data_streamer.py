import numpy as np
import threading
import time

class HumanPoseDataStreamer(threading.Thread):
    def __init__(self, data_file, fps=25):
        super(HumanPoseDataStreamer, self).__init__()
        
        self._frames = np.load(data_file)
        self._fps = fps
        self._joint_position_set = np.array([0.0] * 15)

    def run(self):
        for joint_position_set in self._frames:
            z_axis_min = np.min(joint_position_set, axis=0)[2]
            offset = np.array([0, 0, z_axis_min])

            self._joint_position_set = (joint_position_set - offset) / 1000.0
            
            time.sleep(1./self._fps)

    def get_joint_position_set(self):
        return self._joint_position_set

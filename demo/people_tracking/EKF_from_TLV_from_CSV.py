# Copyright 2019 The OpenRadar Authors. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

# Copy of EKF_from_TLV.py which reads point cloud data from a CSV

from mmwave.dataloader import radars as ti
from mmwave.tracking import ekf
from mmwave.tracking import gtrack_visualize as GTRACK_visualize
import time
import numpy as np

DMODE = '2D'

def unpack_data(data):
    """Unpack data from the radar"""
    data_np = {}
    data_np['range'] = data['range'].to_numpy()
    data_np['doppler'] = np.ones_like(data['doppler'].to_numpy()) / 10
    data_np['snr'] = data['snr'].to_numpy()
    if DMODE == '3D':
        data_np['azimuth'] = data['azimuth'].to_numpy()
        data_np['elevation'] = data['elevation'].to_numpy()
    else:
        data_np['angle'] = data['azimuth'].to_numpy()
    
    return data_np

if __name__ == '__main__':

    tracker = ekf.EKF(mode=DMODE)
    radar = ti.PCCSV()
    radar._initialize()

    #GTRACK_visualize.create()
    while True:
        time.sleep(.1)
        data = radar.sample()        
        if data is not None:
            data_np = unpack_data(data)
            print(len(data_np['range']))
            #frame = GTRACK_visualize.get_empty_frame()
            
            tracker.update_point_cloud(data_np)
            
            targetDescr, tNum = tracker.step()

            '''try:
                frame = GTRACK_visualize.update_frame(targetDescr, int(tNum[0]), frame)
            except:
                pass
            try:
                frame = GTRACK_visualize.draw_points(tracker.point_cloud, len(ranges), frame)
            except:
                pass
            if not GTRACK_visualize.show(frame, wait=10):
                break
            '''
        else:
            break
            '''
            frame = GTRACK_visualize.get_empty_frame()
#            frame = GTRACK_visualize.update_frame(target_desc, 0, frame)
            if not GTRACK_visualize.show(frame, wait=10):
                break
            '''

    #GTRACK_visualize.destroy()
    radar.close()
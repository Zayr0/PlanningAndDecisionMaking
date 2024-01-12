# -*- coding: utf-8 -*-
"""
Created on Wed Jan 10 16:56:32 2024

@author: Maria
"""

import numpy as np
import minimum_snap

def main_demo_v010():
    # v 0.1.0 test
    ax = [0.0, 1.0, 1.0,4.0, 5.0,8.0]
    ay = [0.0, 2.0, 4.0,8.0, 2.0,3.0]
    az = [0.0, 3.0, 1.0,2.0, 2.0,3.0]
    # add a classic case in paper
    # "minimum snap trajectory generation and control for quadrotors"
    ax = [0.0, 5.0,5.0,0.0]
    ay = [0.0, 0.0,6.0,6.0]
    az = [0.0, 0.0,6.0,6.0]
    
    waypts_ori = np.array([ax,ay,az])
    
    T = 10
    v0 = np.array([0,0,0])
    a0 = np.array([0,0,0])
    ve = np.array([0,0,0])
    ae = np.array([0,0,0])
    
    myMistGen = mist_generator()
    xxs,yys,zzs,tts = myMistGen.mist_3d_gen(waypts_ori,v0,a0,ve,ae,T)
    vaj_xy = myMistGen.mist_3d_vaj_gen(xxs,yys,zzs,tts)
    myMistGen.mist_3d_vis(waypts_ori,xxs,yys,zzs,tts,vaj_xy,True,True,True)

if __name__ == '__main__':
    main_demo_v010()
#!/usr/bin/env python

import sys,os
import numpy as np

pose_path = sys.argv[1]

fp = open(pose_path, 'rt')

inv_pose = None

for line in fp:

    pose = np.vstack((np.array(list(map(float,line.strip().split(' ')))).reshape(3,4), [0,0,0,1]))

    if inv_pose is None:
        inv_pose = np.linalg.inv(pose)


    new_pose = np.matmul(inv_pose, pose)

    print(' '.join(map(str,new_pose[:3,:4].flatten())))

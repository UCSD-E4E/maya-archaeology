#!/usr/bin/env python

import sys,os
import numpy as np
import transforms3d

pose_path = sys.argv[1]

fp = open(pose_path, 'rt')


inv_pose = None

for n,line in enumerate(fp):
    linelist = line.strip().split(' ')
    data = np.array(list(map(float,linelist)))
    timestamp = linelist[0]
    xyz = data[[1,2,3]]
    quaternion = data[[7,4,5,6]]
    pose_rot = transforms3d.quaternions.quat2mat(quaternion)
    pose = np.eye(4)
    pose[:3,3] = xyz
    pose[0:3,0:3] = pose_rot

    #pose = np.vstack((np.array(list(map(float,line.strip().split(' ')))).reshape(3,4), [0,0,0,1]))

    if inv_pose is None:
        inv_pose = np.linalg.inv(pose)

    new_pose = np.matmul(inv_pose, pose)

    print(timestamp + ' ' + ' '.join(map(str,new_pose[:3,3])) + ' ' + ' '.join(map(str,transforms3d.quaternions.mat2quat(new_pose[0:3,0:3])[[1,2,3,0]])))



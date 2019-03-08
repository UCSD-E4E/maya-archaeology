#!/usr/bin/env python

import sys,os
import numpy as np
import transforms3d

time_path = sys.argv[1]
pose_path = sys.argv[2]
optional_pose_path = ""
optional_timestamps = []
if len(sys.argv) > 3:
    optional_pose_path = sys.argv[3]
    fo = open(optional_pose_path, 'rt')
    for line in fo:
        optional_timestamps.append(float(line.strip().split()[0]))

ft = open(time_path, 'rt')
fp = open(pose_path, 'rt')

previous_timestamp = ""
previous_pose = np.eye(4)

data = []

min_angle = 0. # Radians
min_dist  = 0. # Meters

# Only keep the latest transform for each timestamp
for linet,linep in zip(ft,fp):
    timestamp = linet.strip().split()[0]
    pose = np.vstack((np.array(list(map(float,linep.strip().split(' ')))).reshape(3,4), [0,0,0,1]))
    if np.isnan(pose).any():
        continue

    if timestamp != previous_timestamp and previous_timestamp:
        data.append((previous_timestamp, previous_pose))
    
    previous_timestamp = timestamp
    previous_pose = pose
    
data.append((previous_timestamp, previous_pose))


previous_i = -1

for i, (timestamp, pose) in enumerate(data):
    line = timestamp + ' ' + ' '.join(map(str,pose[:3,3])) + ' ' + ' '.join(map(str,transforms3d.quaternions.mat2quat(pose[0:3,0:3])[[1,2,3,0]]))

    if previous_i >= 0:
        previous_pose = data[previous_i][1]
        diff = np.matmul(np.linalg.inv(previous_pose), pose)
        direc, angle, point = transforms3d.axangles.aff2axangle(diff)
        trans = diff[:3,3]

        if np.abs(angle) > min_angle or np.linalg.norm(trans) > min_dist:
            previous_i = i
            print(line)


    if i == 0:
        previous_i = 0
        print(line)


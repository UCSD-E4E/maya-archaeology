#!/usr/bin/env python

import argparse
import rosbag
import sys
import os
import cv2
import numpy as np
import warnings
from cv_bridge import CvBridge, CvBridgeError
import multiprocessing as mp
from multiprocessing import Pool
import subprocess
import shutil


def parse_trajectory(trajectory_file):
    f = open(trajectory_file, 'rt')
    timestamps = [float(line.split(' ')[0]) for line in f]
    return timestamps


def get_images(bag_file, timestamps):

    bag = rosbag.Bag(bag_file)
    depth_header_size = 80
    bridge = CvBridge()
    path_depth = 'depth_image'

    output_images = []

    image_idx = 0
    timestamp_idx = 0

    previous_image = None
    previous_timestamp = None

    for topic, msg, t in bag.read_messages(topics=['/camera/depth_registered/image_raw']):

        # Get image and timestamp from bag file
        curr_timestamp = t.to_sec()
        curr_image = bridge.imgmsg_to_cv2(msg, "16UC1")

        # The timestamp we are looking for
        target_t = timestamps[timestamp_idx]

        image_selected = None

        # Is the target in the past? It's not so good, but we'll associate the current image
        if target_t < curr_timestamp:
            image_selected = curr_image
        # Is the target in between current and previous timestamps?
        # If yes, add the image with the closest timestamp
        elif previous_timestamp is not None:
            if target_t >= previous_timestamp and target_t < curr_timestamp:
                if np.abs(target_t - previous_timestamp) < np.abs(target_t - curr_timestamp):
                    image_selected = previous_image
                else:
                    image_selected = curr_image
        # Oh, we do not deal with a target equal or greater than the last timestamp :(
        
        if image_selected is not None:
            output_images.append((target_t,image_selected))
	    #cv2.imwrite("depth_" + str(target_t) + ".png", image_selected)
            timestamp_idx += 1
            if timestamp_idx >= len(timestamps):
                break

        previous_image = curr_image
        previous_timestamp = curr_timestamp

    bag.close()

    if len(timestamps) != len(output_images):
        warnings.warn("PANIC: We did not pull exactly the same number of images as the number of timestamps!")

    return output_images



def generate_pointcloud((depth, ply_file, intrinsics)):

    fx, fy, cx, cy, scalingFactor = intrinsics

    depth = depth.astype(float) / scalingFactor

    print("Generating " + ply_file + "...")

    
    Y = ((np.arange(depth.shape[0]) - cy) * depth.T).T / fy # Multiply columns
    X = ((np.arange(depth.shape[1]) - cx) * depth) / fx # Multiply rows
    Z = depth

    mask = Z != 0.0
    X = X[mask]
    Y = Y[mask]
    Z = Z[mask]

    f = open(ply_file, "w")
    f.write('''ply
format ascii 1.0
element vertex %d
property float x
property float y
property float z
end_header
'''%(Z.size))
    
    for x,y,z in zip(X.flatten(), Y.flatten(), Z.flatten()):
        f.write("%f %f %f\n"%(x,y,z))


    #points = []    
    #for v in range(depth.size[1]):
    #    for u in range(depth.size[0]):
    #        Z = depth.getpixel((u,v)) / scalingFactor
    #        if Z==0: continue
    #        X = (u - centerX) * Z / focalLength
    #        Y = (v - centerY) * Z / focalLength
    #        points.append("%f %f %f\n"%(X,Y,Z))
    
    f.close()


def generate_pointclouds(images, intrinsics, temp_dir, n_threads = -1):
    if n_threads <= 0:
        n_threads = mp.cpu_count()

    

    data = [(depth, os.path.join(temp_dir, 'pointcloud_{0:.6f}.ply'.format(timestamp)), intrinsics) for timestamp,depth in images]
    pool = Pool(n_threads)
    pool.map(generate_pointcloud, data)
    
    f = open(os.path.join(temp_dir, 'pointclouds.txt'), 'wt')
    f.write('\n'.join(map(os.path.abspath, zip(*data)[1])))


# Main
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='''
    Create point clouds from a bag file and a trajectory file.
    Requires ROS with OpenCV.
    ''')

    parser.add_argument('bag_file', help='Input bag file (format: bag)')
    parser.add_argument('trajectory_file', help='Input trajectory file (format: orbslam txt)')
    parser.add_argument('-j', '--num-threads', type=int, default=-1, help='Number of threads to use (default: max available CPUs)')
    parser.add_argument('-i', '--intrinsics', metavar=('fx', 'fy', 'cx', 'cy', 'scale'), default=[525.0, 525.0, 319.5, 239.5, 5000.0], nargs=5, help='Camera intrinsics [fx fy cx cy scale] (default: %(default)s)')
    parser.add_argument('-mem', '--memory', help='Try to limit the memory usage (a little bit) by not computing normals', action='store_true')
    args = parser.parse_args()

    
    temp_dir = "temp_clouds"
    result_dir = "results"

    #1
    timestamps = parse_trajectory(args.trajectory_file)
    
    #2
    images = get_images(args.bag_file, timestamps)
    
    #3
    generate_pointclouds(images, args.intrinsics, temp_dir, args.num_threads)

    #4
    subprocess.call(" ".join([os.path.join('build', 'transform_concat'), args.trajectory_file, os.path.join(temp_dir, 'pointclouds.txt'), '' if args.memory else '-n']), shell=True)

    #5
    subprocess.call(" ".join([os.path.join('build', 'downsample'), "orbslam_cloud.ply", 'orbslam_cloud_downsampled.ply']), shell=True)
    
    #6
    os.remove(os.path.join(result_dir, "orbslam_cloud.ply"))
    os.remove(os.path.join(result_dir, "orbslam_cloud_downsampled.ply"))
    shutil.move("orbslam_cloud.ply", result_dir)
    shutil.move("orbslam_cloud_downsampled.ply", result_dir)

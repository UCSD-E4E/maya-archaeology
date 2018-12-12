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


#-------------------------------------------------------
def parse_trajectory(trajectory_file):
#-------------------------------------------------------
    f = open(trajectory_file, 'rt')
    timestamps = [float(line.split(' ')[0]) for line in f]
    return timestamps


#-------------------------------------------------------
def get_images(bag_file, timestamps, bag_topic):
#-------------------------------------------------------
    bag = rosbag.Bag(bag_file)
    depth_header_size = 80
    bridge = CvBridge()
    path_depth = 'depth_image'

    output_images = []

    image_idx = 0
    timestamp_idx = 0

    previous_image = None
    previous_timestamp = float("-inf")

    for topic, msg, t in bag.read_messages(topics=[bag_topic]):

        # Get image and timestamp from bag file
        curr_timestamp = t.to_sec()
        curr_image = bridge.imgmsg_to_cv2(msg, "16UC1")

        # The timestamp we are looking for
        target_t = timestamps[timestamp_idx]

        image_selected = None


        while target_t <= curr_timestamp:
            if np.abs(target_t - previous_timestamp) < np.abs(target_t - curr_timestamp):
                image_selected = previous_image
                print(target_t, previous_timestamp)
            else:
                image_selected = curr_image
                print(target_t, curr_timestamp)
            
            output_images.append((target_t,image_selected))
	    #cv2.imwrite("depth_" + str(target_t) + ".png", image_selected)
            timestamp_idx += 1
            if timestamp_idx >= len(timestamps): break
            target_t = timestamps[timestamp_idx]


        if timestamp_idx >= len(timestamps):
            break

        previous_image = curr_image
        previous_timestamp = curr_timestamp

    bag.close()

    if len(timestamps) != len(output_images):
        warnings.warn("PANIC: We did not pull exactly the same number of images as the number of timestamps!")
        print(str(len(output_images)) + " images and " + str(len(timestamps)) + " timestamps")

    
    return output_images



#-------------------------------------------------------
def generate_pointcloud((depth, ply_file, intrinsics, apply_filter)):
#-------------------------------------------------------

    fx, fy, cx, cy, scalingFactor = intrinsics

    depth = depth.astype(float) / scalingFactor

    if apply_filter:
        depth = filter(depth)

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


#-------------------------------------------------------
def filter(depth, max_dist=5.0, kernel_size=7):
#-------------------------------------------------------    
    depth[depth > max_dist] = 0
    
    mask = cv2.morphologyEx(depth, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)))
    
    depth[mask == 0] = 0

    return depth


#-------------------------------------------------------
def generate_all_pointclouds(traj_file, bag_file, bag_topic, intrinsics, temp_dir, apply_filter=False, n_threads = -1):
#-------------------------------------------------------
    timestamps = parse_trajectory(traj_file)
    
    print("Extracting depth images from bag file...")
    images = get_images(bag_file, timestamps, bag_topic)
 
    if n_threads <= 0:
        n_threads = mp.cpu_count()

    data = [(depth, os.path.join(temp_dir, 'pointcloud_{0:.6f}.ply'.format(timestamp)), intrinsics, apply_filter) for timestamp,depth in images]
    pool = Pool(n_threads)
    pool.map_async(generate_pointcloud, data).get(999999999) # map_async is used to catch interrupts

    f = open(os.path.join(temp_dir, 'pointclouds.txt'), 'wt')
    f.write('\n'.join(map(os.path.abspath, zip(*data)[1])))


#-------------------------------------------------------
def move(src, dst):
#-------------------------------------------------------
    if os.path.exists(dst):
        os.remove(dst)
    shutil.move(src, dst)



#-------------------------------------------------------
if __name__ == '__main__':
#-------------------------------------------------------
    parser = argparse.ArgumentParser(description='''
    Create point clouds from a bag file and a trajectory file.
    Requires ROS with OpenCV.
    ''')

    parser.add_argument('bag_file', help='Input bag file (format: bag)')
    parser.add_argument('trajectory_file', help='Input trajectory file (format: orbslam txt)')
    parser.add_argument('-j', '--num-threads', type=int, default=-1, help='Number of threads to use (default: max available CPUs)')
    parser.add_argument('-t', '--topic', default='/camera/depth_registered/image_raw', help='Bag file topic')
    parser.add_argument('-i', '--intrinsics', metavar=('fx', 'fy', 'cx', 'cy', 'scale'), default=[525.0, 525.0, 319.5, 239.5, 5000.0], nargs=5, help='Camera intrinsics [fx fy cx cy scale] (default: %(default)s)')
    parser.add_argument('-n', '--normals', help='Compute normals on the original model (uses a lot of extra memory)', action='store_true')
    parser.add_argument('-f', '--filter', help='Apply a filter on depth images (optimized for ZR300)', action='store_true')
    args = parser.parse_args()

    
    temp_dir = "temp_clouds"
    result_dir = "results"


    # Parse trajectory, extract depth images from bag file, and generate point clouds
    generate_all_pointclouds(args.trajectory_file, args.bag_file, args.topic, list(map(float, args.intrinsics)), temp_dir, args.filter, args.num_threads)


    # Transform and concatenate all point clouds
    subprocess.call(" ".join([os.path.join('build', 'transform_concat'), args.trajectory_file, os.path.join(temp_dir, 'pointclouds.txt'), '-n' if args.normals else '']), shell=True)


    # Downsample the resulting point cloud
    subprocess.call(" ".join([os.path.join('build', 'downsample'), "orbslam_cloud.ply", 'orbslam_cloud_downsampled.ply']), shell=True)


    # Colorize the downsampled point cloud
    subprocess.call(" ".join([os.path.join('build', 'colorize'), "orbslam_cloud_downsampled.ply", 'orbslam_cloud_colored.ply']), shell=True)


    # Copy the results to the results directory
    output_filename1 = os.path.join(result_dir, "orbslam_cloud.ply")
    output_filename2 = os.path.join(result_dir, "orbslam_cloud_downsampled.ply")
    output_filename3 = os.path.join(result_dir, "orbslam_cloud_colored.ply")
    move("orbslam_cloud.ply",             output_filename1)
    move("orbslam_cloud_downsampled.ply", output_filename2)
    move("orbslam_cloud_colored.ply",     output_filename3)

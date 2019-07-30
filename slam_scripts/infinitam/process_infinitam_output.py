#!/usr/bin/env python

import os, sys
import glob
import subprocess
import numpy as np

def main():

    if len(sys.argv) < 2:
        print("Usage: " + sys.argv[0] + " world.txt")
        sys.exit(1)

    poses_filename = sys.argv[1]

    basename, ext = os.path.splitext(poses_filename)

    if not ext:
        poses_filename += ".txt"

    pcd_filenames = glob.glob(basename + "*.pcd")


    matrices = {}

    with open(poses_filename, 'rt') as f:
        for i,line in enumerate(f):
            M = line.split(',')[:-1]
            line = next(f)
            invM = line.split(',')[:-1]
            matrices[i] = np.array(list(map(float, invM))).reshape(4,4)

    all_transformed = []

    for filename in pcd_filenames:
        if 'processed' in filename or 'transformed' in filename or 'scaled' in filename:
            continue
        
        pcd_basename = os.path.splitext(filename)[0]
        idx = int(pcd_basename.split('_')[-1])
        output_pcd_filename = pcd_basename + "_processed.pcd"
        
        if not os.path.exists(output_pcd_filename):
            subprocess.call("/home/e4e/workspace/maya_archaeology/infinitam/ProcessPoints/build/processPoints -v 1 -s 1 " + filename + " " + output_pcd_filename, shell=True)
        
        output_transform_pcd = pcd_basename + "_transformed.pcd"
        
        command = "pcl_transform_point_cloud " + output_pcd_filename + " " + output_transform_pcd + " -matrix " + ','.join(map(str, matrices[idx].flatten()))
        #command = "pcl_transform_point_cloud " + filename + " " + output_transform_pcd + " -matrix " + ','.join(map(str, matrices[idx].flatten()))
        subprocess.call(command, shell=True)
        
        all_transformed.append(output_transform_pcd)
        
        #all_transformed.append(output_pcd_filename)

    subprocess.call("pcl_concatenate_points_pcd " + " ".join(all_transformed), shell=True)


if __name__ == "__main__":
    main()



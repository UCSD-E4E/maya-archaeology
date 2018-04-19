#!/usr/bin/env python

import os, sys
import subprocess
import re
import numpy as np
import argparse
import matplotlib.pyplot as plt

def main():

    maxdist_re = re.compile(r'Max Distance: (\S+)')
    mindist_re = re.compile(r'Min Distance: (\S+)')
    rmse_re = re.compile(r'Distance,RMSE: ([^,]+),(\S+)')

    parser = argparse.ArgumentParser(description='Calculate the RMSE curve between two 3D point clouds.')
    parser.add_argument('filenames', nargs=2, metavar='file.pcd', help='The two files to compare (in PCD format)')
    parser.add_argument('--num-points', '-n', type=int, default=10, help='Number of points in the RMSE curve (default: %(default)s)')
    parser.add_argument('--correspondence', '-c', default="nn", help='Type of point correspondence (default: %(default)s)')
    parser.add_argument('--out-filename', default="out.pcd", help='Output 3D model with colored RMSE (ignoring cutoff) (default: %(default)s)')
    parser.add_argument('--pcl-binary', '-bin', default="/home/e4e/workspace/pcl/build/bin/pcl_compute_cloud_error", help='Location of the PCL tool binary (default: %(default)s)')
    parser.add_argument('--plot', '-p', action='store_true', help='Plot the RMSE curve (default: %(default)s)')
    parser.add_argument('--log', '-log', action='store_true', help='Use log space instead of linear space (default: %(default)s)')

    args = parser.parse_args()


    filename1 = args.filenames[0]
    filename2 = args.filenames[1]
    filename_out = args.out_filename
    binary = args.pcl_binary
    correspondence = args.correspondence
    num_points = args.num_points
    do_plot = args.plot
    use_log = args.log

    command_line = binary + " " + " ".join([filename1, filename2, filename_out]) + " -correspondence " + correspondence


    # Find max distance
    output = subprocess.check_output(command_line + " -m", shell=True).decode()

    max_distance = -1
    min_distance = -1
    for line in output.split('\n'):
        m = maxdist_re.search(line)
        if m: max_distance = float(m.group(1))
        m = mindist_re.search(line)
        if m: min_distance = float(m.group(1))

    if use_log:
        distances = np.geomspace(min_distance, max_distance, num=num_points)
    else:
        distances = np.linspace(min_distance, max_distance, num=num_points)

    distances_str = "\"" + " ".join(map(str, distances)) + "\""


    # Compute RMSE for multiple distances
    output = subprocess.check_output(command_line + " -d " + distances_str, shell=True).decode()


    all_rmse = np.zeros_like(distances)

    # Parse the output
    rmse = -1
    i = 0
    for line in output.split('\n'):
        m = rmse_re.search(line)
        if m:
            dist = float(m.group(1))
            rmse = float(m.group(2))
            if i < all_rmse.shape[0]:
                all_rmse[i] = rmse
                i += 1

            print(str(dist) + ',' + str(rmse))

    if do_plot:
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(distances, all_rmse, marker='o')
        ax.set_xlabel("Distance cutoff")
        ax.set_ylabel("RMSE")

        plt.show()

if __name__ == "__main__":
    main()



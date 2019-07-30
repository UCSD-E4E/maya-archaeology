import argparse
import rosbag
import sys
import os
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

def get_time_stamp(bag_file, timestamp_file):
	bag = rosbag.Bag(bag_file)
	timestamps = []
	depth_header_size = 80
	bridge = CvBridge()
	path_depth = 'depth_image'
	for topic, msg, t in bag.read_messages(topics=['/camera/depth_registered/image_raw']):
		cv_image = bridge.imgmsg_to_cv2(msg, "16UC1")
		file_seq = str(msg.header.seq)
		length = len(file_seq)
		file_seq = '0'*(6-length) + file_seq
		cv2.imwrite(os.path.join(path_depth, "depth" + file_seq + ".png"), cv_image)
                #print '\n=========================================separate line========================================\n'
		timestamps.append("%f\n"%t.to_sec())

	file = open(timestamp_file, "w")
	file.write("".join(timestamps))
	file.close()
	bag.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    This script extracts timestamps from a ros bag file.
    ''')

    parser.add_argument('bag_file', help='input bag file (format: bag)')
    parser.add_argument('timestamp_file', help='output timestamp file (format: txt)')
    args = parser.parse_args()

    get_time_stamp(args.bag_file,args.timestamp_file)

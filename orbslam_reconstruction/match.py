import argparse
import sys
import os
import numpy as np

def match(key_frame_file, all_frame_file):
	index = []

	key_frame_file = open(key_frame_file, "r")
	all_frame_file = open(all_frame_file, "r")
	selected_frame_file = open("selected_frame.txt", "w")

	key_frames = key_frame_file.readlines()
	all_frames = map(float, all_frame_file.readlines())

	print all_frames

	print len(key_frames)
	print len(all_frames)

	print key_frames[0]
	print key_frames[0].split(" ")[0]
	print range(len(key_frames))

	selected_frames = []
	for i in range(len(key_frames)):
 		difference = 0
		time_key = float(key_frames[i].split(" ")[0])
		selected_frame = (np.abs(np.asarray(all_frames)-time_key)).argmin()
		selected_frames.append("%d\n"%selected_frame)

	selected_frame_file.write("".join(selected_frames))
	key_frame_file.close()
	all_frame_file.close()
	selected_frame_file.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    This script find the position of key frames in all the bag file frames
    ''')

    parser.add_argument('key_frame_file', help='input key_frame_file file (format: txt)')
    parser.add_argument('all_frame_file', help='output all_frame_file file (format: txt)')
    args = parser.parse_args()

    match(args.key_frame_file,args.all_frame_file)

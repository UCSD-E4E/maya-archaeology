import argparse
from subprocess import call, Popen

def run(bag_file, key_frame_file):
    call(['mkdir', 'depth_image'])
    call(['mkdir', 'point_clouds'])
    call(['mkdir', 'point_clouds_timestamp'])
    call(['mkdir', 'selected_depth_image'])
    call(['python', 'get_time_stamp_and_depth_images.py',bag_file, 'timestamp.txt'])
    call(['python', 'match.py', key_frame_file, 'timestamp.txt'])
    call(['bash', 'select_depth_image.sh'])
    call(['bash', 'generate_pointclouds.sh'])
    call(['apply_trans_matrix/build/matrix_transform', key_frame_file])
    call(['downsample/build/downsample'])
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    This script extracts timestamps from a ros bag file.
    ''')

    parser.add_argument('bag_file', help='input bag file (format: bag)')
    parser.add_argument('key_traj_file', help='input orbslam2 timestamp file (format: txt)')
    args = parser.parse_args()

    run(args.bag_file,args.key_traj_file)

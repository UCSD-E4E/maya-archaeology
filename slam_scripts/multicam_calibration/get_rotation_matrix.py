import argparse
import sys
import os
import re

def get_rotation_matrix(camera_info_file_name, out_file_path):
    camera_info_file = open(camera_info_file_name)
    rotation = []
    translation = []

    lines = camera_info_file.readlines()

    camera_info_file.close()

    row1 = lines[25]
    row2 = lines[26]
    row3 = lines[27]

    nums1 = re.split("\[|,| |\]", row1)
    rotation.append(nums1[4])
    rotation.append(nums1[6])
    rotation.append(nums1[8])
    translation.append(nums1[10])

    nums2 = re.split("\[|,| |\]", row2)
    rotation.append(nums2[4])
    rotation.append(nums2[6])
    rotation.append(nums2[8])
    translation.append(nums2[10])

    nums3 = re.split("\[|,| |\]", row3)
    rotation.append(nums3[4])
    rotation.append(nums3[6])
    rotation.append(nums3[8])
    translation.append(nums3[10])

    rotation_file_name = out_file_path + "rotation_matrix.txt"
    rotation_file = open(rotation_file_name, 'w')

    for num in rotation:
        rotation_file.write(num + "\n")

    rotation_file.close()

    translation_file_name = out_file_path + "translation_vector.txt"
    translation_file = open(translation_file_name, 'w')

    for num in translation:
        translation_file.write(num + "\n")

    translation_file.close()

args = sys.argv

in_file_array = re.split("/|\.", args[1])
out_file = args[2]
in_file = out_file + "camchain-" + in_file_array[0] + in_file_array[1] + ".yaml"

get_rotation_matrix(in_file, out_file)

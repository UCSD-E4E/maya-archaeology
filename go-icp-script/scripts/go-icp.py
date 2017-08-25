from subprocess import call
import sys
import os

print 'should be lidar(model), slam(data)'

___,plyfile1 = os.path.split(sys.argv[1])
___,plyfile2 = os.path.split(sys.argv[2])

file1 = plyfile1[:-2] + 'cd'
file2 = plyfile2[:-2] + 'cd'
print file1, file2

#if not(os.path.exists(file1) and os.path.exists(file2)):
#    print "files do not exist, exiting."
#    quit()

if not os.path.isdir("../models"):
    call(['mkdir', '../models'])

models_dir = "../models/"+file2[:-4]+"/"

if not os.path.isdir(models_dir):
    call(['mkdir', models_dir])

call(['pcl1.8_ply2pcd', sys.argv[1], models_dir + file1])
call(['pcl1.8_ply2pcd', sys.argv[2], models_dir + file2])

call(['../build/./pcd_normalize', models_dir + file1, models_dir + file2])

call(['cp', 'sc1.pcd', models_dir + 'lidar.txt'])
call(['sed', '-i', '-e', "1,9d;11d", models_dir + 'lidar.txt'])
call(['sed', '-i', 's/POINTS //', models_dir + 'lidar.txt'])

call(['cp', 'sc2.pcd', models_dir + 'slam.txt'])
call(['sed', '-i', '-e', "1,9d;11d", models_dir + 'slam.txt'])
call(['sed', '-i', 's/POINTS //', models_dir + 'slam.txt'])

call(['pcl_normal_estimation', 'sc1.pcd', models_dir + 'sc1-n.pcd', '-k', '6'])
call(['pcl_pcd2ply', models_dir + 'sc1-n.pcd', models_dir + 'lidar.ply'])

f = open("centroid.txt", "r")
dat = f.read()
f.close()
dat2 = dat.split("\n")
NdDownsampled = int(dat2[-2])
print NdDownsampled, "will be sampled"

if not os.path.exists(models_dir+'config.txt'):
    call(['cp', 'config.txt', models_dir + 'config.txt'])

configFile = models_dir + 'config.txt'

call(['touch', models_dir+"out.txt"])

call(['time', 'GoICP', models_dir + 'lidar.txt', models_dir + 'slam.txt', str(NdDownsampled), configFile, models_dir + 'out.txt'])

f = open(models_dir + "out.txt","r")
dat = f.read()
f.close()
dat2 = dat.split(" ")
dat2 = dat2[3:]
remov = ['', '\n']
dat2 = [x for x in dat2 if x not in remov]

text1 = "%s %s %s %s\n" % (dat2[0], dat2[1], dat2[2], dat2[9])
text2 = "%s %s %s %s\n" % (dat2[3], dat2[4], dat2[5], dat2[10])
text3 = "%s %s %s %s\n" % (dat2[6], dat2[7], dat2[8], dat2[11])
text4 = "0 0 0 1\n"
textlist = [text1, text2, text3, text4]
print 'transformation matrix: \n', textlist

f = open("mat.txt","w")
f.writelines(textlist)
f.close()

call(['../build/./pcd_transform'])
call(['pcl_normal_estimation', 'sc2-t.pcd', models_dir + 'sc2-tn.pcd', '-k', '6'])
call(['pcl_pcd2ply', models_dir + 'sc2-tn.pcd', models_dir + 'slam-t.ply'])

call(['pcl_compute_cloud_error', 'sc1.pcd', 'sc2-t.pcd', models_dir + 'error-p2p.pcd', '-correspondence', 'nn'])
call(['pcl_compute_cloud_error', models_dir + 'sc1-n.pcd', models_dir + 'sc2-tn.pcd', models_dir + 'error-p2pl.pcd', '-correspondence', 'nnplane'])

call(['rm','sc1.pcd', 'sc2.pcd', 'sc2-t.pcd', models_dir + 'sc1-n.pcd', models_dir + 'sc2-tn.pcd'])
call(['mv','mat.txt', 'centroid.txt', models_dir])
call(['rm', models_dir + 'out.txt', models_dir + 'lidar.txt', models_dir + 'slam.txt'])
call(['rm', models_dir + file1, models_dir + file2])
call(['cp', sys.argv[2], models_dir])

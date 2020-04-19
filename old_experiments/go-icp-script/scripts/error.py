from subprocess import call
import sys
import os

print 'should be lidar(model), slam(data)'

___,ofile1 = os.path.split(sys.argv[1])
___,ofile2 = os.path.split(sys.argv[2])

ftype1 = ofile1[-3:]
ftype2 = ofile2[-3:]
ftypes = ['vtk', 'ply', 'pcd']

if not(ftype1 in ftypes and ftype2 in ftypes):
  print "file type", ftype1, "or", ftype2, "not supported. Only vtk and ply files are supported currently."
  quit()

file1 = ofile1[:-3] + 'pcd'
file2 = ofile2[:-3] + 'pcd'
print file1, file2

if not os.path.isdir("../models"):
    call(['mkdir', '../models'])

models_dir = "../models/e-"+file2[:-4]+"/"

if not os.path.isdir(models_dir):
    call(['mkdir', models_dir])

if ftype1 != 'pcd':
  call(['pcl1.8_'+ftype1+'2pcd', sys.argv[1], models_dir + file1])
else:
  call(['cp', sys.argv[1], models_dir])
if ftype2 != 'pcd':
  call(['pcl1.8_'+ftype2+'2pcd', sys.argv[2], models_dir + file2])
else:
  call(['cp', sys.argv[2], models_dir])

call(['pcl_normal_estimation', models_dir + file1, models_dir + ofile1[:-4] + '-n.pcd', '-k', '6'])
call(['pcl_normal_estimation', models_dir + file2, models_dir + ofile2[:-4] + '-n.pcd', '-k', '6'])

call(['pcl_compute_cloud_error', models_dir + file1, models_dir + file2, models_dir + 'error-p2p.pcd', '-correspondence', 'nn'])
call(['pcl_compute_cloud_error', models_dir + ofile1[:-4] + '-n.pcd', models_dir + ofile2[:-4] + '-n.pcd', models_dir + 'error-p2pl.pcd', '-correspondence', 'nnplane'])

call(['pcl1.8_pcd2ply', models_dir + 'error-p2p.pcd', models_dir + 'error-p2p.ply'])
call(['pcl1.8_pcd2ply', models_dir + 'error-p2pl.pcd', models_dir + 'error-p2pl.ply'])
